#!/usr/bin/env python3
"""Pose-only MAICON device referee. Device truth never becomes perception input.

The pure core has no ROS or Gazebo dependency. The optional node consumes only
dynamic transforms, publishes state/events, and adds no ray/contact sensors.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

from arena_common import load_yaml, expand_boxes, fixed_geometry, target_metadata


def ray_box(origin, direction, bounds):
    """Nearest nonnegative intersection with an axis-aligned 3-D box."""
    lo, hi = 0., math.inf
    for axis, (lower, upper) in enumerate(zip(bounds[::2], bounds[1::2])):
        if abs(direction[axis]) < 1e-12:
            if not lower <= origin[axis] <= upper:
                return None
        else:
            a, b = (lower-origin[axis])/direction[axis], (upper-origin[axis])/direction[axis]
            lo, hi = max(lo, min(a, b)), min(hi, max(a, b))
            if lo > hi:
                return None
    return lo


def ray_sphere(origin, direction, center, radius):
    delta = [origin[i]-center[i] for i in range(3)]
    b = sum(delta[i]*direction[i] for i in range(3))
    discriminant = b*b-sum(x*x for x in delta)+radius*radius
    if discriminant < 0:
        return None
    near, far = -b-math.sqrt(discriminant), -b+math.sqrt(discriminant)
    return max(0., near) if far >= 0 else None


def ray_hemisphere(origin, direction, center, radius, facing):
    """Nearest exposed dome surface; the flat/back side never receives a hit."""
    distance = ray_sphere(origin, direction, center, radius)
    if distance is None or sum(a*b for a, b in zip(direction, facing)) >= 0:
        return None
    point = [o+distance*d-c for o, d, c in zip(origin, direction, center)]
    return distance if sum(a*b for a, b in zip(point, facing)) >= -1e-10 else None


def bounds(center, size):
    return tuple(v for c, s in zip(center, size) for v in (c-s/2, c+s/2))


def local_point(point, pose):
    x, y, z, yaw = pose
    dx, dy = point[0]-x, point[1]-y
    return (math.cos(yaw)*dx+math.sin(yaw)*dy,
            -math.sin(yaw)*dx+math.cos(yaw)*dy, point[2]-z)


def world_point(point, pose):
    x, y, z, yaw = pose
    return (x+math.cos(yaw)*point[0]-math.sin(yaw)*point[1],
            y+math.sin(yaw)*point[0]+math.cos(yaw)*point[1], z+point[2])


def beam_intersects_robot(segment, height, pose, footprint):
    """Finite beam segment against the rotated UGV footprint, with height gate."""
    if not pose[2] <= height <= pose[2]+footprint[2]:
        return False
    first = local_point((segment[0], segment[1], height), pose)
    last = local_point((segment[0], segment[2], height), pose)
    length = math.dist(first, last)
    if length <= 0:
        return False
    direction = tuple((b-a)/length for a, b in zip(first, last))
    hit = ray_box(first, direction, (-footprint[0]/2, footprint[0]/2,
                                    -footprint[1]/2, footprint[1]/2, 0., footprint[2]))
    return hit is not None and hit <= length


class ArenaReferee:
    def __init__(self, spec, layout, assumptions, robot_geometry=None, *, frames=None):
        self.spec, self.layout, self.settings = spec, layout, assumptions
        self.robot_geometry = robot_geometry
        self.frames = frames
        if frames is None and robot_geometry is None:
            raise ValueError("Provide generated WINGBOT frames or explicit legacy robot geometry")
        self.footprint = tuple(frames.data["footprint_lwh"] if frames is not None else spec["ugv"]["footprint_lwh"])
        self.targets = target_metadata(spec, layout)
        geometry = fixed_geometry(spec)
        self.static_bounds = [bounds(item["center"], item["size"]) for item in geometry]
        self.static_bounds += [bounds(item["center"], item["size"]) for item in expand_boxes(spec, layout)]
        self.reset()

    def reset(self):
        self.phase, self.started_at, self.reporting_at = "ready", None, None
        self.paused, self.last_t = False, None
        self.objectives, self.reported = {}, set()
        self.destroyed, self.target_hits = {"BLUE": 0, "RED": 0}, set()
        self.contacts, self.beams, self.gimbals, self.capture_since = {}, {}, {}, {}
        self.base_beams = {base: False for base in self.spec["bases"]["photoelectric_beam"]["segments"]}
        self.events, self.poses = [], {}
        self.ugvs = {slot: {"hp": self.spec["ugv"]["hit_rules"]["hp"],
                            "ammo": self.spec["ugv"]["laser"]["magazine"],
                            "lock_until": 0., "drive_lock_until": 0., "last_fire": -math.inf}
                     for slot in self.spec["bases"]["start_pose"]}

    def event(self, event_type, now, **fields):
        value = {"type": event_type, "time": float(now), **fields}
        self.events.append(value)
        return value

    def start(self, now, objectives=None):
        if self.phase != "ready":
            return False
        self.phase, self.started_at = "running", float(now)
        if objectives is not None:
            self.set_objectives(objectives)
        self.event("match_start", now)
        return True

    def set_objectives(self, objectives):
        valid = self.spec["bases"]["photoelectric_beam"]["segments"]
        for camp, base in objectives.items():
            if camp not in ("BLUE", "RED") or base not in valid or base[0] == camp[0]:
                raise ValueError("Objective must be a base in the opposing camp")
        self.objectives = dict(objectives)

    def report(self, now, reason="time"):
        if self.phase == "running":
            self.phase, self.reporting_at, self.paused = "reporting", float(now), False
            self.beams.clear()
            self.contacts.clear()
            self.event("match_report", now, reason=reason)

    def fire(self, slot, now, pan_rad=0., tilt_rad=0.):
        if slot not in self.ugvs:
            return "locked", 0
        state, rules = self.ugvs[slot], self.spec["ugv"]["laser"]
        if self.phase != "running" or self.paused or now < state["lock_until"]:
            return "locked", max(0, math.ceil((state["lock_until"]-now)*1000))
        if state["ammo"] <= 0:
            return "empty", 0
        remaining = rules["refire_interval_s"]-(now-state["last_fire"])
        if remaining > 0:
            return "reloading", math.ceil(remaining*1000)
        state["ammo"] -= 1
        state["last_fire"] = now
        self.gimbals[slot] = (pan_rad, tilt_rad)
        self.beams[slot] = now+rules["emit_s"]
        self.event("laser_fired", now, slot=slot, ammo=state["ammo"])
        return "fired", 0

    def _first_receiver(self, shooter):
        pose = self.poses[shooter]
        pan, tilt = self.gimbals.get(shooter, (0., 0.))
        if self.frames is not None:
            local_origin, local_direction = self.frames.laser(pan, tilt)
            origin = world_point(local_origin, pose)
            direction = world_point(local_direction, (0., 0., 0., pose[3]))
        else:
            offset = self.robot_geometry["camera"]["offset"]
            forward = offset[0]*math.cos(tilt)-offset[2]*math.sin(tilt)
            mount = (forward*math.cos(pan)-offset[1]*math.sin(pan),
                     forward*math.sin(pan)+offset[1]*math.cos(pan),
                     offset[0]*math.sin(tilt)+offset[2]*math.cos(tilt))
            pivot = self.robot_geometry["gimbal"]["pivot"]
            origin = world_point(tuple(a+b for a, b in zip(pivot, mount)), pose)
            yaw = pose[3]+pan
            direction = (math.cos(yaw)*math.cos(tilt), math.sin(yaw)*math.cos(tilt), math.sin(tilt))
        blockers = [d for box in self.static_bounds if (d := ray_box(origin, direction, box)) is not None]
        for slot, other in self.poses.items():
            if slot == shooter:
                continue
            local_origin = local_point(origin, other)
            c, s = math.cos(other[3]), math.sin(other[3])
            local_direction = (c*direction[0]+s*direction[1], -s*direction[0]+c*direction[1], direction[2])
            if self.frames is not None:
                for matrix, size in self.frames.collision_transforms(*self.gimbals.get(slot, (0., 0.))):
                    inverse = np.linalg.inv(matrix)
                    point = inverse @ [*local_origin, 1.]
                    ray = inverse[:3, :3] @ local_direction
                    hit = ray_box(point[:3], ray, bounds((0., 0., 0.), size))
                    if hit is not None:
                        blockers.append(hit)
            else:
                body = self.robot_geometry["body"]
                hit = ray_box(local_origin, local_direction, bounds(body["center"], body["size"]))
                if hit is not None:
                    blockers.append(hit)
        maximum = min([self.settings["laser_range"], *blockers])+self.settings["intersection_tolerance"]
        hits = []
        normals = {"N": (0., 1., 0.), "E": (1., 0., 0.), "S": (0., -1., 0.), "W": (-1., 0., 0.)}
        for index, target in enumerate(self.targets):
            if index in self.target_hits:
                continue
            normal, center = normals[target["face"]], target["receiver"]
            denominator = sum(a*b for a, b in zip(direction, normal))
            if denominator >= -1e-10:
                continue
            distance = sum((c-o)*n for c, o, n in zip(center, origin, normal))/denominator
            point = tuple(o+distance*d for o, d in zip(origin, direction))
            if 0 <= distance <= maximum and math.dist(point, center) <= target["diameter"]/2:
                hits.append((distance, ("target", index)))
        for slot, other in self.poses.items():
            if slot == shooter:
                continue
            domes = (self.frames.receivers() if self.frames is not None else
                     ((center, self.robot_geometry["receiver"]["radius"], None)
                      for center in self.robot_geometry["receiver"]["centers"]))
            for dome, radius, facing in domes:
                center = world_point(dome, other)
                distance = (ray_hemisphere(origin, direction, center, radius,
                            world_point(facing, (0., 0., 0., other[3]))) if facing is not None else
                            ray_sphere(origin, direction, center, radius))
                if distance is not None and distance <= maximum:
                    hits.append((distance, ("ugv", slot)))
        return min(hits)[1] if hits else None

    def _apply_hit(self, shooter, receiver, now):
        kind, key = receiver
        if kind == "target":
            if key in self.target_hits:
                return
            self.target_hits.add(key)
            target = self.targets[key]
            camp = "BLUE" if str(target["camp"]).startswith("B") else "RED"
            self.destroyed[camp] += 1
            self.event("target_hit", now, shooter=shooter, target=key, camp=camp, kind=target["kind"])
            return
        state, rules = self.ugvs[key], self.spec["ugv"]["hit_rules"]
        if now < state.get("invulnerable_until", 0):
            return
        state["hits_taken"] = state.get("hits_taken", rules["hp"]-state["hp"]) + 1
        state["hp"] = max(0, state["hp"]-1)
        state["invulnerable_until"] = now+rules["invuln_s"]
        state["lock_until"] = now+rules["lock_laser_tilt_s"]
        if state["hits_taken"] == 4:
            state["drive_lock_until"] = now+rules["on_4th_hit_lock_drive_s"]
        self.beams.pop(key, None)
        self.event("ugv_hit", now, shooter=shooter, victim=key, hp=state["hp"], friendly_fire=shooter[0] == key[0])

    def step(self, now, poses):
        if not math.isfinite(now) or (self.last_t is not None and now < self.last_t):
            raise ValueError("Referee needs a monotonic simulation clock")
        dt = 0. if self.last_t is None else now-self.last_t
        if dt > self.settings["max_contact_step_s"]:
            self.contacts.clear()  # Missing pose samples cannot prove continuous illumination.
        self.last_t = now
        self.poses = {slot: tuple(pose) for slot, pose in poses.items() if slot in self.ugvs}
        # Beam hardware is active before the match as well. Publish changes only;
        # these anonymous device events do not award a capture before 4:00.
        beams = self.spec["bases"]["photoelectric_beam"]
        for base, segment in beams["segments"].items():
            blocked = any(beam_intersects_robot(segment, beams["height"], pose, self.footprint)
                          for pose in self.poses.values())
            if blocked != self.base_beams[base]:
                self.base_beams[base] = blocked
                self.event("base_beam", now, base=base, blocked=blocked)
        if self.phase == "reporting":
            if now-self.reporting_at >= self.spec["match_timeline"]["reporting_s"] or len(self.reported) == len(self.ugvs):
                self.phase = "finished"
                self.event("match_finished", now)
            return
        if self.phase != "running" or self.paused:
            self.contacts.clear()
            return
        elapsed = now-self.started_at
        if elapsed >= self.spec["match_timeline"]["running_s"]:
            self.report(now)
            return
        active = set()
        for shooter, until in list(self.beams.items()):
            if now >= until or shooter not in self.poses or now < self.ugvs[shooter]["lock_until"]:
                continue
            receiver = self._first_receiver(shooter)
            if receiver is not None:
                key = (shooter, receiver)
                active.add(key)
                since = self.contacts.setdefault(key, now)
                if now-since+1e-9 >= self.spec["ugv"]["laser"]["hit_continuous_s"]:
                    self._apply_hit(shooter, receiver, now)
        self.contacts = {k: v for k, v in self.contacts.items() if k in active}
        if elapsed >= self.spec["match_timeline"]["objective_post_at_s"]:
            for camp, base in self.objectives.items():
                blocked = self.base_beams[base]
                if not blocked:
                    self.capture_since.pop(base, None)
                    continue
                since = self.capture_since.setdefault(base, now)
                if now-since >= self.settings["base_dwell_s"]:
                    self.event("base_captured", now, base=base, alliance=camp)
                    self.report(now, "m5_capture")
                    break

    def state(self, now):
        return {"phase": self.phase, "paused": self.paused,
                "elapsed": None if self.started_at is None else max(0., now-self.started_at),
                "ugvs": {slot: {**{k: v for k, v in state.items() if k != "last_fire"},
                                 "reload_until": (state["last_fire"]+self.spec["ugv"]["laser"]["refire_interval_s"]
                                                  if math.isfinite(state["last_fire"]) else 0.)}
                         for slot, state in self.ugvs.items()},
                "destroyed": dict(self.destroyed)}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--spec", type=Path, required=True)
    parser.add_argument("--layout", type=Path, required=True)
    parser.add_argument("--assumptions", type=Path, required=True)
    parser.add_argument("--robot-assumptions", type=Path)
    parser.add_argument("--frames", type=Path, help="Authoritative generated WINGBOT frames, replacing robot assumptions")
    args, ros_args = parser.parse_known_args(argv)
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage

    class RefereeNode(Node):
        def __init__(self):
            super().__init__("maicon_arena_referee")
            if args.frames:
                from wingbot_runtime import WingbotFrames
                self.core = ArenaReferee(load_yaml(args.spec), load_yaml(args.layout), load_yaml(args.assumptions),
                                        frames=WingbotFrames(args.frames))
            else:
                robot_path = args.robot_assumptions or args.assumptions.with_name("robot_assumptions.yaml")
                self.core = ArenaReferee(load_yaml(args.spec), load_yaml(args.layout), load_yaml(args.assumptions),
                                        load_yaml(robot_path))
            self.poses, self.pose_times, self.event_offset = {}, {}, 0
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.state_pub = self.create_publisher(String, "/arena/state", qos)
            self.events_pub = self.create_publisher(String, "/arena/events", 50)
            self.create_subscription(TFMessage, "/arena/dynamic_pose", self.receive_poses, 10)
            self.create_subscription(String, "/arena/control", self.control, 10)
            self.create_subscription(String, "/arena/fire", self.fire, 10)
            self.create_subscription(String, "/arena/gimbal", self.gimbal, 10)
            self.create_timer(1/self.core.settings["referee_hz"], self.tick)

        def now(self):
            return self.get_clock().now().nanoseconds*1e-9

        def receive_poses(self, msg):
            now = self.now()
            for transform in msg.transforms:
                if "::" in transform.child_frame_id:
                    continue  # Link-local transforms are not model world poses.
                name = transform.child_frame_id.split("::")[0].replace("ugv_", "")
                slot = name if name in self.core.ugvs else name[0:1]+"-"+name[1:]
                if slot not in self.core.ugvs:
                    continue
                p, q = transform.transform.translation, transform.transform.rotation
                yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
                self.poses[slot], self.pose_times[slot] = (p.x, p.y, p.z, yaw), now

        def control(self, msg):
            try:
                value, now = json.loads(msg.data), self.now()
                if value.get("cmd") == "reset":
                    self.core.reset()
                    self.event_offset = 0
                elif value.get("cmd") == "start":
                    self.core.start(now, value.get("objectives"))
                elif value.get("cmd") == "report":
                    self.core.report(now, "operator")
                elif value.get("cmd") in ("pause", "resume"):
                    self.core.paused = value["cmd"] == "pause"
                elif value.get("cmd") == "objectives":
                    self.core.set_objectives(value["objectives"])
                elif value.get("cmd") == "m6_received" and value.get("slot") in self.core.ugvs:
                    self.core.reported.add(value["slot"])
            except (ValueError, KeyError, TypeError):
                self.get_logger().warning("Invalid local referee control message")

        def fire(self, msg):
            try:
                value = json.loads(msg.data)
                self.core.fire(value["slot"], self.now(), float(value.get("pan_rad", 0)), float(value.get("tilt_rad", 0)))
            except (ValueError, KeyError, TypeError):
                self.get_logger().warning("Invalid local laser request")

        def gimbal(self, msg):
            try:
                value = json.loads(msg.data)
                slot = value["slot"]
                if slot in self.core.ugvs and self.now() >= self.core.ugvs[slot]["lock_until"]:
                    self.core.gimbals[slot] = (float(value["pan_rad"]), float(value["tilt_rad"]))
            except (ValueError, KeyError, TypeError):
                self.get_logger().warning("Invalid local gimbal request")

        def tick(self):
            now = self.now()
            fresh = {slot: pose for slot, pose in self.poses.items()
                     if now-self.pose_times[slot] <= self.core.settings["max_pose_age_s"]}
            self.core.step(now, fresh)
            for event in self.core.events[self.event_offset:]:
                self.events_pub.publish(String(data=json.dumps(event)))
            self.event_offset = len(self.core.events)
            self.state_pub.publish(String(data=json.dumps(self.core.state(now), allow_nan=False)))

    rclpy.init(args=ros_args)
    node = RefereeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
