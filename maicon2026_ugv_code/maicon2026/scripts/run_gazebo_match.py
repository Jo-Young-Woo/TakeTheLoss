#!/usr/bin/env python3
"""Fixed MAICON Gazebo rehearsal with one physical UGV by default.

B2 runs the current Agent + Nav2 against Gazebo LiDAR and odometry. In the
default one-robot test, three fixed virtual opponents provide M4/M6 detections
and judging without more Gazebo physics or LiDAR sensors. The existing World and
SimServer implement mission timing, synthetic vision/UAV, laser dwell,
cooldown, damage and base-entry scoring from the published rules. Camera
detections are synthetic; this does not validate a YOLO model or real server.

Start CONTROL=match ROBOT=B2 LAYOUT=fixed gazebo_nav_demo.sh first.
The fixed SDF structures and referee items come from the same layout data.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
import threading
import time
from dataclasses import replace
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from maicon26.config import Config  # noqa: E402
from maicon26.hal.gazebo import GazeboRobot  # noqa: E402
from maicon26.mission.manager import Agent  # noqa: E402
from maicon26.nav.ros_runtime import shutdown  # noqa: E402
from maicon26.sim.robot import SimServer  # noqa: E402
from maicon26.sim.fixed_maicon import FIXED_STAGE_POINT  # noqa: E402
from maicon26.sim.run import Match, score  # noqa: E402
from maicon26.sim.world import World  # noqa: E402
from maicon26.types import Pose2D  # noqa: E402
from maicon26.util import clamp, wrap  # noqa: E402

IDS = ("B1", "B2", "B3", "R1", "R2", "R3")


def parse_uav_schedule(spec: str):
    """Optional operator throttling, e.g. 90:2,160:5,220:off."""
    if not spec:
        return []
    events = []
    for item in spec.split(","):
        when_text, sep, period_text = item.partition(":")
        if not sep:
            raise ValueError(f"Invalid UAV schedule item: {item!r}")
        when = float(when_text)
        period = math.inf if period_text == "off" else float(period_text)
        if not math.isfinite(when) or when < 0 or period not in (1.0, 2.0, 5.0, math.inf):
            raise ValueError(f"Invalid UAV schedule item: {item!r}")
        events.append((when, period))
    if events != sorted(events):
        raise ValueError("UAV schedule times must be increasing")
    return events


def simulation_delta(previous: float, current: float) -> float:
    """Elapsed simulator time; a pause must never advance mission/referee timers."""
    if not math.isfinite(previous) or not math.isfinite(current):
        raise RuntimeError("Invalid Gazebo /clock; restart the match")
    if current < previous:
        raise RuntimeError("Gazebo /clock moved backwards; restart the match after world reset")
    return current - previous


def ready_step(robots, previous: float, recover_s: float = 0.):
    """Gate the entire mission tick on clock progress and current sensor input."""
    current = robots["B2"].now()
    halted = False
    try:
        delta = simulation_delta(previous, current)
        if delta > 0 and not all(robot.ready() for robot in robots.values()):
            # Different DDS topics can arrive a callback apart. Halt all drives
            # while awaiting fresh, valid data; never run Agent on stale input.
            deadline = time.monotonic() + recover_s
            for robot in robots.values():
                robot.stop()
            halted = True
            while recover_s > 0 and time.monotonic() < deadline:
                time.sleep(.02)
                current = robots["B2"].now()
                delta = simulation_delta(previous, current)
                if all(robot.ready() for robot in robots.values()):
                    for robot in robots.values():
                        robot.sensor_recoveries = getattr(robot, "sensor_recoveries", 0) + 1
                    break
            else:
                details = {ident: (robot.readiness_diagnostics() if hasattr(robot, "readiness_diagnostics")
                                   else getattr(robot, "scan_error", "")) for ident, robot in robots.items()}
                raise RuntimeError(f"Gazebo sensor/clock data stale or invalid; restart match. {details}")
    except RuntimeError:
        if not halted:
            for robot in robots.values():
                robot.stop()
        raise
    if delta == 0:
        for robot in robots.values():
            robot.stop()
    return current, delta


def sync_pose_measurements(ugvs, truth, previous, distances):
    """Measure Gazebo displacement independently of the referee's integration.

    World.step mutates each UGV pose. Neither the current measurement nor the
    previous measurement used for distance may share that mutable Pose2D.
    """
    for ident, pose in truth.items():
        old = previous.get(ident)
        if old is not None:
            step = math.hypot(pose.x - old.x, pose.y - old.y)
            if step < 1.0:  # Preserve the existing rejection of pose jumps.
                distances[ident] += step
        ugvs[ident].pose = pose.copy()
    return {ident: pose.copy() for ident, pose in truth.items()}


def threat_snapshot(agent, now):
    """Explain a blocked dynamic start without changing planner safety."""
    result = []
    for track in agent._av_tracks:
        px, py, radius = track.predict(now, 0.0, agent.cfg.avoid)
        result.append({"tid": track.tid, "ident": track.ident, "friend": track.friend,
                       "from_blob": track.from_blob, "sources": track.t_src,
                       "x": round(px, 3), "y": round(py, 3), "radius": round(radius, 3),
                       "age": round(track.age(now), 3),
                       "distance_to_robot": round(math.hypot(px-agent.stack.pose.x,
                                                             py-agent.stack.pose.y), 3)})
    return result


def object_snapshot(agent):
    """Measured map evidence for diagnosing classification/association errors."""
    return [{"point": [round(f.x, 4), round(f.y, 4)],
             "centre": [round(f.cx, 4), round(f.cy, 4)],
             "normal": [round(f.vx, 4), round(f.vy, 4)], "n": f.n,
             "geometry_key": getattr(f, "geometry_key", None),
             "votes": dict(f.votes), "kind": f.kind, "last_t": f.last_t}
            for f in agent.omap.faces]


class PoseFeed:
    """Gazebo ground truth for the referee only; Agent receives ROS sensors."""
    def __init__(self) -> None:
        self.proc = subprocess.Popen(
            ["ign", "topic", "-e", "--json-output", "-t",
             "/world/maicon_arena/dynamic_pose/info"],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
            text=True, bufsize=1)
        self.lock = threading.Lock()
        self.poses = {}
        self.thread = threading.Thread(target=self._read, name="gazebo-pose-feed", daemon=True)
        self.thread.start()

    def _read(self) -> None:
        wanted = {"wingbot_ugv": "B2", **{f"{i}_ugv": i for i in IDS if i != "B2"}}
        assert self.proc.stdout is not None
        for line in self.proc.stdout:
            try:
                pose_v = json.loads(line)
                current = {}
                for p in pose_v.get("pose", []):
                    ident = wanted.get(p.get("name"))
                    if ident is None:
                        continue
                    pos, q = p.get("position", {}), p.get("orientation", {})
                    x, y, z, w = (float(q.get(k, 0.0)) for k in ("x", "y", "z", "w"))
                    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
                    current[ident] = Pose2D(float(pos.get("x", 0.0)), float(pos.get("y", 0.0)), yaw)
                if current:
                    with self.lock:
                        self.poses.update(current)
            except (ValueError, TypeError, KeyError):
                continue

    def latest(self):
        with self.lock:
            return {k: v.copy() for k, v in self.poses.items()}

    def close(self) -> None:
        self.proc.terminate()
        try:
            self.proc.wait(timeout=2)
        except subprocess.TimeoutExpired:
            self.proc.kill()


class PatrolPilot:
    """Repeatable opposition traffic, not a copy of another team's AI."""
    def __init__(self, ident, robot, ugv, world):
        self.ident, self.robot, self.ugv, self.world = ident, robot, ugv, world
        self.index = 0
        y = world.geo.base_center(f"{ident[0]}-{ident[1]}")[1]
        if ident[0] == "B":
            self.route = [(1.05, y), (2.25, y), (2.65, 2.5), (1.05, y)]
        else:
            self.route = [(5.05, y), (3.85, y), (3.45, 2.5), (5.05, y)]

    def tick(self, now):
        u, w, robot = self.ugv, self.world, self.robot
        if w.ended or u.paused_left > 0:
            robot.stop()
            return
        if w.scenario == "maicon_fixed" and now >= w.cfg.mission.t_base_cmd:
            robot.stop()  # Keep the B2 M5 test from ending on a scripted teammate.
            return
        p = u.pose
        scan = robot.get_scan()
        # During M5 all opponents attempt their assigned opposing base.
        goal = w.geo.base_center(u.goal_base) if u.goal_base else self.route[self.index]
        dist = math.hypot(goal[0] - p.x, goal[1] - p.y)
        if dist < .25 and not u.goal_base:
            self.index = (self.index + 1) % len(self.route)
            goal = self.route[self.index]
        heading = math.atan2(goal[1] - p.y, goal[0] - p.x)
        enemy = min((o for o in w.ugvs if o.team != u.team),
                    key=lambda o: math.hypot(o.pose.x - p.x, o.pose.y - p.y))
        enemy_d = math.hypot(enemy.pose.x - p.x, enemy.pose.y - p.y)
        if enemy_d < 1.65 and not w._blocked(p.x, p.y, enemy.pose.x, enemy.pose.y,
                                            ignore_ugv=(u, enemy)) and u.ammo > 0:
            heading = math.atan2(enemy.pose.y - p.y, enemy.pose.x - p.x)
            err = wrap(heading - p.th)
            robot.set_pan_tilt(0.0, -3.0)
            if abs(err) < .11:
                robot.set_velocity(0.0, clamp(2.0 * err, -.5, .5))
                robot.laser_fire()
                return
        err = wrap(heading - p.th)
        front = scan.min_in_sector(0.0, .35) if scan is not None else 8.0
        if front < .42:
            left = scan.min_in_sector(.8, .25) if scan is not None else 0.0
            right = scan.min_in_sector(-.8, .25) if scan is not None else 0.0
            robot.set_velocity(0.0, .8 if left >= right else -.8)
        else:
            v = min(.28, .6 * dist) * max(0.0, 1.0 - abs(err) / 1.3)
            robot.set_velocity(v, clamp(1.6 * err, -1.2, 1.2))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--seed", type=int, default=2)
    ap.add_argument("--m3-count", type=int, default=4,
                    help="Dedicated random M3 cubes; must match the world generator")
    ap.add_argument("--layout", choices=["fixed", "random"], default="fixed")
    ap.add_argument("--physical-robots", choices=["one", "six"], default="one")
    ap.add_argument("--opponents", choices=["virtual", "none"], default="virtual",
                    help="none: single-UGV M3 drill without invisible opponent detections")
    ap.add_argument("--duration", type=float, default=300.0)
    ap.add_argument("--base-at", type=float, default=240.0)
    ap.add_argument("--smog-at", type=float, default=120.0)
    ap.add_argument("--uav-schedule", default="",
                    help="Optional organizer feed policy, e.g. 90:2,160:5,220:off")
    ap.add_argument("--uav-layout", choices=["image", "off"], default="image",
                    help="M3 footprint hints from calibrated /uav/image, or LiDAR-only fallback")
    ap.add_argument("--uav-calibration", type=Path,
                    default=ROOT / "ros2/gazebo/uav_grid_calibration.json",
                    help="Measured grid calibration JSON for the fixed camera (image mode only)")
    ap.add_argument("--localization-quality", choices=("off", "arena"),
                    default="arena" if os.environ.get("SLAM") == "arena" else "off")
    ap.add_argument("--report", type=Path, default=ROOT / "ros2/gazebo/match_trial.json")
    ap.add_argument("--require-perfect-m3", action="store_true",
                    help="Fail the trial unless all M3 counts are exact and B2 reaches the M5 base")
    args = ap.parse_args()
    if not 0 < args.base_at < args.duration:
        ap.error("require 0 < --base-at < --duration")
    if args.opponents == "none" and args.physical_robots != "one":
        ap.error("--opponents none requires --physical-robots one")
    try:
        uav_schedule = parse_uav_schedule(args.uav_schedule)
    except ValueError as exc:
        ap.error(str(exc))
    cfg = Config()
    cfg.arena = replace(cfg.arena, base_depth=.4, base_height=1.2)
    cfg.team, cfg.slot = "BLUE", 2
    cfg.nav.backend = "nav2"
    cfg.nav2.use_sim_time = True
    bx, by = cfg.arena.base_center("B-2")
    cfg.nav2.start_pose_override = (bx + .15, by, 0.0)
    cfg.nav2.publish_depth_obstacles = False
    cfg.depth.enable = cfg.map3d.enable = False
    cfg.mission.t_total = args.duration
    cfg.mission.t_base_cmd = args.base_at
    cfg.mission.prestage_at = max(5.0, args.base_at - 18.0)
    if args.layout == "fixed":
        cfg.explore.complete_m3_faces = True
        cfg.mission.stage_point_override = FIXED_STAGE_POINT
        cfg.mission.goal_entry_creep_s = 3.0
    cfg.mission.m3_first_s = min(cfg.mission.m3_first_s, args.duration / 3)
    world = World(cfg, seed=args.seed,
                  scenario="maicon_fixed" if args.layout == "fixed" else "random",
                  fixed_m3_count=args.m3_count)
    ids = (("B2",) if args.opponents == "none" else ("B2", "R1", "R2", "R3")) if args.physical_robots == "one" else IDS
    physical_ids = ("B2",) if args.physical_robots == "one" else IDS
    ugvs = {}
    for ident in ids:
        ugvs[ident] = world.add_ugv("BLUE" if ident[0] == "B" else "RED", int(ident[1]))
    if args.physical_robots == "one" and args.opponents == "virtual":
        for ident in ("R1", "R2", "R3"):
            u = ugvs[ident]
            # Inside the tracker's 0.75 m start-base identity gate.
            u.pose = Pose2D(5.25, world.geo.base_center(f"R-{ident[1]}")[1], math.pi)
    robots = {ident: GazeboRobot("" if ident == "B2" else ident,
                                require_arena_localization=ident == "B2" and args.localization_quality == "arena")
              for ident in physical_ids}
    feed = PoseFeed()
    uav_feed = None
    agent = None
    wall0 = time.monotonic()
    try:
        while time.monotonic() - wall0 < 30:
            if all(r.ready() for r in robots.values()) and len(feed.latest()) == len(physical_ids):
                break
            time.sleep(.1)
        if not all(r.ready() for r in robots.values()) or len(feed.latest()) != len(physical_ids):
            details = "; ".join(r.scan_error for r in robots.values() if r.scan_error)
            raise RuntimeError("Gazebo robot topics not ready; match ROBOT / CONTROL / LAYOUT settings. " + details)
        for ident in physical_ids:
            robots[ident].bind_referee(world, ugvs[ident])
        server = SimServer(world, ugvs["B2"])
        server.uav_image_only = True
        # Never use World.cubes as UAV observations in the Gazebo experiment.
        server.uav_layout_enabled = args.uav_layout == "image"
        if server.uav_layout_enabled:
            from maicon26.hal.gazebo_uav import GazeboUavFeed
            uav_feed = GazeboUavFeed(calibration_path=args.uav_calibration)
            server.uav_layout_provider = uav_feed.latest
        start_clock = robots["B2"].now()
        server.uav_stamp_offset = start_clock - world.t
        agent = Agent(robots["B2"], server, cfg=cfg, name="B2")
        pilots = [PatrolPilot(i, robots[i], ugvs[i], world)
                  for i in physical_ids if i != "B2"]
        last_clock = start_clock
        last_print = -10.0
        last_log = 0
        last_event = 0
        distances = {i: 0.0 for i in ids}
        prev = feed.latest()
        smog_off = False
        next_uav_event = 0
        uav_policy_events = []
        trajectory = []
        last_path_sample = -math.inf
        trial_error = None
        args.report.parent.mkdir(parents=True, exist_ok=True)
        print(f"[gazebo-match] start seed={args.seed} layout={world.layout_meta['used_layout_seed']} "
              f"physical_robots={len(robots)} virtual_opponents={len(ids)-len(robots)} "
              f"duration={args.duration}s base_at={args.base_at}s", flush=True)
        while time.monotonic() - wall0 < max(60.0, args.duration * 3):
            try:
                last_clock, delta = ready_step(robots, last_clock, recover_s=2.)
            except RuntimeError as error:
                trial_error = str(error)
                print(f"[gazebo-match] stopped: {trial_error}", flush=True)
                break
            if delta == 0:
                time.sleep(.05)
                continue
            truth = feed.latest()
            prev = sync_pose_measurements(ugvs, truth, prev, distances)
            while next_uav_event < len(uav_schedule) and world.t >= uav_schedule[next_uav_event][0]:
                _, period = uav_schedule[next_uav_event]
                world.set_uav_period(period)
                event = {"t": round(world.t, 2), "interval_s": None if math.isinf(period) else period,
                         "source": "operator_schedule"}
                uav_policy_events.append(event)
                print(f"[gazebo-match] UAV policy {event}", flush=True)
                next_uav_event += 1
            if args.smog_at > 0 and args.smog_at <= world.t < args.smog_at + 15 and not world.smog:
                world.smog = True
                world.events.append({"t": world.t, "type": "smog", "on": True, "broadcast": True})
            if world.smog and world.t >= args.smog_at + 15 and not smog_off:
                smog_off = True
                world.smog = False
                world.events.append({"t": world.t, "type": "smog", "on": False, "broadcast": True})
            if world.t >= args.base_at:
                for ident, u in ugvs.items():
                    if ident == "B2" or u.goal_base or args.layout == "fixed" or args.physical_robots == "one":
                        continue
                    opp = "RED" if u.team == "BLUE" else "BLUE"
                    u.goal_base = world.base_of_type(opp, world.base_types[f"{opp[0]}-{u.slot}"])
            for pilot in pilots:
                pilot.tick(world.t)
            agent.tick()
            if world.t - last_path_sample >= 1.0:
                last_path_sample = world.t
                q = truth.get("B2", Pose2D())
                p, od = agent.stack.pose, robots["B2"].get_odom()
                trajectory.append({"t": round(world.t, 2), "x": round(q.x, 3),
                                   "y": round(q.y, 3), "yaw": round(q.th, 4), "mode": agent.mode.value,
                                   "estimated": [round(p.x, 3), round(p.y, 3), round(p.th, 4)],
                                   "odom": [round(od.x, 3), round(od.y, 3), round(od.th, 4)],
                                   "localization": dict(getattr(robots["B2"], "localization_quality", {})),
                                   "position_error_m": round(math.hypot(q.x-p.x, q.y-p.y), 3)})
            steps = max(1, math.ceil(delta / .05))
            for _ in range(steps):
                world.step(delta / steps)
            if world.ended:
                for pilot in pilots:
                    pilot.robot.stop()
            if world.t - last_print >= 5:
                last_print = world.t
                p = agent.stack.pose
                q = truth.get("B2", Pose2D())
                print(f"[gazebo-match] t={world.t:5.1f} mode={agent.mode.value:9s} "
                      f"B2=({q.x:.2f},{q.y:.2f}) est=({p.x:.2f},{p.y:.2f}) "
                      f"err={math.hypot(q.x-p.x,q.y-p.y):.2f}m "
                      f"travel={distances['B2']:.2f}m hits={ugvs['B2'].hits_taken} "
                      f"shots={ugvs['B2'].shots} events={len(world.events)}", flush=True)
                # Preserve diagnostics even if the next sensor fault or external
                # shutdown aborts the run. A checkpoint is never a success report.
                args.report.with_suffix(".checkpoint.json").write_text(json.dumps({
                    "completed": False, "elapsed_sim_s": round(world.t, 2),
                    "B2_coverage": agent.coverage.summary(),
                    "B2_m3_counts": agent.coverage.m3_counts(agent.omap),
                    "B2_m3_submission": world.submissions.get(ugvs["B2"].id, {}).get("m3"),
                    "B2_object_faces": object_snapshot(agent),
                    "B2_object_association": dict(getattr(agent.omap, "association_stats", {})),
                    "B2_vision_audit": list(robots["B2"].vision_audit.values()),
                    "B2_exploration": agent.explorer.diagnostics,
                    "B2_trajectory": trajectory, "B2_log": agent.log,
                    "B2_threats": threat_snapshot(agent, last_clock),
                    "B2_threat_filter": getattr(agent.board, "stats", {}),
                    "B2_localization": dict(getattr(robots["B2"], "localization_quality", {})),
                    "UAV": uav_feed.diagnostics() if uav_feed else None,
                }, ensure_ascii=False, indent=2), encoding="utf-8")
            for t, msg in agent.log[last_log:]:
                if any(k in msg for k in ("mode ", "M1", "M2", "M3", "M5", "M6", "STOP", "SAFE_STOP", "SMOG", "UAV", "EXPLORE")):
                    print(f"  [{t:6.2f}] {msg}", flush=True)
            last_log = len(agent.log)
            for event in world.events[last_event:]:
                print(f"  [event {world.t:.1f}] {event}", flush=True)
            last_event = len(world.events)
            if world.ended and agent.mode.value == "DONE":
                break
            time.sleep(.05)
        for robot in robots.values():
            robot.stop()
        # Final pose/error and any final displacement must use the latest
        # Gazebo measurement, not the referee's last predicted pose.
        sync_pose_measurements(ugvs, feed.latest(), prev, distances)
        for ident, u in ugvs.items():
            u.distance = distances[ident]
        match = Match(world, [agent], [server], [robots["B2"]])
        result = score(match)
        result.update({
            "seed": args.seed, "layout": args.layout,
            "completed": agent.mode.value == "DONE" and trial_error is None,
            "error": trial_error if trial_error else (None if agent.mode.value == "DONE" else "wall_time_limit"),
            "physical_robots": args.physical_robots,
            "opponents": args.opponents,
            "controllers": {ident: ("current_agent_nav2" if ident == "B2" else
                                     "scripted_patrol_and_engagement" if ident in physical_ids else
                                     "stationary_virtual_opponent") for ident in ids},
            "simulation_clock_at_start": start_clock,
            "layout_seed": world.layout_meta["used_layout_seed"],
            "layout_meta": world.layout_meta,
            "arena": {"width": world.geo.W, "height": world.geo.H,
                      "structures": [{"cx": c.cx, "cy": c.cy, "sx": c.sx, "sy": c.sy,
                                      "m3": any(i.kind == "object" for i in c.items.values())}
                                     for c in world.cubes]},
            "requested_duration_s": args.duration, "base_command_at_s": args.base_at,
            "elapsed_sim_s": round(world.t, 2),
            "elapsed_wall_s": round(time.monotonic() - wall0, 2),
            "all_robot_travel_m": {k: round(v, 3) for k, v in distances.items()},
            "all_robot_truth": {k: vars(u.pose) for k, u in ugvs.items()},
            "B2_estimated_pose": vars(agent.stack.pose),
            "B2_pose_error_m": round(math.hypot(agent.stack.pose.x - ugvs['B2'].pose.x,
                                                agent.stack.pose.y - ugvs['B2'].pose.y), 3),
            "B2_nav": agent.stack.diagnostics(),
            "B2_localization": dict(getattr(robots["B2"], "localization_quality", {})),
            "sensor_recoveries": {ident: getattr(robot, "sensor_recoveries", 0) for ident, robot in robots.items()},
            "B2_mode": agent.mode.value,
            "B2_tick_over_budget": agent.stats["tick_over_budget"],
            "B2_trajectory": trajectory,
            "B2_coverage": agent.coverage.summary(),
            "B2_vision_audit": list(robots["B2"].vision_audit.values()),
            "B2_m3_counts": agent.coverage.m3_counts(agent.omap),
            "B2_m3_submission": world.submissions.get(ugvs["B2"].id, {}).get("m3"),
            "B2_object_faces": object_snapshot(agent),
            "B2_object_association": dict(getattr(agent.omap, "association_stats", {})),
            "B2_exploration": getattr(agent.explorer, "diagnostics", {}),
            "B2_threats": threat_snapshot(agent, last_clock),
            "B2_threat_filter": getattr(agent.board, "stats", {}),
            "B2_uav_self_filter": {"filtered": getattr(agent.tracker, "self_filtered", 0),
                                  "pruned": getattr(agent.tracker, "self_pruned", 0)},
            "UAV": {"camera": "/uav/image", "camera_height_m": 2.5,
                    "detections_source": "image_provider_only_no_ugv_detector",
                    "layout_source": "gazebo_uav_image" if uav_feed is not None else "disabled",
                    "layout_frames": server.uav_layout_frames,
                    "layout_boxes_total": server.uav_layout_boxes,
                    "image_feed": uav_feed.diagnostics() if uav_feed is not None else None,
                    "frames_generated": server.uav_frames_generated,
                    "frames_delivered": server.uav_frames_delivered,
                    "final_interval_s": world.uav_frame_interval("BLUE"),
                    "operator_policy_events": uav_policy_events,
                    "agent_state": agent.stats["uav_state"],
                    "state_changes": [[round(t - start_clock, 2), state]
                                      for t, state in agent.uavmon.changes]},
            "B2_log": agent.log,
        })
        args.report.parent.mkdir(parents=True, exist_ok=True)
        b2_score = result["agents"]["B2"]
        m3 = b2_score.get("m3") or {}
        result["acceptance"] = {"m3_exact": m3.get("exact", False),
                                "m5_reached": b2_score.get("m5_reached") is not None,
                                "requested": args.require_perfect_m3}
        result["acceptance"]["passed"] = (result["completed"] and result["acceptance"]["m3_exact"]
                                            and result["acceptance"]["m5_reached"])
        args.report.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
        print(f"[gazebo-match] report={args.report} B2={result['agents']['B2']} ", flush=True)
        return (0 if result["acceptance"]["passed"] else 2) if args.require_perfect_m3 else (0 if result["completed"] else 1)
    finally:
        if agent is not None:
            agent.stack.close()
        if uav_feed is not None:
            uav_feed.close()
        for robot in robots.values():
            robot.close()
        feed.close()
        shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
