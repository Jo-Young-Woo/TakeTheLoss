#!/usr/bin/env python3
"""Published MIL field contract over a local Gazebo referee; no truth navigation.

Six-agent launches remap the absolute single-robot API and TF topics using
multi_agent_wiring.emulator_remappings. Each process still owns exactly one
slot's command watchdog, measured joints, scan filtering and MIL status.
"""
import argparse
import copy
import importlib
import json
import math
from pathlib import Path

import yaml


def measured_transform(parent, child, xyz, rpy, stamp):
    """Construct strict ROS float64 fields from YAML (which may contain ints)."""
    from geometry_msgs.msg import TransformStamped
    transform = TransformStamped()
    transform.header.stamp = stamp
    transform.header.frame_id, transform.child_frame_id = parent, child
    transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = map(float, xyz)
    cr, cp, cy = (math.cos(float(v)/2) for v in rpy)
    sr, sp, sy = (math.sin(float(v)/2) for v in rpy)
    q = transform.transform.rotation
    q.x, q.y, q.z, q.w = map(float, (sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy,
                                    cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy))
    return transform


class EmulatorCore:
    """Pure command gating; referee remains authoritative for HP, ammo and locks."""
    def __init__(self, spec, settings, slot, frames=None):
        if slot not in spec["bases"]["start_pose"]:
            raise ValueError("Unknown ego slot")
        self.spec, self.settings, self.slot = spec, settings, slot
        self.frames = frames
        self.state, self.state_t, self.cmd_t = {}, -math.inf, -math.inf
        self.command = (0., 0.)
        self.pan, self.tilt, self.joint_t = 0., 0., -math.inf
        self._gimbal_was_unlocked = False
        self.target = (0., 0.)
        self.sent = (0., 0.)
        self.speed = spec["ugv"]["pan_tilt"]["speed_dps"][0]
        self.local_reload_until = 0.
        self.running_cmd = "start"

    def receive_state(self, state, now):
        if not isinstance(state, dict) or not isinstance(state.get("ugvs"), dict):
            raise ValueError("Referee state must contain per-slot state")
        if state.get("phase") == "running" and self.state.get("paused") and not state.get("paused"):
            self.running_cmd = "resume"
        elif state.get("phase") == "ready":
            self.running_cmd = "start"
        if state.get("phase") == "ready" and self.state.get("phase") != "ready":
            self.local_reload_until = 0.
            self.command, self.cmd_t = (0., 0.), -math.inf
        self.state, self.state_t = copy.deepcopy(state), now

    def fresh(self, now):
        return 0 <= now-self.state_t <= self.settings["emulator"]["stale_referee_s"]

    def own(self):
        return self.state.get("ugvs", {}).get(self.slot, {})

    def unlocked(self, now, drive=False):
        lock = "drive_lock_until" if drive else "lock_until"
        return (self.fresh(now) and bool(self.own()) and self.state.get("phase") == "running"
                and not self.state.get("paused", False) and now >= self.own().get(lock, math.inf))

    def velocity(self, v, w, now):
        limits = self.spec["ugv"]["cmd_vel_limits"]
        if not all(math.isfinite(n) for n in (v, w)):
            v, w = 0., 0.
        self.command = (max(-limits["linear_x"], min(v, limits["linear_x"])),
                        max(-limits["angular_z"], min(w, limits["angular_z"])))
        self.cmd_t = now

    def output_velocity(self, now):
        if not self.unlocked(now, True) or not 0 <= now-self.cmd_t <= self.spec["ugv"]["cmd_vel_timeout"]:
            return 0., 0.
        return self.command

    def gimbal(self, pan, tilt, speed, acc, now):
        if not self.unlocked(now) or not all(math.isfinite(v) for v in (pan, tilt, speed, acc)):
            return "locked"
        config = self.spec["ugv"]["pan_tilt"]
        low, high = config["pan_units"]
        signs = ((self.frames.gimbal["pan_api_sign"], self.frames.gimbal["tilt_api_sign"])
                 if self.frames is not None else (-1., 1.))
        self.target = (signs[0]*math.radians(max(low, min(high, pan))*config["pan_deg_per_unit"]),
                       signs[1]*math.radians(max(low, min(high, tilt))*config["tilt_deg_per_unit"]))
        self.speed = max(config["speed_dps"][0], min(config["speed_dps"][1], speed))
        return "ok"

    def output_gimbal(self, now, dt):
        if not self.unlocked(now):
            # Freeze once on lock entry. Re-latching every tick integrates
            # gravity sag until the head reaches its lower limit in READY.
            if self._gimbal_was_unlocked:
                self.target = self.pan, self.tilt
                self.sent = self.target
            self._gimbal_was_unlocked = False
            return self.sent
        self._gimbal_was_unlocked = True
        step = math.radians(self.speed)*max(0., dt)
        self.sent = tuple(p+max(-step, min(step, target-p)) for p, target in zip(self.sent, self.target))
        return self.sent

    def fire(self, now):
        if not self.unlocked(now) or not 0 <= now-self.joint_t <= self.settings["emulator"]["stale_referee_s"]:
            return "locked", 0
        if self.own().get("ammo", 0) <= 0:
            return "empty", 0
        until = max(self.local_reload_until, self.own().get("reload_until", 0.))
        if now < until:
            return "reloading", math.ceil((until-now)*1000)
        self.local_reload_until = now+self.spec["ugv"]["laser"]["refire_interval_s"]
        return "fired", math.ceil((self.local_reload_until-now)*1000)

    def match_command(self):
        if self.state.get("paused"):
            return "pause"
        return {"ready": "standby", "running": self.running_cmd, "reporting": "report", "finished": "stop"}.get(self.state.get("phase"), "idle")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--spec", type=Path, required=True)
    parser.add_argument("--assumptions", type=Path, required=True)
    parser.add_argument("--frames", type=Path, help="Generated WINGBOT frame authority; omit only for legacy model")
    parser.add_argument("--ego-slot", default="B-2")
    parser.add_argument("--interface-package", default="mil_interfaces_sim")
    args, ros_args = parser.parse_known_args(argv)
    spec = yaml.safe_load(args.spec.read_text(encoding="utf-8"))
    settings = yaml.safe_load(args.assumptions.read_text(encoding="utf-8"))
    frames = None
    if args.frames:
        from wingbot_runtime import WingbotFrames, crop_scan, mask_self_returns
        frames = WingbotFrames(args.frames)
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
    from geometry_msgs.msg import Twist, TransformStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Image, JointState, LaserScan
    from std_msgs.msg import String, Float64
    from tf2_ros import TransformBroadcaster
    msg_types = importlib.import_module(args.interface_package+".msg")
    srv_types = importlib.import_module(args.interface_package+".srv")

    class Emulator(Node):
        def __init__(self):
            super().__init__("maicon_mil_emulator")
            self.core = EmulatorCore(spec, settings, args.ego_slot, frames)
            self.last_tick = self.now()
            self.last_match_command = None
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.match_pub = self.create_publisher(msg_types.Match, "/mil/match", qos)
            self.state_pub = self.create_publisher(msg_types.State, "/mil/state", qos)
            self.velocity_pub = self.create_publisher(Twist, "/arena/ego/cmd_vel", 10)
            self.pan_pub = self.create_publisher(Float64, "/arena/ego/pan_cmd", 10)
            self.tilt_pub = self.create_publisher(Float64, "/arena/ego/tilt_cmd", 10)
            self.fire_pub = self.create_publisher(String, "/arena/fire", 10)
            self.gimbal_pub = self.create_publisher(String, "/arena/gimbal", 10)
            self.image_pub = self.create_publisher(Image, "/image_raw", qos_profile_sensor_data)
            self.odom_pub = self.create_publisher(Odometry, "/odom", qos_profile_sensor_data)
            self.tf = TransformBroadcaster(self)
            self.create_subscription(String, "/arena/state", self.state, qos)
            self.create_subscription(Twist, "/cmd_vel", lambda m: self.core.velocity(m.linear.x, m.angular.z, self.now()), 10)
            self.create_subscription(Odometry, "/arena/ego/odom", self.odom, qos_profile_sensor_data)
            self.create_subscription(Image, "/arena/ego/image", self.image, qos_profile_sensor_data)
            self.create_subscription(JointState, "/arena/ego/joint_states", self.joints, qos_profile_sensor_data)
            if frames is not None:
                self.scan_pub = self.create_publisher(LaserScan, "/scan", qos_profile_sensor_data)
                self.create_subscription(LaserScan, "/arena/ego/scan_raw", self.scan, qos_profile_sensor_data)
                self.scan_error = None
            self.create_subscription(msg_types.OledLine, "/mil/oled", lambda m: self.get_logger().info(f"OLED[{m.line}]: {m.text[:21]}"), 10)
            self.create_service(srv_types.Gimbal, "/mil/gimbal", self.gimbal)
            self.create_service(srv_types.Fire, "/mil/fire", self.fire)
            self.create_timer(1/settings["emulator"]["control_hz"], self.tick)
            self.create_timer(1/settings["emulator"]["state_hz"], self.publish_state)

        def now(self):
            return self.get_clock().now().nanoseconds*1e-9

        def state(self, message):
            try:
                state = json.loads(message.data)
                self.core.receive_state(state, self.now())
                self.publish_match(self.core.match_command())
            except (ValueError, TypeError):
                self.get_logger().warning("Invalid referee state; commands remain gated by freshness")

        def publish_match(self, command):
            if command != self.last_match_command:
                self.match_pub.publish(msg_types.Match(mode="mission", cmd=command))
                self.last_match_command = command

        def publish_state(self):
            if not self.core.fresh(self.now()):
                self.publish_match("pause")
                return
            self.state_pub.publish(msg_types.State(hp=int(self.core.own().get("hp", 0)),
                hp_max=int(spec["ugv"]["hit_rules"]["hp"]), battery_pct=float(settings["emulator"]["battery_pct"])))

        def odom(self, message):
            # DiffDrive publishes measured wheel odometry relative to its initial pose.
            self.odom_pub.publish(message)
            tf = TransformStamped()
            tf.header, tf.child_frame_id = message.header, message.child_frame_id
            p, q = message.pose.pose.position, message.pose.pose.orientation
            tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = float(p.x), float(p.y), float(p.z)
            tf.transform.rotation = q
            self.tf.sendTransform(tf)

        def image(self, message):
            # Preserve actual pixels and row stride. Padding must not be channel-swapped.
            if message.encoding.lower() == "rgb8":
                data = bytearray(message.data)
                for y in range(message.height):
                    start = y*message.step
                    row = data[start:start+message.width*3]
                    row[0::3], row[2::3] = row[2::3], row[0::3]
                    data[start:start+message.width*3] = row
                message.data, message.encoding = bytes(data), "bgr8"
            if message.encoding.lower() == "bgr8":
                self.image_pub.publish(message)

        def joints(self, message):
            positions = dict(zip(message.name, message.position))
            pan_name, tilt_name = frames.joint_names if frames is not None else ("pan_joint", "tilt_joint")
            pan = next((p for n, p in positions.items() if n.split("::")[-1] == pan_name), None)
            tilt = next((p for n, p in positions.items() if n.split("::")[-1] == tilt_name), None)
            if pan is None or tilt is None or not all(math.isfinite(v) for v in (pan, tilt)):
                return
            self.core.pan, self.core.tilt, self.core.joint_t = pan, tilt, self.now()
            self.gimbal_pub.publish(String(data=json.dumps({"slot": args.ego_slot, "pan_rad": pan, "tilt_rad": tilt})))
            if frames is not None:
                transforms = [measured_transform(parent, child, xyz, rpy, message.header.stamp)
                              for parent, child, xyz, rpy in frames.tf_edges(pan, tilt)]
                self.tf.sendTransform(transforms)
                return
            base = settings["body"]["center"]
            pivot = settings["gimbal"]["pivot"]
            lidar = settings["lidar"]["pose"]
            transforms = []
            edges = [("base_link", "pan_link", [pivot[i]-base[i] for i in range(3)], [0, 0, pan]),
                      ("pan_link", "tilt_link", [0, 0, 0], [0, -tilt, 0]),
                      ("tilt_link", "camera_optical_frame", settings["camera"]["offset"], [-math.pi/2, 0, -math.pi/2]),
                      ("base_link", "laser_frame", [lidar[i]-base[i] for i in range(3)], lidar[3:])]
            edges += [("laser_frame", f"laser_sector_{i}", [0, 0, 0], [0, 0, yaw])
                       for i, yaw in enumerate(settings["lidar"]["sector_yaws"])]
            for parent, child, xyz, rpy in edges:
                transforms.append(measured_transform(parent, child, xyz, rpy, message.header.stamp))
            self.tf.sendTransform(transforms)

        def scan(self, message):
            try:
                ranges, masked = crop_scan(message.ranges, message.angle_min, message.angle_increment,
                                           message.header.frame_id, frames.data)
                ranges, self_hits = mask_self_returns(ranges, message.angle_min, message.angle_increment,
                                                     message.header.frame_id, frames)
                masked += self_hits
            except ValueError as error:
                if self.scan_error != str(error):
                    self.get_logger().error(str(error))
                    self.scan_error = str(error)
                return
            output = copy.deepcopy(message)
            output.ranges = ranges
            if len(output.intensities) == len(ranges):
                intensity = list(output.intensities)
                for index in masked:
                    intensity[index] = math.nan
                output.intensities = intensity
            self.scan_pub.publish(output)
            self.scan_error = None

        def gimbal(self, request, response):
            response.message = self.core.gimbal(request.pan, request.tilt, request.speed, request.acc, self.now())
            return response

        def fire(self, request, response):
            response.message, response.left_ms = self.core.fire(self.now())
            if response.message == "fired":
                self.fire_pub.publish(String(data=json.dumps({"slot": args.ego_slot, "pan_rad": self.core.pan, "tilt_rad": self.core.tilt})))
            return response

        def tick(self):
            now = self.now()
            dt, self.last_tick = max(0., now-self.last_tick), now
            v, w = self.core.output_velocity(now)
            message = Twist()
            message.linear.x, message.angular.z = float(v), float(w)
            self.velocity_pub.publish(message)
            pan, tilt = self.core.output_gimbal(now, dt)
            self.pan_pub.publish(Float64(data=float(pan)))
            self.tilt_pub.publish(Float64(data=float(tilt)))

    rclpy.init(args=ros_args)
    node = Emulator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # SIGINT can shut down the context before spin returns. The launch
        # watchdog tears down Gazebo too; publishing on a dead context errors.
        if rclpy.ok():
            node.velocity_pub.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
