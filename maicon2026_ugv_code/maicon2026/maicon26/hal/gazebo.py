"""ROS 2 Gazebo topics as the Robot HAL used by the mission Agent.

Only driving, LiDAR, odometry and simulation time are available. A Gazebo
visual target is not a camera detection or a functioning laser target.
"""
from __future__ import annotations

import math
import json
import threading
import time
from dataclasses import replace
from typing import Optional, Tuple

import numpy as np

from ..nav.ros_runtime import add_node, ensure_ros
from ..types import Frame, Pose2D, RobotStatus, Scan
from .base import Robot
from .gazebo_model import LIDAR_RPY, LIDAR_XYZ, lidar_frame
from .ros_mil import scan_in_base


def _seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class GazeboRobot(Robot):
    def __init__(self, topic_prefix: str = "", *, max_sensor_age: float = .75,
                 max_receive_age: float = 2.0, require_arena_localization: bool = False) -> None:
        if any(not math.isfinite(v) or v <= 0 for v in (max_sensor_age, max_receive_age)):
            raise ValueError("Gazebo freshness limits must be positive finite seconds")
        ensure_ros()
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import LaserScan

        prefix = f"/{topic_prefix.strip('/')}" if topic_prefix.strip("/") else ""
        self.node = Node(f"maicon26_gazebo_robot{prefix.replace('/', '_')}")
        self._Twist = Twist
        self._lock = threading.Lock()
        self._clock = 0.0
        self._clock_received_at = self._scan_received_at = self._odom_received_at = -math.inf
        self._odom_stamp = -math.inf
        self._clock_reset = False
        self._max_sensor_age = max_sensor_age
        self._max_receive_age = max_receive_age
        self._require_arena_localization = require_arena_localization
        self._localization_received_at = -math.inf
        self.localization_quality = {"valid": False, "status": "waiting_for_quality"}
        self._lidar_frame = lidar_frame(prefix)
        self.scan_error = ""
        self._scan: Optional[Scan] = None
        self._odom: Optional[Pose2D] = None
        self._pan = self._tilt = 0.0
        self._sim_world = self._sim_ugv = None
        self._frame: Optional[Frame] = None
        self._frame_t = -1e9
        self.vision_audit = {}  # Evaluation only; never passed in detection extras.
        self.cmd_count = self.nonzero_count = 0
        self.last_cmd = (0.0, 0.0)
        self.max_cmd_v = 0.0
        self.node.create_subscription(Clock, "/clock", self._on_clock, qos_profile_sensor_data)
        self.node.create_subscription(LaserScan, f"{prefix}/scan", self._on_scan, qos_profile_sensor_data)
        self.node.create_subscription(Odometry, f"{prefix}/odom", self._on_odom, qos_profile_sensor_data)
        if require_arena_localization:
            from std_msgs.msg import String
            self.node.create_subscription(String, "/arena_localization/quality", self._on_localization_quality, 10)
        self._pub_cmd = self.node.create_publisher(Twist, f"{prefix}/cmd_vel", 10)
        # This HAL node uses wall time, so the watchdog also runs while /clock is paused.
        self._watchdog_timer = self.node.create_timer(.1, self._watchdog)
        add_node(self.node)

    def bind_referee(self, world, ugv) -> None:
        """Attach the existing match rules and synthetic camera detections."""
        self._sim_world, self._sim_ugv = world, ugv

    def _on_clock(self, msg) -> None:
        stamp = _seconds(msg.clock)
        if not math.isfinite(stamp):
            return
        with self._lock:
            if stamp < self._clock:
                # A reset invalidates the referee, Agent timers and sensor history.
                # Require a new run instead of mixing two simulation epochs.
                self._clock_reset = True
                self._scan = self._odom = self._frame = None
                self._frame_t = -1e9
            self._clock = stamp
            self._clock_received_at = time.monotonic()

    def _on_scan(self, msg) -> None:
        try:
            if msg.header.frame_id != self._lidar_frame:
                raise ValueError(f"Expected scan frame {self._lidar_frame}, got {msg.header.frame_id!r}; "
                                 "regenerate the Fortress SDF and start its LiDAR static TF")
            amin, inc, ranges, rmax = scan_in_base(
                msg.angle_min, msg.angle_increment, msg.ranges, msg.range_min, msg.range_max,
                LIDAR_XYZ[0], LIDAR_XYZ[1], LIDAR_RPY[2])
        except (TypeError, ValueError) as error:
            with self._lock:
                self._scan = None
                self.scan_error = str(error)
            return
        scan = Scan(amin, inc, np.asarray(ranges, dtype=np.float32),
                    _seconds(msg.header.stamp), rmax)
        with self._lock:
            self._scan = scan
            self._scan_received_at = time.monotonic()
            self.scan_error = ""

    def _on_odom(self, msg) -> None:
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        th = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        with self._lock:
            self._odom = Pose2D(float(p.x), float(p.y), th)
            self._odom_stamp = _seconds(msg.header.stamp)
            self._odom_received_at = time.monotonic()

    def _fresh_unlocked(self, stamp: float, received: float, wall_now: float) -> bool:
        # DDS callbacks can deliver a sensor slightly ahead of the latest /clock.
        return (math.isfinite(stamp) and -.1 <= self._clock - stamp <= self._max_sensor_age
                and 0 <= wall_now - received <= self._max_receive_age)

    def _on_localization_quality(self, message) -> None:
        try:
            quality = json.loads(message.data)
            if (not isinstance(quality, dict) or not isinstance(quality.get("valid"), bool)
                    or not math.isfinite(float(quality.get("stamp")))):
                raise ValueError("Malformed localization quality")
        except (ValueError, TypeError):
            quality = {"valid": False, "status": "invalid_quality", "stamp": None}
        with self._lock:
            self.localization_quality = quality
            self._localization_received_at = time.monotonic()

    def _localization_ready_unlocked(self, wall_now: float) -> bool:
        if not self._require_arena_localization:
            return True
        stamp = self.localization_quality.get("stamp", -math.inf)
        return (self.localization_quality.get("valid") is True
                and self._fresh_unlocked(float(stamp), self._localization_received_at, wall_now)
                and -.1 <= self._clock-float(stamp) <= .5)

    def _ready_unlocked(self, wall_now: float) -> bool:
        return (not self._clock_reset and self._clock > 0
                and self._localization_ready_unlocked(wall_now)
                and 0 <= wall_now - self._clock_received_at <= self._max_receive_age
                and self._scan is not None and self._odom is not None
                and self._fresh_unlocked(self._scan.stamp, self._scan_received_at, wall_now)
                and self._fresh_unlocked(self._odom_stamp, self._odom_received_at, wall_now))

    def ready(self) -> bool:
        with self._lock:
            return self._ready_unlocked(time.monotonic())

    def readiness_diagnostics(self) -> dict:
        with self._lock:
            wall = time.monotonic()
            return {"ready": self._ready_unlocked(wall), "clock": self._clock,
                    "clock_reset": self._clock_reset, "scan_error": self.scan_error,
                    "scan_age_s": None if self._scan is None else self._clock-self._scan.stamp,
                    "odom_age_s": self._clock-self._odom_stamp,
                    "receive_ages_s": {"clock": wall-self._clock_received_at,
                                       "scan": wall-self._scan_received_at,
                                       "odom": wall-self._odom_received_at},
                    "localization": dict(self.localization_quality)}

    def _watchdog(self) -> None:
        if self.last_cmd != (0.0, 0.0) and not self.ready():
            self.stop()

    def now(self) -> float:
        with self._lock:
            return self._clock

    def set_velocity(self, v: float, w: float) -> None:
        if (v or w) and not self.ready():
            v = w = 0.0
        m = self._Twist()
        m.linear.x, m.angular.z = float(v), float(w)
        self._pub_cmd.publish(m)
        self.cmd_count += 1
        self.nonzero_count += abs(v) > 1e-4 or abs(w) > 1e-4
        self.last_cmd = (float(v), float(w))
        self.max_cmd_v = max(self.max_cmd_v, abs(v))
        if self._sim_ugv is not None:
            self._sim_ugv.cmd_v, self._sim_ugv.cmd_w = float(v), float(w)

    def set_pan_tilt(self, pan_deg: float, tilt_deg: float) -> None:
        self._pan, self._tilt = float(pan_deg), float(tilt_deg)
        if self._sim_ugv is not None:
            self._sim_ugv.pan_cmd, self._sim_ugv.tilt_cmd = self._pan, self._tilt

    def get_pan_tilt(self) -> Tuple[float, float]:
        if self._sim_ugv is not None:
            return self._sim_ugv.pan, self._sim_ugv.tilt
        return self._pan, self._tilt

    def laser_fire(self) -> bool:
        return self._sim_ugv.fire() if self.ready() and self._sim_ugv is not None else False

    def laser_abort(self) -> None:
        if self._sim_ugv is not None:
            self._sim_ugv.abort()

    def get_scan(self) -> Optional[Scan]:
        with self._lock:
            return self._scan if self._ready_unlocked(time.monotonic()) else None

    def get_frame(self) -> Optional[Frame]:
        world, ugv = self._sim_world, self._sim_ugv
        if world is None or ugv is None:
            return None
        now = self.now()
        if now - self._frame_t >= .1:
            self._frame_t = now
            cam = world.cfg.camera
            detections = world.camera_detections(ugv)
            for det in detections:
                truth = det.extra.get("_gt")
                if det.extra.get("kind") == "object" and isinstance(truth, tuple) and len(truth) == 3:
                    key = tuple(truth)
                    audit = self.vision_audit.setdefault(key, {"cx": key[0], "cy": key[1], "face": key[2],
                                                              "frames": 0, "usable_frames": 0})
                    audit["frames"] += 1
                    x0, y0, x1, y1 = det.box
                    if min(x0, y0) > 6 and x1 < cam.width-6 and y1 < cam.height-6 and det.conf >= world.cfg.perception.conf_min:
                        audit["usable_frames"] += 1
            detections = [replace(det, extra={k: v for k, v in det.extra.items() if k != "_gt"})
                          for det in detections]
            self._frame = Frame(image=None, stamp=now, pan=ugv.pan, tilt=ugv.tilt,
                                width=cam.width, height=cam.height,
                                meta={"sim_detections": detections})
        return self._frame

    def get_odom(self) -> Pose2D:
        # Robot.get_odom cannot return None: readiness and the output watchdog
        # prevent driving on this retained pose when reception has stopped.
        with self._lock:
            return self._odom.copy() if self._odom is not None else Pose2D()

    def status(self) -> RobotStatus:
        return self._sim_ugv.status() if self._sim_ugv is not None else RobotStatus()

    def close(self) -> None:
        self.stop()
        self.node.destroy_node()
