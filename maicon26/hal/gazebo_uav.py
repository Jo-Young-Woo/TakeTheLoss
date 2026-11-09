"""Read the actual fixed Gazebo UAV image for optional M3 footprint hints."""
from __future__ import annotations

import math
import threading
import time

import numpy as np

from ..link.base import UavFrame
from ..nav.ros_runtime import add_node, ensure_ros
from ..perception.uav_layout import FixedUavLayoutDetector
from ..perception.uav_calibration import GridCalibration, load_calibration


def decode_bgr(msg) -> np.ndarray:
    """Decode padded ROS rgb8/bgr8 images without assuming a packed row."""
    if msg.encoding not in ("rgb8", "bgr8"):
        raise ValueError(f"Unsupported UAV encoding: {msg.encoding}")
    if msg.width <= 0 or msg.height <= 0 or msg.step < msg.width * 3:
        raise ValueError("Invalid UAV image dimensions or row stride")
    data = np.frombuffer(msg.data, dtype=np.uint8)
    if data.size != msg.height * msg.step:
        raise ValueError("UAV image buffer does not match its row stride")
    rgb = data.reshape(msg.height, msg.step)[:, :msg.width * 3]
    rgb = rgb.reshape(msg.height, msg.width, 3)
    return rgb[:, :, ::-1].copy() if msg.encoding == "rgb8" else rgb.copy()


class GazeboUavFeed:
    """Latest camera snapshot; match server applies frame cadence and latency.

    This node does not accept World/PoseFeed or any simulator truth object.
    A stale stream returns None. A new, valid image with no reliable boxes
    returns a frame with an empty list, never invented fallback coordinates.
    """

    def __init__(self, topic: str = "/uav/image", max_receive_age: float = 3.0,
                 calibration_path: str | None = None) -> None:
        if not math.isfinite(max_receive_age) or max_receive_age <= 0:
            raise ValueError("max_receive_age must be positive and finite")
        ensure_ros()
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image

        self.node = Node("maicon26_gazebo_uav_layout")
        calibration = load_calibration(calibration_path) if calibration_path else None
        self.detector = FixedUavLayoutDetector(calibration)
        projection = "measured_grid" if isinstance(calibration, GridCalibration) else "four_corners"
        self.calibration_report = ({"projection": projection, "path": str(calibration_path),
                                    **calibration.validation_report()} if calibration else
                                   {"projection": "fixed_nadir_intrinsics"})
        self._lock = threading.Lock()
        self._latest = None
        self._received_at = -math.inf
        self._max_receive_age = max_receive_age
        self.frames_received = self.frames_rejected = 0
        self.error = ""
        self.node.create_subscription(Image, topic, self._receive, qos_profile_sensor_data)
        add_node(self.node)

    def _receive(self, msg) -> None:
        try:
            stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
            if not math.isfinite(stamp):
                raise ValueError("Invalid UAV image timestamp")
            image = decode_bgr(msg)
            boxes = self.detector.detect(image)
            frame = UavFrame(stamp=stamp, image=image, boxes=boxes,
                             layout_source="gazebo_uav_image")
        except (ValueError, TypeError) as error:
            with self._lock:
                self.frames_rejected += 1
                self.error = str(error)
            return
        with self._lock:
            if self._latest is not None and stamp <= self._latest.stamp:
                return
            self._latest = frame
            self._received_at = time.monotonic()
            self.frames_received += 1
            self.error = ""

    def latest(self) -> UavFrame | None:
        with self._lock:
            if time.monotonic() - self._received_at > self._max_receive_age:
                return None
            return self._latest

    def diagnostics(self) -> dict:
        with self._lock:
            return {"source": "gazebo_uav_image", "frames_received": self.frames_received,
                    "calibration": self.calibration_report,
                    "frames_rejected": self.frames_rejected,
                    "last_stamp": self._latest.stamp if self._latest else None,
                    "last_boxes": len(self._latest.boxes) if self._latest else 0,
                    "receive_age_s": (round(time.monotonic() - self._received_at, 3)
                                      if math.isfinite(self._received_at) else None),
                    "error": self.error}

    def close(self) -> None:
        self.node.destroy_node()
