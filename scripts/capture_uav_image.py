#!/usr/bin/env python3
"""Save one frame from the fixed Gazebo UAV camera for visual inspection."""
from __future__ import annotations

import argparse
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class Capture(Node):
    def __init__(self, topic="/uav/image") -> None:
        super().__init__("maicon_uav_capture")
        self.frame = None
        self.create_subscription(Image, topic, self.receive, 10)

    def receive(self, msg: Image) -> None:
        if msg.encoding not in ("rgb8", "bgr8"):
            raise ValueError(f"Unsupported UAV encoding: {msg.encoding}")
        rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
        rgb = rgb[:, : msg.width * 3].reshape(msg.height, msg.width, 3)
        self.frame = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR) if msg.encoding == "rgb8" else rgb.copy()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument("--topic", default="/uav/image")
    args = parser.parse_args()
    rclpy.init()
    node = Capture(args.topic)
    try:
        end = time.monotonic() + args.timeout
        while node.frame is None and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.2)
        if node.frame is None:
            raise TimeoutError("No camera frame received on the selected topic")
        args.output.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(args.output), node.frame):
            raise OSError(f"Failed to write {args.output}")
        print(f"{args.output} {node.frame.shape[1]}x{node.frame.shape[0]}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
