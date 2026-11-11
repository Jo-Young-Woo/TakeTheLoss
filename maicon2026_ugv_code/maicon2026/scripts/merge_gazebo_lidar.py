#!/usr/bin/env python3
"""ROS bridge for three real 120-degree Gazebo sectors per UGV."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from maicon26.hal.lidar_sectors import LidarSectorMerger, SectorScan, mask_rover_self_returns, sector_topic


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--samples", type=int, choices=(360, 640), default=360)
    parser.add_argument("--robots", nargs="+", choices=("B1", "B2", "B3", "R1", "R2", "R3"), default=["B2"])
    parser.add_argument("--max-skew", type=float, default=.02)
    args = parser.parse_args()
    import numpy as np
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import LaserScan

    class MergeNode(Node):
        def __init__(self):
            super().__init__("maicon_lidar_sector_merger")
            self.clock = None
            self.clock_received = -float("inf")
            self.mergers = {}
            self.publishers_by_robot = {}
            self.last_stamp = {}
            self.last_warn = {}
            self.create_subscription(Clock, "/clock", self.on_clock, qos_profile_sensor_data)
            for ident in dict.fromkeys(args.robots):
                prefix = "" if ident == "B2" else ident
                self.mergers[ident] = LidarSectorMerger(args.samples, prefix, args.max_skew)
                self.mergers[ident].stats.update(self_returns_masked=0, last_self_returns_masked=0)
                topic = "/scan" if ident == "B2" else f"/{ident}/scan"
                self.publishers_by_robot[ident] = self.create_publisher(LaserScan, topic, qos_profile_sensor_data)
                for index in range(3):
                    self.create_subscription(LaserScan, sector_topic(prefix, index),
                                             lambda msg, i=ident, s=index: self.receive(i, s, msg), qos_profile_sensor_data)
            self.create_timer(.02, self.publish_ready)
            self.create_timer(5., self.report)

        def on_clock(self, msg):
            now = msg.clock.sec + msg.clock.nanosec * 1e-9
            # Clear the epoch before accepting any post-reset sector callbacks.
            if self.clock is not None and now < self.clock:
                for merger in self.mergers.values():
                    merger.reset_clock(now)
                self.last_stamp.clear()
            self.clock = now
            self.clock_received = time.monotonic()

        def receive(self, ident, index, msg):
            packet = SectorScan(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                                msg.header.frame_id, msg.angle_min, msg.angle_max, msg.angle_increment,
                                msg.range_min, msg.range_max, np.asarray(msg.ranges, dtype=np.float32))
            try:
                self.mergers[ident].push(index, packet, time.monotonic())
            except (ValueError, TypeError) as exc:
                if time.monotonic() - self.last_warn.get(ident, -float("inf")) > 2.:
                    self.get_logger().warning(f"{ident} rejected sector {index}: {exc}")
                    self.last_warn[ident] = time.monotonic()

        def publish_ready(self):
            if self.clock is None or time.monotonic() - self.clock_received > .5:
                return
            for ident, merger in self.mergers.items():
                packet = merger.merge(self.clock, time.monotonic())
                if packet is None:
                    continue
                # Filter the common ROS output once so Nav2, HAL and localization
                # all receive the same sensor evidence. Do not mask real walls
                # or boxes by a broad distance threshold around the LiDAR.
                packet, masked = mask_rover_self_returns(packet)
                merger.stats["self_returns_masked"] += masked
                merger.stats["last_self_returns_masked"] = masked
                msg = LaserScan()
                ns = round(packet.stamp * 1e9)
                msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(ns, 1_000_000_000)
                msg.header.frame_id = packet.frame_id
                msg.angle_min, msg.angle_max, msg.angle_increment = packet.angle_min, packet.angle_max, packet.angle_increment
                msg.range_min, msg.range_max = packet.range_min, packet.range_max
                msg.time_increment = 0.  # Gazebo sector snapshots are instantaneous, not a simulated mechanical sweep.
                msg.scan_time = packet.stamp - self.last_stamp.get(ident, packet.stamp - .1)
                msg.ranges = packet.ranges.tolist()
                self.publishers_by_robot[ident].publish(msg)
                self.last_stamp[ident] = packet.stamp

        def report(self):
            print(json.dumps({i: m.diagnostics() for i, m in self.mergers.items()}), flush=True)

    rclpy.init()
    node = MergeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.report()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
