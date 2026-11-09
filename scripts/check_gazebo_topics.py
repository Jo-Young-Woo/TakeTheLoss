#!/usr/bin/env python3
"""Check the live ROS topics used by the single UGV Gazebo rehearsal."""
from __future__ import annotations

import argparse
import json
import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class TopicProbe(Node):
    def __init__(self):
        super().__init__("maicon_gazebo_topic_probe")
        self.times = {"scan": [], "odom": [], "clock": []}
        self.beams = 0
        self.valid_beams = 0
        self.quadrants = [0, 0, 0, 0]
        self.cardinal_ranges = {}
        self.create_subscription(LaserScan, "/scan", self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/odom", self.on_odom, qos_profile_sensor_data)
        self.create_subscription(Clock, "/clock", self.on_clock, qos_profile_sensor_data)

    def on_scan(self, msg):
        self.times["scan"].append(time.monotonic())
        self.beams = len(msg.ranges)
        self.valid_beams = sum(math.isfinite(r) and msg.range_min < r < msg.range_max
                               for r in msg.ranges)
        self.quadrants = [0, 0, 0, 0]
        for i, r in enumerate(msg.ranges):
            angle = (msg.angle_min+i*msg.angle_increment+math.pi) % (2*math.pi)-math.pi
            if math.isfinite(r) and max(.15, msg.range_min) < r < msg.range_max:
                self.quadrants[min(3, int((angle+math.pi)/(math.pi/2)))] += 1
        self.cardinal_ranges = {}
        for name, angle in (("front", 0.), ("left", math.pi/2), ("rear", -math.pi), ("right", -math.pi/2)):
            candidates = [(abs(math.atan2(math.sin(msg.angle_min+i*msg.angle_increment-angle),
                                          math.cos(msg.angle_min+i*msg.angle_increment-angle))), r)
                          for i, r in enumerate(msg.ranges) if math.isfinite(r)]
            self.cardinal_ranges[name] = round(min(candidates)[1], 4) if candidates else None

    def on_odom(self, _msg):
        self.times["odom"].append(time.monotonic())

    def on_clock(self, _msg):
        self.times["clock"].append(time.monotonic())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=8.0)
    parser.add_argument("--lidar-samples", type=int, default=360)
    parser.add_argument("--expect-360", action="store_true")
    args = parser.parse_args()
    rclpy.init()
    probe = TopicProbe()
    try:
        until = time.monotonic() + args.seconds
        while time.monotonic() < until:
            rclpy.spin_once(probe, timeout_sec=.1)
        result = {}
        for name, stamps in probe.times.items():
            span = stamps[-1] - stamps[0] if len(stamps) > 1 else 0.0
            result[name] = {"messages": len(stamps),
                            "hz": round((len(stamps) - 1) / span, 1) if span > 0 else 0.0}
        result["scan"].update({"beams": probe.beams, "valid_beams": probe.valid_beams})
        result["scan"].update(quadrants_valid_over_15cm=probe.quadrants, cardinal_ranges_m=probe.cardinal_ranges)
        result["ok"] = (result["scan"]["messages"] >= 5
                        and probe.beams == args.lidar_samples
                        and probe.valid_beams > 0
                        and result["odom"]["messages"] >= 5
                        and result["clock"]["messages"] >= 5)
        if args.expect_360:
            result["ok"] = result["ok"] and min(probe.quadrants) > args.lidar_samples//20
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0 if result["ok"] else 1
    finally:
        probe.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
