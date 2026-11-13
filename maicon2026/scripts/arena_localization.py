#!/usr/bin/env python3
"""Optional fixed-field LiDAR localization; publishes TF and explicit quality.

Inputs are only scan/odom, published static arena geometry, calibrated LiDAR
extrinsics, and the selected starting base. No world-state/UAV subscriber.
"""
from __future__ import annotations

import argparse
from collections import deque
import json
import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from maicon26.hal.gazebo_model import LIDAR_RPY, LIDAR_XYZ
from maicon26.hal.ros_mil import scan_in_base
from maicon26.nav.arena_localization import ArenaLocalizer
from maicon26.types import Pose2D, Scan
from maicon26.util import wrap


def interpolate_odom(samples, stamp):
    """Match encoder pose to LiDAR time; permit at most 60 ms extrapolation."""
    if not samples or stamp < samples[0][0]-.01 or stamp > samples[-1][0]+.06:
        return None
    if stamp <= samples[0][0]:
        return samples[0][1].copy()
    pair = None
    for previous, current in zip(samples, list(samples)[1:]):
        if current[0] >= stamp:
            pair = previous, current
            break
    if pair is None:
        if len(samples) < 2 or samples[-1][0]-samples[-2][0] > .2:
            return samples[-1][1].copy()
        pair = samples[-2], samples[-1]
    (ta, a), (tb, b) = pair
    if tb <= ta:
        return None
    ratio = (stamp-ta)/(tb-ta)
    return Pose2D(a.x+ratio*(b.x-a.x), a.y+ratio*(b.y-a.y), wrap(a.th+ratio*wrap(b.th-a.th)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scan-topic", default="/scan")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--quality-topic", default="/arena_localization/quality")
    parser.add_argument("--start-x", type=float, default=.35)
    parser.add_argument("--start-y", type=float, default=2.5)
    parser.add_argument("--start-yaw", type=float, default=0.)
    parser.add_argument("--use-sim-time", action="store_true")
    args = parser.parse_args()
    import numpy as np
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    from tf2_ros import TransformBroadcaster

    def seconds(stamp):
        return float(stamp.sec)+float(stamp.nanosec)*1e-9

    class ArenaNode(Node):
        def __init__(self):
            super().__init__("maicon_arena_localization", parameter_overrides=[Parameter("use_sim_time", value=args.use_sim_time)])
            self.tracker = ArenaLocalizer(Pose2D(args.start_x, args.start_y, args.start_yaw))
            self.odometry = deque(maxlen=400)
            self.latest_scan = None
            self.processed_stamp = -math.inf
            self.last_transform = None
            self.quality = {"valid": False, "status": "waiting_for_scan_and_odom", "stamp": 0.}
            self.pub = self.create_publisher(String, args.quality_topic, 10)
            self.tf = TransformBroadcaster(self)
            self.create_subscription(Odometry, args.odom_topic, self.on_odom, qos_profile_sensor_data)
            self.create_subscription(LaserScan, args.scan_topic, self.on_scan, qos_profile_sensor_data)
            self.create_timer(.05, self.tick)
            self.get_logger().info("Fixed infrastructure LiDAR tracking enabled; random cubes and world poses are not map inputs")

        def on_odom(self, message):
            if message.header.frame_id != "odom" or message.child_frame_id != "base_link":
                self.quality = {"valid": False, "status": "wrong_odometry_frames", "stamp": seconds(message.header.stamp)}
                return
            stamp = seconds(message.header.stamp)
            p, q = message.pose.pose.position, message.pose.pose.orientation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            if not all(math.isfinite(v) for v in (stamp, p.x, p.y, yaw)):
                return
            if not self.odometry or stamp > self.odometry[-1][0]:
                self.odometry.append((stamp, Pose2D(p.x, p.y, yaw)))

        def on_scan(self, message):
            self.latest_scan = message

        def tick(self):
            now = self.get_clock().now()
            current = now.nanoseconds*1e-9
            message = self.latest_scan
            stamp = seconds(message.header.stamp) if message else -math.inf
            if message and stamp > self.processed_stamp:
                odom = interpolate_odom(self.odometry, stamp)
                if odom is not None:
                    self.processed_stamp = stamp
                    try:
                        if message.header.frame_id != "gazebo_lidar":
                            raise ValueError("Expected calibrated gazebo_lidar frame")
                        amin, inc, ranges, rmax = scan_in_base(
                            message.angle_min, message.angle_increment, message.ranges,
                            message.range_min, message.range_max, LIDAR_XYZ[0], LIDAR_XYZ[1], LIDAR_RPY[2])
                        scan = Scan(amin, inc, np.asarray(ranges, dtype=np.float32), stamp, rmax)
                        self.quality = self.tracker.update(scan, odom)
                        if self.quality["valid"]:
                            # Compose with the same encoder sample as the scan;
                            # later odom motion then propagates correctly in TF.
                            self.last_transform = self.tracker.map_to_odom(odom)
                    except (ValueError, TypeError, np.linalg.LinAlgError) as error:
                        self.quality = {"valid": False, "status": "invalid_scan", "detail": str(error), "stamp": stamp}
            quality = dict(self.quality)
            if current-self.processed_stamp > .5 or not self.odometry or current-self.odometry[-1][0] > .3:
                quality.update(valid=False, status="stale_scan_or_odometry")
            self.pub.publish(String(data=json.dumps(quality, allow_nan=False)))
            if quality["valid"] and self.last_transform is not None:
                pose = self.last_transform
                transform = TransformStamped()
                transform.header.stamp = now.to_msg()
                transform.header.frame_id, transform.child_frame_id = "map", "odom"
                transform.transform.translation.x, transform.transform.translation.y = pose.x, pose.y
                transform.transform.rotation.z, transform.transform.rotation.w = math.sin(pose.th/2), math.cos(pose.th/2)
                self.tf.sendTransform(transform)

    rclpy.init()
    node = ArenaNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
