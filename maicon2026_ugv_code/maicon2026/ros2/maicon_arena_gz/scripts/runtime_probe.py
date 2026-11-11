#!/usr/bin/env python3
"""Read-only sensor/model checks against a running arena; never commands motion."""
import argparse
import json
import math
from pathlib import Path
import time


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=8.)
    parser.add_argument("--output", type=Path, required=True)
    args, ros_args = parser.parse_known_args(argv)
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
    from sensor_msgs.msg import LaserScan, Image, Imu
    from nav_msgs.msg import Odometry
    from tf2_msgs.msg import TFMessage
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import String

    rclpy.init(args=ros_args)
    node = Node("maicon_arena_runtime_probe")
    counts, latest, poses, clocks = {}, {}, {}, []
    def receive(topic, message):
        counts[topic] = counts.get(topic, 0)+1
        latest[topic] = message
        if topic == "/clock":
            clocks.append(message.clock.sec+message.clock.nanosec*1e-9)
        elif topic == "/arena/dynamic_pose":
            for transform in message.transforms:
                if "::" not in transform.child_frame_id and transform.child_frame_id.startswith("ugv_"):
                    p, q = transform.transform.translation, transform.transform.rotation
                    poses[transform.child_frame_id] = [p.x, p.y, p.z,
                        math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))]
    topics = {"/clock": Clock, "/scan": LaserScan, "/uav/image_raw": Image,
              "/image_raw": Image, "/odom": Odometry, "/imu/data": Imu,
              "/arena/dynamic_pose": TFMessage, "/arena/state": String}
    for topic, cls in topics.items():
        qos = (QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
               if topic == "/arena/state" else qos_profile_sensor_data)
        node.create_subscription(cls, topic, lambda message, topic=topic: receive(topic, message), qos)
    started = time.monotonic()
    try:
        while time.monotonic()-started < args.duration:
            rclpy.spin_once(node, timeout_sec=.1)
        result = {"duration_wall_s": time.monotonic()-started, "received_messages": counts,
                  "models": poses, "clock_advances": len(clocks) > 1 and clocks[-1] > clocks[0]}
        for topic in ("/uav/image_raw", "/image_raw"):
            msg = latest.get(topic)
            if msg is not None:
                result[topic] = {"width": msg.width, "height": msg.height, "encoding": msg.encoding}
        scan = latest.get("/scan")
        if scan is not None:
            finite = [r for r in scan.ranges if math.isfinite(r)]
            useful = [r for r in finite if scan.range_min+.001 < r < scan.range_max]
            result["scan"] = {"samples": len(scan.ranges), "finite": len(finite), "useful": len(useful),
                "range_min": scan.range_min, "range_max": scan.range_max,
                "min_return": min(finite) if finite else None, "max_return": max(finite) if finite else None,
                "angle_span": scan.angle_max-scan.angle_min, "frame_id": scan.header.frame_id}
        if "/arena/state" in latest:
            result["referee"] = json.loads(latest["/arena/state"].data)
        result["all_topics_received"] = set(topics).issubset(latest)
        result["scan_not_collapsed_at_minimum"] = result.get("scan", {}).get("useful", 0) > 10
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2, allow_nan=False)+"\n", encoding="utf-8")
        print(json.dumps(result, indent=2, allow_nan=False))
        return 0 if result["all_topics_received"] and result["scan_not_collapsed_at_minimum"] else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
