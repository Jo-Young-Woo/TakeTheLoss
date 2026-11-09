#!/usr/bin/env python3
"""Count actual ROS image timestamps without converting or retaining pixels."""
import argparse
import json
import math
from pathlib import Path
import statistics
import time


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=30.)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--topics", nargs="+", default=["/arena/ego/image", "/image_raw", "/uav/image_raw"])
    parser.add_argument("--qos-depth", type=int, default=5)
    args, ros_args = parser.parse_known_args(argv)
    if not math.isfinite(args.duration) or not 0 < args.duration <= 3600 or not 1 <= args.qos_depth <= 100:
        parser.error("duration must be (0,3600] and qos-depth in [1,100]")
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from sensor_msgs.msg import Image

    rclpy.init(args=ros_args)
    node = Node("arena_image_delivery_probe")
    start = time.monotonic()
    data = {topic: {"count": 0, "duplicate_stamps": 0, "backwards_stamps": 0,
                    "gaps": [], "first_64_stamps": []} for topic in args.topics}

    def receive(topic, message):
        s = data[topic]
        stamp = message.header.stamp.sec + message.header.stamp.nanosec*1e-9
        received = time.monotonic()-start
        if s["count"]:
            gap = stamp-s["last_stamp"]
            if gap == 0:
                s["duplicate_stamps"] += 1
            elif gap < 0:
                s["backwards_stamps"] += 1
            elif len(s["gaps"]) < 100000:
                s["gaps"].append(gap)
        else:
            s.update(first_stamp=stamp, first_received_wall_s=received)
        s["count"] += 1
        s.update(last_stamp=stamp, last_received_wall_s=received, width=message.width,
                 height=message.height, encoding=message.encoding, bytes=len(message.data))
        if len(s["first_64_stamps"]) < 64:
            s["first_64_stamps"].append(stamp)

    qos = QoSProfile(depth=args.qos_depth, reliability=ReliabilityPolicy.BEST_EFFORT,
                     history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.VOLATILE)
    subscriptions = [node.create_subscription(Image, topic, lambda msg, name=topic: receive(name, msg), qos)
                     for topic in args.topics]
    try:
        while rclpy.ok() and time.monotonic()-start < args.duration:
            rclpy.spin_once(node, timeout_sec=min(.1, max(0., args.duration-(time.monotonic()-start))))
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        elapsed = time.monotonic()-start
        for s in data.values():
            gaps = s.pop("gaps")
            span = s.get("last_stamp", 0)-s.get("first_stamp", 0)
            s.update(wall_fps=s["count"]/elapsed, stamp_span_fps=(s["count"]-1)/span if span > 0 else 0.,
                     median_positive_stamp_gap_s=statistics.median(gaps) if gaps else None,
                     max_stamp_gap_s=max(gaps) if gaps else None)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps({"transport": "ros2_best_effort", "qos_depth": args.qos_depth,
                                           "wall_seconds": elapsed, "topics": data}, indent=2)+"\n", encoding="utf-8")
        for sub in subscriptions:
            node.destroy_subscription(sub)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
