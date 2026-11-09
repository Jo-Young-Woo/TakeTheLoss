#!/usr/bin/env python3
"""Merge three co-located measured120deg scans; no interpolation or body mask."""
import argparse
from collections import Counter
import json
import math
from pathlib import Path

import yaml


class ScanRejected(ValueError):
    pass


def merge_scans(scans, samples, now, settings):
    """ROS-free validator/rebinner returning original-stamp common-frame ranges."""
    yaws, span = settings["sector_yaws"], settings["sector_span"]
    if (len(yaws) != 3 or not isinstance(samples, int) or samples < 3
            or abs(span*3-2*math.pi) > 1e-6
            or any(abs((yaws[(i+1) % 3]-yaws[i]) % (2*math.pi)-span) > 1e-6 for i in range(3))):
        raise ScanRejected("configuration")
    count = math.ceil(samples/len(yaws))+1
    if len(scans) != len(yaws):
        raise ScanRejected("incomplete")
    stamps = [scan["stamp"] for scan in scans]
    if not all(math.isfinite(s) and 0 <= now-s <= settings["max_age_s"] for s in stamps):
        raise ScanRejected("stale_or_future")
    if max(stamps)-min(stamps) > settings["max_skew_s"]:
        raise ScanRejected("skew")
    output = [math.nan]*samples
    delta = 2*math.pi/samples
    minimum, maximum = settings["range"]
    for index, (scan, yaw) in enumerate(zip(scans, yaws)):
        amin, inc, amax = scan["angle_min"], scan["angle_increment"], scan["angle_max"]
        expected_inc = span/(count-1)
        if (scan["frame_id"] != f"laser_sector_{index}" or len(scan["ranges"]) != count
                or not all(math.isfinite(v) for v in (amin, inc, amax, scan["range_min"], scan["range_max"]))
                or abs(amin+span/2) > 1e-5 or abs(amax-span/2) > 1e-5
                or abs(inc-expected_inc) > 1e-6 or abs(scan["range_min"]-minimum) > 1e-5
                or abs(scan["range_max"]-maximum) > 1e-5):
            raise ScanRejected("geometry")
        finite = [r for r in scan["ranges"] if math.isfinite(r) and minimum <= r <= maximum]
        if len(finite) >= .9*count and max(finite) <= minimum+settings["min_collapse_epsilon_m"]:
            raise ScanRejected("range_min_collapse")
        for ray, distance in enumerate(scan["ranges"]):
            if not math.isfinite(distance) or not minimum <= distance <= maximum or distance <= 0:
                continue
            angle = amin+ray*inc+yaw
            cell = int(math.floor(((angle+math.pi) % (2*math.pi))/delta+.5)) % samples
            if math.isnan(output[cell]) or distance < output[cell]:
                output[cell] = float(distance)
    return {"stamp": max(stamps), "frame_id": "laser_frame", "angle_min": -math.pi,
            "angle_max": math.pi-delta, "angle_increment": delta,
            "range_min": minimum, "range_max": maximum, "ranges": output}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--spec", type=Path, required=True)
    parser.add_argument("--profile", type=Path, required=True)
    parser.add_argument("--assumptions", type=Path, required=True)
    args, ros_args = parser.parse_known_args(argv)
    spec = yaml.safe_load(args.spec.read_text(encoding="utf-8"))
    profile = yaml.safe_load(args.profile.read_text(encoding="utf-8"))
    settings = yaml.safe_load(args.assumptions.read_text(encoding="utf-8"))["lidar"]
    samples = profile["ugv"].get("lidar_samples", spec["ugv"]["lidar"]["sim_samples_assumed"])
    rate = profile["ugv"]["lidar_rate"]
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.executors import ExternalShutdownException
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String

    class Merger(Node):
        def __init__(self):
            super().__init__("maicon_lidar_sector_merger")
            self.latest, self.last_tuple, self.counts = {}, None, Counter()
            self.scan_pub = self.create_publisher(LaserScan, "/scan", qos_profile_sensor_data)
            self.health_pub = self.create_publisher(String, "/arena/lidar_diagnostics", 10)
            for index in range(len(settings["sector_yaws"])):
                self.create_subscription(LaserScan, f"/arena/scan_sector_{index}",
                    lambda message, i=index: self.receive(i, message), qos_profile_sensor_data)
            self.create_timer(1., self.health)
            self.create_timer(1/(2*rate), self.try_publish)

        def health(self):
            self.health_pub.publish(String(data=json.dumps(dict(self.counts), sort_keys=True)))

        def receive(self, index, message):
            stamp = message.header.stamp.sec+message.header.stamp.nanosec*1e-9
            self.counts["received"] += 1
            self.latest[index] = {"stamp": stamp, "frame_id": message.header.frame_id,
                "angle_min": message.angle_min, "angle_max": message.angle_max,
                "angle_increment": message.angle_increment, "range_min": message.range_min,
                "range_max": message.range_max, "ranges": message.ranges, "header": message.header}
            self.try_publish()

        def try_publish(self):
            now = self.get_clock().now().nanoseconds*1e-9
            if len(self.latest) != len(settings["sector_yaws"]):
                return
            frames = [self.latest[i] for i in range(len(settings["sector_yaws"]))]
            stamps = tuple(frame["stamp"] for frame in frames)
            if self.last_tuple == stamps:
                return
            try:
                merged = merge_scans(frames, samples, now, settings)
            except ScanRejected as error:
                self.counts[str(error)] += 1
                return
            output = LaserScan()
            output.header.stamp = max(frames, key=lambda f: f["stamp"])["header"].stamp
            output.header.frame_id = merged["frame_id"]
            for key in ("angle_min", "angle_max", "angle_increment", "range_min", "range_max"):
                setattr(output, key, float(merged[key]))
            output.scan_time, output.time_increment = float(1/rate), 0.
            output.ranges = merged["ranges"]
            self.scan_pub.publish(output)
            self.last_tuple = stamps
            self.counts["published"] += 1
            self.counts["last_valid_beams"] = sum(math.isfinite(r) for r in output.ranges)

    rclpy.init(args=ros_args)
    node = Merger()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
