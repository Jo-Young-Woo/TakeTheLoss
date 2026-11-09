#!/usr/bin/env python3
"""Measure wall-clock RTF from /clock and received camera FPS, without inventing GUI FPS."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import time


def summarize(samples, image_count, *, startup_s=None):
    if len(samples) < 2:
        raise ValueError("At least two clock samples are required")
    start_wall, start_sim = samples[0]
    wall_span, sim_span = samples[-1][0]-start_wall, samples[-1][1]-start_sim
    if wall_span <= 0 or sim_span < 0:
        raise ValueError("Nonpositive observation interval or simulation clock reset")
    intervals = []
    anchor = samples[0]
    for wall, sim in samples[1:]:
        if wall-anchor[0] >= 1.:
            intervals.append((sim-anchor[1])/(wall-anchor[0]))
            anchor = (wall, sim)
    return {"wall_seconds": wall_span, "sim_seconds": sim_span,
            "rtf_mean": sim_span/wall_span,
            "rtf_min_one_second_window": min(intervals) if intervals else None,
            "received_camera_fps": image_count/wall_span,
            "startup_to_first_step_s": startup_s,
            "gui_fps": None, "gui_fps_note": "Not measured; camera delivery FPS is not GUI FPS",
            "clock_sample_count": len(samples), "one_second_window_count": len(intervals)}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=60.)
    parser.add_argument("--warmup", type=float, default=0., help="Wall seconds after first clock, excluded from the steady-state window")
    parser.add_argument("--clock-timeout", type=float, default=30.)
    parser.add_argument("--image-topic", default="/uav/image_raw")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--launch-wall-time", type=float,
                        help="Optional time.monotonic() captured immediately before launching Gazebo")
    parser.add_argument("--minimum-mean", type=float, default=.98)
    parser.add_argument("--minimum-window", type=float, default=.90)
    args, ros_args = parser.parse_known_args(argv)
    if not math.isfinite(args.duration) or args.duration < 1:
        parser.error("duration must be finite and at least one second")
    if not math.isfinite(args.warmup) or args.warmup < 0:
        parser.error("warmup must be finite and nonnegative")
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import Image

    class Probe(Node):
        def __init__(self):
            super().__init__("maicon_arena_perf_probe")
            self.samples, self.images, self.error = [], 0, None
            self.first_clock_wall = None
            self.create_subscription(Clock, "/clock", self.on_clock, qos_profile_sensor_data)
            self.create_subscription(Image, args.image_topic, self.on_image, qos_profile_sensor_data)

        def on_clock(self, msg):
            now, sim = time.monotonic(), msg.clock.sec+msg.clock.nanosec*1e-9
            if self.first_clock_wall is None:
                self.first_clock_wall = now
            if now-self.first_clock_wall < args.warmup:
                return
            if self.samples and sim < self.samples[-1][1]:
                self.error = "Simulation clock reset during measurement"
            # Keep at most roughly 100 observations per second for bounded memory.
            if not self.samples or now-self.samples[-1][0] >= .01:
                self.samples.append((now, sim))

        def on_image(self, _):
            if self.samples:
                self.images += 1

    rclpy.init(args=ros_args)
    probe, opened = Probe(), time.monotonic()
    try:
        while rclpy.ok():
            rclpy.spin_once(probe, timeout_sec=.05)
            if probe.error:
                raise ValueError(probe.error)
            if probe.first_clock_wall is None and time.monotonic()-opened > args.clock_timeout:
                raise TimeoutError("No simulation clock received")
            if probe.samples and time.monotonic()-probe.samples[0][0] >= args.duration:
                break
        startup = None if args.launch_wall_time is None else probe.first_clock_wall-args.launch_wall_time
        result = summarize(probe.samples, probe.images, startup_s=startup)
        result["warmup_wall_seconds"] = args.warmup
        result["thresholds"] = {"mean": args.minimum_mean, "minimum_window": args.minimum_window}
        result["passed_rtf"] = (result["rtf_mean"] >= args.minimum_mean and
                                result["rtf_min_one_second_window"] is not None and
                                result["rtf_min_one_second_window"] >= args.minimum_window)
        result["full_sixty_second_probe"] = result["wall_seconds"] >= 59.9
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2)+"\n", encoding="utf-8")
        print(json.dumps(result, indent=2))
        return 0 if result["passed_rtf"] else 1
    finally:
        probe.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
