#!/usr/bin/env python3
"""Record actual Gazebo camera pixels. Stop with SIGINT or SIGTERM.

The default is B2's onboard camera. --topic /recording/arena/image --fps 5
records the independent third-person arena overview instead.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import signal
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from maicon26.hal.fpv_recording import SimulationVideoWriter, decode_bgr


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--topic", default="/B2/fpv/image")
    parser.add_argument("--fps", type=float, default=10.0)
    parser.add_argument("--label", default="", help="optional video overlay; raw camera pixels by default")
    parser.add_argument("--max-hold", type=float, default=1.0, help="simulation seconds before marking a missing frame")
    parser.add_argument("--first-frame-timeout", type=float, default=90.0)
    parser.add_argument("--idle-timeout", type=float, default=60.0, help="wall seconds without a new camera frame")
    parser.add_argument("--max-wall", type=float, default=1800.0)
    parser.add_argument("--ready-file", type=Path, help="created only after the first actual image is encoded")
    args = parser.parse_args()
    if any(not math.isfinite(v) or v <= 0 for v in (args.first_frame_timeout, args.idle_timeout, args.max_wall)):
        parser.error("timeouts must be positive and finite")
    if args.ready_file and args.ready_file.exists():
        parser.error("--ready-file already exists; choose a unique path")
    recorder = SimulationVideoWriter(args.output, args.fps, args.max_hold, args.label)
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import Image

    status = {"stop": False, "reason": "", "error": None, "rejected_frames": 0}
    start = last_receive = time.monotonic()

    def stop(signum, _frame):
        status.update(stop=True, reason=signal.Signals(signum).name)

    def receive(msg):
        nonlocal last_receive
        try:
            image = decode_bgr(msg)
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            accepted = recorder.add(image, stamp)
            if accepted:
                last_receive = time.monotonic()
                if args.ready_file and not args.ready_file.exists():
                    args.ready_file.parent.mkdir(parents=True, exist_ok=True)
                    args.ready_file.write_text(json.dumps({"topic": args.topic, "first_image_sim_s": stamp}), encoding="utf-8")
                if recorder.source_frames == 1:
                    print(f"Camera ready: {args.topic} {image.shape[1]}x{image.shape[0]} -> {args.output}", flush=True)
        except (ValueError, TypeError) as exc:
            status["rejected_frames"] += 1
            # Timestamp resets and changing resolution invalidate this timeline.
            if "clock moved" in str(exc) or "resolution changed" in str(exc) or "gap exceeded" in str(exc):
                status.update(stop=True, error=str(exc), reason="invalid_timeline")
            elif status["rejected_frames"] <= 5:
                print(f"Rejected camera image: {exc}", file=sys.stderr, flush=True)
        except Exception as exc:
            status.update(stop=True, error=str(exc), reason="recording_error")

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node("maicon_fpv_recorder")
    node.create_subscription(Image, args.topic, receive, qos_profile_sensor_data)
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        while not status["stop"] and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.1)
            now = time.monotonic()
            limit = args.idle_timeout if recorder.source_frames else args.first_frame_timeout
            if now - last_receive > limit:
                status.update(stop=True, error=f"No new camera image for {limit:g} wall seconds", reason="camera_timeout")
            elif now - start > args.max_wall:
                status.update(stop=True, error="Recording wall-time limit exceeded", reason="wall_timeout")
    except Exception as exc:
        status.update(stop=True, error=str(exc), reason="exception")
    finally:
        summary = recorder.close()
        summary.update(topic=args.topic, wall_elapsed_s=time.monotonic() - start,
                       completed=recorder.source_frames > 0 and status["error"] is None,
                       stop_reason=status["reason"], error=status["error"],
                       rejected_frames=status["rejected_frames"])
        recorder.metadata.write_text(json.dumps(summary, indent=2), encoding="utf-8")
        node.destroy_node()
        rclpy.shutdown()
        print(json.dumps(summary), flush=True)
    return 0 if summary["completed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
