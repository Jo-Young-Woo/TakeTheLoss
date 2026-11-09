#!/usr/bin/env python3
"""Compare scripted Gazebo wheel odometry with referee pose for calibration only.

Start an idle B2 Gazebo world first, with no Agent/teleop driving it. This
script never launches Gazebo and never sends truth positions to a controller.
Commands are fixed straight / positive-spin / negative-spin segments. LiDAR
and HAL freshness gates can stop the probe; truth is only logged and analysed.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import signal
import statistics
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from maicon26.hal.gazebo import GazeboRobot
from maicon26.nav.ros_runtime import shutdown
from maicon26.util import wrap
from scripts.run_gazebo_match import PoseFeed


def summarize_segment(rows, v, w, wheel_track):
    """Independent ground truth is used here for offline calibration, not control."""
    result = {"command_v_mps": v, "command_w_radps": w, "samples": len(rows)}
    if len(rows) < 2:
        return result
    result["elapsed_sim_s"] = rows[-1]["clock_s"] - rows[0]["clock_s"]
    for key in ("odom", "truth"):
        values = [row[key] for row in rows]
        yaw = sum(wrap(b[2] - a[2]) for a, b in zip(values, values[1:]))
        travel = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(values, values[1:]))
        result[f"{key}_yaw_delta_rad"] = yaw
        result[f"{key}_path_m"] = travel
        result[f"{key}_displacement_m"] = math.hypot(values[-1][0] - values[0][0], values[-1][1] - values[0][1])
    actual, odom = result["truth_yaw_delta_rad"], result["odom_yaw_delta_rad"]
    if w and abs(actual) >= .4 and actual * odom > 0:
        result["effective_wheel_track_m"] = wheel_track * abs(odom / actual)
        result["odom_over_actual_yaw_ratio"] = abs(odom / actual)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--wheel-track", type=float, default=.17452, help="current DiffDrive wheel_separation, metres")
    parser.add_argument("--speed", type=float, default=.12)
    parser.add_argument("--turn-rate", type=float, default=.5)
    parser.add_argument("--straight-seconds", type=float, default=2.)
    parser.add_argument("--turn-seconds", type=float, default=4.)
    parser.add_argument("--settle-seconds", type=float, default=1.)
    parser.add_argument("--wait-ready", type=float, default=60.)
    parser.add_argument("--max-wall", type=float, default=180.)
    args = parser.parse_args()
    numeric = (args.wheel_track, args.speed, args.turn_rate, args.straight_seconds,
               args.turn_seconds, args.settle_seconds, args.wait_ready, args.max_wall)
    if any(not math.isfinite(value) or value <= 0 for value in numeric):
        parser.error("all numeric parameters must be positive and finite")
    if args.speed > .2 or args.turn_rate > .8 or args.straight_seconds * args.speed > .5:
        parser.error("probe limits: speed <= .2m/s, turn-rate <= .8rad/s, straight travel <= .5m")
    sample_path = args.output.with_suffix(".samples.jsonl")
    for path in (args.output, sample_path):
        if path.exists():
            raise FileExistsError(f"Refusing to overwrite {path}")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    # Keep ROS alive until the finally block has sent the zero-velocity stop.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    robot = GazeboRobot()
    feed = PoseFeed()
    wall_start = time.monotonic()
    last_truth_object = None
    last_truth_received = -math.inf
    segments = []
    error = None

    def interrupt(_signum, _frame):
        raise KeyboardInterrupt("Calibration interrupted")

    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)

    def sample():
        nonlocal last_truth_object, last_truth_received
        with feed.lock:
            truth_object = feed.poses.get("B2")
            if truth_object is not None and truth_object is not last_truth_object:
                # Holding the previous object prevents Python identity reuse.
                last_truth_object = truth_object
                last_truth_received = time.monotonic()
            truth = truth_object.copy() if truth_object is not None else None
        if truth is None or time.monotonic() - last_truth_received > 1.0 or feed.proc.poll() is not None:
            raise RuntimeError("Ground truth recorder is stale; calibration is invalid")
        if not robot.ready():
            raise RuntimeError("LiDAR, odometry or simulation clock became stale")
        with robot._lock:
            odom = robot._odom.copy()
            odom_stamp = robot._odom_stamp
            now = robot._clock
        return {"clock_s": now, "odom_header_s": odom_stamp,
                "odom": [odom.x, odom.y, odom.th], "truth": [truth.x, truth.y, truth.th]}

    def guard_motion(v, w):
        scan = robot.get_scan()
        if scan is None:
            raise RuntimeError("No fresh LiDAR for probe clearance")
        import numpy as np
        valid = np.isfinite(scan.ranges) & (scan.ranges > .05)
        if v:
            valid &= np.abs(scan.angles) < .45
        clearance = float(np.min(scan.ranges[valid])) if np.any(valid) else 0.
        if clearance < (.28 if v else .22):
            raise RuntimeError(f"Probe stopped by LiDAR clearance ({clearance:.3f}m)")

    def run_phase(name, v, w, seconds, log, rows):
        start = sample()["clock_s"]
        previous = -math.inf
        while True:
            row = sample()
            if row["clock_s"] < start:
                raise RuntimeError("Simulation clock reset during probe")
            if time.monotonic() - wall_start > args.max_wall:
                raise TimeoutError("Probe wall-time limit reached")
            if row["clock_s"] > previous:
                row.update(phase=name, command=[v, w])
                log.write(json.dumps(row) + "\n")
                log.flush()
                rows.append(row)
                previous = row["clock_s"]
            if row["clock_s"] - start >= seconds:
                break
            if v or w:
                guard_motion(v, w)
            robot.set_velocity(v, w)
            time.sleep(.025)
        robot.stop()

    try:
        while time.monotonic() - wall_start < args.wait_ready:
            if robot.ready() and "B2" in feed.latest():
                break
            time.sleep(.05)
        if not robot.ready() or "B2" not in feed.latest():
            raise TimeoutError("Idle Gazebo B2 did not become ready")
        if robot.node.count_publishers("/cmd_vel") > 1:
            raise RuntimeError("Another /cmd_vel publisher is active; stop Agent/teleop before calibration")
        with sample_path.open("x", encoding="utf-8") as log:
            run_phase("initial_settle", 0., 0., args.settle_seconds, log, [])
            for name, v, w, seconds in (("straight", args.speed, 0., args.straight_seconds),
                                         ("spin_positive", 0., args.turn_rate, args.turn_seconds),
                                         ("spin_negative", 0., -args.turn_rate, args.turn_seconds)):
                rows = []
                run_phase(name, v, w, seconds, log, rows)
                run_phase(name + "_settle", 0., 0., args.settle_seconds, log, rows)
                report = summarize_segment(rows, v, w, args.wheel_track)
                report["name"] = name
                segments.append(report)
                print(json.dumps(report), flush=True)
    except (Exception, KeyboardInterrupt) as exc:
        error = str(exc) or type(exc).__name__
    finally:
        robot.stop()
        feed.close()
        robot.close()
        shutdown()
        track_values = [s["effective_wheel_track_m"] for s in segments if "effective_wheel_track_m" in s]
        recommendation = statistics.median(track_values) if len(track_values) == 2 else None
        spread = (max(track_values) - min(track_values)) / recommendation if recommendation else None
        if spread is not None and spread > .20:
            recommendation = None
        result = {"purpose": "calibration_only", "truth_used_for_control": False,
                  "completed": error is None and len(segments) == 3, "error": error,
                  "configured_wheel_track_m": args.wheel_track, "segments": segments,
                  "recommended_effective_wheel_track_m": recommendation,
                  "opposite_turn_relative_spread": spread,
                  "recommendation_note": "Estimate only; changes no configuration. Review contact/slip and repeat both directions before applying.",
                  "samples": str(sample_path), "wall_elapsed_s": time.monotonic() - wall_start}
        args.output.write_text(json.dumps(result, indent=2), encoding="utf-8")
        print(json.dumps(result), flush=True)
    return 0 if result["completed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
