#!/usr/bin/env python3
"""Offline, time-aligned evaluation of a recorded autonomous WINGBOT trial.

Reads existing artifacts only. Never connects to ROS, Gazebo, or an Agent.
Truth timestamps in the current runner are the observer's latest simulation
clock at callback arrival, not the original Pose_V acquisition timestamp.
Consequently these results measure agreement at recorded times, not externally
certified synchronization accuracy.
"""
from __future__ import annotations

import argparse
from bisect import bisect_left
import hashlib
import json
import math
from pathlib import Path
import statistics


def read_jsonl(path: Path) -> list[dict]:
    rows = []
    with path.open(encoding="utf-8") as stream:
        for number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            try:
                row = json.loads(line)
            except ValueError as exc:
                raise ValueError(f"{path.name}:{number}: incomplete or invalid JSON") from exc
            if not isinstance(row, dict):
                raise ValueError(f"{path.name}:{number}: expected a JSON object")
            rows.append(row)
    return rows


def finite_pose(row: dict, key: str) -> tuple[float, tuple[float, ...]]:
    stamp = float(row["sim_s"])
    pose = tuple(float(value) for value in row[key])
    if len(pose) != 3 or not all(math.isfinite(value) for value in (stamp, *pose)):
        raise ValueError(f"Nonfinite/invalid {key} or sim_s")
    return stamp, pose


def metrics(values: list[float]) -> dict:
    if not values:
        return {"count": 0, "mean": None, "median": None, "rms": None,
                "p95": None, "max": None}
    ordered = sorted(values)
    # Empirical nearest-rank 95th percentile, not interpolated trajectory data.
    return {"count": len(values), "mean": statistics.fmean(values),
            "median": statistics.median(values),
            "rms": math.sqrt(statistics.fmean(value * value for value in values)),
            "p95": ordered[math.ceil(.95 * len(ordered)) - 1], "max": ordered[-1]}


def align(trace: list[dict], truth: list[dict], world_offset: tuple[float, float],
          max_delta_s: float = .05) -> tuple[dict, list[dict]]:
    """Match each estimate to its closest recorded truth time, without synthesis."""
    if not math.isfinite(max_delta_s) or max_delta_s <= 0:
        raise ValueError("max_delta_s must be finite and positive")
    if len(world_offset) != 2 or not all(math.isfinite(value) for value in world_offset):
        raise ValueError("world_offset must contain two finite coordinates")
    if not trace or not truth:
        raise ValueError("Both trace and truth must contain recorded samples")
    truth_samples = [finite_pose(row, "world_xy_yaw") for row in truth]
    if any(later[0] < earlier[0] for earlier, later in zip(truth_samples, truth_samples[1:])):
        raise ValueError("Truth clock moved backwards; separate reset episodes before evaluation")
    stamps = [sample[0] for sample in truth_samples]
    matches = []
    unmatched = []
    previous = -math.inf
    for row in trace:
        stamp, estimated = finite_pose(row, "estimated_nav_xy_yaw")
        if stamp < previous:
            raise ValueError("Agent trace clock moved backwards")
        previous = stamp
        insertion = bisect_left(stamps, stamp)
        candidates = [index for index in (insertion - 1, insertion) if 0 <= index < len(stamps)]
        index = min(candidates, key=lambda item: (abs(stamps[item] - stamp), stamps[item]))
        truth_stamp, world_pose = truth_samples[index]
        delta = truth_stamp - stamp
        if abs(delta) > max_delta_s + 1e-12:
            unmatched.append(stamp)
            continue
        actual = (world_pose[0] - world_offset[0], world_pose[1] - world_offset[1], world_pose[2])
        yaw_error = math.atan2(math.sin(estimated[2] - actual[2]), math.cos(estimated[2] - actual[2]))
        matches.append({"sim_s": stamp, "elapsed_s": row.get("elapsed"),
                        "mode": row.get("mode"), "phase": row.get("phase"),
                        "truth_sim_s": truth_stamp, "truth_minus_estimate_time_s": delta,
                        "estimated_nav_xy_yaw": estimated, "truth_nav_xy_yaw": actual,
                        "position_error_m": math.dist(estimated[:2], actual[:2]),
                        "yaw_error_rad": yaw_error, "absolute_yaw_error_rad": abs(yaw_error)})
    summary = {"trace_samples": len(trace), "truth_samples": len(truth),
               "matched_samples": len(matches), "unmatched_samples": len(unmatched),
               "matched_fraction": len(matches) / len(trace),
               "unmatched_sim_s": unmatched,
               "position_error_m": metrics([row["position_error_m"] for row in matches]),
               "absolute_yaw_error_rad": metrics([row["absolute_yaw_error_rad"] for row in matches]),
               "absolute_time_difference_s": metrics([abs(row["truth_minus_estimate_time_s"]) for row in matches]),
               "trace_interval_sim_s": [trace[0]["sim_s"], trace[-1]["sim_s"]],
               "matched_interval_sim_s": [matches[0]["sim_s"], matches[-1]["sim_s"]] if matches else None,
               "worst_position_samples": sorted(matches, key=lambda row: row["position_error_m"], reverse=True)[:10]}
    return summary, matches


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trial_dir", type=Path)
    parser.add_argument("--max-delta-s", type=float, default=.05)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--matches", type=Path)
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args(argv)
    directory = args.trial_dir.resolve()
    output = args.output or directory / "time_aligned_evaluation.json"
    matches_path = args.matches or directory / "time_aligned_samples.jsonl"
    if output.resolve() == matches_path.resolve():
        parser.error("Summary and sample paths must differ")
    inputs = {name: directory / name for name in
              ("agent_trace.jsonl", "evaluation_truth.jsonl", "trial_config.json", "mission_report.json")}
    for destination in (output, matches_path):
        if destination.resolve() in (path.resolve() for path in inputs.values()):
            parser.error("An output cannot replace a trial input")
        if destination.exists() and not args.overwrite:
            parser.error(f"Output already exists: {destination.name}")
    # The final report is written only after recorder shutdown and closed traces.
    # Refuse a running trial, where partially buffered data would bias statistics.
    report = json.loads(inputs["mission_report.json"].read_text(encoding="utf-8"))
    config = json.loads(inputs["trial_config.json"].read_text(encoding="utf-8"))
    offset = tuple(float(value) for value in config["config"]["arena"]["world_offset"])
    summary, matches = align(read_jsonl(inputs["agent_trace.jsonl"]),
                             read_jsonl(inputs["evaluation_truth.jsonl"]), offset, args.max_delta_s)
    summary.update(schema="wingbot_time_aligned_evaluation_v1", evaluation_only=True,
                   method="Nearest recorded truth timestamp; earlier timestamp wins equal-distance ties; no interpolation or extrapolation",
                   max_time_difference_s=args.max_delta_s, world_offset=offset,
                   coordinate_transform="navigation_xy = world_xy - trial_config.config.arena.world_offset; yaw unchanged",
                   timestamp_limitation="Truth sim_s is observer /clock at callback arrival; estimate sim_s is loop /clock before Agent.tick. Neither is an independently synchronized acquisition timestamp.",
                   weighting="Each recorded trace sample has equal weight; these are sampled-error statistics, not continuous time integration.",
                   mission_completed_without_runner_error=report.get("error") is None and not report.get("interrupted") and report.get("agent_mode") == "DONE",
                   all_missions_success_claimed=False,
                   recording_evidence=report.get("recording"),
                   physical_motion_evidence=report.get("evaluation"),
                   inputs={name: {"bytes": path.stat().st_size,
                                  "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}
                           for name, path in inputs.items()},
                   matched_samples_file=str(matches_path.resolve()))
    for path in (output, matches_path):
        path.parent.mkdir(parents=True, exist_ok=True)
    with matches_path.open("w" if args.overwrite else "x", encoding="utf-8") as stream:
        for row in matches:
            stream.write(json.dumps(row, ensure_ascii=False, allow_nan=False) + "\n")
    output.write_text(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(json.dumps({"output": str(output), "matched_samples": summary["matched_samples"],
                      "position_error_m": summary["position_error_m"],
                      "absolute_yaw_error_rad": summary["absolute_yaw_error_rad"]}, ensure_ascii=False))
    return 0 if matches else 1


if __name__ == "__main__":
    raise SystemExit(main())
