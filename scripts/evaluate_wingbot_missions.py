#!/usr/bin/env python3
"""Offline mission evidence: submitted aggregates versus layout, referee versus guesses.

This tool reads layout truth exclusively after a run. It never feeds geometry,
face identities, or referee positions back to the Agent or perception pipeline.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "ros2/maicon_arena_gz/scripts"))
from arena_common import expand_boxes, load_layout, load_spec, parse_face, sector_of
from maicon26.link.schema import OBJECT_CODES, scoring_sectors, server_sector


def aggregate_truth(spec, layout, team):
    """Count actually rendered, exposed M3 face labels, including physical layers."""
    classes = sorted({value.upper() for value in OBJECT_CODES.values()})
    counts = {sector: dict.fromkeys(classes, 0) for sector in scoring_sectors(team)}
    faces = []
    for box in expand_boxes(spec, layout):
        sector = server_sector(sector_of(spec, *box["center"][:2]))
        declared = server_sector(box.get("sector"))
        if declared != sector:
            raise ValueError(f"Layout sector disagrees with physical cube centre: {box['name']}")
        if sector not in counts:
            continue
        for face, value in box["faces"].items():
            kind, label, _ = parse_face(value)
            if kind == "m3":
                code = label.upper()
                if code not in classes:
                    raise ValueError(f"Unknown M3 class: {code}")
                counts[sector][code] += 1
                faces.append({"cube": box["name"], "face": face, "sector": sector, "type": code})
    return counts, faces


def compare_m3(expected, submitted):
    reported = {sector: dict.fromkeys(classes, 0) for sector, classes in expected.items()}
    present = set()
    for row in (submitted or {}).get("sectors", []):
        sector = server_sector(row["sector"])
        if sector not in reported:
            continue
        if sector in present:
            raise ValueError("Duplicate M3 sector in accepted snapshot")
        present.add(sector)
        seen = set()
        for obj in row["objects"]:
            kind, count = str(obj["type"]).upper(), obj["count"]
            if kind not in reported[sector] or kind in seen or type(count) is not int or count < 0:
                raise ValueError("Invalid/duplicate class or count in accepted M3 snapshot")
            seen.add(kind)
            reported[sector][kind] = count
    rows = [{"sector": sector, "type": kind, "expected": expected[sector][kind],
             "reported": reported[sector][kind], "difference": reported[sector][kind] - expected[sector][kind]}
            for sector in expected for kind in expected[sector]]
    return {"accepted_snapshot_present": submitted is not None,
            "scoring_sectors": list(expected), "expected_counts": expected,
            "accepted_counts": reported, "per_sector_class": rows,
            "expected_total": sum(row["expected"] for row in rows),
            "reported_total": sum(row["reported"] for row in rows),
            "sum_minimum_bin_counts": sum(min(row["expected"], row["reported"]) for row in rows),
            "undercount_total": sum(max(-row["difference"], 0) for row in rows),
            "overcount_total": sum(max(row["difference"], 0) for row in rows),
            "exact_bins": sum(row["difference"] == 0 for row in rows), "total_bins": len(rows),
            "all_counts_equal": submitted is not None and all(row["difference"] == 0 for row in rows),
            "identity_level_recall_available": False,
            "interpretation": "Aggregate sector/class counts only. A duplicate recognition can replace a missed face in the same bin, so sum_minimum_bin_counts is NOT individual-face recall, precision, or a confirmed identity match."}


def evaluate(report, config, spec, layout, m2_target=None):
    team = str(config["config"]["team"]).upper()
    slot = f"{team[0]}-{config['config']['slot']}"
    accepted = report.get("submitted_reports", {})
    events = report.get("referee_events", [])
    expected, faces = aggregate_truth(spec, layout, team)
    m3 = compare_m3(expected, accepted.get("m3"))
    m3["expected_face_ids_for_offline_audit"] = faces
    m3["agent_local_final_counts_not_used_for_scoring"] = report.get("m3_counts")
    target = m2_target or accepted.get("m1", {}).get("target")
    target_hits = [event for event in events if event.get("type") == "target_hit"]
    correct_hits = [event for event in target_hits if event.get("shooter") == slot
                    and f"{event.get('camp')}_{str(event.get('kind')).upper()}" == target]
    objective = config["operator_fixture"]["targets"][team]["objective"]
    camp, label = objective.split("_", 1)
    rows = [row for row, value in spec["bases"]["label_map"].items() if value == label]
    if len(rows) != 1:
        raise ValueError("Objective does not resolve uniquely in the supplied arena specification")
    base = f"{camp[0]}-{rows[0].removeprefix('row')}"
    captures = [event for event in events if event.get("type") == "base_captured"]
    own_captures = [event for event in captures if event.get("alliance") == team and event.get("base") == base]
    local_entries = [row for row in report.get("agent_log", [])
                     if len(row) >= 2 and "M5 local entry estimate" in str(row[1])]
    return {"schema": "wingbot_offline_mission_evaluation_v1", "evaluation_only": True,
            "scope": "Recorded local rehearsal fixture; not an official competition score",
            "team": team, "slot": slot,
            "m1": {"local_api_accepted": "m1" in accepted, "accepted_payload": accepted.get("m1"),
                   "interpretation_semantics_independently_graded": False,
                   "limitation": "API acceptance validates submission, not correct natural-language interpretation or physical success."},
            "m2": {"designated_target": target,
                   "target_basis": "explicit evaluator argument" if m2_target else "accepted M1 interpretation; not an independent M1 correctness check",
                   "referee_confirmed_designated_hit": bool(correct_hits), "designated_hit_events": correct_hits,
                   "all_target_hit_events": target_hits, "agent_target_done": report.get("target_done"),
                   "agent_target_failed": report.get("target_failed")},
            "m3": m3,
            "m4": {"ugv_hit_events": [event for event in events if event.get("type") == "ugv_hit"],
                   "limitation": "One physical B2 trial has no physical opposing UGV; opponent engagement is not validated."},
            "m5": {"posted_objective": objective, "expected_base": base,
                   "referee_confirmed_capture": bool(own_captures), "matching_capture_events": own_captures,
                   "all_capture_events": captures, "agent_local_entry_estimates": local_entries,
                   "limitation": "A local position/entry estimate or a base_beam event is not a scored capture; require matching base_captured."},
            "m6": {"local_api_accepted": "m6" in accepted, "accepted_payload": accepted.get("m6"),
                   "agent_result": report.get("m6_result"), "ranking_correctness_graded": False,
                   "limitation": "No observed physical opponents in this trial; ACK does not validate ranking quality."},
            "runner_completed": report.get("agent_mode") == "DONE" and report.get("error") is None and not report.get("interrupted"),
            "all_missions_success_claimed": False,
            "evidence_limitation": "Referee conclusions use captured event records; event absence means no recorded confirmation. Supplied spec/layout must match the launched world; source hashes below identify exactly what was evaluated."}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trial_dir", type=Path)
    parser.add_argument("--layout", type=Path, default=ROOT / "ros2/maicon_arena_gz/config/layouts/layout_default.yaml")
    parser.add_argument("--spec", type=Path, default=ROOT / "ros2/maicon_arena_gz/config/arena_spec.yaml")
    parser.add_argument("--m2-target", help="Independently supplied designated target, e.g. RED_RADAR")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args(argv)
    paths = {"mission_report": args.trial_dir / "mission_report.json",
             "trial_config": args.trial_dir / "trial_config.json", "layout": args.layout, "spec": args.spec}
    output = args.output or args.trial_dir / "mission_evaluation.json"
    if output.resolve() in {path.resolve() for path in paths.values()}:
        parser.error("Output cannot replace an evaluation input")
    if output.exists() and not args.overwrite:
        parser.error("Output exists; use --overwrite to replace only this evaluation")
    report = json.loads(paths["mission_report"].read_text(encoding="utf-8"))
    config = json.loads(paths["trial_config"].read_text(encoding="utf-8"))
    spec, layout = load_spec(args.spec), load_layout(args.layout)
    if list(map(float, spec["floor"]["size"])) != [config["config"]["arena"][key] for key in ("W", "H")]:
        raise ValueError("Evaluator specification dimensions differ from recorded trial")
    result = evaluate(report, config, spec, layout, args.m2_target)
    result["inputs"] = {key: {"path": str(path.resolve()), "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}
                        for key, path in paths.items()}
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w" if args.overwrite else "x", encoding="utf-8") as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")
    print(json.dumps({"output": str(output), "m3_expected": result["m3"]["expected_total"],
                      "m3_reported": result["m3"]["reported_total"], "m3_all_counts_equal": result["m3"]["all_counts_equal"],
                      "m2_confirmed": result["m2"]["referee_confirmed_designated_hit"],
                      "m5_confirmed": result["m5"]["referee_confirmed_capture"]}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
