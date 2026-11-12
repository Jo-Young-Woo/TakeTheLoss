#!/usr/bin/env python3
"""Offline evaluation of six physical Agents; never feeds truth into control.

The internal monitor's stall time is reported as such. Older trace schemas
without observed-face/shot counters cannot independently prove all progress
events, so this evaluator explicitly withholds that acceptance claim.
"""
from __future__ import annotations

import argparse
from bisect import bisect_left
from collections import Counter, deque
import html
import json
import math
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/"scripts"))
sys.path.insert(0,str(ROOT))
sys.path.insert(0,str(ROOT/"ros2/maicon_arena_gz/scripts"))
from arena_common import load_layout, load_spec
from evaluate_wingbot_missions import evaluate

SLOTS = ("B-1","B-2","B-3","R-1","R-2","R-3")
INACTIVE_MODES = ("WAIT","HALT","REPORT","DONE")


def json_lines(path):
    with path.open(encoding="utf8") as handle:
        for line in handle:
            if line.strip(): yield json.loads(line)


def independent_audit(rows, slot, truth_rows):
    """Rebuild progress without trusting controller resets when evidence exists."""
    active_rows=[row for row in rows if row.get("mode") not in INACTIVE_MODES]
    if not active_rows or not all("observed_faces" in row.get("decision", {}) and
                           "shots" in row.get("decision", {}) for row in active_rows):
        return {"verified":False,"reason":"Trace lacks observed-face/shot progress counters"}
    from compare_sim import Monitor
    monitor=Monitor()
    stamps=[row["sim_s"] for row in truth_rows]
    missing=0
    previous_normalized=None
    for row in rows:
        inactive=row.get("mode") in INACTIVE_MODES
        if inactive and previous_normalized is None:
            continue
        j=bisect_left(stamps,row["sim_s"])
        indexes=[i for i in (j-1,j) if 0<=i<len(stamps)]
        if not indexes:
            missing+=1; continue
        k=min(indexes,key=lambda i:abs(stamps[i]-row["sim_s"]))
        true=truth_rows[k]
        if abs(stamps[k]-row["sim_s"])>.15 or slot not in true["poses"]:
            missing+=1; continue
        decision=row["decision"]
        point=true["poses"][slot]
        nearest=min((math.dist(point[:2],p[:2]) for s,p in true["poses"].items() if s!=slot),default=math.inf)
        normalized=dict(decision,t=float(row.get("elapsed") or 0.),robot=slot,
            remaining_m=decision.get("remaining_path_m"),confirmed_faces=decision.get("confirmed_faces",0),
            mode=row.get("mode"),v=row.get("cmd_vw",[0,0])[0],
            normal_wait=decision.get("normal_wait",False) or inactive)
        if inactive:
            # Reporting/halting resets are not navigation decisions. Keep the
            # last evidence counters and advance only the paused clock.
            normalized=dict(previous_normalized,t=normalized["t"],normal_wait=True,v=0.)
        else:
            previous_normalized=normalized
        monitor.update(normalized,dict(pose=point,nearest_ugv_m=nearest))
    result=monitor.result()
    result.update(verified=missing==0,unmatched_rows=missing,trace_rows=len(rows),
                  reason="Independent qualifying route/face/shot progress; goal changes alone never reset stall")
    return result


def analyze_trace(rows, slot, truth_rows, world_offset=(-3., -2.4)):
    stamps = [row["sim_s"] for row in truth_rows]
    previous_t = previous_decision = None
    direction = 0
    transitions, reversals = deque(), deque()
    oscillating = stalled = False
    active_s = stall_s = 0.
    incidents, flips, errors, distances, health = [], [], [], [], Counter()
    last_true = None
    distance = 0.
    complete_progress_fields = True
    for row in rows:
        t = float(row.get("elapsed") or 0.)
        decision = row.get("decision", {})
        complete_progress_fields &= "observed_faces" in decision and "shots" in decision
        dt = max(0., t-previous_t) if previous_t is not None else 0.
        previous_t = t
        active_decision=row.get("mode") not in INACTIVE_MODES
        pair = decision.get("posture"), decision.get("level")
        if active_decision:
            if previous_decision is not None and pair != previous_decision: transitions.append(t)
            previous_decision = pair
        else:
            previous_decision=None
        v = row.get("cmd_vw", [0,0])[0]
        sign = 1 if v>.02 else -1 if v<-.02 else 0
        if active_decision:
            if sign and direction and sign!=direction: reversals.append(t)
            if sign: direction=sign
        for sequence in (transitions,reversals):
            while sequence and t-sequence[0]>10.: sequence.popleft()
        current_oscillation = active_decision and (len(transitions)>3 or len(reversals)>2)
        if current_oscillation and not oscillating:
            flips.append(dict(robot=slot,t=t,kind="B",reason="More than 3 posture/level changes or 2 direction reversals in 10 s"))
        oscillating = current_oscillation
        match = None
        if stamps:
            j = bisect_left(stamps,row["sim_s"])
            indexes=[i for i in (j-1,j) if 0<=i<len(stamps)]
            k=min(indexes,key=lambda i:abs(stamps[i]-row["sim_s"]))
            if abs(stamps[k]-row["sim_s"])<=.15: match=truth_rows[k]
        nearest = None
        if match and slot in match["poses"]:
            point = match["poses"][slot]
            nearest = min((math.dist(point[:2],p[:2]) for s,p in match["poses"].items() if s!=slot),default=math.inf)
            est = row.get("estimated_nav_xy_yaw")
            if est is not None: errors.append(math.hypot(est[0]-(point[0]-world_offset[0]),est[1]-(point[1]-world_offset[1])))
            if last_true is not None:
                delta=math.dist(point[:2],last_true[:2])
                if delta<.5: distance+=delta
            last_true=point
        wait=decision.get("normal_wait",False) or not active_decision
        no_progress=float(decision.get("no_progress_s") or 0.)
        if not wait:
            active_s+=dt
            if no_progress>=3.: stall_s+=dt
            if no_progress>12. and not stalled:
                kind = "D" if nearest is not None and nearest<.7 else "E"
                incidents.append(dict(robot=slot,t=t,kind=kind,classification="provisional; proximity is not proof of obstruction",
                                      no_progress_s=no_progress,goal=decision.get("goal"),mode=row.get("mode"),nearest_ugv_m=nearest,
                                      reason="Agent reports more than 12 s without qualifying progress"))
                stalled=True
        if no_progress<=12.: stalled=False
        readiness=row.get("readiness")
        if readiness and not readiness[0]: health["not_ready_trace_rows"]+=1
    return dict(robot=slot,trace_rows=len(rows),reported_deadlock_events=incidents,oscillation_events=flips,
                active_s=active_s,reported_stall_s=stall_s,reported_stall_ratio=stall_s/max(active_s,1e-9),
                independent_progress_fields_complete=complete_progress_fields,
                physical_distance_m=distance,localization_samples=len(errors),
                localization_error_m={"mean":float(np.mean(errors)),"p95":float(np.percentile(errors,95)),"max":float(max(errors))} if errors else None,
                health=dict(health),independent_audit=independent_audit(rows,slot,truth_rows))


def physical_match_evidence(report):
    """Keep match termination, physical firing and video duration separate."""
    start = report.get("match_start_sim_s")
    events = report.get("referee_events", [])
    timeline = []
    for event in events:
        if event.get("type") not in ("match_start", "base_captured", "match_report", "match_finished"):
            continue
        item = dict(event)
        if start is not None and event.get("time") is not None:
            item["elapsed_s"] = event["time"] - start
        timeline.append(item)
    return dict(
        duration_requested_s=report.get("duration_requested_s"),
        video_duration_s=report.get("recording", {}).get("video_duration_s"),
        chronology=timeline,
        shots_recorded=sum(event.get("type") == "laser_fired" for event in events),
        target_hit_events=[event for event in events if event.get("type") == "target_hit"],
        ugv_hit_events=[event for event in events if event.get("type") == "ugv_hit"],
        engagement_validated=any(event.get("type") == "ugv_hit" for event in events),
        note="A six-robot run without recorded UGV hits does not validate combat performance. Capture events are alliance-level, not success by all three robots.")


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output",type=Path)
    args=parser.parse_args()
    out=args.output
    report=json.loads((out/"selfplay_report.json").read_text(encoding="utf8"))
    truth=list(json_lines(out/"evaluation_truth_all.jsonl"))
    spec_path=out/"arena_spec.yaml"
    layout_path=out/"layout_default.yaml"
    spec=load_spec(spec_path if spec_path.is_file() else ROOT/"ros2/maicon_arena_gz/config/arena_spec.yaml")
    layout=load_layout(layout_path if layout_path.is_file() else ROOT/"ros2/maicon_arena_gz/config/layouts/layout_default.yaml")
    agents={}
    for slot in SLOTS:
        folder=out/slot
        rows=list(json_lines(folder/"agent_trace.jsonl"))
        mission=json.loads((folder/"mission_report.json").read_text(encoding="utf8"))
        config=json.loads((folder/"trial_config.json").read_text(encoding="utf8"))
        # Older recordings still carry their original frame in trial_config.
        analysis=analyze_trace(rows,slot,truth,config["config"]["arena"]["world_offset"])
        evaluated=evaluate(mission,config,spec,layout,m2_target=("RED_RADAR" if slot.startswith("B") else "BLUE_RADAR"))
        evaluated["m4"]["limitation"]="Six physical UGVs are present. Only recorded referee ugv_hit events confirm hits; no event means engagement performance is unvalidated."
        evaluated["m6"]["limitation"]="Six physical UGVs are present. API acceptance does not independently grade opponent ranking quality."
        evaluated["m5"]["capture_scope"]="Alliance-level referee event; does not prove that this particular robot entered the base."
        (folder/"mission_evaluation.json").write_text(json.dumps(evaluated,ensure_ascii=False,indent=2),encoding="utf8")
        analysis["mission_evaluation"]=evaluated
        analysis["mode"]=mission.get("agent_mode")
        analysis["error"]=mission.get("error")
        analysis["cleanup_errors"]=mission.get("cleanup_errors",[])
        agents[slot]=analysis
    events=[event for item in agents.values() for event in item["reported_deadlock_events"]+item["oscillation_events"]]
    active=sum(item["active_s"] for item in agents.values())
    stall=sum(item["reported_stall_s"] for item in agents.values())
    ratio=stall/max(active,1e-9)
    evaluated_slots=[slot for slot in SLOTS if report.get("policies",{}).get(slot,"agent")=="agent"]
    evaluated_agents=[agents[slot] for slot in evaluated_slots]
    evaluated_ratio=sum(item["reported_stall_s"] for item in evaluated_agents)/max(1e-9,sum(item["active_s"] for item in evaluated_agents))
    result=dict(metric_version=4,agents=agents,physical_match=physical_match_evidence(report),reported_deadlock_count=sum(len(item["reported_deadlock_events"]) for item in agents.values()),
                oscillation_count=sum(len(item["oscillation_events"]) for item in agents.values()),
                categories=dict(Counter(event["kind"] for event in events)),reported_stall_ratio=ratio,
                limitations=["Stall duration is from each Agent monitor; older traces lack independent observed-face/shot counters",
                             "D/E incident classes are provisional. M3 aggregate matches are not individual-face recall",
                             "No same-seed v2 Gazebo baseline in this artifact; A/B criteria remain unverified"],
                acceptance_evaluated_robots=evaluated_slots,acceptance_stall_ratio=evaluated_ratio,
                acceptance=dict(zero_reported_deadlocks=not any(item["reported_deadlock_events"] for item in evaluated_agents),
                                stall_ratio_at_most_5pct=evaluated_ratio<=.05,
                                zero_oscillation_events=not any(item["oscillation_events"] for item in evaluated_agents),
                                individual_face_coverage_100pct_at_210s="unverified",
                                v2_comparison="unverified",overall_passed=False))
    result["independent_audit_scope"] = evaluated_slots
    result["independent_audit_complete"] = all(item["independent_audit"]["verified"] for item in evaluated_agents)
    if result["independent_audit_complete"]:
        audits=[item["independent_audit"] for item in evaluated_agents]
        result["independent_deadlock_events"]=[event for audit in audits for event in audit["deadlock_events"]]
        result["independent_oscillation_events"]=[event for audit in audits for event in audit["oscillation_events"]]
        result["independent_stall_ratio"]=sum(audit["stall_s"] for audit in audits)/max(1.,sum(audit["active_s"] for audit in audits))
        result["acceptance"].update(zero_independent_deadlocks=not result["independent_deadlock_events"],
                                    independent_stall_ratio_at_most_5pct=result["independent_stall_ratio"]<=.05)
    (out/"selfplay_analysis.json").write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding="utf8")
    table=[]
    for slot,item in agents.items():
        m3=item["mission_evaluation"]["m3"]
        policy=report.get("policies",{}).get(slot,"agent")
        table.append(f"<tr><td>{slot}</td><td>{html.escape(policy)}</td><td>{item['mode']}</td><td>{len(item['reported_deadlock_events'])}</td><td>{len(item['oscillation_events'])}</td><td>{item['reported_stall_ratio']:.1%}</td><td>{m3['reported_total']} / {m3['expected_total']}</td></tr>")
    links=[]
    video_offset = ((report.get("recording", {}).get("first_image_sim_s") or 0.)-
                    (report.get("match_start_sim_s") or 0.))
    for event in events:
        t=max(0.,event['t']-video_offset)
        links.append(f"<li><a href='wingbot_selfplay_third_person.mp4#t={t:.2f}'>{event['robot']} {event['kind']} {t:.2f} s</a> {html.escape(event['reason'])}</li>")
    page="""<!doctype html><html lang='ko'><meta charset='utf-8'><title>Gazebo 자가 대전 검증</title>
<style>body{font:16px system-ui;max-width:1100px;margin:32px auto;background:#15181d;color:#eee}table{border-collapse:collapse}td,th{border:1px solid #555;padding:9px}a{color:#8bc9ff}video{width:100%}</style>
<h1>실제 Gazebo 여섯 로봇 시험</h1><p>센서 입력: 실제 RGB·LiDAR·odom·IMU. 실제 모델 위치는 사후 평가와 영상 식별 표시만 사용합니다. 혼합 시험에서 더미 제어 방식의 로봇은 별도 진단하며, 임무 알고리즘 합격 기준은 agent 로봇에 적용합니다.</p>
<video controls src='wingbot_selfplay_third_person.mp4'></video>
<p>교착은 Agent의 진척 감시 로그 기준입니다. 독립 면별 진척 검증과 3:30의 100% 면 커버리지, 동일 시드 v2 대비 성능은 이 자료만으로 통과 판정하지 않습니다. 아래 M3 수치는 제출된 집계와 실제 집계의 비교이며, 개별 표적 탐지율이 아닙니다.</p>
<table><tr><th>로봇</th><th>제어 방식</th><th>종료 상태</th><th>12 s 초과 정체</th><th>진동 사건</th><th>정체 비율</th><th>M3 보고 수 / 실제 수</th></tr>"""+"".join(table)+"</table><h2>사건 영상 시각</h2><ul>"+"".join(links)+"</ul><p><a href='selfplay_analysis.json'>전체 수치와 한계</a></p></html>"
    (out/"report.html").write_text(page,encoding="utf8")
    print(json.dumps({"output":str(out),"reported_deadlock_count":result["reported_deadlock_count"],
                      "oscillation_count":result["oscillation_count"],"reported_stall_ratio":ratio,
                      "acceptance":result["acceptance"]},ensure_ascii=False,indent=2))


if __name__=="__main__": main()
