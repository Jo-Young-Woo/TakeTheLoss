"""Acceptance evidence must retain failures and distinguish internal telemetry."""
from pathlib import Path
import sys

import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"scripts"))
from analyze_wingbot_selfplay import analyze_trace, independent_audit, physical_match_evidence


def row(t, *, wait=False, posture="AVOID", level=0, v=.1):
    return dict(sim_s=t,elapsed=t,mode="EXPLORE",cmd_vw=[v,0],estimated_nav_xy_yaw=[3.,2.4,0.],
                readiness=[True,[]],decision=dict(posture=posture,level=level,no_progress_s=t,normal_wait=wait,goal=[4.,2.4]))


def truth(t):
    return dict(sim_s=t,poses={"B-2":[0.,0.,0.,0.],"R-2":[.6,0.,0.,0.]})


def test_failed_internal_stall_retained_but_not_called_independent_validation():
    result=analyze_trace([row(t) for t in range(16)],"B-2",[truth(t) for t in range(16)])
    assert len(result["reported_deadlock_events"])==1
    event=result["reported_deadlock_events"][0]
    assert event["t"]==13 and event["kind"]=="D"
    assert "provisional" in event["classification"]
    assert result["reported_stall_ratio"]>.05
    assert not result["independent_progress_fields_complete"]
    assert result["localization_error_m"]["max"]==pytest.approx(0.)


def test_normal_wait_is_excluded_but_oscillation_not_erased():
    rows=[row(t,wait=True,posture="ENGAGE_OK" if t%2 else "AVOID") for t in range(16)]
    result=analyze_trace(rows,"B-2",[truth(t) for t in range(16)])
    assert result["active_s"]==0 and not result["reported_deadlock_events"]
    assert result["oscillation_events"]


def test_sparse_truth_is_not_silently_used_for_localization_accuracy():
    result=analyze_trace([row(1),row(2)],"B-2",[truth(0),truth(3)])
    assert result["localization_samples"]==0 and result["localization_error_m"] is None


def test_independent_monitor_does_not_accept_goal_churn_as_progress():
    rows=[row(t) for t in range(16)]
    for i,value in enumerate(rows):
        value["decision"].update(observed_faces=0,confirmed_faces=0,shots=0,
                                 goal=[4.+.2*(i%2),2.4],remaining_path_m=1.)
        value["decision"]["no_progress_s"]=0.  # Controller reset cannot hide this.
    result=independent_audit(rows,"B-2",[truth(t) for t in range(16)])
    assert result["verified"]
    assert len(result["deadlock_events"])==1
    assert result["stall_ratio"]>.05


def test_early_capture_is_not_presented_as_300_seconds_or_combat_success():
    report=dict(match_start_sim_s=1.2,duration_requested_s=300.,
                recording={"video_duration_s":251.2},referee_events=[
                    {"type":"base_captured","time":251.7,"base":"B-2","alliance":"RED"},
                    {"type":"match_report","time":251.7,"reason":"m5_capture"}])
    result=physical_match_evidence(report)
    assert result["chronology"][0]["elapsed_s"]==pytest.approx(250.5)
    assert result["video_duration_s"]==251.2
    assert result["duration_requested_s"]==300.
    assert not result["engagement_validated"]


def test_reporting_reset_does_not_add_an_extra_navigation_flip():
    rows=[row(0),row(1,level=3),row(2,level=2),row(3,level=3),row(4)]
    rows[-1]["mode"]="REPORT"
    result=analyze_trace(rows,"B-2",[truth(t) for t in range(5)])
    assert not result["oscillation_events"]


def test_initial_wait_without_counters_does_not_invalidate_active_evidence():
    rows=[row(t) for t in range(16)]
    rows[0].update(mode="WAIT",decision={})
    for value in rows[1:]:
        value["decision"].update(observed_faces=0,confirmed_faces=0,shots=0,remaining_path_m=1.)
    result=independent_audit(rows,"B-2",[truth(t) for t in range(16)])
    assert result["verified"]
    assert len(result["deadlock_events"])==1
