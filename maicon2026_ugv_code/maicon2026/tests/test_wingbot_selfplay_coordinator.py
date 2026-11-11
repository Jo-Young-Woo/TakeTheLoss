"""Scenarios keep our single competition UGV independent of partner behavior."""
from pathlib import Path
import sys

import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"scripts"))
from run_wingbot_selfplay import policies_for, source_manifest
from run_wingbot_autonomous_trial import json_value, write_json, apply_trial_variant, processing_summary
import run_wingbot_autonomous_trial as trial


@pytest.mark.parametrize("scenario,policy",[("vs_evaders","evader"),("vs_hunters","hunter"),("vs_blockers","blocker")])
def test_opponent_policies_leave_blue_agents_intact(scenario,policy):
    roles=policies_for(scenario)
    assert {roles[slot] for slot in ("B-1","B-2","B-3")}=={"agent"}
    assert {roles[slot] for slot in ("R-1","R-2","R-3")}=={policy}


def test_selfplay_uses_same_mission_code_but_mixed_allies_only_our_robot():
    assert set(policies_for("selfplay").values())=={"agent"}
    roles=policies_for("mixed_allies")
    assert roles["B-2"]=="agent" and sum(value=="agent" for value in roles.values())==1
    assert roles["B-1"]=="idle" and roles["B-3"]=="patrol_own"
    with pytest.raises(ValueError): policies_for("scripted_truth_controller")


def test_actual_observed_face_defaultdict_can_be_saved_in_final_report(tmp_path):
    import json
    from maicon26.perception.object_map import Face
    face=Face(1.,2.,1.2,2.2,n=3,geometry_key=(4,2))
    face.votes["TANK"]=2.8
    report={"observed_classified_faces":[face],"unbounded_diagnostic":float("inf")}
    path=tmp_path/"mission_report.json"
    write_json(path,report)
    saved=json.loads(path.read_text(encoding="utf8"))
    assert saved["observed_classified_faces"][0]["votes"]=={"TANK":2.8}
    assert saved["observed_classified_faces"][0]["geometry_key"]==[4,2]
    assert saved["unbounded_diagnostic"] is None
    assert face.votes.default_factory is float
    assert json_value(face)["n"]==3


def test_relative_imu_is_explicit_candidate_only_not_production_or_v2():
    from maicon26.config import Config
    baseline=Config()
    assert not baseline.hal.imu_heading_fusion and not baseline.progress.enabled
    apply_trial_variant(baseline,"v2")
    assert not baseline.hal.imu_heading_fusion and not baseline.progress.enabled
    candidate=Config()
    apply_trial_variant(candidate,"progress")
    assert candidate.hal.imu_heading_fusion and candidate.progress.enabled
    assert not candidate.engage.fire_on_blocker


def test_processing_latency_rolling_deques_are_serializable(tmp_path):
    from collections import deque
    import json
    from types import SimpleNamespace
    worker=SimpleNamespace(latency_ms=deque([10.,20.]),build_ms=deque([5.,10.]),errors=deque(["diagnostic error"]))
    agent=SimpleNamespace(blobs=SimpleNamespace(latency_ms=[1.,3.]))
    path=tmp_path/"risk_worker_summary.json"
    write_json(path,processing_summary(agent,worker))
    saved=json.loads(path.read_text(encoding="utf8"))
    assert saved["risk_submit_to_publish"]["p95_ms"]==pytest.approx(19.5)
    assert saved["worker_errors"]==["diagnostic error"]
    assert saved["lidar_blob_processing"]["count"]==2


def test_launch_manifest_fingerprints_actual_source_changes(tmp_path):
    import hashlib
    directory=tmp_path/"maicon26"
    directory.mkdir()
    path=directory/"policy.py"
    path.write_text("first",encoding="utf8")
    first=source_manifest(tmp_path)
    assert first["sha256"]["repo/maicon26/policy.py"]==hashlib.sha256(b"first").hexdigest()
    path.write_text("second",encoding="utf8")
    second=source_manifest(tmp_path)
    assert first["sha256"]!=second["sha256"]


def test_transient_permission_error_retries_critical_report(tmp_path,monkeypatch):
    import json
    original=Path.replace
    calls=[]
    monkeypatch.setattr(trial.time,"sleep",lambda _:None)
    def locked_once(self,target):
        calls.append(str(target))
        if len(calls)==1:
            raise PermissionError("temporary sharing lock")
        return original(self,target)
    monkeypatch.setattr(Path,"replace",locked_once)
    path=tmp_path/"mission_report.json"
    write_json(path,{"completed":True})
    assert len(calls)==2 and json.loads(path.read_text())["completed"]


def test_persistent_preview_lock_is_visible_and_bounded_but_critical_report_fails(tmp_path,monkeypatch):
    calls=[]
    monkeypatch.setattr(trial.time,"sleep",lambda _:None)
    def always_locked(self,target):
        calls.append(str(target))
        raise PermissionError("persistent sharing lock")
    monkeypatch.setattr(Path,"replace",always_locked)
    stats={'replace_retries':0,'preview_skipped':0}
    monkeypatch.setattr(trial,"ARTIFACT_IO_STATS",stats)
    assert trial.write_preview_json(tmp_path/"overlay.json",{"tick":1}) is False
    assert len(calls)==5 and stats['preview_skipped']==1 and stats['replace_retries']==4
    with pytest.raises(PermissionError):
        write_json(tmp_path/"mission_report.json",{"completed":True})
    assert len(calls)==10


def test_non_permission_preview_io_error_is_not_hidden(tmp_path,monkeypatch):
    import errno
    def disk_full(self,target):
        raise OSError(errno.ENOSPC,"disk full")
    monkeypatch.setattr(Path,"replace",disk_full)
    with pytest.raises(OSError) as caught:
        trial.write_preview_json(tmp_path/"overlay.json",{"tick":1})
    assert caught.value.errno==errno.ENOSPC


def test_risk_preview_archive_is_atomic_and_locked_previous_frame_survives(tmp_path,monkeypatch):
    import numpy as np
    path=tmp_path/"risk_overlay.npz"
    assert trial.write_preview_npz(path,risk=np.ones((2,2)))
    monkeypatch.setattr(trial.time,"sleep",lambda _:None)
    def locked(self,target):
        raise PermissionError("reader sharing lock")
    monkeypatch.setattr(Path,"replace",locked)
    assert not trial.write_preview_npz(path,risk=np.zeros((2,2)))
    with np.load(path) as payload:
        assert np.array_equal(payload["risk"],np.ones((2,2)))


def test_bounded_aim_output_keeps_exact_source_pixels_and_caps_saving(tmp_path):
    import cv2
    import json
    import numpy as np
    from trial_evidence import BoundedAimEvidence
    writer=BoundedAimEvidence(tmp_path,trial.replace_artifact,trial.write_preview_json,limit=1)
    pixels=np.arange(60,dtype=np.uint8).reshape(4,5,3)
    snapshot=dict(mode='STRIKE',updated_this_tick=True,now=10.,consumed_frame_stamp=7.25,source_frame_available=True)
    assert writer.offer(snapshot,pixels)
    pixels[:]=0
    assert not writer.offer(dict(snapshot,now=12.,consumed_frame_stamp=8.),pixels)
    report=writer.close()
    assert report['saved']==1 and not report['errors']
    decoded=cv2.imdecode(np.frombuffer((tmp_path/'aim_source_000.png').read_bytes(),dtype=np.uint8),cv2.IMREAD_COLOR)
    assert np.array_equal(decoded,np.arange(60,dtype=np.uint8).reshape(4,5,3))
    metadata=json.loads((tmp_path/'aim_source_000.json').read_text())
    assert metadata['consumed_frame_stamp']==7.25 and metadata['exact_capture_stamp_match']


def test_missing_exact_aim_source_is_reported_without_substitution(tmp_path):
    from trial_evidence import BoundedAimEvidence
    writer=BoundedAimEvidence(tmp_path,trial.replace_artifact,trial.write_preview_json)
    snapshot=dict(mode='STRIKE',updated_this_tick=True,now=10.,consumed_frame_stamp=7.25,source_frame_available=False)
    assert not writer.offer(snapshot,None)
    assert not writer.wants(dict(snapshot,now=10.2))
    report=writer.close()
    assert report['missing_exact_source']==1 and report['saved']==0
    assert not list(tmp_path.iterdir())


def test_processing_summary_does_not_mix_receipt_latency_with_worker_time():
    from types import SimpleNamespace
    worker=SimpleNamespace(latency_ms=[20.],build_ms=[10.],errors=[],lineage_snapshot=lambda:{
        'records':[{'category':'urgent_fresh_scan','scan_callback_to_publish_ms':120.},
                   {'category':'timer_same_scan','scan_callback_to_publish_ms':300.},
                   {'category':'unmatched_scan','scan_callback_to_publish_ms':None}],
        'counts':{'published_requests':3},'truncated_records':0})
    summary=processing_summary(SimpleNamespace(blobs=None),worker)
    assert summary['risk_submit_to_publish']['p95_ms']==20.
    assert summary['scan_callback_to_publish']['count']==2
    assert summary['scan_callback_to_publish_by_category']['urgent_fresh_scan']['p95_ms']==120.
    assert summary['scan_lineage']['truncated_records']==0


def test_route_provenance_is_read_only_and_reports_actual_selected_endpoint():
    from types import SimpleNamespace
    decision={'goal':(2.,3.)}
    agent=SimpleNamespace(deadlock_telemetry=lambda:dict(decision),_remaining_path_route=lambda goal:('navigator',[(1.,1.),goal]))
    result=trial.decision_with_route(agent)
    assert result['remaining_path_source']=='navigator' and result['remaining_path_endpoint']==(2.,3.)
    assert result['remaining_path_endpoint_error_m']==0.
    assert decision=={'goal':(2.,3.)}
