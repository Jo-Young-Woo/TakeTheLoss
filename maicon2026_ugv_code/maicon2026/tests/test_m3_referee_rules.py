from types import SimpleNamespace as NS
import pytest
from maicon26.sim.robot import SimServer
from maicon26.sim.run import score_m3


def server():
    out = object.__new__(SimServer)
    out.u = NS(id="B-UGV-2", team="BLUE")
    out.world = NS(t=10., ended=False, end_time=None, submissions={},
                   cfg=NS(mission=NS(t_total=300., t_report=30.)))
    return out


@pytest.mark.parametrize("team,opponent,home", [("BLUE", "R-S1", "B-S1"), ("RED", "B-S1", "R-S1")])
def test_only_opponent_and_common_sectors_affect_score(team, opponent, home):
    truth = {opponent: {"TANK": 2}, "COMMON": {"DRUM": 1}, home: {"MISSILE": 8}}
    result = score_m3(truth, {"sectors": {opponent: {"TANK": 2}, "CM": {"DRUM": 1},
                                         home: {"MISSILE": 99}}}, team)
    assert result["exact"] and result["n_truth"] == result["n_sub"] == result["n_correct"] == 3


def test_missing_sector_is_zero_in_latest_complete_snapshot():
    truth = {"R-S1": {"TANK": 2}, "COMMON": {"DRUM": 1}}
    result = score_m3(truth, {"sectors": {"COMMON": {"DRUM": 1}}}, "BLUE")
    assert result["n_correct"] == 1 and result["n_truth"] == 3 and not result["exact"]


def test_exactness_is_not_inferred_from_rounded_recall():
    truth = {"COMMON": {"TANK": 250}}
    result = score_m3(truth, {"sectors": {"COMMON": {"TANK": 249}}}, "BLUE")
    assert result["recall"] == 1. and not result["exact"]


def test_resubmission_replaces_whole_snapshot_and_storage_is_immutable():
    sv = server()
    first = {"sectors": {"R-S1": {"TANK": 2}, "COMMON": {"DRUM": 1}}}
    assert sv.submit("m3", first)
    first["sectors"]["R-S1"]["TANK"] = 99
    stored = sv.world.submissions[sv.u.id]["m3"]
    assert stored["payload"]["sectors"]["R-S1"]["TANK"] == 2
    assert sv.submit("m3", {"sectors": {"COMMON": {"DRUM": 2}}})
    assert stored["payload"] == {"sectors": {"COMMON": {"DRUM": 2}}}
    assert stored["n"] == 2


@pytest.mark.parametrize("receipt,ended", [(300., False), (300.1, False), (250., True)])
def test_m3_received_at_deadline_or_after_early_end_is_rejected(receipt, ended):
    sv = server()
    original = {"sectors": {"COMMON": {"DRUM": 1}}}
    assert sv.submit("m3", original)
    sv.world.t, sv.world.ended = receipt, ended
    sv.world.end_time = receipt if ended else None
    assert not sv.submit("m3", {"sectors": {"COMMON": {"DRUM": 2}}})
    assert sv.last_submit_status == 409
    assert sv.world.submissions[sv.u.id]["m3"]["payload"] == original


@pytest.mark.parametrize("duration,receipt", [(400., 300.), (400., 310.), (240., 240.)])
def test_rehearsal_duration_cannot_extend_official_m3_receive_deadline(duration, receipt):
    sv = server()
    sv.world.cfg.mission.t_total = duration
    original = {"sectors": {"COMMON": {"DRUM": 1}}}
    assert sv.submit("m3", original)
    sv.world.t = receipt
    assert not sv.submit("m3", {"sectors": {"R-S1": {"TANK": 2}}})
    assert sv.last_submit_status == 409
    assert sv.world.submissions[sv.u.id]["m3"]["payload"] == original


@pytest.mark.parametrize("bad", [{"DESTROYED_BUILDING": 1}, {"BLOCK": 1}, {"TANK": -1}, {"TANK": True}])
def test_invalid_picture_class_or_quantity_cannot_replace_good_report(bad):
    sv = server()
    assert sv.submit("m3", {"sectors": {"COMMON": {"DRUM": 1}}})
    assert not sv.submit("m3", {"sectors": {"COMMON": bad}})
    assert sv.last_submit_status == 422
    assert sv.world.submissions[sv.u.id]["m3"]["payload"]["sectors"]["COMMON"] == {"DRUM": 1}
