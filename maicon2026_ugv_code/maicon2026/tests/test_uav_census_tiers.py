"""UAV 초기 영상 장부(미탐지 → 탐지 전환)와 M1 지정 구역 우선순위(적 섹터 먼저, CM 은 그 다음). ROS·시뮬 없이 돈다."""
import pytest

from maicon26.config import Config
from maicon26.link.schema import server_sector
from maicon26.nav.grid import OccupancyGrid
from maicon26.perception.coverage import CoverageMap
from maicon26.mission.explore_online import OnlineExplorer
from maicon26.types import Pose2D

from test_sector_ledger_m1 import _built, accept, box_in, cov_on, fp, layout, mark_empty, resolve, sector_point


# ====================================================================== 초기 영상 장부
def census_cov(**over):
    return cov_on(uav_census=True, **over)


def test_first_layout_faces_are_recorded_as_census_and_later_boxes_are_new():
    cov, grid = census_cov()
    cfg = cov.cfg
    accept(cov, fp(*sector_point(cfg, "R-S2")), fp(*sector_point(cfg, "COMMON")), t=1.0)
    assert cov.census_stats["registered"] == 8 and all(r.census for r in cov.registry.values())
    accept(cov, fp(*sector_point(cfg, "R-S3")), t=20.0)                    # 창(4 s) 뒤에 새로 받은 상자
    late = [r for r in cov.registry.values() if r.sector == "R-S3"]
    assert late and not any(r.census for r in late)
    rep = cov.uav_map_report()
    assert rep["registered"] == 8 and rep["undetected"] == 8 and rep["sectors"]["R-S3"]["new_faces"] == 4
    assert rep["sectors"]["R-S2"] == {"census": 4, "detected": 0, "undetected": 4, "hidden": 0, "new_faces": 0, "new_detected": 0}


def test_census_off_by_default_leaves_the_ledger_untouched():
    cov, grid = cov_on()
    accept(cov, fp(*sector_point(cov.cfg, "R-S2")), t=1.0)
    assert cov.census_stats == {"registered": 0, "converted": 0} and not any(r.census for r in cov.registry.values())
    assert cov.pop_conversions() == [] and cov.uav_map_report()["enabled"] is False


def test_identified_census_face_converts_once_with_sector_class_and_time():
    cov, grid = census_cov()
    cfg = cov.cfg
    accept(cov, fp(*sector_point(cfg, "R-S2")), t=1.0)
    b = box_in(cov, "R-S2")
    resolve(b.faces[0], "TANK")
    cov._update_registry(30.0)
    ev = cov.pop_conversions()
    assert len(ev) == 1 and ev[0]["sector"] == "R-S2" and ev[0]["status"] == "identified" and ev[0]["cls"] == "TANK" and ev[0]["idx"] == b.faces[0].idx
    assert ev[0]["waited_s"] == pytest.approx(28.0)
    cov._update_registry(31.0)
    assert cov.pop_conversions() == []                                      # 같은 면은 두 번 알리지 않는다
    rep = cov.uav_map_report(["R-S2"])
    assert rep["converted"] == 1 and rep["undetected"] == 3 and rep["sectors"]["R-S2"]["detected"] == 1
    assert len(rep["todo"]) == 3 and all(t[0] == "R-S2" for t in rep["todo"])


def test_empty_face_also_counts_as_detected_and_hidden_is_separate():
    cov, grid = census_cov()
    accept(cov, fp(*sector_point(cov.cfg, "R-S1")), t=1.0)
    b = box_in(cov, "R-S1")
    mark_empty(b.faces[1])
    b.internal.append(b.faces[2].idx)                                       # 이웃에 가려 볼 수 없는 면
    cov._update_registry(10.0)
    row = cov.uav_map_report()["sectors"]["R-S1"]
    assert row["detected"] == 1 and row["hidden"] == 1 and row["undetected"] == 2
    assert cov.pop_conversions()[0]["status"] == "empty"


def test_report_can_be_limited_to_the_m1_sectors():
    cov, grid = census_cov()
    cfg = cov.cfg
    accept(cov, fp(*sector_point(cfg, "R-S1")), fp(*sector_point(cfg, "R-S2")), fp(*sector_point(cfg, "COMMON")), t=1.0)
    rep = cov.uav_map_report(["R-S2", "CM"])
    assert set(rep["sectors"]) == {"R-S2", "CM"} and rep["undetected"] == 8


def test_non_census_faces_never_emit_conversions():
    cov, grid = census_cov()
    accept(cov, fp(*sector_point(cov.cfg, "R-S1")), t=1.0)
    accept(cov, fp(*sector_point(cov.cfg, "R-S3")), t=30.0)
    b = box_in(cov, "R-S3")
    resolve(b.faces[0])
    cov._update_registry(40.0)
    assert cov.pop_conversions() == [] and cov.census_stats["converted"] == 0
    assert cov.uav_map_report()["sectors"]["R-S3"]["new_detected"] == 1


# ====================================================================== 적 섹터 → CM 우선순위
def tier_cfg(**over):
    cfg, cov, grid, names = layout()
    cfg.explore.m1_priority_enemy_first = True
    for k, v in over.items():
        setattr(cfg.explore, k, v)
    return cfg, cov, grid, names


def weights(cfg, cov, m1, now=10.0, explorer=None):
    ex = explorer or OnlineExplorer(cfg, cov)
    return ex, ex._m1_tier_weights(set(m1), "BLUE", now)


def test_default_off_returns_no_tier_weights():
    cfg, cov, grid, names = tier_cfg(m1_priority_enemy_first=False)
    _, w = weights(cfg, cov, {"R-S2", "CM"})
    assert w == {}


def test_enemy_first_then_cm_released_when_enemy_faces_are_done():
    cfg, cov, grid, names = tier_cfg()
    ex, w = weights(cfg, cov, {"R-S2", "CM"})
    assert w["R-S2"] == pytest.approx(2.4) and w["CM"] == pytest.approx(1.6 * 0.30)
    assert ex.m1_tier["cm_deferred"] and not ex.m1_tier["enemy_done"] and ex.m1_tier["enemy"] == ["R-S2"]
    enemy = next(b for b, n in names.items() if server_sector(n) == "R-S2")
    for f in cov.boxes[enemy].faces:
        resolve(f)
    ex, w = weights(cfg, cov, {"R-S2", "CM"}, now=20.0, explorer=ex)
    assert w["CM"] == pytest.approx(1.6) and ex.m1_tier["enemy_done"] and not ex.m1_tier["cm_deferred"]
    for f in cov.boxes[enemy].faces:                                         # 득표가 잠시 흔들려도 한 번 풀린 CM 은 다시 보류하지 않는다
        f.det_n, f.det_votes = 0, {}
    ex, w = weights(cfg, cov, {"R-S2", "CM"}, now=21.0, explorer=ex)
    assert w["CM"] == pytest.approx(1.6)


def test_budget_releases_cm_even_if_enemy_is_unfinished():
    cfg, cov, grid, names = tier_cfg(m1_enemy_budget_s=60.0)
    ex, w = weights(cfg, cov, {"R-S2", "CM"}, now=10.0)
    assert w["CM"] < 1.0
    ex, w = weights(cfg, cov, {"R-S2", "CM"}, now=69.0, explorer=ex)
    assert w["CM"] < 1.0
    ex, w = weights(cfg, cov, {"R-S2", "CM"}, now=71.0, explorer=ex)
    assert w["CM"] == pytest.approx(1.6) and not ex.m1_tier["cm_deferred"]


def test_zero_budget_means_no_time_limit():
    cfg, cov, grid, names = tier_cfg(m1_enemy_budget_s=0.0)
    ex, w = weights(cfg, cov, {"R-S2", "CM"}, now=10.0)
    ex, w = weights(cfg, cov, {"R-S2", "CM"}, now=5000.0, explorer=ex)
    assert w["CM"] < 1.0


def test_exhaustive_mode_releases_m1_boost_for_other_scoring_sectors_after_budget():
    cfg,cov,grid,names = tier_cfg(m1_enemy_budget_s=60.,exhaustive_scoring_faces=True)
    ex,w = weights(cfg,cov,{'R-S1','R-S2'},now=10.)
    assert w == {'R-S1':2.4,'R-S2':2.4}
    ex,w = weights(cfg,cov,{'R-S1','R-S2'},now=71.,explorer=ex)
    assert w == {'R-S1':1.,'R-S2':1.}
    assert ex.m1_tier['enemy_priority_expired']
    assert cov.pending_faces(), 'Unfinished sides remain in the ledger; only the early boost expires.'
    ex,w = weights(cfg,cov,{'R-S3','CM'},now=72.,explorer=ex)
    assert w['R-S3'] == 2.4 and not ex.m1_tier['enemy_priority_expired']


def test_new_m1_command_restarts_the_clock():
    cfg, cov, grid, names = tier_cfg(m1_enemy_budget_s=60.0)
    ex, _ = weights(cfg, cov, {"R-S2", "CM"}, now=10.0)
    ex, w = weights(cfg, cov, {"R-S2", "CM"}, now=100.0, explorer=ex)
    assert w["CM"] == pytest.approx(1.6)
    ex, w = weights(cfg, cov, {"R-S3", "CM"}, now=101.0, explorer=ex)             # M1 이 구역을 바꿨다
    assert w["CM"] < 1.0 and w["R-S3"] == pytest.approx(2.4)


def test_cm_only_or_enemy_only_orders_need_no_deferral():
    cfg, cov, grid, names = tier_cfg()
    _, w = weights(cfg, cov, {"CM"})
    assert w == {"CM": pytest.approx(1.6)}
    _, w = weights(cfg, cov, {"R-S1", "R-S3"})
    assert w == {"R-S1": pytest.approx(2.4), "R-S3": pytest.approx(2.4)}


def test_own_side_sector_in_the_order_keeps_the_plain_m1_weight():
    cfg, cov, grid, names = tier_cfg()
    _, w = weights(cfg, cov, {"B-S2", "R-S2"})
    assert w["B-S2"] == pytest.approx(cfg.explore.m1_sector_weight) and w["R-S2"] == pytest.approx(2.4)


def expected_gain(c, mult):
    """후보의 면 가치 합 = 면마다 (기본 가치 1.0) × 그 면 섹터의 배수. 이 배치에서는 모든 면의 기본 가치가 1.0 이라 면 수와 같다."""
    return sum(mult.get(bid, 1.0) for bid, _ in c.faces)


def test_candidate_gains_follow_the_tier_weights():
    cfg, cov, grid, names = tier_cfg()
    enemy = next(b for b, n in names.items() if server_sector(n) == "R-S2")
    cm = next(b for b, n in names.items() if server_sector(n) == "CM")
    cfg.explore.m1_priority_enemy_first = False
    cfg.explore.m1_sector_weight = 1.0
    _, base = _built(cfg, cov, grid, m1={"R-S2", "CM"})
    assert all(c.gain_f == pytest.approx(len(c.faces)) for c in base if c.kind == "face")
    cfg.explore.m1_priority_enemy_first = True
    ex, tiered = _built(cfg, cov, grid, m1={"R-S2", "CM"})
    mult = {enemy: 2.4, cm: 1.6 * 0.30}
    assert ex.m1_tier["cm_deferred"]
    faces = [c for c in tiered if c.kind == "face"]
    assert faces and all(c.gain_f == pytest.approx(expected_gain(c, mult), rel=1e-6) for c in faces)
    for f in cov.boxes[enemy].faces:
        resolve(f)
    ex, after = _built(cfg, cov, grid, m1={"R-S2", "CM"})
    assert not ex.m1_tier["cm_deferred"] and ex.m1_tier["enemy_done"]
    faces = [c for c in after if c.kind == "face"]
    assert faces and all(c.gain_f == pytest.approx(expected_gain(c, {cm: 1.6}), rel=1e-6) for c in faces)
