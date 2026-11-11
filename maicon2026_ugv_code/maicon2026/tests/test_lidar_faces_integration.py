"""LiDAR 면 지도를 면 장부(CoverageMap)·탐색기(OnlineExplorer)에 연결한 부분: 이어 붙은 큐브 장부, 안쪽 면 제외, 닫히지 않은 그룹의 뒷면 정찰 후보,
팬틸트로 못 푼 면의 OAK 식별 후보. 시뮬을 쓰지 않는다(합성 LiDAR 장면 + 실제 OccupancyGrid)."""
import math

import numpy as np
import pytest

from lidar_scene import around, make_world
from maicon26.config import Config
from maicon26.mission.explore_online import Cand, OnlineExplorer
from maicon26.perception.coverage import CoverageMap, GIVEUP, UNCERTAIN, UNSEEN, VIEWED_NO_DET
from maicon26.types import Pose2D


def build_cov(cubes, poses, lidar=True, mut=None, **kw):
    cfg, g = make_world(cubes, poses, **kw)
    cfg.explore.lidar_faces = lidar
    if mut:
        mut(cfg)
    cov = CoverageMap(cfg)
    cov.refresh(g, None, 10.0, force=True)
    return cfg, g, cov


# ------------------------------------------------------------------------------------------------ 면 장부
def test_pair_becomes_two_boxes_and_internal_faces_are_not_pending():
    cfg, g, cov = build_cov([(3.0, 2.5), (3.4, 2.5)], around(3.2, 2.5, 1.2, 12))
    boxes = sorted(cov.boxes.values(), key=lambda b: b.cx)
    assert len(boxes) == 2 and all(b.closed is True for b in boxes)
    left, right = boxes
    assert 0 in left.internal and 2 in right.internal
    assert not left.faces[0].exterior_open and not right.faces[2].exterior_open
    pend = cov.pending_faces()
    assert len(pend) == 6                                                  # 바깥 면 6개(위 2 + 아래 2 + 양 끝 2), 맞닿은 면 2개는 빠진다
    assert all(f.idx not in cov.boxes[f.bid].internal for f in pend)


def test_l_shape_is_three_boxes_with_lidar_faces_but_dropped_without():
    scene = ([(3.0, 2.5), (3.4, 2.5), (3.4, 2.9)], around(3.3, 2.7, 1.3, 16))
    _, _, cov_old = build_cov(*scene, lidar=False)
    assert len(cov_old.boxes) == 0                                         # 기존 군집은 L 자를 박스로 인정하지 못한다
    _, _, cov_new = build_cov(*scene, lidar=True)
    assert len(cov_new.boxes) == 3
    assert {round(b.cx, 1) for b in cov_new.boxes.values()} == {3.0, 3.4}


def test_default_config_keeps_the_old_cluster_path():
    cfg = Config()
    assert cfg.explore.lidar_faces is False
    cov = CoverageMap(cfg)
    assert cov._lidar is None and cov.lidar_result is None


def test_summary_reports_lidar_stats_only_when_enabled():
    _, _, cov = build_cov([(3.0, 2.5)], around(3.0, 2.5))
    s = cov.summary()["lidar_faces"]
    assert s["groups"] == 1 and s["closed"] == 1 and s["units"] == 1
    _, _, cov0 = build_cov([(3.0, 2.5)], around(3.0, 2.5), lidar=False)
    assert cov0.summary()["lidar_faces"] is None


def test_ugv_points_reach_the_face_map_and_exclude_the_ugv():
    ugv = [(3.45, 2.5, 3.45 + 0.125, 2.5 + 0.125)]
    ugv = [(3.325, 2.375, 3.575, 2.625)]
    cfg, g = make_world([(3.0, 2.5)], around(3.0, 2.5, 1.1, 10), ugv=ugv)
    cfg.explore.lidar_faces = True
    cov = CoverageMap(cfg)
    cov.set_ugv_points([(3.45, 2.5, 0.30), (float("nan"), 1.0, 0.3)])
    assert cov._ugv_points == [(3.45, 2.5, 0.3)]                           # 유한하지 않은 값은 버린다
    cov.refresh(g, None, 10.0, force=True)
    assert cov.lidar_result.stats["ugv_cells_removed"] > 0
    b = next(iter(cov.boxes.values()))
    assert b.closed is None                                                # UGV 가 가린 쪽 — 열림으로 오인하지 않는다
    assert all(math.hypot(b.cx - 3.45, b.cy - 2.5) > 0.3 for b in cov.boxes.values())


def test_box_matched_again_keeps_identity_and_updates_closure():
    cfg, g = make_world([(3.0, 2.5)], [(1.6, 2.5, 0.0), (1.5, 2.5, 0.0), (1.7, 2.5, 0.0)])
    cfg.explore.lidar_faces = True
    cov = CoverageMap(cfg)
    cov.refresh(g, None, 10.0, force=True)
    b = next(iter(cov.boxes.values()))
    assert b.closed is False and b.kind == "face"
    bid = b.bid
    cfg2, g2 = make_world([(3.0, 2.5)], around(3.0, 2.5))
    cov.cfg = cfg2
    cov.refresh(g2, None, 12.0, force=True)
    again = next(iter(cov.boxes.values()))
    assert again.bid == bid and again.closed is True and again.kind == "full"


# ------------------------------------------------------------------------------------------------ 뒷면 정찰
def explorer_for(cubes, poses, mut=None, **kw):
    cfg, g, cov = build_cov(cubes, poses, mut=mut, **kw)
    return cfg, g, cov, OnlineExplorer(cfg, cov)


def cands_of(cfg, g, cov, ex, pose=(1.5, 2.5, 0.0), now=11.0):
    blocked, _ = cov.planning_cost_maps(g)
    return ex.build(Pose2D(*pose), g, blocked, now, team=None)


def test_open_group_gets_scout_candidates_on_the_hidden_side():
    cfg, g, cov, ex = explorer_for([(3.0, 2.5)], [(1.6, 2.5, 0.0), (1.5, 2.5, 0.0), (1.7, 2.5, 0.0)])
    scouts = [c for c in cands_of(cfg, g, cov, ex) if c.kind == "scout"]
    assert scouts, "닫히지 않은 그룹은 뒷면 정찰 후보가 있어야 한다"
    for c in scouts:
        assert c.mode == "look" and c.scout_key is not None
        assert c.vp[0] > 3.0                                               # 로봇이 본 서쪽 반대편 = 동쪽 뒷면 쪽
        assert abs(math.atan2(2.5 - c.vp[1], 3.1 - c.vp[0]) - c.heading) < 0.6      # 그 쪽을 본다
        assert c.gain >= cfg.explore.lidar_open_bonus


def test_closed_group_has_no_scout():
    cfg, g, cov, ex = explorer_for([(3.0, 2.5)], around(3.0, 2.5))
    assert not [c for c in cands_of(cfg, g, cov, ex) if c.kind == "scout"]


def test_scout_stops_after_max_attempts_via_note_observation_and_mark_bad():
    cfg, g, cov, ex = explorer_for([(3.0, 2.5)], [(1.6, 2.5, 0.0), (1.5, 2.5, 0.0), (1.7, 2.5, 0.0)])
    first = [c for c in cands_of(cfg, g, cov, ex) if c.kind == "scout"]
    assert first
    ex.note_observation(first[0], 11.0)                                    # 도착해서 관측을 끝낸 한 번
    second = [c for c in cands_of(cfg, g, cov, ex, now=12.0) if c.kind == "scout"]
    assert second
    ex.mark_bad(second[0].vp, 12.0)                                        # 못 닿거나 실패한 한 번
    third = [c for c in cands_of(cfg, g, cov, ex, now=13.0) if c.kind == "scout"]
    assert third == []                                                      # lidar_scout_max_attempts(2) 소진


def test_scout_not_offered_when_ugv_is_near_the_group():
    cfg, g = make_world([(3.0, 2.5)], [(1.6, 2.5, 0.0), (1.5, 2.5, 0.0), (1.7, 2.5, 0.0)])
    cfg.explore.lidar_faces = True
    cov = CoverageMap(cfg)
    cov.set_ugv_points([(3.4, 2.5, 0.3)])
    cov.refresh(g, None, 10.0, force=True)
    ex = OnlineExplorer(cfg, cov)
    assert not [c for c in cands_of(cfg, g, cov, ex) if c.kind == "scout"]


def test_scout_disabled_without_lidar_faces():
    cfg, g, cov, ex = explorer_for([(3.0, 2.5)], [(1.6, 2.5, 0.0), (1.5, 2.5, 0.0), (1.7, 2.5, 0.0)], lidar=False)
    assert not [c for c in cands_of(cfg, g, cov, ex) if c.kind == "scout"]


def test_scout_incumbent_is_refreshed_and_dropped_when_group_closes():
    cfg, g, cov, ex = explorer_for([(3.0, 2.5)], [(1.6, 2.5, 0.0), (1.5, 2.5, 0.0), (1.7, 2.5, 0.0)])
    cur = next(c for c in cands_of(cfg, g, cov, ex) if c.kind == "scout")
    again = [c for c in cands_of(cfg, g, cov, ex, now=12.0) if c.kind == "scout" and c.incumbent is False]
    assert again
    blocked, _ = cov.planning_cost_maps(g)
    out = ex.build(Pose2D(1.5, 2.5, 0.0), g, blocked, 12.0, current=cur, team=None)
    inc = [c for c in out if c.incumbent]
    assert inc and inc[0].kind == "scout" and inc[0].gain_u == cfg.explore.lidar_open_bonus and inc[0].scout_key == cur.scout_key
    ex._open_scouts = {}                                                    # 그룹이 닫혀 열린 정찰 목록에서 빠지면 가치가 0 이 된다
    cur2 = Cand("scout", cur.vp, cur.heading, "look", [], scout_key=cur.scout_key)
    inc2 = ex._refresh_incumbent(cur2, [], cov.visibility_occupancy(g), g, blocked, cov.seen.solid(g), cov.seen.unseen(cov.seen.solid(g)), 12.0,
                                 lambda f, vp: 0.0)
    assert inc2 is not None and inc2.gain == 0.0                            # evaluate 의 min_gain 에서 걸러진다


# ------------------------------------------------------------------------------------------------ OAK 식별 후보
def identify_cfg(cfg):
    cfg.oak_det.enable = True
    cfg.oak_det.identify_fallback = True


def make_unresolved(cfg, g, cov, face_idx=2, attempts=2):
    b = next(iter(cov.boxes.values()))
    f = b.faces[face_idx]
    f.arrive_attempts = attempts
    return b, f


def test_identify_candidate_in_front_of_unresolved_face_only_when_enabled_and_ready():
    cfg, g, cov, ex = explorer_for([(3.0, 2.5)], around(3.0, 2.5), mut=identify_cfg)
    b, f = make_unresolved(cfg, g, cov)
    assert cov.face_state(f) in (UNSEEN, VIEWED_NO_DET)
    ids = [c for c in cands_of(cfg, g, cov, ex, pose=(2.0, 2.5, 0.0)) if c.kind == "identify"]
    assert ids
    for c in ids:
        assert c.faces == [f.key] and c.mode == "look" and c.gain_f == cfg.oak_det.identify_gain
        d = math.hypot(c.vp[0] - f.cx, c.vp[1] - f.cy)
        assert abs(d - cfg.oak_det.identify_dist_m) < 0.3
        assert c.vp[0] < f.cx                                               # 서쪽 면(W) 앞
    ex.identify_ready = False
    assert not [c for c in cands_of(cfg, g, cov, ex, pose=(2.0, 2.5, 0.0)) if c.kind == "identify"]


def test_identify_not_offered_before_pantilt_gave_it_min_attempts():
    cfg, g, cov, ex = explorer_for([(3.0, 2.5)], around(3.0, 2.5), mut=identify_cfg)
    make_unresolved(cfg, g, cov, attempts=1)
    assert not [c for c in cands_of(cfg, g, cov, ex) if c.kind == "identify"]


def test_identify_off_by_default_and_when_oak_det_disabled():
    cfg, g, cov, ex = explorer_for([(3.0, 2.5)], around(3.0, 2.5))
    make_unresolved(cfg, g, cov)
    assert not [c for c in cands_of(cfg, g, cov, ex) if c.kind == "identify"]
    cfg.oak_det.identify_fallback = True                                    # enable 이 꺼져 있으면 여전히 없다
    assert not [c for c in cands_of(cfg, g, cov, ex) if c.kind == "identify"]


def test_identify_includes_giveup_faces_and_stops_after_max_attempts():
    cfg, g, cov, ex = explorer_for([(3.0, 2.5)], around(3.0, 2.5), mut=identify_cfg)
    b, f = make_unresolved(cfg, g, cov, attempts=3)
    assert cov.face_state(f) == GIVEUP and cov.face_weight(f) == 0.0        # 팬틸트 쪽 가치는 0 이지만
    ids = [c for c in cands_of(cfg, g, cov, ex) if c.kind == "identify"]
    assert ids                                                              # OAK 식별 대상이다
    ex.note_observation(ids[0], 11.0)
    assert f.id_tries == 1 and f.arrive_attempts == 3
    ids = [c for c in cands_of(cfg, g, cov, ex, now=12.0) if c.kind == "identify"]
    assert ids
    ex.mark_bad(ids[0].vp, 12.0)                                            # 도착 실패도 시도로 센다
    assert f.id_tries == 2
    assert not [c for c in cands_of(cfg, g, cov, ex, now=13.0) if c.kind == "identify"]


def test_identify_skips_confirmed_empty_internal_and_other_faces():
    cfg, g, cov, ex = explorer_for([(3.0, 2.5), (3.4, 2.5)], around(3.2, 2.5, 1.2, 12), mut=identify_cfg)
    for b in cov.boxes.values():
        for f in b.faces:
            f.arrive_attempts = 2
    keys = {f.key for f in cov.identify_faces()}
    for b in cov.boxes.values():
        for i in b.internal:
            assert b.faces[i].key not in keys                              # 맞닿은 안쪽 면은 식별할 수 없다
    assert len(keys) == 6
    f0 = next(iter(keys))
    cov.get_face(f0).det_n, cov.get_face(f0).det_votes = 6, {"cls_a": 6.0}
    cov.get_face(f0).det_kind = "object"
    st = cov.face_state(cov.get_face(f0))
    assert st == "CONFIRMED"
    assert f0 not in {f.key for f in cov.identify_faces()}                 # 확정된 면은 대상이 아니다


def test_identify_incumbent_survives_while_face_is_still_eligible():
    cfg, g, cov, ex = explorer_for([(3.0, 2.5)], around(3.0, 2.5), mut=identify_cfg)
    b, f = make_unresolved(cfg, g, cov, attempts=3)
    cur = next(c for c in cands_of(cfg, g, cov, ex) if c.kind == "identify")
    blocked, _ = cov.planning_cost_maps(g)
    out = ex.build(Pose2D(2.0, 2.5, 0.0), g, blocked, 12.0, current=cur, team=None)
    inc = [c for c in out if c.incumbent]
    assert inc and inc[0].kind == "identify" and inc[0].gain_f == cfg.oak_det.identify_gain
    f.id_tries = cfg.oak_det.identify_max_attempts                          # 더는 식별 대상이 아니면 현재 목표에서 내려온다
    out = ex.build(Pose2D(2.0, 2.5, 0.0), g, blocked, 13.0, current=cur, team=None)
    assert not [c for c in out if c.incumbent]
