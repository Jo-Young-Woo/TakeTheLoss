"""① 섹터 완성 우선(explore.sector_finish_bonus) ② M3 제출 직전 섹터별 객체 리스트 중복 제거(explore.m3_dedupe)
③ 지정표적(M2) 접근 포기 조건(mission.m2_robust_abandon). ROS·네트워크·시뮬 월드 없이 돈다. YOLO 분류는 장부에 득표를 직접 넣는다."""
from __future__ import annotations

from types import SimpleNamespace

import pytest

from maicon26.config import Config
from maicon26.link.schema import scoring_sectors, server_sector
from maicon26.mission.explore_online import OnlineExplorer
from maicon26.mission.manager import Agent
from maicon26.nav.grid import OccupancyGrid
from maicon26.perception.coverage import BoxModel, CONFIRMED, CoverageMap
from maicon26.perception.object_map import ObjectMap
from maicon26.types import Pose2D


def resolve(f, cls="TANK", votes=6):
    f.det_n, f.det_votes, f.det_kind = votes, {cls: float(votes)}, "object"


def weak(f, cls="TANK"):
    f.det_n, f.det_votes, f.det_kind = 2, {cls: 2.0}, "object"       # 득표 부족 — 아직 확정 전(후보)


def add_box(cov, bid, x, y):
    b = BoxModel(bid, x, y, "full", [], 1., 1., full_seen=3, last_full_t=1.)
    cov._make_faces(b)
    cov.boxes[bid] = b
    return b


def sector_xy(cfg, sector, dx=0.0, dy=0.0):
    x, y = cfg.arena.sector_center(sector)
    return x + dx, y + dy


# ====================================================================== ② 섹터별 객체 리스트 + 제출 전 중복 제거
def cov_blue():
    cfg = Config()
    cfg.explore.complete_m3_faces = True
    return cfg, CoverageMap(cfg), ObjectMap(cfg)


def test_sector_object_list_marks_confirmed_and_keeps_unconfirmed_as_pending_not_counted():
    cfg, cov, omap = cov_blue()
    b = add_box(cov, 1, *sector_xy(cfg, "R-S1"))
    resolve(b.faces[0], "TANK")
    weak(b.faces[1], "DRUM")                                                         # 객체는 보였지만 득표 부족
    lst = cov.sector_object_list(omap, sectors=scoring_sectors("BLUE"))
    sec = server_sector("R-S1")
    ent = lst[sec]
    assert [(e["cls"], e["status"]) for e in ent] == [("TANK", "confirmed"), ("DRUM", "pending")]
    assert cov.m3_counts(omap) == {"R-S1": {"TANK": 1}}                              # 후보는 세지 않는다
    assert cov.face_state(b.faces[0]) == CONFIRMED


def test_object_lists_cover_enemy_sectors_and_cm_only_when_asked():
    cfg, cov, omap = cov_blue()
    for bid, sec in enumerate(["R-S1", "R-S2", "R-S3", "B-S1", "COMMON"], 1):
        b = add_box(cov, bid, *sector_xy(cfg, sec))
        resolve(b.faces[0], "MISSILE")
    lst = cov.sector_object_list(omap, sectors=scoring_sectors("BLUE"))
    assert set(lst) == {server_sector(s) for s in scoring_sectors("BLUE")}
    assert "B-S1" not in lst                                                        # 아군 구역은 제출 목록에 없다
    assert set(cov.sector_object_list(omap)) >= {"B-S1"}                            # 지정하지 않으면 전체 장부


def _twin_boxes(cfg, cov, omap_unused, offset):
    x, y = sector_xy(cfg, "R-S2")
    a = add_box(cov, 1, x, y)
    b = add_box(cov, 2, x + offset[0], y + offset[1])                                # 같은 상자가 둘로 갈라져 장부에 들어간 경우
    resolve(a.faces[1], "TRUCK")
    resolve(b.faces[1], "TRUCK")
    return a, b


def test_same_face_recorded_by_two_split_boxes_is_counted_once_when_dedupe_is_on():
    cfg, cov, omap = cov_blue()
    _twin_boxes(cfg, cov, omap, (0.06, 0.04))
    assert cov.m3_counts(omap) == {"R-S2": {"TRUCK": 2}}                             # 끔: 기존 동작 그대로
    out = cov.m3_counts(omap, dedupe=True, dedupe_dist=cfg.explore.m3_dedupe_dist_m)
    assert out == {"R-S2": {"TRUCK": 1}}
    assert cov.m3_dedupe_last["merged"] == 1


def test_split_boxes_with_conflicting_classes_keep_the_better_voted_one():
    cfg, cov, omap = cov_blue()
    a, b = _twin_boxes(cfg, cov, omap, (0.05, 0.0))
    resolve(b.faces[1], "DRUM", votes=9)
    out = cov.m3_counts(omap, dedupe=True)
    assert out == {"R-S2": {"DRUM": 1}}


def test_neighbouring_boxes_and_faces_of_one_box_are_never_merged():
    cfg, cov, omap = cov_blue()
    x, y = sector_xy(cfg, "R-S2")
    a, b = add_box(cov, 1, x, y), add_box(cov, 2, x, y + 0.4)                        # 이웃 상자(같은 방향 면이 0.4 m 간격)
    for f in a.faces + b.faces:
        resolve(f, "TANK")                                                           # 한 상자의 네 면(법선이 다르다)
    assert cov.m3_counts(omap, dedupe=False) == {"R-S2": {"TANK": 8}}
    assert cov.m3_counts(omap, dedupe=True, dedupe_dist=0.25) == {"R-S2": {"TANK": 8}}
    assert cov.m3_dedupe_last["merged"] == 0


def test_dedupe_never_merges_across_sectors():
    e = [{"sec": "R-S1", "cls": "TANK", "x": 1.0, "y": 1.0, "nx": 1., "ny": 0., "conf": .9, "n": 5, "src": "ledger", "key": (1, 0), "status": "confirmed"},
         {"sec": "R-S2", "cls": "TANK", "x": 1.02, "y": 1.0, "nx": 1., "ny": 0., "conf": .9, "n": 5, "src": "ledger", "key": (2, 0), "status": "confirmed"}]
    kept, merged = CoverageMap.dedupe_entries(e, 0.25)
    assert len(kept) == 2 and merged == []


def test_agent_snapshot_uses_dedupe_only_when_configured():
    cfg = Config()
    cfg.explore.complete_m3_faces = True
    cov, omap = CoverageMap(cfg), ObjectMap(cfg)
    _twin_boxes(cfg, cov, omap, (0.06, 0.04))
    agent = Agent.__new__(Agent)
    agent.cfg, agent.team, agent.coverage, agent.omap = cfg, "BLUE", cov, omap
    assert cfg.explore.m3_dedupe is False and cfg.explore.m3_counting == "ledger"
    assert agent._m3_counts()["R-S2"]["TRUCK"] == 2
    cfg.explore.m3_dedupe = True
    counts = agent._m3_counts()
    assert counts["R-S2"]["TRUCK"] == 1
    assert set(counts) == {"R-S1", "R-S2", "R-S3", "COMMON"}                         # 채점 구역 전부(0 포함)가 매번 들어간다


# ====================================================================== ① 섹터 완성 우선
def layout(bonus):
    cfg = Config()
    cfg.explore.sector_finish_bonus = bonus
    cov, grid = CoverageMap(cfg), OccupancyGrid(cfg.arena)
    boxes = {}
    for bid, sector in enumerate(cfg.arena.sector_names(), 1):
        cx, cy = cfg.arena.sector_center(sector)
        boxes[server_sector(sector)] = add_box(cov, bid, cx, cy)
        for iy in range(grid.ny):
            for ix in range(grid.nx):
                x, y = grid.to_world(iy, ix)
                if abs(x - cx) < .2 and abs(y - cy) < .2:
                    grid.L[iy, ix] = grid.L_MAX
    return cfg, cov, grid, boxes


def built(cfg, cov, grid, team="BLUE"):
    ex = OnlineExplorer(cfg, cov)
    blocked, _ = cov.planning_cost_maps(grid)
    return ex, ex.build(Pose2D(.4, 2.5, 0.), grid, blocked, 10., team=team)


def ratios(base, boosted):
    """같은 시점(kind, vp)의 이득 비율 — 두 번의 build 는 같은 배치라 후보가 같다."""
    ref = {(c.kind, round(c.vp[0], 3), round(c.vp[1], 3)): c for c in base}
    out = []
    for c in boosted:
        r = ref.get((c.kind, round(c.vp[0], 3), round(c.vp[1], 3)))
        if r is not None and r.gain_f > 0:
            out.append((c, c.gain_f / r.gain_f))
    return out


def setup_progress(boxes):
    for f in boxes["R-S1"].faces[:3]:
        resolve(f, "TANK")                                                           # R-S1: 4 면 중 3 면 끝, 1 면 남음 → 가산
    resolve(boxes["R-S2"].faces[0], "TANK")                                          # R-S2: 1 면 끝, 3 면 남음 → 끝난 비율 미달이라 가산 없음
    # R-S3: 아무것도 안 봤다(4 면 남음 > max_todo)


def test_sector_finish_bonus_favours_the_nearly_done_scoring_sector():
    cfg0, cov0, grid0, b0 = layout(0.0)
    setup_progress(b0)
    ex0, base = built(cfg0, cov0, grid0)
    assert ex0.finish_sectors == {}
    cfg1, cov1, grid1, b1 = layout(1.0)
    setup_progress(b1)
    ex1, boosted = built(cfg1, cov1, grid1)
    assert ex1.finish_sectors == {"R-S1": 1.0}                                       # 남은 면 1개 → 가산 전부
    rs = ratios(base, boosted)
    pending = (b1["R-S1"].bid, next(f.idx for f in b1["R-S1"].faces if f.det_n == 0))
    with_it = [r for c, r in rs if pending in c.faces]
    without = [r for c, r in rs if pending not in c.faces]
    assert with_it and without
    assert max(with_it) == pytest.approx(2.0, rel=0.02) or max(with_it) > 1.2          # 그 면이 들어간 시점은 이득이 늘고(그 면만 보면 2배)
    assert all(r == pytest.approx(1.0, rel=1e-6) for r in without)                  # 그 면이 없는 시점은 그대로


def test_sector_finish_bonus_scales_with_remaining_faces_and_needs_enough_done():
    cfg, cov, grid, boxes = layout(0.9)
    cfg.explore.sector_finish_max_todo = 3
    cfg.explore.sector_finish_min_done = 0.25
    for f in boxes["R-S1"].faces[:2]:
        resolve(f, "TANK")                                                           # 2 면 남음 → 0.9·(3+1-2)/3 = 0.6
    resolve(boxes["R-S2"].faces[0], "TANK")                                          # 3 면 남음, 끝난 비율 0.25 → 0.9·1/3 = 0.3
    ex, _ = built(cfg, cov, grid)
    assert ex.finish_sectors == {"R-S1": pytest.approx(0.6), "R-S2": pytest.approx(0.3)}


def test_sector_finish_ignores_sectors_outside_the_scoring_territory_and_is_off_by_default():
    assert Config().explore.sector_finish_bonus == 0.0
    cfg, cov, grid, boxes = layout(1.0)
    for f in boxes["B-S1"].faces[:3]:
        resolve(f, "TANK")                                                           # 아군 구역(BLUE 의 채점 구역이 아니다)
    ex, _ = built(cfg, cov, grid)
    assert ex.finish_sectors == {}
    ex2, _ = built(cfg, cov, grid, team=None)                                        # 팀을 모르는 도구 호출은 기존 그대로
    assert ex2.finish_sectors == {}


# ====================================================================== ③ M2 접근 포기 조건
def m2_agent(**over):
    a = Agent.__new__(Agent)
    a.cfg = Config()
    a.cfg.mission.m2_robust_abandon = True
    for k, v in over.items():
        setattr(a.cfg.mission, k, v)
    a.target_failed, a._m2_defer_s, a._m2_unreach_n, a._m2_retry_until = False, 0.0, 0, -1e9
    a.stats = {}
    return a


def test_defaults_keep_legacy_m2_abandon_and_real_config_enables_the_new_one():
    mc = Config().mission
    assert mc.m2_robust_abandon is False and (mc.m2_nav_budget_s, mc.m2_blocked_s, mc.m2_unreach_max) == (12.0, 4.0, 3)
    real = Config.load("config/waveshare_real.json")
    assert real.mission.m2_robust_abandon is True and real.explore.m3_dedupe is True and real.explore.sector_finish_bonus > 0


def test_one_blocked_tick_after_12s_no_longer_abandons_the_target():
    a = m2_agent()
    S = {"t0": 0.0, "defer0": 0.0}
    assert a._m2_blocked_verdict(13.0, S, "blocked") is None                         # 예전: 여기서 영구 포기
    assert a._m2_blocked_verdict(13.2, S, "moving") is None                          # 막힘이 풀리면 타이머를 되돌린다
    assert S["blocked_since"] is None
    assert a._m2_blocked_verdict(14.0, S, "blocked") is None and a.target_failed is False


def test_avoidance_time_is_not_charged_to_the_approach_budget():
    a = m2_agent()
    S = {"t0": 0.0, "defer0": 0.0}
    a._m2_defer_s = 9.0                                                              # 12 s 중 9 s 는 회피에 썼다
    assert a._m2_blocked_verdict(13.0, S, "blocked") is None
    assert a._m2_blocked_verdict(18.0, S, "blocked") is None                         # 실제 접근 시간 9 s < 12 s
    a._m2_defer_s = 9.0
    verdict = a._m2_blocked_verdict(25.0, S, "blocked")                              # 실제 접근 16 s, 막힘 12 s 이어짐
    assert verdict is not None and "retry" in verdict and a.target_failed is False


def test_sustained_block_retries_after_cooldown_and_only_the_last_attempt_is_permanent():
    a = m2_agent(m2_unreach_max=3)
    cd = a.cfg.progress.retry_after_s
    for i in range(1, 3):
        S = {"t0": 0.0, "defer0": 0.0}
        assert a._m2_blocked_verdict(20.0, S, "blocked") is None
        v = a._m2_blocked_verdict(25.0, S, "blocked")
        assert v is not None and f"({i}/3)" in v and a.target_failed is False
        assert a._m2_retry_until == pytest.approx(25.0 + cd)
    S = {"t0": 0.0, "defer0": 0.0}
    a._m2_blocked_verdict(20.0, S, "blocked")
    v = a._m2_blocked_verdict(25.0, S, "blocked")
    assert "permanent" in v and a.target_failed is True and a.stats["m2_unreach_n"] == 3


def test_decision_waits_for_the_m2_retry_cooldown_even_if_avoid_deferral_is_exhausted():
    """STRIKE 진입 판정이 쿨다운(_m2_retry_until)을 지키는지 — 회피 연기 합계 상한과 무관하게."""
    import inspect
    src = inspect.getsource(Agent)
    i = src.index("if now < getattr(self, \"_m2_retry_until\", -1e9):")
    assert "return Mode.EXPLORE" in src[i:i + 200]
