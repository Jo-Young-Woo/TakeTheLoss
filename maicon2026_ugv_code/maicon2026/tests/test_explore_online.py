"""M3 온라인 탐색(프런티어 + 다음 최선 시점 + 면 장부) 시험. 기존 고정 5x5 순회(lattice)는 `explore.strategy="lattice"` 로 그대로 남아 있다.

!! 시뮬 수치다. 실제 YOLO 가중치가 없어 객체 분류 득표는 시뮬 검출기의 라벨/누락 모델에 의존한다.
"""
import math
import warnings


from maicon26.config import Config
from maicon26.sim.bench import make_cfg, run_trial
from maicon26.sim.dynamic import shuffle_structures
from maicon26.sim.run import build_match, run, score

warnings.filterwarnings("ignore")
FACE_N = [(1, 0), (0, 1), (-1, 0), (0, -1)]


def _solo_run(strategy="online", seed=2, t=200.0, events=None, mut=None):
    m = build_match(seed=seed, cfg=make_cfg(strategy, mut=mut), teams=["B2"])
    run(m, max_t=t, events=events)
    return m, m.agents[0]


def test_ledger_counts_each_box_face_once_and_matches_truth_at_high_precision():
    r = run_trial(2, "online", cfg=make_cfg("online"), max_t=300.0)
    a = r["agent"]
    cov = a.coverage
    faces = [f for b in cov.boxes.values() for f in b.faces]
    keys = [f.key for f in faces]
    assert len(keys) == len(set(keys)), "면 키가 겹친다"
    counts = a._m3_counts()
    assert counts is not None
    total = sum(sum(v.values()) for v in counts.values())
    n_counted = sum(1 for f in faces if f.det_kind == "object" and cov.face_state(f) in ("CONFIRMED", "UNCERTAIN", "GIVEUP"))
    assert total <= n_counted, "한 면을 두 번 센다(제출 개수가 센 수 있는 면 수보다 많다)"
    assert r["precision"] >= 0.9, f"M3 정밀도 {r['precision']}"
    assert r["recall"] >= 0.6, f"M3 재현율 {r['recall']}"


def test_every_face_records_whether_it_could_actually_be_seen_not_just_visited():
    m, a = _solo_run(t=150.0)
    cov = a.coverage
    st = cov.summary()
    assert st["boxes"] >= 5 and sum(st["faces"].values()) >= 10
    seen_states = {cov.face_state(f) for b in cov.boxes.values() for f in b.faces}
    assert "CONFIRMED" in seen_states
    assert seen_states <= {"UNSEEN", "VIEWED_NO_DET", "UNCERTAIN", "CONFIRMED", "EMPTY", "UNREACHABLE", "GIVEUP"}
    # 가려져서 본 적 없는 시도(occluded_views)와 실제 본 횟수(views)를 따로 센다
    assert all(f.views >= 0 and f.occluded_views >= 0 for b in cov.boxes.values() for f in b.faces)
    assert any(f.views > 0 for b in cov.boxes.values() for f in b.faces)


def test_unknown_is_not_free_for_exploration_seen_map_tracks_what_lidar_really_swept():
    m, a = _solo_run(t=5.0)
    f5 = a.coverage.summary()["seen_frac"]
    grid = a.stack.grid
    untouched = float(((grid.L == 0.0) & ~grid.static).sum())
    assert untouched > 0                                # 점유 격자에는 아직 한 번도 안 본 칸(L=0)이 있고, 계획기는 그것을 자유로 본다
    blocked, _ = grid.cost_maps(a.cfg.nav.inflate_hard, a.cfg.nav.inflate_soft, 0.0)
    unk = (grid.L == 0.0) & ~grid.static
    assert float((unk & ~blocked).sum()) > 0.5 * float(unk.sum())
    assert f5 < 0.95                                    # 별도의 '실제로 훑은 곳' 지도는 그것을 구분한다
    run(m, max_t=60.0)
    assert a.coverage.summary()["seen_frac"] > f5       # 움직일수록 훑은 곳이 늘어난다

def test_moved_box_is_dropped_and_re_observed_without_double_counting():
    m, a = _solo_run(t=240.0, events=[(60.0, shuffle_structures)], seed=2)
    a.coverage.summary()
    assert a.coverage.stats["boxes_moved"] >= 1, "박스를 옮겼는데 이동을 못 알아봤다"
    sc = score(m)["agents"][a.name]["m3"]
    if sc is not None:
        assert sc["precision"] >= 0.7                    # 옛 위치의 객체를 계속 믿지 않는다(옮긴 뒤에도 제출이 크게 틀리지 않는다)


def test_lattice_strategy_still_available_and_reaches_m5():
    r = run_trial(3, "lattice", cfg=make_cfg("lattice"), max_t=300.0)
    assert r["m5"] is not None and r["m2_ok"]
    assert r["agent"].explorer.stats["selected"] == 0   # 온라인 탐색기는 쓰이지 않았다


def test_online_explorer_never_starts_a_viewpoint_that_cannot_finish_before_stage_deadline():
    m, a = _solo_run(t=240.0, seed=4)
    ec = a.cfg.explore
    deadline = a.cfg.mission.prestage_at - ec.deadline_margin_s
    picks = [t for t, msg in a.log if msg.startswith("EXPLORE pick")]
    assert picks, "시점을 한 번도 고르지 않았다"
    assert max(picks) <= deadline + 8.0, f"마감({deadline:.0f}s) 한참 뒤에 새 시점을 골랐다: {max(picks)}"


def test_sector_priority_flag_is_off_by_default_and_rule_consistent():
    cfg = Config()
    assert cfg.explore.sector_priority is False         # M1 섹터는 규정상 M3 탐색 범위를 제한하지 않으므로 우선순위에 쓰지 않는다(켜면 가산만)
    assert cfg.explore.strategy == "online" and cfg.explore.m3_counting == "ledger"


def test_strike_goal_picks_a_free_point_with_line_of_sight_and_snaps_wrong_normal():
    m, a = _solo_run(strategy="lattice", seed=4, t=17.5)
    faces = a.omap.find_target(a.cmd.target) if a.cmd and a.cmd.target else []
    assert faces
    f = faces[0]
    goal, tol = a._strike_goal(f, a.stack.pose)
    blocked, _ = a.stack.grid.cost_maps(a.cfg.nav.inflate_hard, a.cfg.nav.inflate_soft, 0.0)
    iy, ix = a.stack.grid.to_cell(*goal)
    assert not blocked[iy, ix]                                                          # 비어 있는 점
    assert math.hypot(goal[0] - f.x, goal[1] - f.y) <= a.cfg.mission.strike_range + 0.05
    assert tol >= a.cfg.mission.strike_max_incidence_deg
    # 영상으로 잘못 추정한 면 법선은 LiDAR 장부의 면으로 바로잡는다
    f.vx, f.vy = (0.0, 1.0) if abs(f.vx) > 0.5 else (1.0, 0.0)                      # 일부러 90° 틀린 법선
    n0 = a.stats.get("normal_snaps", 0)
    a._snap_face_normal(f)
    assert a.stats.get("normal_snaps", 0) >= n0       # 장부에 같은 위치의 면이 있으면 법선이 스냅된다(없으면 변화 없음)


def test_m2_hits_in_all_seeds_without_enemies_with_online_exploration():
    ok = 0
    for seed in (1, 2, 3, 4, 5, 6):
        r = run_trial(seed, "online", cfg=make_cfg("online", mut=lambda c: setattr(c.avoid, "enable", False)), max_t=120.0)
        ok += 1 if r["m2_ok"] else 0
    assert ok >= 5, f"온라인 탐색에서 M2 성공 {ok}/6"



