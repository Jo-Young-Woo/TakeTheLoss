"""적 UGV 회피(안전 계층) 시험: 위협 보드·위협장·위험 인지 경로 추정·회피 제어기·LiDAR 동적 군집, 그리고 시뮬 시나리오.

!! 시뮬 결과는 '그 상대 모델(완벽 정보 추격·조준)에서의 반응'이지 실기 성능이 아니다. 실제 YOLO 가중치·UAV 영상 검출기가 없어서
   카메라/UAV 입력은 시뮬이 주는 라벨로 대신한다(README '적 UGV 회피' 절).
"""
import math
import warnings

import numpy as np
import pytest

from maicon26.config import Config
from maicon26.mission import avoid as av
from maicon26.mission.threat_field import body_hit, build_field, path_exposure, risk_at
from maicon26.nav import risk_astar
from maicon26.nav.grid import OccupancyGrid, astar
from maicon26.perception.lidar_blobs import BlobDetector
from maicon26.perception.threat_board import EV_ENEMY, EV_OBJECT, EV_UGV, ThreatBoard, ThreatTrack
from maicon26.perception.tracks import UgvTracker
from maicon26.sim.bench import make_cfg, run_trial
from maicon26.sim.run import build_match
from maicon26.sim.scenarios import build
from maicon26.types import Pose2D, Scan

warnings.filterwarnings("ignore")


def _grid(cfg=None):
    cfg = cfg or Config()
    return cfg, OccupancyGrid(cfg.arena, cfg.nav.res)


def _tt(x, y, vx=0.0, vy=0.0, w=0.7, ev=EV_UGV, sigma=0.08, t=60.0, friend=False, p=0.5):
    tr = ThreatTrack(1, x, y, vx, vy, sigma, ev, friend, p, t, {"cam": t})
    tr.weight = w
    return tr


# ====================================================================== 위협 보드: 증거 사다리
def test_board_team_belief_with_label_noise_and_unknown_is_conservative():
    cfg = Config()
    tracker = UgvTracker(cfg, "BLUE")
    b = ThreatBoard(cfg, "BLUE")
    # 적(RED) 라벨 10번 중 1번 뒤집힘(10 %): 적으로 수렴
    for i in range(10):
        tracker._upsert(3.0, 2.0, 1.0 + 0.1 * i, "BLUE" if i == 4 else "RED", 0.95, "cam")
    # 라벨이 한 번도 없는 UGV(팀 미상)
    tracker._upsert(1.0, 1.0, 1.0, None, 0.9, "cam")
    # 아군(BLUE) 라벨만 계속
    for i in range(6):
        tracker._upsert(5.0, 4.0, 1.0 + 0.1 * i, "BLUE", 0.95, "cam")
    out = b.update(tracker, [], 2.0, Pose2D(0.5, 0.5, 0.0))
    by = {round(t.x): t for t in out}
    assert by[3].evidence == EV_ENEMY and by[3].p_enemy >= cfg.avoid.enemy_p and by[3].weight >= cfg.avoid.p_cons
    assert by[1].evidence == EV_UGV and not by[1].friend and by[1].weight == pytest.approx(cfg.avoid.p_cons)   # 미상 = 적과 같은 보수 가중
    assert by[5].friend and by[5].weight == pytest.approx(cfg.avoid.friend_w)                                   # 아군: 위험 가중만 낮춘다


def test_board_lidar_blob_without_camera_becomes_object_and_ghost_decays():
    cfg = Config()
    from maicon26.perception.lidar_blobs import Blob
    b = ThreatBoard(cfg, "BLUE")
    out = b.update(None, [Blob(7, 2.0, 2.0, 0.3, 0.0, 0.25, 5, 1.0, 0.3)], 1.0, Pose2D(0, 0, 0))
    assert len(out) == 1 and out[0].evidence == EV_OBJECT and out[0].weight == pytest.approx(cfg.avoid.p_cons)
    late = b.update(None, [], 1.0 + cfg.avoid.hold_track_s + 0.5 * cfg.avoid.ghost_s, Pose2D(0, 0, 0))
    assert late and late[0].weight < cfg.avoid.p_cons          # 관측이 끊기면 ghost 로 위험 가중이 줄어든다
    gone = b.update(None, [], 1.0 + cfg.avoid.hold_track_s + cfg.avoid.ghost_s + 1.0, Pose2D(0, 0, 0))
    assert gone == []


# ====================================================================== 위협장
def test_field_body_follows_prediction_and_box_blocks_risk():
    cfg, g = _grid()
    ac = cfg.avoid
    tt = _tt(2.0, 2.5, vx=0.4)                                 # +x 로 0.4 m/s
    f = build_field([tt], g, ac, cfg.laser.max_range, 0.0, cfg.nav.inflate_hard)
    assert f is not None and f.K == ac.layers
    assert body_hit(f, g, 2.0, 2.5, 0.0)                       # 현재 위치는 점유
    assert not body_hit(f, g, 2.0 + 0.4 * 2.25 + 1.2, 2.5, 0.0)
    assert body_hit(f, g, 2.0 + 0.4 * 2.25, 2.5, 2.25)         # 마지막 층에서는 예측 위치(+0.9 m)가 점유
    near = risk_at(f, g, 3.0, 2.5, 0.0)
    assert near > 0.6                                          # 사정거리·시야 안: 위험 큼
    # 박스가 시야를 가리면 위험이 크게 준다
    g.add_static_box((2.5, 2.2, 2.9, 2.8))
    f2 = build_field([tt], g, ac, cfg.laser.max_range, 0.0, cfg.nav.inflate_hard)
    assert risk_at(f2, g, 3.4, 2.5, 0.0) < 0.35 * risk_at(f, g, 3.4, 2.5, 0.0)
    assert build_field([], g, ac, cfg.laser.max_range, 0.0, 0.17) is None


def test_risk_astar_equals_plain_astar_without_field_and_avoids_body():
    cfg, g = _grid()
    blocked, cost = g.cost_maps(cfg.nav.inflate_hard, cfg.nav.inflate_soft, cfg.nav.soft_weight)
    p0 = astar(g, blocked, cost, (0.8, 2.5), (5.0, 2.5))
    r0 = risk_astar.plan(g, blocked, cost, (0.8, 2.5), (5.0, 2.5), 0.35, None)
    assert p0 and r0 and abs(r0.length - sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(p0[:-1], p0[1:]))) < 0.3
    tt = _tt(3.0, 2.5)                                         # 일직선 경로 한가운데에 정지한 적
    f = build_field([tt], g, cfg.avoid, cfg.laser.max_range, 0.0, cfg.nav.inflate_hard)
    r1 = risk_astar.plan(g, blocked, cost, (0.8, 2.5), (5.0, 2.5), 0.35, f, 3.0, 1.5)
    assert r1 is not None
    assert all(math.hypot(x - 3.0, y - 2.5) > 0.4 for x, y in r1.path)   # 적 차체를 통과하지 않는다
    assert r1.length > r0.length                                         # 돌아간다
    ri, mx = path_exposure(r1.path, 0.35, f, g)
    assert ri >= 0.0 and mx >= 0.0


# ====================================================================== 회피 제어기
def _ctx(cfg, g, tracks, pose=None, t=60.0, now=60.0, slack=1e9, in_goal=False):
    pose = pose or Pose2D(1.0, 2.5, 0.0)
    blocked, cost = g.cost_maps(cfg.nav.inflate_hard + cfg.avoid.risk_extra_inflate_m, cfg.nav.inflate_soft, cfg.nav.soft_weight)
    fld = build_field(tracks, g, cfg.avoid, cfg.laser.max_range, now, cfg.nav.inflate_hard) if tracks else None
    return av.AvoidCtx(now, 0.05, t, pose, 0.0, 0.0, None, g, blocked, cost, fld, tracks, 0.3, slack, in_goal, False, None)


def test_avoid_levels_have_hysteresis_and_step_down_one_at_a_time():
    cfg, g = _grid()
    ctl = av.AvoidController(cfg)
    seq = []
    now = 10.0
    for d in (4.0, 2.5, 1.5, 1.7, 1.55, 1.9, 2.0, 2.3, 2.6, 3.5, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0):
        for _ in range(int(1.0 / 0.05)):                 # 거리를 1 s 유지
            ctl.assess(_ctx(cfg, g, [_tt(1.0 + d, 2.5, t=now, w=0.7)], now=now))
            now += 0.05
        seq.append(int(ctl.level))
    assert seq[1] == av.Level.AWARE and seq[2] == av.Level.EVADE
    assert seq[3] == av.Level.EVADE and seq[4] == av.Level.EVADE      # 1.55~1.9 m: 진입 문턱(1.6)을 넘나들어도 EVADE 유지(해제 문턱은 +0.6 m)
    drops = [a - b for a, b in zip(seq[:-1], seq[1:]) if a > b]
    assert max(drops) <= 1, f"한 번에 두 단계 이상 내려갔다: {seq}"
    assert seq[-1] == av.Level.CLEAR


def test_retreat_candidates_do_not_approach_the_enemy_and_prefer_lower_risk():
    cfg, g = _grid()
    en = _tt(3.2, 2.5)
    pose = Pose2D(2.0, 2.5, 0.0)                       # 적이 정면 1.2 m
    ctx = _ctx(cfg, g, [en], pose=pose)
    ctl = av.AvoidController(cfg)
    ctl.assess(ctx)
    assert ctl.main is not None
    pick = ctl.pick_retreat(ctx, ctl.main[1])
    assert pick is not None
    goal, path = pick
    assert math.hypot(goal[0] - en.x, goal[1] - en.y) > math.hypot(pose.x - en.x, pose.y - en.y)   # 적에게서 멀어진다
    assert all(not body_hit(ctx.field, g, x, y, 0.0) for x, y in path[1:])


def test_recover_hint_turns_away_and_forbids_reverse_toward_rear_enemy():
    cfg, g = _grid()
    ctl = av.AvoidController(cfg)
    pose = Pose2D(2.0, 2.5, 0.0)
    left = _tt(2.0, 3.5)                               # 왼쪽(+y) 1 m
    tp, back = ctl.recover_hint(pose, [left], 0.0, 0.0, 0.0)
    assert tp == -1.0 and back is True                 # 적이 왼쪽이면 오른쪽으로
    rear = _tt(1.0, 2.5)                               # 뒤 1 m
    tp2, back2 = ctl.recover_hint(pose, [rear], 0.0, 0.0, 0.0)
    assert back2 is False                              # 적이 뒤에 있으면 후진 금지
    assert ctl.recover_hint(pose, [], 0.0, 0.0, 0.0) == (None, True)   # 위협 없으면 기존 동작


def test_avoid_does_not_take_ownership_when_m5_commit_or_goal_evade():
    cfg, g = _grid()
    ctl = av.AvoidController(cfg)
    en = _tt(2.9, 2.5)
    for slack, in_goal in ((-5.0, True), (30.0, True)):
        ctl2 = av.AvoidController(cfg)
        ctx = _ctx(cfg, g, [en], pose=Pose2D(2.0, 2.5, 0.0), slack=slack, in_goal=in_goal)
        ctl2.assess(ctx)
        assert ctl2.level >= av.Level.EVADE
        assert ctl2.step(ctx) is None and ctl2.stats["commit_skips"] >= 1     # 거점 접근 중엔 정지·후퇴로 임무를 막지 않는다
    ctx = _ctx(cfg, g, [en], pose=Pose2D(2.0, 2.5, 0.0), slack=100.0, in_goal=False)
    ctl.assess(ctx)
    assert ctl.step(ctx) is not None                                          # 평시(탐색 중)엔 소유권을 가져간다


def test_stale_observation_never_takes_ownership_but_still_raises_awareness():
    cfg, g = _grid()
    stale = _tt(2.9, 2.5, t=55.0)                       # now=60 → 5 s 전 관측(fresh_s 1.6 초과, hold_track_s+1 초과도 근접)
    stale.t_last = 59.0 - cfg.avoid.fresh_s             # 나이 = 1 + fresh_s (> fresh_s) 이면서 유지 시간(3 s) 안
    ctl = av.AvoidController(cfg)
    ctx = _ctx(cfg, g, [stale], pose=Pose2D(2.0, 2.5, 0.0), now=60.0)
    ctl.assess(ctx)
    assert ctl.level == av.Level.AWARE                  # 가까운 오래된 위치는 경로 비용에만 반영
    assert ctl.step(ctx) is None
    fresh = _tt(2.9, 2.5, t=60.0)
    ctl2 = av.AvoidController(cfg)
    ctx2 = _ctx(cfg, g, [fresh], pose=Pose2D(2.0, 2.5, 0.0), now=60.0)
    ctl2.assess(ctx2)
    assert ctl2.level >= av.Level.EVADE


def test_budget_kappa_shrinks_when_time_is_short():
    cfg = Config()
    ctl = av.AvoidController(cfg)
    k_early, s_early = ctl.budget(60.0, 3.0)
    k_late, s_late = ctl.budget(285.0, 3.0)
    assert k_early == 1.0 and s_early > 0
    assert s_late < 0 and k_late == 0.0


def test_carrot_point_is_ahead_on_the_path_and_never_past_its_end():
    path = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (2.0, 1.0)]
    assert av.carrot_point(path, Pose2D(0.1, 0.0, 0.0), 1.0) == pytest.approx((1.1, 0.0), abs=1e-6)
    assert av.carrot_point(path, Pose2D(2.0, 0.9, 0.0), 1.0) == (2.0, 1.0)


# ====================================================================== LiDAR 동적 군집
def _scan_with(cfg, pose, objs):
    """경기장 외벽 + 원기둥(반경 r) 들을 360 빔으로 쏜 합성 스캔."""
    n = 360
    ang = -math.pi + np.arange(n) * (2 * math.pi / n)
    W, H = cfg.arena.W, cfg.arena.H
    ranges = np.full(n, np.inf)
    for i, a in enumerate(ang + pose.th):
        dx, dy = math.cos(a), math.sin(a)
        best = 8.0
        for t in ([(0 - pose.x) / dx] if dx < -1e-9 else []) + ([(W - pose.x) / dx] if dx > 1e-9 else []) + \
                ([(0 - pose.y) / dy] if dy < -1e-9 else []) + ([(H - pose.y) / dy] if dy > 1e-9 else []):
            if 0 < t < best:
                best = t
        for (ox, oy, r) in objs:
            fx, fy = ox - pose.x, oy - pose.y
            b = fx * dx + fy * dy
            disc = b * b - (fx * fx + fy * fy - r * r)
            if disc >= 0 and b > 0:
                t = b - math.sqrt(disc)
                if 0 < t < best:
                    best = t
        ranges[i] = best
    return Scan(-math.pi, 2 * math.pi / n, ranges, 0.0, 8.0)


def test_lidar_blobs_detect_moving_small_object_but_ignore_static_box_and_walls():
    cfg, g = _grid()
    det = BlobDetector(cfg)
    pose = Pose2D(1.0, 2.5, 0.0)
    now = 0.0
    seen = []
    # 로봇이 지나온 자리를 자유로 만든다(자유 확인 칸이 있어야 '새 반사'를 알아본다) — 격자를 직접 갱신
    for k in range(60):                                   # 6 s 동안 정지한 관측으로 격자와 H 를 안정화 (정적 박스는 처음부터 있었다)
        sc = _scan_with(cfg, pose, [(3.0, 1.0, 0.2)])    # 정적 기둥(박스 대용)
        g.update(sc, pose)
        det.update(sc, pose, g, now, 0.0)
        now += 0.1
    assert det.blobs(now) == []                           # 정적 구조·벽은 군집이 아니다
    # 같은 방에서 움직이는 UGV(반경 0.13)가 y=3.5 를 0.35 m/s 로 +x 방향
    x = 2.0
    for k in range(60):
        sc = _scan_with(cfg, pose, [(3.0, 1.0, 0.2), (x, 3.6, 0.13)])
        g.update(sc, pose, )
        seen = det.update(sc, pose, g, now, 0.0)
        x += 0.35 * 0.1
        now += 0.1
    assert seen, "움직이는 작은 물체를 LiDAR 군집으로 못 잡았다"
    b = min(seen, key=lambda q: math.hypot(q.x - x, q.y - 3.6))
    assert math.hypot(b.x - x, b.y - 3.6) < 0.35 and b.speed_est > 0.15
    # 회전이 빠르면(자세 오차) 새 반사를 판정하지 않는다
    det2 = BlobDetector(cfg)
    det2.update(_scan_with(cfg, pose, [(2.0, 3.6, 0.13)]), pose, g, 100.0, 2.0)
    assert det2.stats["skipped_rot"] == 1


def test_lidar_new_returns_without_prior_free_space_are_not_dynamic():
    cfg, grid = _grid()
    detector = BlobDetector(cfg)
    pose = Pose2D(1.0, 2.5, 0.0)
    # No historical free-space map is available in this newly observed area.
    # Apparent movement alone must not turn newly exposed structure edges into
    # a moving-object track. Camera/UAV detection is a separate input.
    for k in range(90):
        scan = _scan_with(cfg, pose, [(2.0 + .035 * k, 3.6, .13)])
        assert detector.update(scan, pose, grid, k * .1, 0.0) == []
    assert detector.stats["candidates"] == 0


def test_lidar_static_square_with_small_pose_jitter_is_not_a_moving_blob():
    from maicon26.sim.world import _ray_aabb

    cfg, grid = _grid()
    detector = BlobDetector(cfg)
    true_pose = Pose2D(1.0, 2.5, 0.0)
    scan = _scan_with(cfg, true_pose, [])
    for i, angle in enumerate(scan.angles):
        hit = _ray_aabb(true_pose.x, true_pose.y, math.cos(angle), math.sin(angle),
                        (2.8, .8, 3.2, 1.2), scan.range_max)
        if hit is not None:
            scan.ranges[i] = min(scan.ranges[i], hit[0])
    for k in range(120):
        estimated = Pose2D(true_pose.x + .025 * math.sin(k * .31),
                           true_pose.y + .025 * math.cos(k * .43), .008 * math.sin(k * .2))
        grid.update(scan, estimated)
        assert detector.update(scan, estimated, grid, k * .1, 0.0) == []


# ====================================================================== 시뮬 시나리오
def _hits_and_m5(strategy_kw, seed, scen, avoid_on, max_t=300.0):
    def mut(c):
        c.avoid.enable = avoid_on
    r = run_trial(seed, "online", cfg=make_cfg("online", mut=mut), setup=build(scen, **strategy_kw), max_t=max_t)
    return r


def test_scenario_front_blocker_avoidance_reduces_hits_and_keeps_m5():
    base = _hits_and_m5({}, 3, "S1b_front_blocker", False)
    on = _hits_and_m5({}, 3, "S1b_front_blocker", True)
    assert base["hits_taken"] >= 2, "기준선에서 피격이 있어야 비교가 의미 있다(시나리오가 상대 사정거리 안에 들어간다)"
    assert on["hits_taken"] <= base["hits_taken"]
    assert on["m5"] is not None and on["m5"] <= base["m5"] + 20.0           # M5 도착 시각 악화 한도(시뮬 기준 +20 s)


def test_scenario_no_hostile_has_few_false_alarms_and_reaches_m5():
    r = run_trial(2, "online", cfg=make_cfg("online"), max_t=300.0)
    a = r["agent"]
    assert r["m5"] is not None
    assert a.avoid.stats["owner_s"] < 15.0, "적이 없는데 회피가 구동을 오래 쥐었다(오탐)"


def test_scenario_stop_during_avoid_zeroes_output_and_resets_state():
    m = build_match(seed=3, cfg=make_cfg("online"), teams=["B2"])
    hs = build("S1b_front_blocker", t_trig=20.0)(m)
    a = m.agents[0]
    w = m.world
    for _ in range(int(120.0 / 0.05)):
        for h in hs:
            h.step(m)
        a.tick()
        if a.avoid.cmd is not None and a.stats["avoid_ticks"] > 5:
            break
        w.step(0.05)
    assert a.avoid.cmd is not None, "회피가 시작되지 않았다 — 시나리오 확인 필요"
    w.end()                                                  # 서버 STOP
    for _ in range(3):
        a.tick()
        w.step(0.05)
    assert a.mode.value in ("HALT", "REPORT", "DONE")
    assert a.avoid.level == av.Level.CLEAR and a.avoid.cmd is None   # 회피 상태 폐기
    assert abs(w.ugvs[0].cmd_v) < 1e-9 and abs(w.ugvs[0].cmd_w) < 1e-9


def test_scenario_world_noise_knobs_exist_and_default_to_previous_values():
    m = build_match(seed=1, cfg=make_cfg("online"), teams=["B2"])
    w = m.world
    assert (w.ugv_miss_base, w.ugv_miss_slope, w.team_flip, w.uav_sigma, w.uav_latency, w.uav_enabled) == (0.05, 0.04, 0.04, 0.04, 0.4, True)
    w.uav_enabled = False
    assert m.servers[0].get_uav() is None                        # 실기 get_uav 가 None 인 상황을 흉내 낼 수 있다

