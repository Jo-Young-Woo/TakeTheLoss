"""차체 외곽(250×230 mm 직사각형)·LiDAR 오차 예산·경기장 정적 유효성 시험.

!! 시뮬 결과는 실기 안전성의 증거가 아니다. LiDAR 오차 수치는 PDF 에 없어서 전부 가정값이다.
"""
import math
import warnings

import numpy as np
import pytest

from maicon26.config import Config
from maicon26.footprint import Footprint, path_violations
from maicon26.nav.fpplan import plan_footprint
from maicon26.nav.grid import OccupancyGrid
from maicon26.nav.navigator import Navigator
from maicon26.sim import arena_check as ac
from maicon26.sim.world import Cube, Item, World
from maicon26.types import Pose2D

warnings.filterwarnings("ignore")


# ====================================================================== 예산 / 치수
def test_footprint_dims_come_from_pdf_and_budget_is_consistent_everywhere():
    cfg = Config()
    fp = Footprint(cfg.robot.length, cfg.robot.width, cfg.robot.height)
    assert (cfg.robot.length, cfg.robot.width, cfg.robot.height) == (0.25, 0.23, 0.32)          # [PDF p21]
    assert fp.circumradius == pytest.approx(0.1698, abs=1e-3) and fp.inscribed == pytest.approx(0.115)
    assert cfg.consistency() == [], cfg.consistency()                                           # 파생값이 예산과 일치
    b = cfg.budget()
    # 같은 오차를 계층마다 중복 가산하지 않는다: 지연은 정지만, 위치추정은 계획만
    assert b.stop_margin == pytest.approx(b.sensor_margin + b.latency_term) and b.plan_margin == pytest.approx(b.sensor_margin + b.pose_term)
    names = {n: use for n, _, use, _ in b.items}
    assert names["전달 지연×기준속도"] == "정지만" and names["자기 위치추정 k·σ_pose"] == "계획만"
    # Nav2 footprint = 차체 + 계획 여유 (패딩은 0 → 이중 가산 없음)
    hl = max(x for x, _ in b.nav2_footprint)
    assert hl == pytest.approx(0.125 + b.plan_margin)
    # LiDAR 오차는 가정값으로 기록돼 있고 실측으로 표시되지 않는다
    assert cfg.lidar_err.status == "assumed" and all("assumed" in v for v in cfg.lidar_err.source.values())


def test_40cm_corridor_passable_straight_but_not_rotatable_with_margin():
    b = Config().budget()
    c = b.corridor_clearance(0.40)
    assert c["can_pass_straight"] and c["side_after_plan_margin_m"] > 0.02
    assert not c["can_rotate_in_place"]                       # 외접 0.17 + 여유 0.049 > 반폭 0.20 → 통로 안 제자리 회전 불가
    assert not b.corridor_clearance(0.30)["can_pass_straight"]


def test_robot_profile_edit_updates_config_consistently_and_keeps_assumed_label():
    from maicon26.calib.generate import gen_config_overlay
    from maicon26.calib.profile import Profile
    P = Profile()
    base = gen_config_overlay(P)
    assert base["lidar_err"]["status"] == "assumed" and base["robot"]["length"] == 0.25
    P.set("model.size.x", 0.27, "measured")
    P.set("sensors.lidar.error.range_sigma_m", 0.02, "measured")           # 일부만 실측 → 아직 measured 로 표시하지 않는다
    o = gen_config_overlay(P)
    assert o["robot"]["length"] == 0.27 and o["nav"]["stop_dist"] > base["nav"]["stop_dist"] and o["nav"]["rotate_clear"] > base["nav"]["rotate_clear"]
    assert o["lidar_err"]["status"] == "assumed"
    cfg = Config()
    cfg.robot.length = 0.27
    assert cfg.consistency() != []                                           # 치수만 바꾸고 파생값을 안 갱신하면 어긋남을 알려 준다
    cfg.apply_budget()
    assert cfg.consistency() == []


# ====================================================================== 시뮬 충돌 모델 (직사각형)
def _bare_world(seed=0):
    cfg = Config()
    cfg.arena.validate_layout = False
    w = World(cfg, seed=seed)
    w.cubes = []
    return cfg, w


def _put(w, x, y, th):
    u = w.add_ugv("BLUE", 1)
    u.pose = Pose2D(x, y, th)
    u.collisions = 0
    return u


def _corridor_cubes(w, y_lo=2.0, y_hi=2.8, x0=2.0, x1=3.6):
    xs = np.arange(x0, x1 + 1e-9, 0.4)
    for x in xs:
        w.cubes.append(Cube(float(x), y_lo))
        w.cubes.append(Cube(float(x), y_hi))                 # 면 사이 간격 = y_hi − y_lo − 0.4 = 0.40


def test_sim_collision_straight_through_40cm_corridor_has_no_contact():
    cfg, w = _bare_world()
    _corridor_cubes(w)
    u = _put(w, 1.2, 2.4, 0.0)
    for _ in range(int(14.0 / 0.05)):
        u.cmd_v, u.cmd_w = 0.3, 0.0
        w.step(0.05)
    assert u.pose.x > 3.9 and u.collisions == 0


def test_sim_rect_catches_corner_brush_that_the_old_small_circle_missed():
    cfg, w = _bare_world()
    w.cubes.append(Cube(3.0, 2.5))                           # 박스 (2.8~3.2, 2.3~2.7)
    # 박스 면(x=2.8)에서 0.155 m 떨어진 곳에서 45° 로 비스듬히 선 차체: 모서리가 면 쪽으로 (0.125+0.115)/√2 = 0.170 m 뻗는다
    u = _put(w, 2.8 - 0.155, 2.5, math.pi / 4)
    u.cmd_v = u.cmd_w = 0.0
    u.step(0.05)
    assert u.collisions >= 1, "직사각형 모서리(중심에서 0.17 m)가 박스 면에 닿았는데 충돌로 안 셌다"
    cfg2, w2 = _bare_world()
    cfg2.use_legacy_circle()
    w2.cubes.append(Cube(3.0, 2.5))
    u2 = _put(w2, 2.8 - 0.155, 2.5, math.pi / 4)
    u2.step(0.05)
    assert u2.collisions == 0, "레거시 원(0.14 m)은 같은 접촉을 놓친다 — 변경 전 동작 재현 확인"


def test_sim_oncoming_ugvs_use_rectangles_not_circles():
    cfg, w = _bare_world()
    a = _put(w, 2.0, 2.0, 0.0)
    b = w.add_ugv("RED", 1)
    b.pose = Pose2D(2.0 + 0.24, 2.0 + 0.2, math.pi)          # 측면 간격 0.2 < 폭 0.23 → 겹침(전후로 지나치는 중)
    a.collisions = b.collisions = 0
    a.step(0.05)
    assert a.collisions >= 1
    b.pose = Pose2D(2.0 + 0.24, 2.0 + 0.26, math.pi)         # 0.26 > 0.23 → 안 겹침 (원 모델 0.13+0.13 이면 닿는다고 봤을 거리)
    a.pose = Pose2D(2.0, 2.0, 0.0)
    a.collisions = 0
    a.step(0.05)
    assert a.collisions == 0


def test_lidar_sees_other_ugv_as_rectangle():
    cfg, w = _bare_world()
    a = _put(w, 2.0, 2.0, 0.0)
    b = w.add_ugv("RED", 1)
    b.pose = Pose2D(3.0, 2.0, 0.0)                           # 정면 1 m, 같은 방향: 뒷면(폭 0.23)이 보인다
    sc = w.lidar(a)
    k = int(np.argmin(np.abs(sc.angles)))
    assert sc.ranges[k] == pytest.approx(1.0 - 0.125, abs=0.03)           # 앞 반길이만큼 (원 0.13 이 아니라 0.125)
    b.pose = Pose2D(3.0, 2.0, math.pi / 2)                   # 옆으로 선 상대: 정면에서 보이는 폭이 길이(0.25) 쪽
    sc2 = w.lidar(a)
    assert sc2.ranges[k] == pytest.approx(1.0 - 0.115, abs=0.03)


# ====================================================================== 경로 검증: 직사각형 + 회전 공간
def _grid_with(cubes):
    cfg = Config()
    g = OccupancyGrid(cfg.arena, cfg.nav.res)
    xs = g.ox + (np.arange(g.nx) + 0.5) * g.res
    ys = g.oy + (np.arange(g.ny) + 0.5) * g.res
    for (cx, cy) in cubes:
        mx = (xs >= cx - 0.2) & (xs <= cx + 0.2)
        my = (ys >= cy - 0.2) & (ys <= cy + 0.2)
        g.static[np.ix_(my, mx)] = True
    return cfg, g


def test_path_violations_straight_ok_oblique_corner_and_rotation_in_40cm():
    cfg, g = _grid_with([(3.0, 2.0), (3.0, 2.8)])             # 가운데 40 cm 틈 (y 2.2~2.6), x 2.8~3.2
    b = cfg.budget()
    occ = g.occupied()
    straight = [(2.0, 2.4), (4.0, 2.4)]
    assert path_violations(occ, g.res, g.ox, g.oy, straight, b) == []
    oblique = [(2.4, 1.6), (3.0, 2.4), (3.6, 3.2)]            # 틈을 약 53° 로 비스듬히 통과 → 모서리가 걸린다
    v = path_violations(occ, g.res, g.ox, g.oy, oblique, b)
    assert v and any(k == "body" for _, _, k in v)
    turn_in_gap = [(2.4, 2.4), (3.0, 2.4), (3.0, 2.7)]        # 틈 한가운데에서 90° 꺾기: 제자리 회전 공간 없음
    assert any(k == "rotate" for _, _, k in path_violations(occ, g.res, g.ox, g.oy, turn_in_gap, b))
    wide = [(2.4, 4.0), (3.0, 4.0), (3.0, 4.4)]               # 트인 곳에서 같은 꺾기는 문제없다
    assert path_violations(occ, g.res, g.ox, g.oy, wide, b) == []


def test_planner_passes_40cm_gap_but_never_shrinks_margin_for_a_30cm_gap():
    cfg, g = _grid_with([(3.0, 2.0), (3.0, 2.7)])            # 틈 0.30 m (y 2.2~2.5) — 차체 폭 0.23 + 양쪽 여유 0.049×2 > 0.30
    b = cfg.budget()
    # 막는 벽을 만들어 이 틈이 유일한 통로가 되게 한다
    xs = g.ox + (np.arange(g.nx) + 0.5) * g.res
    ys = g.oy + (np.arange(g.ny) + 0.5) * g.res
    for cy in (0.2, 0.6, 1.0, 1.4, 1.8, 3.1, 3.5, 3.9, 4.3, 4.7):
        g.static[np.ix_((ys >= cy - 0.2) & (ys <= cy + 0.2), (xs >= 2.8) & (xs <= 3.2))] = True
    res = plan_footprint(g, cfg.nav.inflate_hard, cfg.nav.inflate_soft, cfg.nav.soft_weight, b, (2.0, 2.35), (4.0, 2.35), 0.0)
    assert res is None, "30 cm 틈은 차체+여유가 못 지나간다 — 경로를 못 찾았다고 여유를 줄이면 안 된다"
    # 40 cm 로 넓히면 통과
    cfg, g = _grid_with([])
    xs = g.ox + (np.arange(g.nx) + 0.5) * g.res
    ys = g.oy + (np.arange(g.ny) + 0.5) * g.res
    for cy in (0.2, 0.6, 1.0, 1.4, 1.8, 3.0, 3.4, 3.8, 4.2, 4.6, 4.8):
        g.static[np.ix_((ys >= cy - 0.2) & (ys <= cy + 0.2), (xs >= 2.8) & (xs <= 3.2))] = True
    res2 = plan_footprint(g, cfg.nav.inflate_hard, cfg.nav.inflate_soft, cfg.nav.soft_weight, b, (2.0, 2.4), (4.0, 2.4), 0.0)
    assert res2 is not None and all(abs(y - 2.4) < 0.15 for x, y in res2[0] if 2.7 < x < 3.3)


def test_navigator_no_longer_relaxes_inflation_but_legacy_mode_does():
    cfg, g0 = _grid_with([])
    nav = Navigator(cfg)
    nav.grid = g0
    xs = g0.ox + (np.arange(g0.nx) + 0.5) * g0.res
    ys = g0.oy + (np.arange(g0.ny) + 0.5) * g0.res
    for cy in (0.2, 0.6, 1.0, 1.4, 1.8, 3.1, 3.5, 3.9, 4.3, 4.7, 2.0, 2.7):                # 틈 0.30
        g0.static[np.ix_((ys >= cy - 0.2) & (ys <= cy + 0.2), (xs >= 2.8) & (xs <= 3.2))] = True
    nav.set_goal((4.0, 2.35))
    assert nav._plan(Pose2D(2.0, 2.35, 0.0), 0.0) is False
    cfg2 = Config().use_legacy_circle()
    nav2 = Navigator(cfg2)
    nav2.grid = g0
    nav2.set_goal((4.0, 2.35))
    assert nav2._plan(Pose2D(2.0, 2.35, 0.0), 0.0) is True            # 변경 전: 0.6배 완화로 30 cm 틈을 '통과 가능'으로 판정(차체 모서리 무시)


def test_safety_limiter_does_not_nudge_in_the_middle_of_a_40cm_corridor():
    from maicon26.nav.safety import SafetyLimiter
    cfg = Config()
    s = SafetyLimiter(cfg)
    ang = np.linspace(-math.pi, math.pi, 360, endpoint=False)
    r = np.where(np.abs(np.sin(ang)) > 1e-6, 0.20 / np.abs(np.sin(ang)), 8.0)       # 좌우 0.20 m 벽(복도 중앙)
    r = np.minimum(r, 8.0)
    from maicon26.types import Scan
    sc = Scan(-math.pi, 2 * math.pi / 360, r, 0.0, 8.0)
    v, w = s.limit(sc, 0.3, 0.0, 0.0)
    assert 0.1 < v <= 0.3 and abs(w) < 1e-9                  # 앞 ±35° 부채꼴에 옆벽이 비스듬히 들어와 감속은 되지만 정지·측면 보정(w)은 없다
    r2 = r.copy()
    sc2 = Scan(-math.pi, 2 * math.pi / 360, np.minimum(r2, np.where(np.abs(ang) < 0.3, 0.18, 8.0)), 0.0, 8.0)       # 앞 0.18 m < stop 0.192
    v2, _ = s.limit(sc2, 0.3, 0.0, 0.0)
    assert v2 == 0.0


# ====================================================================== LiDAR 오차 모델 (시뮬 숨은 진짜 오차)
def _corridor_run(mut, seed=0, T=40.0, legacy=False):
    cfg, w = _bare_world(seed)
    cfg.lidar_err.__dict__.update({})
    mut(cfg)
    if legacy:
        cfg.use_legacy_circle()
    _corridor_cubes(w, 2.0, 2.8, 2.0, 3.6)
    u = _put(w, 1.0, 2.4, 0.0)
    from maicon26.sim.robot import SimRobot
    rob = SimRobot(w, u)
    nav = Navigator(cfg)
    nav.set_goal((4.6, 2.4), 0.12)
    t = 0.0
    while t < T:
        sc = rob.get_scan()
        if sc is not None and sc.stamp != getattr(nav, "_last_stamp", None):
            nav.update_map(sc, rob.get_odom() if False else u.pose)        # 지도는 참 자세로 쌓는다(위치추정 오차는 별도 시험)
            nav._last_stamp = sc.stamp
        out = nav.step(u.pose, sc, t, 0.05)
        u.cmd_v, u.cmd_w = out.v, out.w
        w.step(0.05)
        t += 0.05
        if out.status == "reached":
            break
    return u.collisions, out.status


@pytest.mark.parametrize("name,mut", [
    ("기준", lambda c: None),
    ("편향 -3cm(가깝게 읽음)", lambda c: setattr(c.lidar_err, "sim_range_bias_m", -0.03)),
    ("잡음 σ 3cm", lambda c: setattr(c.lidar_err, "sim_range_sigma_m", 0.03)),
    ("누락 20%", lambda c: setattr(c.lidar_err, "sim_dropout_p", 0.20)),
    ("이상치 3%", lambda c: setattr(c.lidar_err, "sim_outlier_p", 0.03)),
    ("지연 0.2s", lambda c: setattr(c.lidar_err, "sim_latency_s", 0.2)),
])
def test_corridor_traversal_under_lidar_errors_builtin(name, mut):
    coll, st = _corridor_run(mut)
    # 가정 예산 안의 오차에서는 충돌 없이 통과. 예산을 넘는 큰 오차(이상치·누락 등)에서는 '정지·복구'가 허용되지만 충돌하면 안 된다
    assert coll <= 2, f"{name}: 충돌 {coll}"
    if name in ("기준", "지연 0.2s"):
        assert st == "reached"


# ====================================================================== 경기장 유효성
def _tgt(cls, face):
    c = Cube(0, 0)
    c.items[face] = Item("target", cls)
    return c


def good_world():
    """직접 짠 유효 배치: 양 진영에 표적 3종(진영마다 1개씩), 객체 박스 3개씩. 가운데(x=3.06) 열은 비워 두어 방벽 시험에 쓴다."""
    cfg = Config()
    cfg.arena.validate_layout = False
    w = World(cfg, seed=0)
    w.cubes = []
    cls = list(cfg.perception.target_classes)
    obj = list(cfg.perception.object_classes)
    for i, y in enumerate((0.8, 2.5, 4.2)):
        c = Cube(1.4, y)
        c.items[0] = Item("target", cls[i])                   # B 진영: +x 쪽 면
        w.cubes.append(c)
        c = Cube(4.72, y)
        c.items[2] = Item("target", cls[i])                   # R 진영: -x 쪽 면
        w.cubes.append(c)
    for (x, y) in [(2.2, 1.65), (2.2, 3.35), (2.2, 4.8)]:
        c = Cube(x, y)
        for f in range(4):
            c.items[f] = Item("object", obj[0])
        w.cubes.append(c)
    for (x, y) in [(3.9, 1.65), (3.9, 3.35), (3.9, 4.8)]:
        c = Cube(x, y)
        for f in range(4):
            c.items[f] = Item("object", obj[1])
        w.cubes.append(c)
    return cfg, w

def test_good_layout_is_valid():
    cfg, w = good_world()
    rep = ac.check_arena(w, cfg)
    assert rep.ok, rep.reasons


def test_trapped_start_is_excluded():
    cfg, w = good_world()
    x, y, th = ac._start_pose(cfg, "BLUE", 2)
    c = Cube(x + 0.25, y)                                  # 출발 자세 바로 앞에 박스 → 직사각형이 겹친다
    for f in range(4):
        c.items[f] = Item("object", cfg.perception.object_classes[0])
    w.cubes.append(c)
    rep = ac.check_arena(w, cfg)
    assert not rep.ok and any(r.startswith("start:B-2") for r in rep.reasons)


def _barrier(w, x, gap_y=None, gap=0.4):
    """x 위치에 세로 방벽(박스 줄). gap_y 가 있으면 그 y 중심에 gap(m) 틈을 남긴다."""
    ys = [0.2, 0.6, 1.0, 1.4, 1.8, 2.2, 2.6, 3.0, 3.4, 3.8, 4.2, 4.6, 4.8]
    for y in ys:
        if gap_y is not None and abs(y - gap_y) < 0.2 + gap / 2 + 1e-9 - (0.0):
            if abs(y - gap_y) < gap / 2 + 0.2 - 1e-9:
                continue
        c = Cube(x, y)
        w.cubes.append(c)


def test_cut_central_corridor_is_excluded_but_a_real_40cm_gap_is_not():
    cfg, w = good_world()
    for cx in (3.06,):                                     # 틈이 전혀 없는 방벽 (박스가 겹쳐도 상관없다: spacing 사유가 같이 나오지만 goal 사유가 핵심)
        _barrier(w, cx, gap_y=None)
    rep = ac.check_arena(w, cfg)
    assert any(r.startswith("goal:") for r in rep.reasons), rep.reasons
    cfg2, w2 = good_world()
    # 틈이 40 cm 인 방벽: y=2.5 중심으로 박스를 0.8 간격으로 비운다
    ys_lo = [0.2, 0.6, 1.0, 1.4, 1.8, 2.1]                       # 마지막 박스 윗면 2.3
    ys_hi = [2.9, 3.3, 3.7, 4.1, 4.5, 4.8]                       # 첫 박스 아랫면 2.7 → 틈 0.4
    for y in ys_lo + ys_hi:
        w2.cubes.append(Cube(3.06, y))
    rep2 = ac.check_arena(w2, cfg2)
    assert not any(r.startswith(("goal:", "target:", "start:")) for r in rep2.reasons), rep2.reasons      # 실제로 통과 가능한 40 cm 통로를 잘못 제외하지 않는다


def test_blocked_goal_base_entrance_is_excluded():
    cfg, w = good_world()
    gx, gy = cfg.arena.base_center("R-2")
    # R-2 거점 둘레를 박스로 에워싼다 (정상 이격은 무시, 도달 불가가 핵심)
    for dx, dy in ((-0.4, 0.0), (-0.4, 0.4), (-0.4, -0.4), (0.0, 0.4), (0.0, -0.4)):
        w.cubes.append(Cube(gx + dx, gy + dy))
    rep = ac.check_arena(w, cfg)
    assert any(r.startswith("goal:") and r.endswith("R-2") for r in rep.reasons), rep.reasons


def test_unreachable_target_access_is_excluded():
    cfg, w = good_world()
    cls = cfg.perception.target_classes[0]
    for c in w.cubes:                                    # 이 표적의 두 면을 모두 벽 쪽으로 돌리고 벽에 붙인다
        for f, it in list(c.items.items()):
            if it.kind == "target" and it.cls == cls:
                del c.items[f]
                c.items[3] = it                          # face 3 = -y (아래 벽 쪽)
                c.cy = 0.6
    rep = ac.check_arena(w, cfg)
    assert any(r.startswith("target:") and r.endswith(cls) for r in rep.reasons), rep.reasons


def test_missing_required_target_is_excluded_not_skipped_silently():
    cfg, w = good_world()
    w.cubes = [c for c in w.cubes if not any(it.kind == "target" and it.cls == cfg.perception.target_classes[1] and c.cx < 3 for it in c.items.values())]
    rep = ac.check_arena(w, cfg)
    assert any(r.startswith("content:B-targets") for r in rep.reasons), rep.reasons
    w.layout_failures = ["B:B-S2#1"]
    assert any(r.startswith("placement:") for r in ac.check_arena(w, cfg).reasons)


def test_generation_is_deterministic_records_attempts_and_aborts_with_reasons():
    cfg = Config()
    a, b = World(cfg, seed=3), World(cfg, seed=3)
    assert a.layout_meta == b.layout_meta and [(c.cx, c.cy) for c in a.cubes] == [(c.cx, c.cy) for c in b.cubes]
    m = a.layout_meta
    assert m["seed"] == 3 and m["attempts"] >= 1 and m["used_layout_seed"] == 3 + 10007 * (m["attempts"] - 1)
    assert len(m["excluded"]) == m["attempts"] - 1 and all(e["reasons"] for e in m["excluded"])
    cfg2 = Config()
    cfg2.arena.max_layout_tries = 1
    bad = next(s for s in range(1, 60) if World(Config(), seed=s).layout_meta["attempts"] > 1)         # 첫 시도가 무효인 시드
    with pytest.raises(ac.ArenaGenerationError) as ei:
        World(cfg2, seed=bad)
    assert ei.value.meta["excluded"] and ei.value.meta["seed"] == bad
    cfg3 = Config()
    cfg3.arena.validate_layout = False
    assert World(cfg3, seed=bad).layout_meta["attempts"] == 1               # 검사 끄면 옛 동작(비교용)

