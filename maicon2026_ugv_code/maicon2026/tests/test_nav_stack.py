"""항법 스택(SLAM + Nav2 경계) 시험. ROS 없이 돌아간다.
- Nav2Stack 의 좌표 변환·상태 기계·속도 중재를 스텁 클라이언트로 검증
- 구조물이 바뀔 때 점유격자·재계획이 따라오는지 (내장 백엔드)
- nav2-fake 백엔드(시뮬)로 RED/BLUE 전체 흐름(map 프레임이 경기장과 180° 어긋난 경우 포함)
"""
import math
from typing import Optional, Tuple

import numpy as np
import pytest

from maicon26.config import Config
from maicon26.mission.explore import CubeFinder
from maicon26.nav.local_stack import LocalStack
from maicon26.nav.nav2_stack import Anchor, MapData, Nav2Client, Nav2Stack
from maicon26.nav.navigator import Navigator
from maicon26.sim.dynamic import shuffle_structures
from maicon26.sim.run import build_match, run, score
from maicon26.sim.world import Cube, World
from maicon26.types import Pose2D, Scan


# ====================================================================== 스텁 클라이언트
class StubClient(Nav2Client):
    def __init__(self, m0=(0.0, 0.0, 0.0)):
        self.m = m0
        self.map: Optional[MapData] = None
        self.cmd: Optional[Tuple[float, float]] = (0.2, 0.1)
        self.state = "none"
        self.rec = False
        self.sent = []
        self.cancels = 0
        self.walls = None

    def lookup_map_to_base(self):
        return self.m

    def latest_map(self):
        return self.map

    def latest_cmd(self, now):
        return self.cmd

    def send_goal(self, x, y, yaw):
        self.sent.append((x, y, yaw))
        self.state = "active"

    def cancel(self):
        self.cancels += 1
        self.state = "none"

    def goal_state(self):
        return self.state

    def recovering(self):
        return self.rec

    def set_arena_walls(self, pts):
        self.walls = pts


def _free_scan():
    n = 360
    return Scan(angle_min=-math.pi, angle_inc=2 * math.pi / n, ranges=np.full(n, np.inf), stamp=1.0, range_max=8.0)


def _stack_with_anchor(arena_pose: Pose2D, m0):
    cfg = Config()
    cl = StubClient(m0)
    st = Nav2Stack(cfg, cl)
    st.anchor = Anchor.from_pair(arena_pose, m0)
    st.pose = arena_pose.copy()
    return st, cl


# ====================================================================== 좌표
def test_anchor_roundtrip():
    for arena, m0 in [(Pose2D(5.57, 2.5, math.pi), (0.0, 0.0, 0.0)),
                      (Pose2D(0.55, 4.16, 0.03), (0.1, -0.2, 0.4)),
                      (Pose2D(5.0, 1.0, -2.0), (1.0, 2.0, -3.0))]:
        a = Anchor.from_pair(arena, m0)
        x, y, th = a.to_arena(*m0)
        assert (x, y) == pytest.approx((arena.x, arena.y), abs=1e-9)
        assert math.cos(th - arena.th) == pytest.approx(1.0, abs=1e-9)
        bx, by, bth = a.to_map(arena.x, arena.y, arena.th)
        assert (bx, by) == pytest.approx(m0[:2], abs=1e-9)
        pts = np.array([[1.0, 2.0], [3.0, 0.5]])
        back = a.points_to_map(pts)
        for p, q in zip(pts, back):
            assert a.to_arena(q[0], q[1], 0.0)[:2] == pytest.approx(tuple(p), abs=1e-9)


def test_goal_sent_in_map_frame_and_walls_published():
    st, cl = _stack_with_anchor(Pose2D(5.57, 2.5, math.pi), (0.0, 0.0, 0.0))   # RED 시작: map 은 경기장과 180° 어긋남
    st.set_goal((4.0, 2.5), tol=0.1)
    out = st.step(_free_scan(), 1.0, 0.05)
    assert out.status == "moving" and len(cl.sent) == 1
    # 경기장 (4.0, 2.5) 는 시작(5.57, 2.5)의 앞쪽(-x) 1.57 m → map 에서는 +x 1.57 m
    assert cl.sent[0][0] == pytest.approx(1.57, abs=0.02) and cl.sent[0][1] == pytest.approx(0.0, abs=0.02)
    # 같은 목표를 계속 호출해도 다시 보내지 않는다
    for k in range(5):
        st.step(_free_scan(), 1.1 + 0.05 * k, 0.05)
    assert len(cl.sent) == 1


def test_walls_published_on_start():
    cfg = Config()
    cl = StubClient((0.0, 0.0, 0.0))
    st = Nav2Stack(cfg, cl)
    n = 360
    ang = np.linspace(-math.pi, math.pi, n, endpoint=False)
    # 시작 자세 (0.55, 2.5, 0): 외벽까지 거리로 가짜 스캔 생성
    x, y = 0.55, 2.5
    r = np.full(n, 8.0)
    for i, a in enumerate(ang):
        dx, dy = math.cos(a), math.sin(a)
        ts = []
        if dx > 1e-9: ts.append((cfg.arena.W - x) / dx)
        if dx < -1e-9: ts.append((0 - x) / dx)
        if dy > 1e-9: ts.append((cfg.arena.H - y) / dy)
        if dy < -1e-9: ts.append((0 - y) / dy)
        r[i] = min(ts)
    p0 = st.start(Scan(-math.pi, 2 * math.pi / n, r, 0.0, 8.0), Pose2D(), (x, y))
    assert p0 is not None and math.hypot(p0.x - x, p0.y - y) < 0.03 and abs(p0.th) < 0.03
    assert cl.walls is not None and cl.walls.shape[1] == 2 and len(cl.walls) > 400


def test_start_waits_for_slam_tf():
    cl = StubClient()
    cl.m = None
    st = Nav2Stack(Config(), cl)
    assert st.start(_free_scan(), Pose2D(), (0.55, 2.5)) is None and not st.initialized


def test_project_map_finds_box_with_rotated_frame():
    """SLAM 지도(map 프레임, 180° 회전)의 박스가 경기장 좌표 격자의 올바른 위치에 나타나는가."""
    st, cl = _stack_with_anchor(Pose2D(5.57, 2.5, math.pi), (0.0, 0.0, 0.0))
    res, half = 0.05, 8.0
    n = int(2 * half / res)
    data = np.full((n, n), -1, dtype=np.int8)
    # 경기장의 박스 (3.0, 2.0) 크기 0.4 → map 좌표로 변환해 윤곽(테두리) 셀만 점유로
    box = (3.0, 2.0)
    for t in np.linspace(-0.2, 0.2, 41):
        for (bx, by) in [(box[0] + t, box[1] - 0.2), (box[0] + t, box[1] + 0.2),
                         (box[0] - 0.2, box[1] + t), (box[0] + 0.2, box[1] + t)]:
            mx, my, _ = st.anchor.to_map(bx, by)
            data[int((my + half) / res), int((mx + half) / res)] = 100
    data[data == -1] = 0                                   # 나머지는 자유로 관측됨
    # 박스 내부는 미관측이어야 하므로 중심부를 미지로
    mx, my, _ = st.anchor.to_map(*box)
    data[int((my + half) / res) - 3:int((my + half) / res) + 4, int((mx + half) / res) - 3:int((mx + half) / res) + 4] = -1
    st._project_map(MapData(data, res, -half, -half, 1.0))
    cubes = CubeFinder(st.geo).find(st.grid)
    assert len(cubes) == 1 and math.hypot(cubes[0][0] - box[0], cubes[0][1] - box[1]) < 0.08


# ====================================================================== 상태 기계 / 속도 중재
def test_stale_cmd_is_zero_and_speed_scaled():
    st, cl = _stack_with_anchor(Pose2D(0.7, 2.5, 0.0), (0.0, 0.0, 0.0))
    st.set_goal((3.0, 2.5), tol=0.1, speed_scale=1.0)
    st.step(_free_scan(), 1.0, 0.05)                      # 목표 전송
    out = st.step(_free_scan(), 1.05, 0.05)
    assert out.status == "moving" and out.v == pytest.approx(0.2) and out.w == pytest.approx(0.1)
    cl.cmd = None                                         # Nav2 속도가 끊김 → 정지
    out = st.step(_free_scan(), 1.1, 0.05)
    assert out.v == 0.0 and out.w == 0.0
    st.set_goal((3.0, 2.5), tol=0.1, speed_scale=1.2)
    cl.cmd = (0.3, 0.0)
    assert st.step(_free_scan(), 1.2, 0.05).v == pytest.approx(0.36)


def test_lidar_safety_overrides_nav2():
    """Nav2 가 전진을 명령해도 전방 장애물이 가까우면 Agent 쪽 안전 제한이 막는다."""
    st, cl = _stack_with_anchor(Pose2D(0.7, 2.5, 0.0), (0.0, 0.0, 0.0))
    st.set_goal((3.0, 2.5), tol=0.1)
    st.step(_free_scan(), 1.0, 0.05)
    n = 360
    r = np.full(n, np.inf)
    ang = np.linspace(-math.pi, math.pi, n, endpoint=False)
    r[np.abs(ang) < 0.3] = 0.10                           # 정면 10 cm
    sc = Scan(-math.pi, 2 * math.pi / n, r, 2.0, 8.0)
    cl.cmd = (0.35, 0.0)
    out = st.step(sc, 1.1, 0.05)
    assert out.v == 0.0                                   # 전진 금지(제자리 회전만)


def test_aborted_goal_retries_once_then_falls_back_to_builtin():
    st, cl = _stack_with_anchor(Pose2D(0.7, 2.5, 0.0), (0.0, 0.0, 0.0))
    st.set_goal((3.0, 2.5), tol=0.1)
    st.step(_free_scan(), 1.0, 0.05)
    cl.state = "aborted"
    assert st.step(_free_scan(), 1.1, 0.05).status == "recovering" and len(cl.sent) == 2
    cl.state = "aborted"
    out = st.step(_free_scan(), 1.2, 0.05)                # 두 번째 실패 → 내장 A* 가 같은 목표를 이어받는다
    assert st.fallbacks == 1 and cl.cancels >= 1
    assert out.status in ("moving", "recovering")
    st.set_goal((3.0, 3.0), tol=0.1)                      # 새 목표는 다시 Nav2 로 먼저 시도
    st.step(_free_scan(), 1.3, 0.05)
    assert len(cl.sent) == 3 and not st._fallback


def test_fallback_when_nav2_server_missing():
    st, cl = _stack_with_anchor(Pose2D(0.7, 2.5, 0.0), (0.0, 0.0, 0.0))
    cl.available = lambda: False
    st.set_goal((3.0, 2.5), tol=0.1)
    outs = [st.step(_free_scan(), 1.0 + 0.1 * k, 0.1) for k in range(25)]       # 2.4 s
    assert st.fallbacks == 1 and "서버" in st.fallback_reason
    assert any(o.v > 0 for o in outs[-5:])                # 내장 항법기가 실제로 전진 명령을 낸다
    assert cl.sent == [] or len(cl.sent) <= 1


def test_active_nav2_stall_falls_back_despite_repeated_goal_changes():
    st, cl = _stack_with_anchor(Pose2D(0.7, 2.5, 0.0), (0.0, 0.0, 0.0))
    cl.cmd = (0.0, 0.0)
    cl.rec = True
    for tick in range(18):
        if tick % 4 == 0:
            st.clear_goal()
            st.set_goal((3.0, 2.5 + (tick % 8) * 0.05), tol=0.1)
        st.step(_free_scan(), tick * 0.5, 0.5)
    assert st._fallback and st._stall_fallback
    assert st.fallbacks == 1
    assert "translational progress" in st.fallback_reason
    sent = len(cl.sent)
    st.clear_goal()
    st.set_goal((3.0, 3.0), tol=0.1)
    st.step(_free_scan(), 9.0, 0.5)
    assert len(cl.sent) == sent, "Switching viewpoints must not restart the stalled Nav2 controller"


def test_nav2_motion_and_intentional_observation_gaps_reset_stall_clock():
    st, cl = _stack_with_anchor(Pose2D(0.7, 2.5, 0.0), (0.0, 0.0, 0.0))
    cl.cmd = (0.0, 0.0)
    st.set_goal((4.0, 2.5), tol=0.1)
    for tick in range(14):
        st.pose.th += 0.1  # An ordinary short turn is allowed within the timeout.
        st.step(_free_scan(), tick * 0.5, 0.5)
    assert not st._fallback
    st.step(_free_scan(), 20.0, 0.5)  # Deliberate camera/mission pause.
    for tick in range(1, 17):
        if tick == 10:
            st.pose.x += 0.12
        st.step(_free_scan(), 20.0 + tick * 0.5, 0.5)
    assert not st._fallback


def test_nav2_success_outside_tolerance_resends_then_accepts():
    st, cl = _stack_with_anchor(Pose2D(0.7, 2.5, 0.0), (0.0, 0.0, 0.0))
    st.set_goal((1.2, 2.5), tol=0.1)
    st.step(_free_scan(), 1.0, 0.05)
    n0 = len(cl.sent)
    cl.state = "succeeded"                                # Nav2는 '도착'이라지만 아직 0.5 m 떨어져 있다
    st.step(_free_scan(), 1.1, 0.05)
    assert len(cl.sent) == n0 + 1
    cl.state = "succeeded"
    st.step(_free_scan(), 1.2, 0.05)
    cl.state = "succeeded"
    out = st.step(_free_scan(), 1.3, 0.05)                # 재전송 한도 소진 → 멀면 blocked
    assert out.status == "blocked"


def test_reached_cancels_nav2_goal():
    st, cl = _stack_with_anchor(Pose2D(0.7, 2.5, 0.0), (0.0, 0.0, 0.0))
    st.set_goal((0.75, 2.5), tol=0.1)
    assert st.step(_free_scan(), 1.0, 0.05).status == "reached"
    assert cl.cancels >= 1


# ====================================================================== 구조물 변경
def _world_with_robot(seed=0):
    cfg = Config()
    w = World(cfg, seed=seed)
    w.cubes.clear()
    u = w.add_ugv("BLUE", 2)
    return cfg, w, u


def _occupied_near(grid, x, y, r=1):
    iy, ix = grid.to_cell(x, y)
    return bool(grid.occupied()[iy - r:iy + r + 1, ix - r:ix + r + 1].any())


def test_grid_follows_moved_box():
    cfg, w, u = _world_with_robot()
    c = Cube(2.0, 2.5)                                    # 정면 앞쪽 박스
    w.cubes.append(c)
    st = LocalStack(cfg)
    st.start(w.lidar(u), u.odom, (u.pose.x, u.pose.y))

    def scans(n):
        for _ in range(n):
            w.t += 0.1
            st.update(u.odom, w.lidar(u), w.t)

    scans(15)
    assert _occupied_near(st.grid, 1.8, 2.5)              # 앞쪽 면
    w.move_cube(0, 2.0, 3.6)                              # 박스를 옆으로 옮김(시야 안)
    scans(25)
    assert not _occupied_near(st.grid, 1.8, 2.5)          # 옛 자리는 비워졌다
    assert _occupied_near(st.grid, 1.8, 3.6, r=2) or _occupied_near(st.grid, 2.0, 3.4, r=2)   # 새 자리는 점유


def test_replans_when_box_appears_on_path():
    cfg, w, u = _world_with_robot()
    nav = Navigator(cfg)
    goal = (4.8, 2.5)

    def tick(n, t0):
        out = None
        for k in range(n):
            w.t = t0 + 0.1 * k
            sc = w.lidar(u)
            nav.update_map(sc, Pose2D(u.pose.x, u.pose.y, u.pose.th))
            nav.set_goal(goal, 0.1)
            out = nav.step(Pose2D(u.pose.x, u.pose.y, u.pose.th), sc, w.t, 0.1)
        return out, t0 + 0.1 * n

    out, t = tick(15, 0.0)
    assert out.status == "moving"
    n0, inv0 = nav.replans, nav.invalidations
    w.add_blank_cube(1.9, 2.5)                            # 직선 경로 위에 새 구조물
    out, t = tick(12, t)                                  # 1 s 안에(replan_s=1.0 이전에) 재계획되어야 함
    assert nav.invalidations > inv0 and nav.replans > n0
    assert all(not (1.7 - 0.17 <= x <= 2.1 + 0.17 and 2.3 - 0.17 <= y <= 2.7 + 0.17) for x, y in nav.path)


# ====================================================================== 시뮬 통합 (백엔드별)
def _cfg(backend):
    cfg = Config()
    cfg.mission.spin_w, cfg.mission.tour_nx, cfg.mission.tour_ny = 1.8, 3, 3
    cfg.nav.backend = backend
    return cfg


@pytest.mark.parametrize("backend,team", [("nav2-fake", "R2"), ("nav2-fake", "B2"), ("builtin", "R2")])
def test_solo_reaches_goal_base(backend, team):
    m = build_match(seed=1, cfg=_cfg(backend), teams=[team])
    run(m, max_t=340)
    v = score(m)["agents"][m.agents[0].name]
    assert v["m5_reached"] is not None, f"{backend}/{team}: M5 미도달"
    assert v["collisions"] <= 3


@pytest.mark.parametrize("backend", ["builtin", "nav2-fake"])
def test_structure_changes_mid_match(backend):
    """경기 시작 후 구조물이 바뀌어도(60 s, 150 s) 거점 진입(M5)까지 간다."""
    m = build_match(seed=2, cfg=_cfg(backend), teams=["B2"])
    run(m, max_t=340, events=[(60.0, shuffle_structures), (150.0, shuffle_structures)])
    v = score(m)["agents"]["B-UGV-2"]
    assert v["m5_reached"] is not None
    assert v["collisions"] <= 6

