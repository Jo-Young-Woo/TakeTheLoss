"""Nav2Stack 에 건 PoseGuard: 꺼져 있으면 예전과 같고, 켜면 TF 끊김 추측항법·LiDAR 틀어짐 보정·내장 A* 전환·복구 보정이 된다.
ROS 없이 스텁 클라이언트로 돈다."""
import math

import numpy as np
import pytest

from maicon26.config import Config
from maicon26.nav.nav2_stack import Anchor, Nav2Stack
from maicon26.types import Pose2D, Scan

from test_nav_stack import StubClient, _free_scan


def wall_scan(r=1.0, stamp=1.0):
    n = 360
    return Scan(angle_min=-math.pi, angle_inc=2 * math.pi / n, ranges=np.full(n, r), stamp=stamp, range_max=8.0)


def make(guard=True, arena=Pose2D(3.0, 1.5, 0.0), m0=(0.0, 0.0, 0.0), **over):
    cfg = Config()
    cfg.pose_guard.enable = guard
    cfg.pose_guard.link_check = False                               # 합성 스캔(원)은 경기장 정합과 무관하다 — 경기장 연결 점검은 test_pose_guard 가 맡는다
    for k, v in over.items():
        setattr(cfg.pose_guard, k, v)
    cl = StubClient(m0)
    st = Nav2Stack(cfg, cl)
    st.anchor = Anchor.from_pair(arena, m0)
    st.pose = arena.copy()
    return st, cl


def run_updates(st, cl, t0, seconds, odom, m=None, hz=10.0, scan=True):
    t = t0
    for _ in range(int(seconds * hz)):
        t += 1.0 / hz
        if m is not None:
            cl.m = m
        st.update(odom, wall_scan(stamp=t) if scan else None, t)
    return t


def test_default_has_no_guard_and_pose_follows_tf():
    st, cl = make(guard=False)
    assert st.guard is None
    cl.m = (0.3, 0.1, 0.2)
    st.update(Pose2D(), None, 1.0)
    assert (st.pose.x, st.pose.y, st.pose.th) == pytest.approx(st.anchor.to_arena(0.3, 0.1, 0.2))
    cl.m = None                                                     # TF 가 끊기면 직전 자세 유지(예전과 같다)
    p = st.pose.copy()
    st.update(Pose2D(0.2, 0.0, 0.0), None, 1.1)
    assert (st.pose.x, st.pose.y) == (p.x, p.y)
    assert st.diagnostics()["pose_guard"] is None


def test_guard_follows_tf_when_nothing_is_wrong():
    st, cl = make()
    t = run_updates(st, cl, 0.0, 3.0, Pose2D(0, 0, 0), m=(0.0, 0.0, 0.0))
    assert st.guard.state == "ok" and (st.pose.x, st.pose.y, st.pose.th) == pytest.approx((3.0, 1.5, 0.0), abs=1e-6)
    assert st.diagnostics()["pose_guard"]["state"] == "ok"


def test_tf_loss_is_bridged_by_odometry_then_declared_lost():
    st, cl = make()
    t = run_updates(st, cl, 0.0, 2.0, Pose2D(0, 0, 0), m=(0.0, 0.0, 0.0))
    cl.m = None
    odom = Pose2D(0, 0, 0)
    for i in range(10):                                              # 1 s 동안 0.2 m/s 로 전진
        t += 0.1
        odom = Pose2D(odom.x + 0.02, 0.0, 0.0)
        st.update(odom, None, t)
    assert st.pose.x == pytest.approx(3.0 + 0.2, abs=0.02) and st.guard.state == "degraded"
    for i in range(40):                                              # 더 오래 끊기면 lost
        t += 0.1
        st.update(odom, None, t)
    assert st.guard.state == "lost" and "tf_lost" in st.guard.reasons


def test_lidar_shift_after_impact_is_compensated_and_forces_builtin_astar():
    st, cl = make()
    g = st.guard
    odom = Pose2D(0, 0, 0)
    t = run_updates(st, cl, 0.0, 8.0, odom, m=(0.0, 0.0, 0.0))        # 충격 전 안정
    delta = math.radians(7.0)
    g.mark_impact(t, "major", 0.0, "accel")
    t = run_updates(st, cl, t, 3.0, odom, m=(0.0, 0.0, delta))        # 충격 뒤 SLAM 헤딩만 7° 틀어짐(차체는 안 돌았다)
    assert g.shift == pytest.approx(delta, abs=math.radians(0.7))
    assert st.pose.th == pytest.approx(0.0, abs=math.radians(0.8))     # 보정된 자세는 차체 헤딩을 따른다
    assert g.needs_builtin_nav
    st.set_goal((4.0, 1.5), tol=0.1)
    st.step(wall_scan(), t + 0.1, 0.05)
    assert st._fallback and "내장 A*" in st.fallback_reason and st.fallbacks == 1


def test_small_shift_stays_on_nav2():
    st, cl = make()
    g = st.guard
    odom = Pose2D(0, 0, 0)
    t = run_updates(st, cl, 0.0, 8.0, odom, m=(0.0, 0.0, 0.0))
    g.mark_impact(t, "minor", 0.0, "accel")
    t = run_updates(st, cl, t, 3.0, odom, m=(0.0, 0.0, math.radians(3.5)))
    assert g.shift == pytest.approx(math.radians(3.5), abs=math.radians(0.7)) and not g.needs_builtin_nav
    st.set_goal((4.0, 1.5), tol=0.1)
    st.step(wall_scan(), t + 0.1, 0.05)
    assert not st._fallback


def test_scan_none_skips_grid_integration_but_not_pose():
    st, cl = make(guard=False)
    st.update(Pose2D(), wall_scan(1.0), 1.0)
    before = st.grid.L.copy()
    assert (before != 0).any()
    cl.m = (0.5, 0.0, 0.0)
    st.update(Pose2D(), None, 1.1)
    assert (st.grid.L == before).all() and st.pose.x == pytest.approx(3.5)


def test_apply_trim_keeps_arena_pose_consistent_and_republishes_walls():
    st, cl = make(m0=(0.4, -0.2, 0.3), arena=Pose2D(3.0, 1.5, 0.1))
    st.update(Pose2D(), None, 1.0)
    p = st.pose.copy()
    walls_before = cl.walls
    cancels = cl.cancels
    st._apply_trim((0.05, -0.03, math.radians(2.0)))
    x, y, th = st.anchor.to_arena(0.4, -0.2, 0.3)                      # 같은 map 자세를 새 앵커로 옮기면
    assert (x, y) == pytest.approx((p.x + 0.05, p.y - 0.03), abs=1e-9)
    assert th == pytest.approx(p.th + math.radians(2.0), abs=1e-9)
    assert (st.pose.x, st.pose.y) == pytest.approx((x, y), abs=1e-9)
    assert cl.cancels == cancels + 1 and st.trims_applied == 1 and st.guard.trims_applied == 1
    assert cl.walls is not None and (walls_before is None or not np.array_equal(cl.walls, walls_before))


def test_diagnostics_carry_guard_report():
    st, cl = make()
    run_updates(st, cl, 0.0, 1.0, Pose2D(), m=(0.0, 0.0, 0.0))
    d = st.diagnostics()
    assert d["pose_guard"]["state"] == "ok" and d["trims_applied"] == 0
