"""앵커 재설정(reanchor): 충격·SLAM 도약·연결 불량 뒤 정지 상태에서 벽·고정 구조물에 다시 맞춰 SLAM 지도→경기장 앵커를 바꾼다.
PoseGuard 판정(복구·거부·원복) → Nav2Stack 적용(앵커·격자) → Agent·CoverageMap 후처리. 시뮬·ROS 를 쓰지 않는다(합성 스캔·스텁 클라이언트)."""
from __future__ import annotations

import math
from unittest.mock import MagicMock

import numpy as np
import pytest

from hw_fakes import make_robot
from maicon26.config import Config
from maicon26.mission.manager import Agent
from maicon26.nav.nav2_stack import Anchor, Nav2Stack
from maicon26.nav.pose_guard import ArenaFit
from maicon26.perception.coverage import BoxModel, CoverageMap
from maicon26.types import Pose2D, Scan
from test_nav_stack import StubClient
from test_pose_guard import fit, guard, make_scan

DEG = math.pi / 180.0
TRUE = (2.2, 2.6, 0.0)
ODOM = Pose2D(0.0, 0.0, 0.0)


def ra_guard(**over):
    over.setdefault("reanchor", True)
    return guard(**over)


def settle(g, pose=TRUE, seconds=6.0, t0=0.0, dt=0.5):
    """true 자세에서 정지해 연결 기준선을 만든다."""
    t = t0
    for _ in range(int(seconds / dt)):
        g.process(t, Pose2D(*pose), ODOM, make_scan(pose, stamp=t))
        t += dt
    return t


def run(g, raw, t0, seconds, dt=0.5, scan_pose=TRUE, **kw):
    t = t0
    for _ in range(int(seconds / dt)):
        t += dt
        g.process(t, Pose2D(*raw), ODOM, make_scan(scan_pose, stamp=t, **kw))
        req = g.take_reanchor()
        if req is not None:
            return t, req
    return t, None


# ====================================================================== PoseGuard 판정
def test_default_is_off_and_leaves_no_trace():
    g = guard()                                                      # reanchor 기본 False
    assert g.c.reanchor is False
    t = settle(g)
    g.mark_impact(t, "major", 0.0)
    t, req = run(g, (TRUE[0] + 0.25, TRUE[1] - 0.2, 0.0), t, 8.0)
    assert req is None and g.take_reanchor() is None and g.wants_still is False
    assert g.report()["reanchor"] is None


def test_impact_then_translation_error_is_recovered_after_confirmed_fits():
    g = ra_guard()
    t = settle(g)
    assert g._link_base is not None
    g.mark_impact(t, "minor", 0.0)
    err = (0.25, -0.20, 0.0)
    t, req = run(g, (TRUE[0] + err[0], TRUE[1] + err[1], 0.0), t, 12.0)
    assert req is not None and req["why"] == "impact" and not req["revert"]
    dx, dy, dth = req["trim"]
    assert dx == pytest.approx(-err[0], abs=0.04) and dy == pytest.approx(-err[1], abs=0.04) and abs(dth) < 1.5 * DEG
    assert req["samples"] == g.c.reanchor_confirm_n and req["ratio_after"] > 0.8
    assert any("앵커 재설정 결정" in m for _, m in g.logs)


def test_slam_jump_with_rotation_error_is_recovered_and_holds_still_for_a_moment():
    g = ra_guard(dead_reckon_s=1.0, jump_m=0.15)                  # 0.25 m 는 도약으로 본다 · 오래 이어지면 2 s 뒤 받아들인다
    t = settle(g)
    wrong = (TRUE[0] + 0.20, TRUE[1] + 0.15, 5 * DEG)
    seen_hold, req = False, None
    for _ in range(40):
        t += 0.5
        g.process(t, Pose2D(*wrong), ODOM, make_scan(TRUE, stamp=t))
        seen_hold = seen_hold or g.wants_still
        req = g.take_reanchor()
        if req:
            break
    assert req is not None and req["why"] == "jump" and seen_hold
    dx, dy, dth = req["trim"]
    assert (dx, dy) == pytest.approx((-0.20, -0.15), abs=0.05) and math.degrees(dth) == pytest.approx(-5.0, abs=1.6)


def test_persistent_link_failure_triggers_reanchor_without_any_impact():
    g = ra_guard()
    t = settle(g)
    wrong = (TRUE[0] + 0.12, TRUE[1] + 0.10, 3 * DEG)
    t, req = run(g, wrong, t, 20.0, dt=1.0)
    assert req is not None and req["why"] == "link"
    assert req["trim"][0] == pytest.approx(-0.12, abs=0.04) and req["trim"][1] == pytest.approx(-0.10, abs=0.04)
    assert g.link_checks > 0 and g.take_trim() is None              # 작은 복구 보정(link_trim)은 reanchor 가 켜지면 쓰지 않는다


def test_nothing_to_fix_ends_the_attempt_without_request():
    g = ra_guard()
    t = settle(g)
    g.mark_impact(t, "major", 0.0)
    t, req = run(g, TRUE, t, 8.0)
    assert req is None and g._ra is None
    assert any("앵커 재설정 불필요" in m for _, m in g.logs)


def test_arena_drawing_error_is_part_of_baseline_not_corrected():
    """도면이 실제와 5 cm 어긋나 있어도(시작 때 잔차) 충격 뒤 같은 잔차면 '고칠 것 없음'이다."""
    g = ra_guard()
    biased = (TRUE[0] + 0.05, TRUE[1] - 0.04, 2 * DEG)
    t = settle(g, pose=TRUE)                                        # 위치 TRUE 의 스캔
    g2 = ra_guard()
    t = 0.0
    for _ in range(12):
        g2.process(t, Pose2D(*biased), ODOM, make_scan(TRUE, stamp=t))
        t += 0.5
    assert g2._link_base is not None
    g2.mark_impact(t, "major", 0.0)
    t, req = run(g2, biased, t, 8.0)
    assert req is None


def test_ambiguous_single_wall_view_is_rejected():
    g = ra_guard()
    t = settle(g)
    sc0 = make_scan((3.06, 0.35, 0.0))
    only = np.where(np.abs(sc0.angles + math.pi / 2) < 0.30, sc0.ranges, np.inf)       # 남쪽 벽 한 조각만 보인다

    def one_wall(stamp):
        return Scan(sc0.angle_min, sc0.angle_inc, only.copy(), stamp=stamp, range_max=9.0)
    g.mark_impact(t, "major", 0.0)
    got = None
    for _ in range(24):
        t += 0.5
        g.process(t, Pose2D(3.06 + 0.25, 0.35, 0.0), ODOM, one_wall(t))
        got = got or g.take_reanchor()
    assert got is None
    assert sum(g.reanchor_rejects.get(k, 0) for k in ("ambiguous", "unobservable", "low_ratio", "few_points", "no_peak", "edge")) > 0


def test_correction_beyond_single_limit_is_rejected_and_reported():
    g = ra_guard(reanchor_max_m=0.15)
    t = settle(g)
    g.mark_impact(t, "minor", 0.0)
    t, req = run(g, (TRUE[0] + 0.30, TRUE[1], 0.0), t, 14.0)
    assert req is None and g.reanchor_rejects.get("too_large", 0) >= 1
    assert any("한 번 한도" in m for _, m in g.logs)


def test_samples_that_do_not_agree_are_not_applied():
    g = ra_guard()
    t = settle(g)
    g.mark_impact(t, "minor", 0.0)
    got = None
    for i in range(24):                                              # 틱마다 어긋남이 달라지는 SLAM(일치하지 않는다)
        t += 0.5
        e = 0.08 + 0.12 * (i % 2)
        g.process(t, Pose2D(TRUE[0] + e, TRUE[1] - e, 0.0), ODOM, make_scan(TRUE, stamp=t))
        got = got or g.take_reanchor()
    assert got is None


def test_motion_resets_the_streak():
    g = ra_guard()
    t = settle(g)
    g.mark_impact(t, "minor", 0.0)
    raw = (TRUE[0] + 0.25, TRUE[1], 0.0)
    odom_x = 0.0
    for i in range(30):
        t += 0.5
        odom_x += 0.2 if i % 2 == 0 else 0.0                         # 격틱으로 움직인다 → 정지 조건이 계속 끊긴다
        g.process(t, Pose2D(*raw), Pose2D(odom_x, 0.0, 0.0), make_scan(TRUE, stamp=t))
    assert g.take_reanchor() is None


def test_cooldown_and_max_count_and_baseline_requirements():
    g = ra_guard()
    g.mark_impact(0.0, "major", 0.0)                                 # 기준선이 아직 없다
    g._finish_verify(0.1, "timeout")
    assert g._ra is None and g.reanchor_rejects.get("no_baseline", 0) == 1
    t = settle(g)
    g._ra_last_t = t - 1.0
    g._ra_arm(t, "impact")
    assert g._ra is None and g.reanchor_rejects.get("cooldown", 0) == 1
    g._ra_last_t = -1e9
    g._ra_count = g.c.reanchor_max_count
    g._ra_arm(t, "impact")
    assert g._ra is None and g.reanchor_rejects.get("max_count", 0) == 1


def test_hold_only_for_major_or_jump_and_only_for_a_while():
    g = ra_guard(reanchor_hold_s=1.5)
    t = settle(g)
    g.mark_impact(t, "minor", 0.0)
    for _ in range(6):
        t += 0.5
        g.process(t, Pose2D(*TRUE), ODOM, make_scan(TRUE, stamp=t))
        assert g.wants_still is False                                # 작은 충격은 멈추게 하지 않는다
    g2 = ra_guard(reanchor_hold_s=1.5)
    t = settle(g2)
    g2.mark_impact(t, "major", 0.0)
    holds = []
    for i in range(14):                                              # 일치하지 않는 어긋남 → 적용되지 않고 시도가 계속된다
        t += 0.5
        e = 0.08 + 0.12 * (i % 2)
        g2.process(t, Pose2D(TRUE[0] + e, TRUE[1] - e, 0.0), ODOM, make_scan(TRUE, stamp=t))
        holds.append(g2.wants_still)
    assert True in holds and holds[-1] is False                      # 잠깐(reanchor_hold_s) 서 있다가 풀린다 — 끝없이 세워 두지 않는다
    first = holds.index(True)
    assert sum(holds) * 0.5 <= 1.5 + 0.5 and all(holds[first:first + sum(holds)])


# ---------------------------------------------------------------------- 적용 뒤 확인·원복
def _applied(g, req, t):
    g.note_reanchor_applied(req)
    return t


def test_applied_reanchor_is_verified_and_kept_when_fit_holds():
    g = ra_guard()
    t = settle(g)
    g.mark_impact(t, "minor", 0.0)
    t, req = run(g, (TRUE[0] + 0.25, TRUE[1] - 0.2, 0.0), t, 12.0)
    assert req is not None
    g.note_reanchor_applied(req)
    assert g.pose.x == pytest.approx(TRUE[0] + 0.25 + req["trim"][0])
    t, req2 = run(g, TRUE, t, 6.0)                                   # 스택이 앵커를 바꾼 뒤의 SLAM 자세 = 실제 자세
    assert req2 is None
    assert any("앵커 재설정 확인" in m for _, m in g.logs) and g.report()["reanchor"]["reverts"] == 0
    assert g.report()["reanchor"]["count"] == 1 and g.report()["reanchor"]["verifying"] is False


def test_worse_fit_after_apply_requests_revert_and_second_revert_locks():
    g = ra_guard(reanchor_cooldown_s=0.0)
    t = settle(g)
    for k in range(2):
        g.mark_impact(t, "minor", 0.0)
        t, req = run(g, (TRUE[0] + 0.25, TRUE[1] - 0.2, 0.0), t, 12.0)
        assert req is not None, k
        g.note_reanchor_applied(req)
        bad = (TRUE[0] + 0.35, TRUE[1] + 0.30, 8 * DEG)              # 적용 뒤에도 맞지 않는 자리 → 맞음 비율이 기대보다 낮다
        t, back = run(g, bad, t, 8.0)
        assert back is not None and back["revert"] and back["why"] == "revert"
        assert back["trim"] == pytest.approx(tuple(-v for v in req["trim"]))
        g.note_reanchor_applied(back)
    assert g._ra_locked is True and g.report()["reanchor"]["locked"] is True
    g.mark_impact(t, "minor", 0.0)
    assert g._ra is None
    assert g._ra_total == pytest.approx([0.0, 0.0, 0.0], abs=1e-9)    # 두 번 적용하고 두 번 원복 → 누적 0


def test_reanchor_ignores_uav_boxes_unless_asked():
    g = ra_guard()
    g.set_uav_rects([(2.0, 2.0, 0.4, 0.4)], raw=[(2.5, 2.5, 0.4, 0.4)])
    assert g._ra_rects() == ()
    g2 = ra_guard(reanchor_use_uav_boxes=True)
    g2.set_uav_rects([(2.0, 2.0, 0.4, 0.4)], raw=[(2.5, 2.5, 0.4, 0.4)])
    assert list(g2._ra_rects()) == [(2.5, 2.5, 0.4, 0.4)]            # 정렬 전 원좌표만 쓴다(정렬이 앵커 오차를 품고 있다)


def test_noise_does_not_break_recovery():
    g = ra_guard()
    t = 0.0
    for i in range(12):
        g.process(t, Pose2D(*TRUE), ODOM, make_scan(TRUE, stamp=t, noise=0.012, seed=i))
        t += 0.5
    g.mark_impact(t, "minor", 0.0)
    got = None
    for i in range(30):
        t += 0.5
        g.process(t, Pose2D(TRUE[0] + 0.22, TRUE[1] + 0.12, 0.0), ODOM, make_scan(TRUE, stamp=t, noise=0.012, seed=50 + i))
        got = got or g.take_reanchor()
    assert got is not None and got["trim"][0] == pytest.approx(-0.22, abs=0.05) and got["trim"][1] == pytest.approx(-0.12, abs=0.05)


def test_pure_lidar_yaw_shift_leaves_the_anchor_alone():
    """LiDAR 가 차체에 대해 돌아가도 벽 끝점의 월드 위치는 같다 → 앵커는 맞다. 재설정은 '불필요'로 끝난다(헤딩은 δ 보정이 맡는다)."""
    g = ra_guard()
    t = settle(g)
    g.mark_impact(t, "major", 0.0)
    laser = 8 * DEG
    got = None
    for _ in range(150):                                              # 10 Hz — 충격 후 검증은 정지 표본 8 개(0.8 s)가 필요하다
        t += 0.1
        raw = Pose2D(TRUE[0], TRUE[1], TRUE[2] + laser)               # SLAM 은 돌아간 LiDAR 를 따라 헤딩이 +δ
        g.process(t, raw, ODOM, g.correct_scan(make_scan(TRUE, stamp=t, laser_yaw=laser)))
        got = got or g.take_reanchor()
    assert got is None
    assert abs(math.degrees(g.shift) - 8.0) < 1.5                    # δ 는 따로 확정·보정된다


# ====================================================================== Nav2Stack 적용
def make_stack(err=(0.0, 0.0, 0.0), **over):
    cfg = Config()
    cfg.pose_guard.enable = True
    cfg.pose_guard.reanchor = True
    cfg.nav2.grid_source = "scan"
    for k, v in over.items():
        setattr(cfg.pose_guard, k, v)
    m0 = (0.4, -0.3, 0.1)                                            # SLAM 지도 프레임의 시작 자세(임의)
    cl = StubClient(m0)
    st = Nav2Stack(cfg, cl)
    st.guard.fit = fit()
    true0 = Pose2D(*TRUE)
    st.anchor = Anchor.from_pair(true0, m0)
    st.pose = true0.copy()
    return st, cl, m0


def map_pose_of(st, arena):
    return st.anchor.to_map(arena[0], arena[1], arena[2])


def test_stack_recovers_wrong_anchor_end_to_end_and_updates_everything():
    st, cl, m0 = make_stack()
    cl.m = map_pose_of(st, TRUE)
    t = 0.0
    for _ in range(14):                                              # 맞는 앵커로 기준선을 만든다
        t += 0.5
        st.update(ODOM, make_scan(TRUE, stamp=t), t)
    assert st.guard._link_base is not None and st.anchor_epoch == 0
    # 지도 프레임에서 로봇은 그대로인데 앵커만 어긋난다(예: SLAM 재국소화로 지도가 통째로 밀림)
    good = st.anchor
    st.anchor = Anchor(good.x + 0.22, good.y - 0.18, good.th + 4 * DEG)
    st.guard.mark_impact(t, "minor", 0.0)
    cl.sent.clear()
    walls_before = None if cl.walls is None else cl.walls.copy()
    st.grid.L[:] = 0.0
    ix = int((TRUE[0] + 1.0 - st.grid.ox) / st.grid.res)
    iy = int((TRUE[1] - st.grid.oy) / st.grid.res)
    st.grid.L[iy, ix] = 4.0                                          # 로봇 앞 1 m 의 점유 셀(옛 앵커 기준으로 쌓인 것)
    st.goal, st.goal_eff, st._sent_goal = (3.0, 3.0), (3.0, 3.0), (3.0, 3.0)
    for _ in range(60):
        t += 0.5
        st.update(ODOM, make_scan(TRUE, stamp=t), t)
        if st.anchor_epoch:
            break
    assert st.anchor_epoch == 1 and st.last_reanchor["why"] in ("impact", "link") and st.last_reanchor["revert"] is False
    assert st.pose.x == pytest.approx(TRUE[0], abs=0.05) and st.pose.y == pytest.approx(TRUE[1], abs=0.05)
    assert wrap_deg(st.pose.th - TRUE[2]) == pytest.approx(0.0, abs=1.6)
    assert st.anchor.x == pytest.approx(good.x, abs=0.05) and st.anchor.y == pytest.approx(good.y, abs=0.05)
    assert st._sent_goal is None and st.goal_eff is None and cl.cancels >= 1          # 보낸 목표는 취소됐다
    assert st.diagnostics()["anchor_epoch"] == 1 and st.diagnostics()["pose_guard"]["reanchor"]["count"] == 1
    if cl.walls is not None and walls_before is not None:
        assert not np.allclose(cl.walls, walls_before)                                # Nav2 에 알리는 외벽도 새 앵커로 다시 냈다


def wrap_deg(a):
    return math.degrees(math.atan2(math.sin(a), math.cos(a)))


def test_stack_grid_is_moved_by_the_same_transform_for_scan_source_only():
    st, cl, _ = make_stack()
    st.guard.note_reanchor_applied = MagicMock()                      # 격자 이동만 본다
    g = st.grid
    g.L[:] = 0.0
    px, py = TRUE[0] + 1.0, TRUE[1]
    g.L[int((py - g.oy) / g.res), int((px - g.ox) / g.res)] = 4.0
    st.guard.pose = Pose2D(*TRUE)
    st._apply_reanchor({"trim": (0.20, 0.0, 0.0), "why": "impact", "revert": False})
    iy, ix = np.nonzero(g.L > 3.0)
    assert len(ix) == 1
    x, y = g.to_world(int(iy[0]), int(ix[0]))
    assert (x, y) == pytest.approx((px + 0.20, py), abs=g.res)
    st2, _, _ = make_stack()
    st2.cfg.nav2.grid_source = "slam_map"
    st2.c2.grid_source = "slam_map"
    st2.guard.note_reanchor_applied = MagicMock()
    st2.grid.L[:] = 0.0
    st2.grid.L[40, 40] = 4.0
    st2.guard.pose = Pose2D(*TRUE)
    st2._apply_reanchor({"trim": (0.20, 0.0, 0.0), "why": "impact", "revert": False})
    assert st2.grid.L[40, 40] == 4.0                                  # slam_map 격자는 다음 지도 갱신에서 새 앵커로 다시 만들어진다


def test_stack_rotation_about_robot_keeps_robot_position_plus_shift():
    st, cl, m0 = make_stack()
    st.guard.note_reanchor_applied = MagicMock()
    st.guard.pose = Pose2D(*TRUE)
    mp = map_pose_of(st, TRUE)
    st._apply_reanchor({"trim": (0.1, -0.05, 6 * DEG), "why": "link", "revert": False})
    nx, ny, nth = st.anchor.to_arena(*mp)
    assert (nx, ny) == pytest.approx((TRUE[0] + 0.1, TRUE[1] - 0.05), abs=1e-9)
    assert nth == pytest.approx(TRUE[2] + 6 * DEG, abs=1e-9)
    # 되돌리면 원래 앵커
    a1 = Anchor(st.anchor.x, st.anchor.y, st.anchor.th)
    st.guard.pose = Pose2D(nx, ny, nth)
    st.pose = Pose2D(nx, ny, nth)
    st._apply_reanchor({"trim": (-0.1, 0.05, -6 * DEG), "why": "revert", "revert": True})
    back = st.anchor.to_arena(*mp)
    assert back == pytest.approx(TRUE, abs=1e-9)
    assert st.anchor_epoch == 2 and a1.th != st.anchor.th


def test_stack_without_reanchor_has_no_epoch_change():
    cfg = Config()
    cfg.pose_guard.enable = True
    st = Nav2Stack(cfg, StubClient())
    assert st.anchor_epoch == 0 and st.last_reanchor is None and st.guard.c.reanchor is False


# ====================================================================== 후처리: Agent·CoverageMap
def make_agent():
    cfg = Config()
    cfg.nav.backend = "builtin"
    robot, tr, clock = make_robot("practice", "idle", cfg=cfg)
    agent = Agent(robot, MagicMock(), cfg=cfg)
    agent.stack = MagicMock()
    agent.stack.anchor_epoch = 0
    return agent


def test_agent_moves_pose_history_with_the_anchor_and_ignores_unchanged_epoch():
    a = make_agent()
    a._hist.clear()
    a._hist.extend([(1.0, 1.0, 1.0, 0.0), (2.0, 2.0, 1.0, 0.5)])
    a._anchor_changed(3.0)
    assert list(a._hist)[0] == (1.0, 1.0, 1.0, 0.0)                  # epoch 그대로 → 변화 없음
    a.stack.anchor_epoch = 1
    a.stack.last_reanchor = {"epoch": 1, "pivot": (2.0, 1.0), "trim": (0.1, 0.0, 90 * DEG), "revert": False, "why": "impact"}
    a._anchor_changed(3.0)
    h = list(a._hist)
    assert h[1][1:] == pytest.approx((2.1, 1.0, 0.5 + math.pi / 2))   # pivot 위의 자세: 위치는 +d, 헤딩은 +dth
    assert h[0][1:3] == pytest.approx((2.0 + 0.1, 1.0 + (-1.0)), abs=1e-9)   # (1,1)-(2,1) = (-1,0) 을 90° 돌리면 (0,-1)
    assert a.stats["anchor_epoch"] == 1 and a.stats["anchor_changes"] == 1
    a._anchor_changed(4.0)
    assert a.stats["anchor_changes"] == 1                             # 같은 epoch 는 두 번 반영하지 않는다


def test_agent_survives_a_mock_stack_and_non_integer_epoch():
    a = make_agent()
    a.stack.anchor_epoch = MagicMock()
    a._anchor_changed(1.0)                                            # 숫자가 아니면 무시
    a.stack.anchor_epoch = True
    a._anchor_changed(1.0)
    assert a.stats.get("anchor_changes", 0) == 0


def cov_with_boxes():
    cfg = Config()
    cm = CoverageMap(cfg)
    lidar = BoxModel(1, 3.0, 2.0, "full", [], 0.0, 0.0, sx=0.4, sy=0.4)
    cm._make_faces(lidar)
    lidar.faces[0].view_poses = [(2.0, 2.0, 0.0)]
    cm.boxes[1] = lidar
    uav = BoxModel(2, 1.0 + 0.30, 4.0 - 0.10, "face", [], 0.0, 0.0, sx=0.4, sy=0.4, uav_t=0.0, arena_cx=1.0, arena_cy=4.0)
    cm._make_faces(uav)
    cm.boxes[2] = uav
    cm._uav_alignment = type("A", (), {"dx": 0.30, "dy": -0.10, "matches": 2, "rms": 0.01})()
    cm.uav_alignment_report = {"dx": 0.3, "dy": -0.1, "matches": 2, "rms": 0.01, "t": 0.0}
    cm._uav_pending = [(1.3, 3.9, 0.4, 0.4, 0.0)]
    return cm


def test_coverage_moves_lidar_boxes_and_resets_uav_only_boxes_and_alignment():
    cm = cov_with_boxes()
    cm.seen.seen[:] = False
    cm.seen.seen[20, 30] = True
    out = cm.on_anchor_change((2.0, 2.0), (0.2, 0.0, 0.0), 5.0)
    assert out == {"moved": 1, "uav_reset": 1}
    b1, b2 = cm.boxes[1], cm.boxes[2]
    assert (b1.cx, b1.cy) == pytest.approx((3.2, 2.0))
    assert b1.faces[0].view_poses[0] == pytest.approx((2.2, 2.0, 0.0))
    assert (b1.faces[0].cx, b1.faces[0].cy) == pytest.approx((3.2 + 0.2, 2.0))   # 면 위치도 박스를 따른다
    assert (b2.cx, b2.cy) == pytest.approx((1.0, 4.0))                # UAV 원좌표로 되돌렸다
    assert (b2.arena_cx, b2.arena_cy) == (1.0, 4.0)                   # UAV 원좌표는 건드리지 않는다
    assert cm._uav_alignment is None and cm.uav_alignment_report == {}
    assert cm._uav_pending[0][:2] == pytest.approx((1.0, 4.0))
    assert cm.seen.seen[20, 30] is np.False_ and cm.seen.seen.sum() == 1       # '봤다' 지도도 옮겨졌다(한 셀 → 한 셀)
    assert cm.stats["anchor_changes"] == 1


def test_coverage_keeps_face_state_when_box_moves():
    cm = cov_with_boxes()
    f = cm.boxes[1].faces[0]
    f.views = 3
    f.det_votes = {"TANK": 2.0}
    f.reg = 7
    cm.on_anchor_change((2.0, 2.0), (-0.35, 0.1, 0.0), 5.0)
    f2 = cm.boxes[1].faces[0]
    assert f2 is f and f2.views == 3 and f2.det_votes == {"TANK": 2.0} and f2.reg == 7      # 식별한 면의 기록은 그대로 — 위치만 옮겨진다


# ====================================================================== 도면(구조물 좌표) 오차 방어
def _shifted_world(shift):
    from maicon26.nav.arena_localization import fixed_infrastructure_rectangles
    return ArenaFit(6.12, 5.0, [(cx + shift[0], cy + shift[1], sx, sy) for cx, cy, sx, sy in fixed_infrastructure_rectangles()])


def _scan_in(world, pose, stamp):
    import test_pose_guard as tp
    old, tp._FIT = tp._FIT, world
    try:
        return tp.make_scan(pose, stamp=stamp)
    finally:
        tp._FIT = old


SPOTS = [(0.9, 0.8, 0.0), (3.3, 2.6, 0.0), (1.0, 4.2, 0.0), (4.5, 2.2, 1.0), (5.0, 0.9, 2.0), (0.9, 2.5, 0.0), (5.2, 4.2, 3.0), (2.8, 0.7, 1.5)]


def _tour(shift, **over):
    w = _shifted_world(shift)
    g = ra_guard(**over)
    t = 0.0
    for A in SPOTS * 2:
        for _ in range(12):
            g.process(t, Pose2D(*A), ODOM, _scan_in(w, A, t))
            t += 0.5
    return g, w, t


def test_plan_spread_is_zero_for_a_correct_drawing_and_grows_with_structure_error():
    sp = {}
    for e in (0.0, 0.10):
        g, _, _ = _tour((e, 0.0))
        sp[e] = g.plan_spread()
        assert sp[e] is not None and sp[e]["n"] >= 8
    assert sp[0.0]["xy"] < 0.01 and sp[0.10]["xy"] > 0.06


def test_reanchor_refuses_when_the_drawing_looks_wrong_and_says_so():
    """구조물이 10 cm 틀려 있으면 앵커가 멀쩡해도 도면에 맞추면 '보정'이 생긴다 — 자리마다 잔차가 다르다는 증거로 막는다."""
    g, w, t = _tour((0.10, 0.0))
    B = (4.5, 2.2, 1.0)
    g.mark_impact(t, "minor", 0.0)
    got = None
    for _ in range(40):
        t += 0.5
        g.process(t, Pose2D(*B), ODOM, _scan_in(w, B, t))
        got = got or g.take_reanchor()
    assert got is None and g.reanchor_rejects.get("plan_inconsistent", 0) >= 1
    assert any("도면 잔차" in m and m.startswith("WARN") or "WARN PoseGuard: 자리마다" in m for _, m in g.logs)
    assert g.report()["reanchor"]["plan_spread"]["xy"] > 0.06


def test_plan_check_can_be_disabled_and_small_drawing_errors_pass():
    g, w, t = _tour((0.05, 0.0))                                      # 5 cm 오차는 문턱 아래
    assert g.plan_spread()["xy"] < g.c.reanchor_plan_spread_m
    g2, w2, t2 = _tour((0.10, 0.0), reanchor_plan_spread_m=0.0)       # 검사 끔
    g2.mark_impact(t2, "minor", 0.0)
    g2._ra_arm(t2, "impact")
    assert g2._ra is not None


def test_genuine_anchor_error_does_not_trip_the_plan_check():
    """앵커 오차는 자리와 무관하게 같은 양이다 — 도면이 맞는 세계에서 앵커가 틀어져도 퍼짐은 0 이라 재설정이 막히지 않는다."""
    g, w, t = _tour((0.0, 0.0))
    assert g.plan_spread()["xy"] < 0.01
    B = (3.3, 2.6, 0.0)
    g.mark_impact(t, "minor", 0.0)
    got = None
    for _ in range(40):
        t += 0.5
        g.process(t, Pose2D(B[0] + 0.2, B[1] - 0.1, 0.0), ODOM, _scan_in(w, B, t))
        got = got or g.take_reanchor()
    assert got is not None and got["trim"][0] == pytest.approx(-0.2, abs=0.04) and got["trim"][1] == pytest.approx(0.1, abs=0.04)


def test_agent_moves_observed_faces_and_survives_anchor_post_errors():
    from maicon26.perception.object_map import Face
    a = make_agent()
    a.omap.faces.append(Face(3.0, 2.0, 3.2, 2.0, n=2, vx=-1.0, vy=0.0))
    a.stack.anchor_epoch = 1
    a.stack.last_reanchor = {"epoch": 1, "pivot": (2.0, 2.0), "trim": (0.1, 0.0, math.pi / 2), "revert": False, "why": "link"}
    a._anchor_changed(5.0)
    f = a.omap.faces[0]
    assert (f.x, f.y) == pytest.approx((2.1, 3.0)) and (f.cx, f.cy) == pytest.approx((2.1, 3.2))     # (1,0) → 90° → (0,1)
    assert (f.vx, f.vy) == pytest.approx((0.0, -1.0)) and f.n == 2                                     # 접근 방향도 돌고 득표는 그대로
    a.coverage.on_anchor_change = MagicMock(side_effect=RuntimeError("boom"))
    a.stack.anchor_epoch = 2
    a._anchor_changed(6.0)                                                                             # 후처리 실패가 제어 루프를 죽이지 않는다
    assert a.stats["anchor_post_errors"] == 1 and a.stats["anchor_epoch"] == 2


def test_require_plan_check_blocks_until_residuals_were_measured_in_several_places():
    g = ra_guard(reanchor_require_plan_check=True)
    t = settle(g)                                                    # 한 자리뿐 — 도면 잔차 표본이 모자란다
    g.mark_impact(t, "minor", 0.0)
    t, req = run(g, (TRUE[0] + 0.25, TRUE[1], 0.0), t, 8.0)
    assert req is None and g.reanchor_rejects.get("plan_unchecked", 0) >= 1
    g2, w, t2 = _tour((0.0, 0.0), reanchor_require_plan_check=True)    # 여러 자리를 돌고 나면 허용된다
    g2.mark_impact(t2, "minor", 0.0)
    B = (3.3, 2.6, 0.0)
    got = None
    for _ in range(40):
        t2 += 0.5
        g2.process(t2, Pose2D(B[0] + 0.2, B[1], 0.0), ODOM, _scan_in(w, B, t2))
        got = got or g2.take_reanchor()
    assert got is not None


# ====================================================================== UGV 는 빼고 구조물만으로 맞춘다
UGV_BOXES = [(TRUE[0], TRUE[1] + 0.55, 0.3, 0.3), (TRUE[0] - 0.6, TRUE[1] - 0.2, 0.3, 0.3)]


def test_scan_points_exclude_drops_endpoints_inside_the_circles_before_thinning():
    sc = make_scan(TRUE, extra=UGV_BOXES)
    all_pts = ArenaFit.scan_points(sc, max_pts=400)
    pts = ArenaFit.scan_points(sc, max_pts=400, exclude=(TRUE, [(b[0], b[1], 0.3) for b in UGV_BOXES]))
    assert len(pts) < len(all_pts)
    world = ArenaFit.to_arena(pts, TRUE)
    for cx, cy, *_ in UGV_BOXES:
        assert np.min(np.hypot(world[:, 0] - cx, world[:, 1] - cy)) >= 0.3 - 1e-9
    near = ArenaFit.to_arena(all_pts, TRUE)
    assert any(np.hypot(near[:, 0] - cx, near[:, 1] - cy).min() < 0.25 for cx, cy, *_ in UGV_BOXES)      # 제외 전에는 UGV 점이 들어 있었다


def test_reanchor_fit_uses_no_endpoints_near_known_ugvs_and_still_recovers():
    g = ra_guard()
    seen = []
    orig = ArenaFit.search_wide

    def spy(self, pts, pose, *a, **k):
        seen.append((np.array(pts), pose))
        return orig(self, pts, pose, *a, **k)
    ArenaFit.search_wide = spy
    try:
        t = 0.0
        for _ in range(12):
            g.set_ugv_points([(x, y, 0.15) for x, y, _, _ in UGV_BOXES])
            g.process(t, Pose2D(*TRUE), ODOM, make_scan(TRUE, stamp=t, extra=UGV_BOXES))
            t += 0.5
        base_ratio = g._link_base["ratio"]
        g.mark_impact(t, "minor", 0.0)
        err, got = (0.25, -0.15), None
        for _ in range(30):
            t += 0.5
            g.process(t, Pose2D(TRUE[0] + err[0], TRUE[1] + err[1], 0.0), ODOM, make_scan(TRUE, stamp=t, extra=UGV_BOXES))
            got = got or g.take_reanchor()
    finally:
        ArenaFit.search_wide = orig
    assert got is not None and got["trim"][0] == pytest.approx(-err[0], abs=0.04) and got["trim"][1] == pytest.approx(-err[1], abs=0.04)
    assert base_ratio > 0.95 and seen                                  # UGV 끝점이 빠져 '구조물에 설명되는 비율' 이 거의 1
    pts, pose = seen[-1]
    world = ArenaFit.to_arena(pts, pose)                               # 적합에 쓴 끝점은 알려진 UGV 반경(0.3+0.45) 밖
    for cx, cy, *_ in UGV_BOXES:
        assert np.min(np.hypot(world[:, 0] - cx, world[:, 1] - cy)) >= 0.30 + 0.45 - 1e-6
    assert g.report()["reanchor"]["ugv_excluded"] == 2


def test_ugv_exclusion_can_be_turned_off_and_is_inert_without_ugvs():
    g = ra_guard(reanchor_ugv_exclude=False)
    g.set_ugv_points([(1.0, 1.0, 0.2)])
    g._ensure_fit()
    pts = g._structure_points(make_scan(TRUE, extra=UGV_BOXES), TRUE, 140)
    assert g.ugv_excluded == 0 and len(pts) > 100
    g2 = ra_guard()
    g2._ensure_fit()
    g2._structure_points(make_scan(TRUE), TRUE, 140)
    assert g2.ugv_excluded == 0                                         # UGV 를 모르면 아무것도 빼지 않는다
    g2.set_ugv_points([(float("nan"), 1.0, 0.2), (2.0, 2.0, 0.2)])
    assert g2.ugv_points == [(2.0, 2.0, 0.2)]


def test_agent_feeds_known_ugvs_to_the_guard_from_tracks_and_moving_blobs():
    from types import SimpleNamespace as NS
    a = make_agent()
    guard = MagicMock()
    guard.c = Config().pose_guard
    a.tracker = NS(tracks=[NS(t=9.5, x=2.0, y=3.0, predicted=lambda now: (2.1, 3.0))])
    a._blobs = [NS(x=4.0, y=1.0, moved=True), NS(x=5.0, y=1.0, moved=False)]       # 움직이지 않는 덩어리는 UGV 로 보지 않는다
    a._feed_guard_ugvs(10.0, guard)
    pts = guard.set_ugv_points.call_args[0][0]
    assert (2.1, 3.0, guard.c.reanchor_ugv_radius_m) in pts and (4.0, 1.0, guard.c.reanchor_ugv_radius_m) in pts and len(pts) == 2
    a.tracker = None
    a._blobs = None
    a._feed_guard_ugvs(11.0, guard)
    assert guard.set_ugv_points.call_args[0][0] == []
