"""PoseGuard(nav/pose_guard.py): 충돌 감지, LiDAR 장착 틀어짐 추정·보정, SLAM 도약 감시, TF 끊김 추측항법, 경기장 연결 점검.
시뮬레이터·ROS 없이 순수 계산만 본다. 스캔은 경기장 외벽 + 고정 구조물 13개를 광선으로 쏴 만든다."""
from __future__ import annotations

import math
import random

import numpy as np
import pytest

from maicon26.config import Config
from maicon26.nav.pose_guard import ArenaFit, ImpactDetector, PoseGuard, wall_heading_residual
from maicon26.types import Pose2D, Scan
from maicon26.util import wrap

DEG = math.pi / 180.0
_FIT = None


def fit() -> ArenaFit:
    global _FIT
    if _FIT is None:
        _FIT = ArenaFit(6.12, 5.0)
    return _FIT


def make_scan(pose, n=450, stamp=0.0, extra=(), noise=0.0, seed=1, laser_yaw=0.0):
    """pose=(x,y,th) 차체 자세. laser_yaw: LiDAR 가 차체에 대해 돌아간 각 — 빔은 laser 프레임에서 재고 차체 기준 각도로 보고된다(보정 전)."""
    segs = list(fit().map.segments)
    for cx, cy, sx, sy in extra:
        x0, y0, x1, y1 = cx - sx / 2, cy - sy / 2, cx + sx / 2, cy + sy / 2
        segs += [(x0, y0, x0, y1), (x1, y0, x1, y1), (x0, y0, x1, y0), (x0, y1, x1, y1)]
    ang = -math.pi + 2 * math.pi / n * np.arange(n)                 # 보고되는 각도(차체 기준 '명목')
    world_dir = pose[2] + laser_yaw + ang                          # 실제 빔 방향: LiDAR 가 laser_yaw 만큼 돌아가 있다
    dx, dy = np.cos(world_dir), np.sin(world_dir)
    best = np.full(n, np.inf)
    rng = random.Random(seed)
    for x0, y0, x1, y1 in segs:
        ex, ey = x1 - x0, y1 - y0
        den = dx * ey - dy * ex
        with np.errstate(divide="ignore", invalid="ignore"):
            t = ((x0 - pose[0]) * ey - (y0 - pose[1]) * ex) / den
            u = ((x0 - pose[0]) * dy - (y0 - pose[1]) * dx) / den
        ok = (np.abs(den) > 1e-9) & (t > 0.05) & (u >= 0) & (u <= 1)
        best = np.where(ok & (t < best), t, best)
    best[~np.isfinite(best)] = 8.0
    if noise:
        best = best + np.array([rng.gauss(0, noise) for _ in range(n)])
    return Scan(-math.pi, 2 * math.pi / n, best, stamp=stamp, range_max=9.0)


def guard(**over):
    cfg = Config()
    cfg.pose_guard.enable = True
    for k, v in over.items():
        setattr(cfg.pose_guard, k, v)
    logs = []
    g = PoseGuard(cfg.pose_guard, cfg.arena, fit=fit(), log=lambda t, m: logs.append((t, m)))
    g.logs = logs
    return g


# ---------------------------------------------------------------------- 충돌 감지
def feed(det, t0, n, g=1.0, ax=0.0, ay=0.0, hz=100.0, noise=0.03, seed=0):
    rng = random.Random(seed)
    ev = []
    for i in range(n):
        t = t0 + i / hz
        e = det.add(t, g + rng.gauss(0, noise), ax + rng.gauss(0, noise), ay + rng.gauss(0, noise), rng.gauss(0, 2.0))
        if e is not None:
            ev.append(e)
    return ev, t0 + n / hz


def test_imu_noise_is_not_an_impact():
    d = ImpactDetector()
    ev, _ = feed(d, 0.0, 600, noise=0.06)
    assert ev == [] and d.count == 0


def test_frontal_collision_gives_one_event_with_front_bearing():
    d = ImpactDetector()
    ev, t = feed(d, 0.0, 200)
    assert ev == []
    got = d.add(t, 1.1, -0.9, 0.02, 30.0)             # 앞이 막혀 급감속: 가속도가 뒤(-x)로
    assert got is not None and got.sector == "front" and abs(got.bearing) < 0.3
    for i in range(1, 8):                             # 같은 충돌의 후속 표본은 새 사건이 아니다
        assert d.add(t + i * 0.01, 1.4, -1.3, 0.0, 20.0) is None
    assert d.count == 1 and got.peak_dg >= 1.2 and got.severity == "major"   # 후속 표본의 더 큰 값이 같은 사건의 peak 에 들어간다


@pytest.mark.parametrize("ax,ay,sector", [(0.8, 0.0, "rear"), (0.0, -0.8, "left"), (0.0, 0.8, "right")])
def test_collision_direction_sectors(ax, ay, sector):
    d = ImpactDetector()
    _, t = feed(d, 0.0, 200)
    e = d.add(t, 1.0, ax, ay, 0.0)
    assert e is not None and e.sector == sector                 # 몸이 받은 가속도의 반대쪽이 접촉 방위


def test_minor_vs_major_and_gyro_only():
    d = ImpactDetector()
    _, t = feed(d, 0.0, 200)
    e = d.add(t, 1.0, -0.6, 0.0, 0.0)
    assert e.severity == "minor"
    d2 = ImpactDetector()
    _, t = feed(d2, 0.0, 200)
    e2 = d2.add(t, 1.0, 0.0, 0.0, 500.0)                        # 자이로만 큰 충격(차체가 휙 돌았다)
    assert e2 is not None and e2.severity == "major" and e2.src == "gyro" and e2.bearing is None


def test_sustained_tilt_is_not_repeated_impacts():
    d = ImpactDetector()
    _, t = feed(d, 0.0, 200)
    ev, _ = feed(d, t, 400, g=1.0, ax=-0.55, ay=0.0)            # 경사로: 기준선이 뒤따라 오기 전까지 편차가 이어진다
    assert len(ev) <= 1 and d.count <= 1
    ev2, _ = feed(d, t + 4.0, 300, g=1.0, ax=-0.55, ay=0.0)
    assert ev2 == []                                            # 기준선을 옮긴 뒤에는 더 안 난다


def test_noisy_floor_raises_threshold():
    d = ImpactDetector(trigger_dg=0.45)
    ev, t = feed(d, 0.0, 600, noise=0.12)                       # 거친 바닥(평균 편차 ~0.1 g)
    assert ev == []
    assert d.add(t, 1.0, -0.5, 0.0, 0.0) is None                # 문턱이 5×잡음 이상으로 올라가 있다
    assert d.add(t + 0.5, 1.0, -1.4, 0.0, 0.0) is not None


def test_detector_rejects_bad_samples():
    d = ImpactDetector()
    assert d.add(float("nan"), 1.0) is None
    d.add(1.0, 1.0)
    assert d.add(0.5, 9.0, 5.0, 5.0) is None                    # 거꾸로 가는 시각은 버린다


# ---------------------------------------------------------------------- 벽 방향
def test_wall_heading_residual_recovers_rotation():
    pose = (3.06, 1.55, 0.0)
    for err in (-4.0, 0.0, 2.5, 6.0):
        s = make_scan(pose)
        pts = ArenaFit.to_arena(ArenaFit.scan_points(s, 450), (pose[0], pose[1], pose[2] + err * DEG))
        r = wall_heading_residual(pts)
        assert r is not None and r[1] > 0.6
        assert abs(math.degrees(r[0]) - err) < 1.0, (err, math.degrees(r[0]))


# ---------------------------------------------------------------------- LiDAR 틀어짐
def run_impact(delta_deg, body_turn_deg=0.0, shift_cfg=None, hz=10.0, n_pre=60, n_post=40):
    """정지한 로봇이 t=6 s 에 충돌. LiDAR 가 차체에 대해 delta 만큼 돌아간다(SLAM 헤딩이 +delta 로 바뀐다). body_turn: 차체 자체가 돈 각."""
    g = guard(**(shift_cfg or {}))
    true = [2.0, 2.0, 0.3]
    out = None
    t = 0.0
    for i in range(n_pre + n_post):
        t = i / hz
        if i == n_pre:
            true[2] += body_turn_deg * DEG
            g.mark_impact(t, "major", 0.0)
        laser = (delta_deg * DEG) if i >= n_pre else 0.0
        raw = Pose2D(true[0], true[1], wrap(true[2] + laser))      # SLAM 은 틀어진 LiDAR 를 따라간다 → 차체 헤딩에 +laser 가 섞인다
        odom = Pose2D(10.0 + true[0], -3.0 + true[1], wrap(true[2] - 0.7))     # 차체 오도메트리: 헤딩 오프셋은 임의(-0.7 rad)
        sc = make_scan(true, stamp=t, laser_yaw=laser)
        sc = g.correct_scan(sc)
        out = g.process(t, raw, odom, sc)
    return g, out, true


def test_lidar_yaw_shift_is_detected_and_compensated():
    g, pose, true = run_impact(6.0)
    assert g.verify_done()
    assert math.degrees(g.shift) == pytest.approx(6.0, abs=0.6)
    assert g.needs_builtin_nav                                  # 6° ≥ 5° → Nav2 대신 내장 A*
    assert wrap(pose.th - true[2]) == pytest.approx(0.0, abs=0.012)          # 보정된 자세 헤딩 = 실제 차체 헤딩
    assert g.report()["last_verify"]["action"] == "shift_confirmed"


def test_small_shift_compensated_without_nav_switch():
    g, pose, true = run_impact(3.5)
    assert math.degrees(g.shift) == pytest.approx(3.5, abs=0.6)
    assert not g.needs_builtin_nav
    assert wrap(pose.th - true[2]) == pytest.approx(0.0, abs=0.012)


def test_negative_shift():
    g, pose, true = run_impact(-8.0)
    assert math.degrees(g.shift) == pytest.approx(-8.0, abs=0.7) and g.needs_builtin_nav


def test_tiny_shift_below_threshold_is_ignored():
    g, pose, true = run_impact(1.2)
    assert g.shift == 0.0 and not g.needs_builtin_nav
    assert g.report()["last_verify"]["action"] == "none"


def test_body_rotation_without_lidar_shift_is_not_a_shift():
    g, pose, true = run_impact(0.0, body_turn_deg=12.0)         # 차체가 돌았지만 SLAM·오도메트리가 같이 돌았다
    assert g.shift == 0.0
    assert g.report()["last_verify"]["action"] == "none"


def test_compensation_can_be_disabled():
    g, pose, true = run_impact(6.0, shift_cfg={"shift_compensate": False})
    assert g.report()["shift_deg"] == pytest.approx(6.0, abs=0.6)            # 감지는 한다
    assert wrap(pose.th - true[2]) == pytest.approx(math.radians(6.0), abs=0.02)   # 자세는 보정하지 않는다
    assert g.correct_scan(make_scan(true)).angle_min == make_scan(true).angle_min


def test_correct_scan_rotates_beams_into_body_frame():
    g, _, true = run_impact(6.0)
    raw = make_scan(true, laser_yaw=6 * DEG)
    fixed = g.correct_scan(raw)
    assert fixed is not raw and fixed.angle_min == pytest.approx(raw.angle_min + g.shift)
    # 보정된 스캔을 보정된 자세로 경기장에 올리면 벽·구조물에 맞는다
    s, ratio = fit().score(ArenaFit.scan_points(fixed), (true[0], true[1], true[2]))
    assert ratio > 0.9
    s2, ratio2 = fit().score(ArenaFit.scan_points(raw), (true[0], true[1], true[2]))
    assert ratio2 < ratio - 0.2                                  # 보정하지 않은 스캔은 맞지 않는다


def test_verify_times_out_when_robot_keeps_moving():
    g = guard()
    for i in range(40):
        g.process(i * 0.1, Pose2D(1, 1, 0), Pose2D(0, 0, 0), make_scan((1, 1, 0)))
    g.mark_impact(4.0, "major", 0.0)
    t = 4.0
    for i in range(30):
        t = 4.0 + i * 0.1
        th = 0.5 * i * 0.1                                        # 계속 도는 중: 정지 표본이 안 모인다
        g.process(t, Pose2D(1, 1, th), Pose2D(0, 0, th), make_scan((1, 1, th)))
    assert g.verify_done()
    last = g.report()["last_verify"]
    assert last["how"] == "timeout" and last["action"] == "unverified"
    assert g.needs_builtin_nav                                    # 큰 충격인데 확인 못 했다 → Nav2 헤딩을 믿지 않는다


def test_yaw_step_without_impact_sensor_is_a_virtual_impact():
    g = guard()
    t = 0.0
    for i in range(80):
        t = i * 0.1
        g.process(t, Pose2D(2, 2, 0.3), Pose2D(0, 0, 0.3 - 0.7), make_scan((2, 2, 0.3), stamp=t))
    for i in range(6):                                            # 충격 센서 없이 SLAM 헤딩만 8° 계단
        t += 0.1
        g.process(t, Pose2D(2, 2, 0.3 + 8 * DEG), Pose2D(0, 0, 0.3 - 0.7), make_scan((2, 2, 0.3), stamp=t, laser_yaw=8 * DEG))
    ev = g.step_detected(t)
    assert ev is not None and ev.src == "yaw_step"
    g.mark_impact(t, "major", None, src="yaw_step")
    for i in range(30):
        t += 0.1
        sc = g.correct_scan(make_scan((2, 2, 0.3), stamp=t, laser_yaw=8 * DEG))
        g.process(t, Pose2D(2, 2, 0.3 + 8 * DEG), Pose2D(0, 0, 0.3 - 0.7), sc)
    assert math.degrees(g.shift) == pytest.approx(8.0, abs=0.7)


# ---------------------------------------------------------------------- SLAM 도약·TF 끊김
def test_single_slam_outlier_is_held_then_ignored():
    g = guard()
    for i in range(20):
        g.process(i * 0.1, Pose2D(2, 2, 0), Pose2D(0, 0, 0), None)
    p = g.process(2.0, Pose2D(3.5, 3.5, 1.0), Pose2D(0, 0, 0), None)      # 한 표본만 튄다
    assert (p.x, p.y) == pytest.approx((2.0, 2.0), abs=0.02)             # 예측(제자리)을 쓴다
    assert g.state == "degraded" and "slam_jump_hold" in g.reasons
    p = g.process(2.1, Pose2D(2.0, 2.0, 0), Pose2D(0, 0, 0), None)
    assert g.state == "ok" and (p.x, p.y) == pytest.approx((2.0, 2.0), abs=0.01)


def test_persistent_relocalization_is_accepted_after_confirmation():
    g = guard(jump_confirm_n=4)
    for i in range(20):
        g.process(i * 0.1, Pose2D(2, 2, 0), Pose2D(0, 0, 0), None)
    t = 2.0
    got = []
    for i in range(6):
        t += 0.1
        got.append(g.process(t, Pose2D(3.0, 2.6, 0.2), Pose2D(0, 0, 0), None))
    assert (got[0].x, got[0].y) == pytest.approx((2.0, 2.0), abs=0.02)
    assert (got[-1].x, got[-1].y) == pytest.approx((3.0, 2.6), abs=0.01) and g.jumps_accepted == 1
    assert g.state == "ok"


def test_jump_rejected_when_prediction_fits_arena_much_better():
    g = guard(jump_confirm_n=3)
    true = (2.0, 2.0, 0.0)
    for i in range(20):
        g.process(i * 0.1, Pose2D(*true), Pose2D(0, 0, 0), make_scan(true, stamp=i * 0.1))
    t = 2.0
    for i in range(8):                                            # SLAM 이 한 곳(잘못된 곳)으로 계속 점프 — 스캔은 실제 자리에서 나온다
        t += 0.1
        out = g.process(t, Pose2D(3.2, 3.0, 0.0), Pose2D(0, 0, 0), make_scan(true, stamp=t))
    assert g.jumps_accepted == 0 and (out.x, out.y) == pytest.approx((2.0, 2.0), abs=0.05)


def test_tf_miss_dead_reckons_then_lost():
    g = guard(dead_reckon_s=2.0)
    for i in range(20):
        g.process(i * 0.1, Pose2D(1.0, 1.0, 0.0), Pose2D(0, 0, 0), None)
    t = 2.0
    for i in range(10):                                           # 1 s 동안 TF 없음, 로봇은 +x 로 0.2 m/s
        t += 0.1
        p = g.process(t, None, Pose2D(0.02 * (i + 1), 0, 0), None)
    assert g.state == "degraded" and "tf_miss_dead_reckoning" in g.reasons
    assert p.x == pytest.approx(1.0 + 0.2, abs=0.01) and p.y == pytest.approx(1.0, abs=0.01)
    for i in range(20):
        t += 0.1
        p = g.process(t, None, Pose2D(0.2, 0, 0), None)
    assert g.state == "lost"
    p = g.process(t + 0.1, Pose2D(1.3, 1.0, 0), Pose2D(0.2, 0, 0), None)
    assert g.state in ("ok", "degraded") and g.state != "lost"


def test_odometry_reset_does_not_move_pose():
    g = guard()
    for i in range(10):
        g.process(i * 0.1, Pose2D(1, 1, 0), Pose2D(0, 0, 0), None)
    p = g.process(1.0, None, Pose2D(50, 50, 3.0), None)           # 오도메트리 리셋(큰 도약)은 이동량으로 쓰지 않는다
    assert (p.x, p.y) == pytest.approx((1, 1), abs=1e-6)


# ---------------------------------------------------------------------- 경기장 연결
def test_link_baseline_then_ok_at_true_pose():
    g = guard()
    pose = (1.0, 2.5, 0.0)
    for i in range(30):
        t = i * 0.5
        g.process(t, Pose2D(*pose), Pose2D(0, 0, 0), make_scan(pose, stamp=t))
    assert g.link["state"] == "ok" and g.link["ratio"] > 0.8
    assert g._link_base is not None and g._link_base["ratio"] > 0.8


def test_link_reports_arena_model_error_as_baseline_not_as_failure():
    """도면 치수가 실제와 달라도(벽이 15 cm 안쪽) 시작 때의 잔차를 기준선에 흡수하고 이상으로 보지 않는다."""
    g = guard()
    pose = (1.0, 2.5, 0.0)
    for i in range(30):
        t = i * 0.5
        g.process(t, Pose2D(pose[0] + 0.05, pose[1] - 0.04, 2 * DEG), Pose2D(0, 0, 0), make_scan(pose, stamp=t))
    assert g._link_base is not None
    assert g.link["state"] == "ok"
    off = g._link_base["off"]
    assert abs(off[0] + 0.05) < 0.04 and abs(off[1] - 0.04) < 0.04       # 잔차(= 보정하면 맞는 양)를 기록한다


def test_link_degrades_when_pose_is_wrong_and_proposes_trim_only_when_enabled():
    pose_true = (1.0, 2.5, 0.0)
    wrong = (1.0 + 0.12, 2.5 + 0.10, 3 * DEG)
    for trim_on in (False, True):
        g = guard(link_trim=trim_on)
        for i in range(10):
            g.process(i * 0.5, Pose2D(*pose_true), Pose2D(0, 0, 0), make_scan(pose_true, stamp=i * 0.5))
        assert g.link["state"] == "ok"
        t = 5.0
        for i in range(8):
            t += 1.0
            g.process(t, Pose2D(*wrong), Pose2D(0, 0, 0), make_scan(pose_true, stamp=t))
        assert g.link["state"] in ("weak", "bad")
        assert g.state == "degraded" and any(r.startswith("link_") for r in g.reasons)
        trim = g.take_trim()
        if not trim_on:
            assert trim is None
        else:
            assert trim is not None
            assert trim[0] == pytest.approx(-0.12, abs=0.04) and trim[1] == pytest.approx(-0.10, abs=0.04)
            assert math.degrees(trim[2]) == pytest.approx(-3.0, abs=1.6)
            assert g.take_trim() is None


def test_link_trim_never_fires_for_unobservable_axis_alone():
    """복도처럼 벽 하나만 보이면 벽을 따르는 방향은 관측 불가 — 그 축은 보정하지 않는다."""
    f = fit()
    pose = (3.06, 0.20, 0.0)                                      # 남쪽 벽 가까이, 정면을 보고 있어도 점이 한 벽 위주가 되도록 앞쪽 범위를 자른다
    sc = make_scan(pose)
    ang = sc.angles
    r = sc.ranges.copy()
    r[(ang < -3.0 * DEG - math.pi / 2 + math.pi / 2) | (ang > 0.2)] = 9.0
    only_south = Scan(sc.angle_min, sc.angle_inc, np.where((np.abs(ang + math.pi / 2) < 0.35), sc.ranges, 9.0), stamp=0.0, range_max=9.0)
    pts = ArenaFit.scan_points(only_south)
    res = f.search(pts, pose, xy=0.15, th=math.radians(6))
    assert res["ok"]
    assert res["obs"][1] is True                                  # 벽에 수직(y)은 관측된다
    assert res["obs"][0] is False                                 # 벽을 따르는 방향(x)은 관측되지 않는다


def test_uav_support_counts_unexplained_points():
    g = guard()
    pose = (1.0, 1.9, 0.0)
    box = (1.6, 1.9, 0.4, 0.4)                                     # 고정 구조물이 아닌 상자
    g.set_uav_rects([box])
    for i in range(6):
        t = i * 1.0
        g.process(t, Pose2D(*pose), Pose2D(0, 0, 0), make_scan(pose, stamp=t, extra=[box]))
    assert g.link["uav_boxes"] == 1
    assert g.link["uav_support"] is not None and g.link["uav_support"] > 0.8     # UAV 상자가 설명하는 점이 있다
    g2 = guard()
    for i in range(6):
        t = i * 1.0
        g2.process(t, Pose2D(*pose), Pose2D(0, 0, 0), make_scan(pose, stamp=t, extra=[box]))
    assert g2.link["uav_support"] is None                          # UAV 상자가 없으면 말하지 않는다


def test_report_is_json_serialisable():
    import json
    g, _, _ = run_impact(6.0)
    json.dumps(g.report())
