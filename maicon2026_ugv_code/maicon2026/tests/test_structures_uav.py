"""구조물 모델 = UAV 가 찍은 상자(structures="uav"): 실제 경기는 구조물 배치가 매번 달라 저장해 둔 고정 구조물 13개가 없다.
정적 모델은 외벽뿐이고, 구조물 윤곽은 경기 시작 때 UAV 가 찍은 것(정렬 전 원좌표)을 기준선 때 굳혀 쓴다. 시뮬·ROS 를 쓰지 않는다(합성 스캔)."""
from __future__ import annotations

import math
import random

import numpy as np
import pytest

from maicon26.config import Config
from maicon26.nav.pose_guard import ArenaFit, PoseGuard
from maicon26.types import Pose2D, Scan
from maicon26.validate import sanitize_config

DEG = math.pi / 180.0
TRUE = (2.2, 2.6, 0.0)
ODOM = Pose2D(0.0, 0.0, 0.0)

# 저장된 13개(sim/fixed_maicon.py)와 전혀 다른 배치 — 로봇 자리(2.2, 2.6)는 비워 둔다
LAYOUT = [(1.0, 1.0, .4, .4), (1.4, 3.4, .4, .8), (2.8, 1.6, .8, .4), (3.8, 3.2, .4, .4),
          (4.8, 1.4, .4, .8), (5.2, 3.8, .4, .4), (3.0, 4.2, .4, .4), (0.8, 2.4, .4, .4)]
_WALLS = ArenaFit(6.12, 5.0, ())


def scene_scan(pose, rects=LAYOUT, n=450, stamp=0.0):
    segs = list(_WALLS.map.segments)
    for cx, cy, sx, sy in rects:
        x0, y0, x1, y1 = cx - sx / 2, cy - sy / 2, cx + sx / 2, cy + sy / 2
        segs += [(x0, y0, x0, y1), (x1, y0, x1, y1), (x0, y0, x1, y0), (x0, y1, x1, y1)]
    ang = -math.pi + 2 * math.pi / n * np.arange(n)
    dx, dy = np.cos(pose[2] + ang), np.sin(pose[2] + ang)
    best = np.full(n, np.inf)
    for x0, y0, x1, y1 in segs:
        ex, ey = x1 - x0, y1 - y0
        den = dx * ey - dy * ex
        with np.errstate(divide="ignore", invalid="ignore"):
            t = ((x0 - pose[0]) * ey - (y0 - pose[1]) * ex) / den
            u = ((x0 - pose[0]) * dy - (y0 - pose[1]) * dx) / den
        ok = (np.abs(den) > 1e-9) & (t > 0.05) & (u >= 0) & (u <= 1)
        best = np.where(ok & (t < best), t, best)
    best[~np.isfinite(best)] = 8.0
    return Scan(-math.pi, 2 * math.pi / n, best, stamp=stamp, range_max=9.0)


def uav_guard(**over):
    cfg = Config()
    cfg.pose_guard.enable = True
    cfg.pose_guard.structures = "uav"
    for k, v in over.items():
        setattr(cfg.pose_guard, k, v)
    logs = []
    g = PoseGuard(cfg.pose_guard, cfg.arena, log=lambda t, m: logs.append((t, m)))     # fit 을 주지 않는다 — 모드에 따라 스스로 만든다
    g.logs = logs
    return g


def settle(g, pose=TRUE, seconds=6.0, t0=0.0, dt=0.5):
    t = t0
    for _ in range(int(seconds / dt)):
        g.process(t, Pose2D(*pose), ODOM, scene_scan(pose, stamp=t))
        t += dt
    return t


def run(g, raw, t0, seconds, dt=0.5):
    t = t0
    for _ in range(int(seconds / dt)):
        t += dt
        g.process(t, Pose2D(*raw), ODOM, scene_scan(TRUE, stamp=t))
        req = g.take_reanchor()
        if req is not None:
            return t, req
    return t, None


# ---------------------------------------------------------------------- 설정
def test_code_default_is_fixed_and_real_config_uses_uav_structures():
    assert Config().pose_guard.structures == "fixed" and Config().pose_guard.structures_min_boxes == 4
    cfg = Config.load("config/waveshare_real.json")
    assert cfg.pose_guard.structures == "uav"                      # 실제 경기는 구조물이 매번 달라 저장된 13개를 믿지 않는다


def test_invalid_structures_value_falls_back_to_default():
    cfg = Config()
    cfg.pose_guard.structures = "13"
    changed = sanitize_config(cfg)
    assert "pose_guard.structures" in changed and cfg.pose_guard.structures == "fixed"


# ---------------------------------------------------------------------- 모델
def test_uav_mode_static_model_is_walls_only():
    g = uav_guard()
    g._ensure_fit()
    assert g.fit.map.rectangles == ()
    assert g.report()["structures"]["mode"] == "uav"


def test_no_baseline_until_uav_boxes_arrive():
    g = uav_guard()
    t = settle(g, seconds=6.0)
    assert g._link_base is None and g.link["state"] == "waiting_uav" and g.link["uav_boxes"] == 0
    g.set_uav_rects(LAYOUT[:2], LAYOUT[:2])                        # min_boxes(4) 미만
    t = settle(g, t0=t, seconds=3.0)
    assert g._link_base is None and g.link["state"] == "waiting_uav"
    g.set_uav_rects(LAYOUT, LAYOUT)
    t = settle(g, t0=t, seconds=4.0)
    assert g._link_base is not None and g._link_base["ratio"] > 0.8
    assert g.link["state"] in ("baseline", "ok")


def test_model_is_latched_at_baseline_and_ignores_later_uav_frames():
    g = uav_guard()
    g.set_uav_rects(LAYOUT, LAYOUT)
    t = settle(g, seconds=6.0)
    rep = g.report()["structures"]
    assert rep["latched"] and rep["model_boxes"] == len(LAYOUT)
    g.set_uav_rects([], [])                                         # UAV 가 막혀(403) 프레임이 끊겨도
    assert len(g._model_rects()) == len(LAYOUT)
    g.set_uav_rects(LAYOUT[:3], LAYOUT[:3])                         # 일부만 검출돼도 굳힌 모델은 그대로
    assert len(g._model_rects()) == len(LAYOUT)


def test_aligned_rects_are_not_used_only_raw_uav_coordinates():
    """정렬(dx,dy)이 반영된 상자 좌표는 앵커 오차를 품고 있다 — 적합에는 원좌표만 쓴다."""
    g = uav_guard()
    shifted = [(x + 0.3, y - 0.2, sx, sy) for x, y, sx, sy in LAYOUT]
    g.set_uav_rects(shifted, LAYOUT)
    settle(g, seconds=6.0)
    assert g._link_base is not None
    ox, oy, oth = g._link_base["off"]
    assert abs(ox) < 0.03 and abs(oy) < 0.03 and abs(oth) < 1.5 * DEG


def test_stored_fixed_13_does_not_explain_a_different_layout_but_uav_boxes_do():
    pts = ArenaFit.scan_points(scene_scan(TRUE))
    _, r_fixed = ArenaFit(6.12, 5.0).score(pts, TRUE)               # 저장된 13개 모델
    _, r_uav = _WALLS.score(pts, TRUE, LAYOUT)                      # UAV 가 찍은 모델
    assert r_uav > 0.95 and r_fixed < r_uav - 0.15


# ---------------------------------------------------------------------- 앵커 재설정(구조물 = UAV 상자)
def test_reanchor_recovers_anchor_error_with_uav_structures():
    g = uav_guard(reanchor=True)
    g.set_uav_rects(LAYOUT, LAYOUT)
    t = settle(g)
    assert g._link_base is not None
    g.mark_impact(t, "minor", 0.0)
    err = (0.25, -0.20)
    t, req = run(g, (TRUE[0] + err[0], TRUE[1] + err[1], 0.0), t, 12.0)
    assert req is not None and not req["revert"] and req["why"] == "impact"
    dx, dy, dth = req["trim"]
    assert dx == pytest.approx(-err[0], abs=0.04) and dy == pytest.approx(-err[1], abs=0.04) and abs(dth) < 1.5 * DEG


def test_reanchor_is_not_attempted_without_uav_boxes():
    """UAV 상자를 한 번도 못 받았으면 기준선이 없어 재설정하지 않는다(벽만으로 앵커를 바꾸지 않는다)."""
    g = uav_guard(reanchor=True)
    t = settle(g)
    g.mark_impact(t, "major", 0.0)
    t, req = run(g, (TRUE[0] + 0.25, TRUE[1], 0.0), t, 8.0)
    assert req is None and g._link_base is None
    assert g.report()["reanchor"]["rejects"].get("no_baseline", 0) >= 1


POSES = [(1.9, 2.0, 0.0), (3.4, 2.6, 0.3), (4.4, 2.6, math.pi), (2.0, 3.6, -0.6), (3.2, 0.6, 1.2), (5.4, 2.4, 2.9)]


def walk(g, t):
    """경기장 여러 자리를 돌며 정지해 도면(=UAV 상자) 잔차를 잰다."""
    for pose in POSES:
        for _ in range(14):
            g.process(t, Pose2D(*pose), ODOM, scene_scan(pose, stamp=t))
            t += 0.5
    return t


def test_uav_detection_error_shows_up_as_plan_spread_and_blocks_reanchor():
    """UAV 상자 좌표가 상자마다 다르게 틀려 있으면(윗면 투영·보정 오차) 자리마다 잔차가 달라진다 — 퍼짐이 문턱을 넘으면 재설정을 거부한다.
    (상자가 여러 개면 오차가 평균돼 퍼짐은 상자 오차보다 작다: 상자마다 ±14 cm 어긋나도 퍼짐은 약 4 cm 다.)"""
    rng = random.Random(3)
    noisy = [(x + rng.uniform(-0.14, 0.14), y + rng.uniform(-0.14, 0.14), sx, sy) for x, y, sx, sy in LAYOUT]
    exact = uav_guard(reanchor=True)
    exact.set_uav_rects(LAYOUT, LAYOUT)
    walk(exact, settle(exact, seconds=4.0))
    bad = uav_guard(reanchor=True, reanchor_plan_spread_m=0.03)
    bad.set_uav_rects(noisy, noisy)
    t = walk(bad, settle(bad, seconds=4.0))
    se, sb = exact.plan_spread(), bad.plan_spread()
    assert se is not None and sb is not None
    assert se["xy"] < 0.01 and sb["xy"] > 0.025 and sb["xy"] > se["xy"] + 0.02
    bad.mark_impact(t, "major", 0.0)
    t, req = run(bad, (TRUE[0] + 0.25, TRUE[1], 0.0), t, 10.0)
    assert req is None and bad.report()["reanchor"]["rejects"].get("plan_inconsistent", 0) >= 1
