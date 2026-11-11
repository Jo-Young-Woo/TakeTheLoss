"""OAK Depth 로 상대 UGV 거리·피아식별(perception/oak_range.py + UgvTracker.update_camera(depth=...)). ROS·카메라 없이 합성 깊이 영상으로 본다."""
from __future__ import annotations

import math

import numpy as np
import pytest

from maicon26.config import Config
from maicon26.perception.camera import oak_obs_camera
from maicon26.perception.oak_range import UgvDepthRanger, depth_range_for_box, pick_frame
from maicon26.perception.tracks import UgvTracker
from maicon26.types import DepthFrame, Detection, Frame, Pose2D

W, H = 1280, 720
FX = (W / 2) / math.tan(math.radians(69.0 / 2))
K = (FX, 0.0, W / 2, 0.0, FX, H / 2, 0.0, 0.0, 1.0)


def scene(z_ugv=1.2, width_m=0.30, cx_px=640.0, wall=3.0, size=(W, H)):
    """벽(wall m) 앞에 폭 width_m 인 UGV(z_ugv m). 반환: 깊이(m), UGV 상자(px)."""
    sw, sh = size
    sx = sw / W
    d = np.full((sh, sw), wall, dtype=np.float32)
    pw = FX * width_m / z_ugv
    box = (cx_px - pw / 2, 360 - 0.55 * pw, cx_px + pw / 2, 360 + 0.45 * pw)
    d[int(box[1] * sx):int(box[3] * sx), int(box[0] * sx):int(box[2] * sx)] = z_ugv
    return d, box


def cfg():
    c = Config()
    c.oak_det.depth_range = True
    return c


def test_range_of_ugv_in_front_of_wall():
    d, box = scene(1.2)
    r = depth_range_for_box(d, box, (W, H), fx=FX, cx=W / 2)
    assert r is not None and r.z == pytest.approx(1.2, abs=0.02) and r.r == pytest.approx(1.2, abs=0.02)
    assert r.width_m == pytest.approx(0.30, abs=0.01) and r.quality > 0.9


def test_off_axis_range_is_horizontal_distance():
    d, box = scene(1.2, cx_px=1000.0)
    r = depth_range_for_box(d, box, (W, H), fx=FX, cx=W / 2)
    assert r.bearing == pytest.approx(math.atan2(1000 - 640, FX), abs=0.01)
    assert r.r == pytest.approx(1.2 / math.cos(r.bearing), abs=0.02) and r.r > r.z


def test_background_inside_the_box_is_ignored_when_ugv_has_enough_mass():
    d, box = scene(1.2)
    big = (box[0] - 150, box[1] - 60, box[2] + 150, box[3] + 60)          # 상자가 뒤 벽을 많이 포함한다
    r = depth_range_for_box(d, big, (W, H), fx=FX, cx=W / 2)
    assert r is not None and r.z == pytest.approx(1.2, abs=0.03)


def test_lower_resolution_depth_scales_the_box():
    d, box = scene(1.5, size=(640, 360))
    r = depth_range_for_box(d, box, (W, H), fx=FX / 2, cx=320.0)
    assert r is not None and r.z == pytest.approx(1.5, abs=0.02) and r.width_m == pytest.approx(0.30, abs=0.015)


def test_invalid_or_sparse_depth_gives_none():
    d, box = scene(1.2)
    assert depth_range_for_box(np.zeros_like(d), box, (W, H), fx=FX, cx=W / 2) is None
    d2 = d.copy()
    d2[:] = 0
    d2[360, 640:645] = 1.2                                                # 유효 픽셀 5개
    assert depth_range_for_box(d2, box, (W, H), fx=FX, cx=W / 2) is None
    too_close = np.full_like(d, 0.1)
    assert depth_range_for_box(too_close, box, (W, H), fx=FX, cx=W / 2) is None          # min_range 안쪽은 믿지 않는다
    assert depth_range_for_box(d, (10, 10, 5, 5), (W, H), fx=FX, cx=W / 2) is None
    assert depth_range_for_box(d, (float("nan"), 0, 5, 5), (W, H), fx=FX, cx=W / 2) is None


def test_pick_frame_uses_nearest_within_limit():
    fr = [DepthFrame(np.zeros((2, 2), np.float32), s) for s in (1.0, 1.2, 1.6)]
    assert pick_frame(fr, 1.25, 0.35).stamp == 1.2
    assert pick_frame(fr, 3.0, 0.35) is None


def test_ranger_reasons():
    c = cfg()
    rg = UgvDepthRanger(c)
    d, box = scene(1.2)
    fr = DepthFrame(d, 5.0, K=K)
    det = Detection("UGV", 0.9, box, team="RED")
    r, why = rg.measure(det, fr, (W, H), ref_range=1.1)
    assert why == "ok" and r.r == pytest.approx(1.2, abs=0.02)
    r, why = rg.measure(det, fr, (W, H), ref_range=2.4)                     # 크기 추정과 45% 넘게 다르다
    assert why == "size_disagree"
    wide = Detection("UGV", 0.9, (box[0] - 400, box[1], box[2] + 400, box[3]))
    d2 = np.full((H, W), 1.2, dtype=np.float32)                              # 상자 전체가 1.2 m — 폭 1.5 m 짜리 면
    r, why = rg.measure(wide, DepthFrame(d2, 5.0, K=K), (W, H))
    assert why == "width_reject" and r is not None
    edge = Detection("UGV", 0.9, (W - 400, box[1], W, box[3]))               # 영상 가장자리에 걸치면 폭을 믿지 않는다
    r, why = rg.measure(edge, DepthFrame(d2, 5.0, K=K), (W, H))
    assert why == "ok"
    assert rg.measure(det, None, (W, H))[1] == "no_frame"
    c.oak_det.depth_aligned_to_rgb = False
    assert UgvDepthRanger(c).measure(det, fr, (W, H))[1] == "not_aligned"


def test_ranger_ring_keeps_latest_unique_frames():
    rg = UgvDepthRanger(cfg())
    for s in (1.0, 1.0, 1.1, 1.2):
        rg.push(DepthFrame(np.zeros((2, 2), np.float32), s))
    assert [f.stamp for f in rg.ring] == [1.0, 1.1, 1.2]
    assert rg.frame_for(1.15).stamp in (1.1, 1.2)
    assert rg.frame_for(9.0) is None and rg.stats["no_frame"] == 1


# ---------------------------------------------------------------------- 트래커 연결
def tracker_case(z, team="RED", width=0.30, wall=3.0, depth=True, conf=0.9, lidar=None):
    c = cfg()
    tr = UgvTracker(c, "BLUE")
    cam = oak_obs_camera(c.oak, c.rig, K, W, H)
    d, box = scene(z, width, wall=wall)
    rg = UgvDepthRanger(c)
    fr = DepthFrame(d, 7.0, K=K)
    det = Detection("UGV", conf, box, team=team)
    frame = Frame(None, 7.0, 0.0, 0.0, W, H)
    pose = Pose2D(2.0, 2.0, 0.0)
    fn = (lambda dt, ref: rg.measure(dt, fr, (W, H), ref)) if depth else None
    tr.update_camera([det], pose, frame, lidar, 0.0, cam=cam, weight=c.oak_det.weight, source="oak", depth=fn)
    return tr, cam, pose


def test_tracker_uses_depth_range_and_origin_at_camera():
    tr, cam, pose = tracker_case(1.0)
    assert len(tr.tracks) == 1
    t = tr.tracks[0]
    assert t.x == pytest.approx(2.0 + cam.x + 1.0 + 0.06, abs=0.05) and t.y == pytest.approx(2.0 + cam.y, abs=0.05)
    assert tr.depth_stats["ok"] == 1 and tr.last_depth_range[1] == pytest.approx(1.0, abs=0.03)


def test_depth_beats_size_estimate_when_they_agree_loosely():
    tr_d, cam, _ = tracker_case(1.4)
    tr_n, _, _ = tracker_case(1.4, depth=False)
    truth = 2.0 + cam.x + 1.4 + 0.06                                          # 카메라 위치 + 거리 + 차체 반폭(기본 0.06)
    err_d = abs(tr_d.tracks[0].x - truth)
    err_n = abs(tr_n.tracks[0].x - truth) if tr_n.tracks else 9.0
    assert err_d <= err_n + 1e-6 and err_d < 0.06


def test_near_ugv_team_vote_is_boosted_far_one_is_withheld():
    near, _, _ = tracker_case(1.2)
    base, _, _ = tracker_case(1.2, depth=False)
    c = cfg()
    assert near.tracks[0].team_votes["RED"] == pytest.approx(0.9 * c.oak_det.weight * c.oak_det.iff_weight_boost)
    assert near.depth_stats["iff_boost"] == 1
    if base.tracks:
        assert base.tracks[0].team_votes["RED"] == pytest.approx(0.9 * c.oak_det.weight)
    far, _, _ = tracker_case(3.0)
    assert far.tracks and far.tracks[0].team is None and not far.tracks[0].team_votes      # 위치 증거는 남고 피아는 보류
    assert far.depth_stats["iff_far"] == 1


def test_non_ugv_sized_object_is_dropped_only_with_strong_evidence():
    c = cfg()
    tr = UgvTracker(c, "BLUE")
    cam = oak_obs_camera(c.oak, c.rig, K, W, H)
    box = (240.0, 200.0, 1040.0, 520.0)                                      # 폭 800 px
    d = np.full((H, W), 1.5, dtype=np.float32)                                # 전부 1.5 m 의 평평한 면 → 폭 1.3 m
    rg = UgvDepthRanger(c)
    det = Detection("UGV", 0.9, box, team="RED")
    tr.update_camera([det], Pose2D(2, 2, 0), Frame(None, 7.0, 0, 0, W, H), None, 0.0, cam=cam, weight=0.5, source="oak",
                     depth=lambda dt, ref: rg.measure(dt, DepthFrame(d, 7.0, K=K), (W, H), ref))
    assert tr.tracks == [] and tr.depth_stats["dropped"] == 1
    sparse = np.zeros((H, W), np.float32)                                     # 깊이가 거의 없으면 버리지 않고 기존 경로(없음) 그대로
    tr2 = UgvTracker(c, "BLUE")
    tr2.update_camera([det], Pose2D(2, 2, 0), Frame(None, 7.0, 0, 0, W, H), None, 0.0, cam=cam, weight=0.5, source="oak",
                      depth=lambda dt, ref: rg.measure(dt, DepthFrame(sparse, 7.0, K=K), (W, H), ref))
    assert tr2.depth_stats["dropped"] == 0


def test_without_depth_argument_behaviour_is_unchanged():
    tr, _, _ = tracker_case(1.2, depth=False)
    assert dict(tr.depth_stats) == {} and tr.last_depth_range is None
