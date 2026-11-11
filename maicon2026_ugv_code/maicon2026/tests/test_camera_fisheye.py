"""7장: 어안 카메라 모델. 핀홀 식 보존, 어안 자기 일관성, OpenCV 교차 확인, 조준 오차, 보정 결과→설정, 성능. 시뮬을 쓰지 않는다."""
import json
import logging
import math
import time
import warnings
from pathlib import Path

import numpy as np
import pytest

from maicon26.calib.generate import RUNTIME_IMAGE_SIZE, camera_overlay, camera_intrinsics, gen_config_overlay
from maicon26.calib.profile import Profile
from maicon26.config import CameraCfg, Config
from maicon26.mission.aim import AimController
from maicon26.perception import camera as camera_mod
from maicon26.perception.camera import CameraModel, estimate_range
from maicon26.perception.coverage import CoverageMap
from maicon26.types import Detection, Pose2D

ROOT = Path(__file__).resolve().parents[1]
K_FISH = (170.0, 0.0, 320.0, 0.0, 168.0, 243.0, 0.0, 0.0, 1.0)             # 시험용 값(실기 K 는 아직 없다)
D_FISH = (0.012, -0.006, 0.0015, -0.0002)


def fisheye_cfg(**kw):
    base = dict(width=640, height=480, K=K_FISH, model="fisheye", D=D_FISH)
    base.update(kw)
    return CameraCfg(**base)


def fish(**kw):
    cm = CameraModel(fisheye_cfg(**kw))
    assert cm.model == "fisheye", cm.fallback_reason
    return cm


def pin(**kw):
    return CameraModel(CameraCfg(**kw))


def direction(theta, az):
    return math.sin(theta) * math.cos(az), math.sin(theta) * math.sin(az), math.cos(theta)      # X 오른쪽, Y 아래, Z 앞


def be_of(X, Y, Z):
    return math.atan2(-X, Z), math.atan2(-Y, math.hypot(X, Z))


# ====================================================================== 핀홀: 지금 식 그대로
@pytest.mark.parametrize("kw", [dict(), dict(K=(300.0, 0, 330.0, 0, 295.0, 235.0, 0, 0, 1)), dict(hfov_deg=90.0)])
def test_pinhole_branch_is_the_old_formulas_literally(kw):
    cm = pin(**kw)
    assert cm.model == "pinhole" and cm.fallback_reason is None
    for u in range(0, 641, 40):
        for v in range(0, 481, 40):
            assert cm.bearing(u) == math.atan2(cm.cx - u, cm.f)
            assert cm.bearing(u, v) == cm.bearing(u)                           # v 는 핀홀에서 쓰지 않는다
            assert cm.elevation(v) == math.atan2(cm.cy - v, cm.fy)
            assert cm.elevation(v, u) == cm.elevation(v)                       # u 도 쓰지 않는다(분리형)
            assert cm.angles(u, v) == (cm.bearing(u), cm.elevation(v))
            assert cm.angular_extent(u, v, 17.5) == math.atan2(17.5, cm.f)
    assert cm.half_fov_h() == math.atan2(cm.w / 2.0, cm.f)
    pose = Pose2D(1.0, 2.0, 0.4)
    assert cm.world_ray(pose, 12.0, 100.0) == pose.th + math.radians(12.0) + cm.bearing(100.0)
    assert cm.world_ray(pose, 12.0, 100.0, 77.0) == cm.world_ray(pose, 12.0, 100.0)


def test_pinhole_project_is_the_closed_form_and_inverts_angles():
    cm = pin()
    for b in (-1.0, -0.3, 0.0, 0.5, 1.2):
        for e in (-0.6, 0.0, 0.4):
            u, v = cm.project(b, e)
            assert u == cm.cx - cm.f * math.tan(b) and v == cm.cy - cm.fy * math.tan(e)
            b2, e2 = cm.angles(u, v)
            assert b2 == pytest.approx(b, abs=1e-12) and e2 == pytest.approx(e, abs=1e-12)
    assert cm.project(math.pi / 2, 0.0) is None and cm.project(0.0, -math.pi / 2) is None      # 앞쪽 반구 밖


def test_pinhole_range_from_size_unchanged_with_and_without_v():
    cm = pin()
    for u in (None, 40.0, 320.0, 600.0):
        for s in (8.0, 40.0, 160.0):
            old = cm.f * 0.4 / max(s, 1.0) if u is None else cm.f * 0.4 / max(s, 1.0) / max(math.cos(cm.bearing(u)), 0.3)
            assert cm.range_from_size(s, 0.4, u) == old
            assert cm.range_from_size(s, 0.4, u, 123.0) == old


def test_usable_hfov_deg_overrides_the_model_for_both_models():
    assert pin(usable_hfov_deg=100.0).half_fov_h() == pytest.approx(math.radians(50.0))
    assert fish(usable_hfov_deg=120.0).half_fov_h() == pytest.approx(math.radians(60.0))
    assert pin(usable_hfov_deg=None).half_fov_h() == math.atan2(320.0, pin().f)


def test_project_point_pinhole_is_the_coverage_formula():
    cm = pin(K=(300.0, 0, 330.0, 0, 295.0, 235.0, 0, 0, 1))
    u, v = cm.project_point(2.0, 0.5, -0.1)
    assert u == cm.cx - cm.f * 0.5 / 2.0 and v == cm.cy - cm.fy * -0.1 / 2.0
    assert cm.project_point(0.0, 0.5, 0.0) is None and cm.project_point(-1.0, 0.0, 0.0) is None


# ====================================================================== 설정 오류는 경고 후 핀홀
@pytest.mark.parametrize("kw,why", [
    (dict(K=None), "K"),
    (dict(D=None), "D"),
    (dict(D=(0.1, 0.0, 0.0)), "D"),
    (dict(D=(0.1, 0.0, 0.0, 0.0, 0.0)), "D"),
    (dict(D=(0.1, float("nan"), 0.0, 0.0)), "D"),
    (dict(K=(0.0, 0, 320.0, 0, 168.0, 243.0, 0, 0, 1)), "fx"),
])
def test_bad_fisheye_config_falls_back_to_pinhole_with_a_warning(kw, why, caplog):
    camera_mod._WARNED.clear()
    with caplog.at_level(logging.WARNING, logger="maicon26.camera"):
        cm = CameraModel(fisheye_cfg(**kw))                                   # 예외 없음
    assert cm.model == "pinhole" and cm.fallback_reason and why in cm.fallback_reason
    ref = CameraModel(CameraCfg(width=640, height=480, K=kw.get("K", K_FISH)) if "K" in kw and kw["K"] else CameraCfg())
    assert any("핀홀" in r.message for r in caplog.records)
    for u in (0.0, 100.0, 320.0, 600.0):
        assert cm.bearing(u, 77.0) == math.atan2(cm.cx - u, cm.f)


def test_fallback_without_k_uses_hfov_exactly_like_the_default():
    camera_mod._WARNED.clear()
    cm = CameraModel(CameraCfg(model="fisheye", D=D_FISH, hfov_deg=110.0))
    ref = CameraModel(CameraCfg(hfov_deg=110.0))
    assert cm.model == "pinhole" and (cm.f, cm.fy, cm.cx, cm.cy) == (ref.f, ref.fy, ref.cx, ref.cy)


def test_unknown_model_name_is_a_warning_too(caplog):
    camera_mod._WARNED.clear()
    with caplog.at_level(logging.WARNING, logger="maicon26.camera"):
        cm = CameraModel(CameraCfg(model="plumb_bob"))
    assert cm.model == "pinhole" and "plumb_bob" in cm.fallback_reason


def test_same_warning_is_logged_once_not_for_every_camera_model(caplog):
    camera_mod._WARNED.clear()
    with caplog.at_level(logging.WARNING, logger="maicon26.camera"):
        for _ in range(6):
            CameraModel(CameraCfg(model="fisheye"))
    assert len([r for r in caplog.records if r.name == "maicon26.camera"]) == 1


def test_model_name_is_case_and_space_tolerant():
    assert CameraModel(fisheye_cfg(model=" FishEye ")).model == "fisheye"


# ====================================================================== 어안 자기 일관성
def test_fisheye_angles_and_project_are_exact_inverses_on_a_0_to_75_degree_grid():
    cm = fish()
    worst = 0.0
    n = 0
    for tdeg in np.arange(0.0, 75.01, 2.5):
        for adeg in range(0, 360, 10):
            b, e = be_of(*direction(math.radians(tdeg), math.radians(adeg)))
            uv = cm.project(b, e)
            assert uv is not None
            b2, e2 = cm.angles(*uv)
            worst = max(worst, abs(b2 - b), abs(e2 - e))
            n += 1
    assert n > 1000 and worst < 1e-9


def test_fisheye_center_and_symmetry():
    cm = fish(K=(170.0, 0, 320.0, 0, 170.0, 240.0, 0, 0, 1))
    assert cm.project(0.0, 0.0) == pytest.approx((320.0, 240.0))
    assert cm.angles(320.0, 240.0) == (0.0, 0.0)
    ul, _ = cm.project(0.7, 0.0)
    ur, _ = cm.project(-0.7, 0.0)
    assert ul + ur == pytest.approx(640.0) and ul < 320.0                        # 왼쪽(+)이면 u 가 작다
    _, vu = cm.project(0.0, 0.5)
    _, vd = cm.project(0.0, -0.5)
    assert vu + vd == pytest.approx(480.0) and vu < 240.0                       # 위(+)이면 v 가 작다


def test_fisheye_bearing_and_elevation_depend_on_both_pixel_coordinates():
    cm = fish()
    b_true, e_true = 0.9, 0.55                                                  # 비스듬한 방향
    u, v = cm.project(b_true, e_true)
    assert cm.bearing(u, v) == pytest.approx(b_true, abs=1e-9) and cm.elevation(v, u) == pytest.approx(e_true, abs=1e-9)
    assert abs(cm.bearing(u) - b_true) > 0.02                                    # v 를 빼면(주점 높이로 치면) 방위가 틀린다 — 그래서 v 를 넘긴다
    pose = Pose2D(0.0, 0.0, 0.3)
    assert cm.world_ray(pose, 20.0, u, v) == pytest.approx(0.3 + math.radians(20.0) + b_true, abs=1e-9)


def test_pixels_outside_the_model_are_clamped_and_directions_outside_are_none():
    cm = fish()
    tmax = cm.theta_max
    assert 1.5 < tmax < 3.2
    just_out = direction(tmax + 0.05, 0.3)
    assert cm.project(*be_of(*just_out)) is None
    just_in = direction(tmax - 0.02, 0.3)
    assert cm.project(*be_of(*just_in)) is not None
    b, e = cm.angles(-5000.0, -5000.0)                                          # 영상 밖: 예외 없이 가장자리 각도로
    X, Y, Z = -math.sin(b) * math.cos(e), -math.sin(e), math.cos(b) * math.cos(e)
    assert math.acos(max(min(Z, 1.0), -1.0)) == pytest.approx(tmax, abs=1e-6)


def test_non_monotonic_polynomial_shrinks_the_model_to_the_monotonic_range():
    cm = CameraModel(fisheye_cfg(D=(-0.2, 0.0, 0.0, 0.0)))                      # θd = θ(1 - 0.2θ²) 는 θ > 1.29 에서 줄어든다
    assert cm.model == "fisheye" and cm.theta_max < 1.3
    assert cm.project(*be_of(*direction(1.5, 0.0))) is None


def test_angular_extent_is_half_the_angle_between_the_two_edge_directions():
    # 계약: 같은 반폭(px)이 덮는 각은 위치에 따라 다르다(핀홀 식 atan(px/f)는 위치와 무관해서 이를 못 본다).
    # 어안에서는 방향·위치에 따라 더 크거나 작을 수 있으므로 "커진다"가 아니라 두 방향 사이각의 절반과 같은지를 본다.
    cm = fish(D=(0.08, -0.03, 0.01, -0.002))                                    # 등거리에서 충분히 벗어난 값
    pinhole_like = math.atan2(20.0, cm.f)
    differs = False
    for bdeg, edeg in ((0.0, 0.0), (30.0, 10.0), (65.0, 0.0), (-60.0, 20.0)):
        u, v = cm.project(math.radians(bdeg), math.radians(edeg))
        ext = cm.angular_extent(u, v, 20.0)
        b1, e1 = cm.angles(u - 20.0, v)
        b2, e2 = cm.angles(u + 20.0, v)
        d1, d2 = np.array(direction_from(b1, e1)), np.array(direction_from(b2, e2))
        assert ext == pytest.approx(0.5 * math.acos(float(np.clip(d1 @ d2, -1, 1))), abs=1e-12)
        assert ext > 0.0
        differs = differs or abs(ext - pinhole_like) > 0.002
    assert differs                                                              # 핀홀 식과 실제로 다른 값이 나오는 위치가 있다


def direction_from(b, e):
    return -math.sin(b) * math.cos(e), -math.sin(e), math.cos(b) * math.cos(e)


def test_half_fov_h_is_the_horizontal_half_angle_of_the_image():
    cm = fish(K=(170.0, 0, 320.0, 0, 170.0, 240.0, 0, 0, 1), D=(0.0, 0.0, 0.0, 0.0))
    assert cm.half_fov_h() == pytest.approx(320.0 / 170.0, abs=1e-9)             # 등거리(D=0): θ = 가장자리 픽셀 / f


# ====================================================================== OpenCV 와 비교
cv2 = pytest.importorskip("cv2")
HAS_FISHEYE = hasattr(cv2, "fisheye") and hasattr(cv2.fisheye, "projectPoints") and hasattr(cv2.fisheye, "undistortPoints")


@pytest.mark.skipif(not HAS_FISHEYE, reason="이 cv2 에 fisheye 모듈이 없다")
def test_project_matches_cv2_fisheye_project_points_within_0p01_px():
    cm = fish()
    rng = np.random.default_rng(3)
    ths = rng.uniform(0.0, math.radians(75.0), 400)
    azs = rng.uniform(0.0, 2 * math.pi, 400)
    P = np.stack([np.sin(ths) * np.cos(azs), np.sin(ths) * np.sin(azs), np.cos(ths)], 1)
    Km = np.array(K_FISH).reshape(3, 3)
    ref, _ = cv2.fisheye.projectPoints(P.reshape(-1, 1, 3), np.zeros(3), np.zeros(3), Km, np.array(D_FISH).reshape(4, 1))
    ref = ref.reshape(-1, 2)
    mine = np.array([cm.project(*be_of(*p)) for p in P])
    assert np.abs(mine - ref).max() < 0.01


@pytest.mark.skipif(not HAS_FISHEYE, reason="이 cv2 에 fisheye 모듈이 없다")
def test_angles_match_cv2_fisheye_undistort_points_within_0p01_px():
    # cv2 는 정규화 좌표(X/Z, Y/Z)를 쓰므로 θ 가 90° 에 가까우면 오차가 폭주한다(θ>90° 는 아예 못 다룬다). 그래서 θ ≤ 80° 영역만 비교한다.
    cm = fish()
    rng = np.random.default_rng(4)
    pts = np.stack([rng.uniform(0, 640, 2000), rng.uniform(0, 480, 2000)], 1)
    Km = np.array(K_FISH).reshape(3, 3)
    und = cv2.fisheye.undistortPoints(pts.reshape(-1, 1, 2), Km, np.array(D_FISH).reshape(4, 1)).reshape(-1, 2)
    worst_px, used = 0.0, 0
    for (uu, vv), (xn, yn) in zip(pts, und):
        b, e = cm.angles(uu, vv)
        X, Y, Z = direction_from(b, e)
        if Z <= math.cos(math.radians(80.0)):
            continue
        used += 1
        worst_px = max(worst_px, math.hypot(X / Z - xn, Y / Z - yn) * K_FISH[0])        # 정규화 좌표 오차를 픽셀로
    assert used > 500 and worst_px < 0.01, (used, worst_px)


@pytest.mark.skipif(not HAS_FISHEYE, reason="이 cv2 에 fisheye 모듈이 없다")
def test_angles_then_cv2_project_points_returns_the_same_pixel():
    cm = fish()
    rng = np.random.default_rng(5)
    pts = np.stack([rng.uniform(0, 640, 2000), rng.uniform(0, 480, 2000)], 1)
    Km, Dm = np.array(K_FISH).reshape(3, 3), np.array(D_FISH).reshape(4, 1)
    dirs, kept = [], []
    for uu, vv in pts:
        b, e = cm.angles(uu, vv)
        X, Y, Z = direction_from(b, e)
        if Z > math.cos(math.radians(80.0)):                                            # cv2.projectPoints 는 Z>0 영역만 맞다
            dirs.append((X, Y, Z))
            kept.append((uu, vv))
    ref, _ = cv2.fisheye.projectPoints(np.array(dirs).reshape(-1, 1, 3), np.zeros(3), np.zeros(3), Km, Dm)
    assert len(kept) > 500 and np.abs(ref.reshape(-1, 2) - np.array(kept)).max() < 0.01


@pytest.mark.skipif(not HAS_FISHEYE, reason="이 cv2 에 fisheye 모듈이 없다")
def test_cv2_roundtrip_agrees_for_a_second_k_d():
    K2, D2 = (210.0, 0.0, 322.0, 0.0, 208.0, 243.0, 0.0, 0.0, 1.0), (0.05, -0.03, 0.01, -0.002)
    cm = CameraModel(CameraCfg(K=K2, model="fisheye", D=D2))
    assert cm.model == "fisheye"
    P = np.array([direction(math.radians(t), math.radians(a)) for t in (0, 15, 40, 70) for a in (0, 90, 200)])
    ref, _ = cv2.fisheye.projectPoints(P.reshape(-1, 1, 3), np.zeros(3), np.zeros(3), np.array(K2).reshape(3, 3), np.array(D2).reshape(4, 1))
    mine = np.array([cm.project(*be_of(*p)) for p in P])
    assert np.abs(mine - ref.reshape(-1, 2)).max() < 0.01


# ====================================================================== 조준: 알려진 각도(가장자리 70° 포함)
def aim_cfg(model):
    cfg = Config()
    cfg.camera = fisheye_cfg() if model == "fisheye" else CameraCfg(K=K_FISH)
    cfg.aim.laser_lateral_m = cfg.aim.laser_vertical_m = 0.0
    cfg.aim.laser_offset_pan_deg = cfg.aim.laser_offset_tilt_deg = 0.0
    return cfg


def aim_error(cfg, b_deg, e_deg, rng_m=2.0):
    cam = CameraModel(cfg.camera)
    uv = cam.project(math.radians(b_deg), math.radians(e_deg))
    assert uv is not None
    u, v = uv
    det = Detection("UGV", 0.9, (u - 10, v - 10, u + 10, v + 10), 0.0, None, {"sensors": [(u, v, 6.0)]})
    ctl = AimController(cfg)
    _, _, res = ctl.update(det, 0.0, 0.0, 1.0, rng_m)
    return res.err_pan, res.err_tilt


@pytest.mark.parametrize("b,e", [(0.0, 0.0), (10.0, 5.0), (-25.0, 12.0), (40.0, -20.0), (55.0, 0.0), (70.0, 0.0), (-70.0, 3.0), (68.0, 14.0)])
def test_fisheye_aim_error_equals_the_angle_difference_within_1e6_deg(b, e):
    ep, et = aim_error(aim_cfg("fisheye"), b, e)
    assert ep == pytest.approx(b, abs=1e-6) and et == pytest.approx(e, abs=1e-6)


def test_fisheye_aim_error_includes_the_laser_parallax_offsets_as_angles():
    cfg = aim_cfg("fisheye")
    cfg.aim.laser_lateral_m, cfg.aim.laser_vertical_m = 0.03, 0.02
    cfg.aim.laser_offset_pan_deg, cfg.aim.laser_offset_tilt_deg = 0.4, -0.2
    b_des = 0.4 + math.degrees(math.atan2(0.03, 2.0))
    e_des = -0.2 + math.degrees(math.atan2(0.02, 2.0))
    ep, et = aim_error(cfg, 70.0, 5.0, rng_m=2.0)
    assert ep == pytest.approx(70.0 - b_des, abs=1e-6) and et == pytest.approx(5.0 - e_des, abs=1e-6)


def test_pinhole_aim_is_still_the_old_formula():
    cfg = aim_cfg("pinhole")
    cam = CameraModel(cfg.camera)
    for b, e in ((0.0, 0.0), (20.0, 5.0), (-35.0, 10.0)):
        ep, et = aim_error(cfg, b, e)
        u, v = cam.project(math.radians(b), math.radians(e))
        assert ep == math.degrees(math.atan2(cam.cx - u, cam.f)) and et == math.degrees(math.atan2(cam.cy - v, cam.fy))


@pytest.mark.parametrize("centre_deg", [0.0, 30.0, 65.0])
def test_aim_alignment_flips_at_the_tolerance_given_by_angular_extent_at_the_sensor(centre_deg):
    # 센서 반경이 작으면 허용치 = deg(angular_extent(u, v, sensor_radius_frac·r_px)) 가 거리별 허용치보다 좁아 이쪽이 결정한다.
    cfg = aim_cfg("fisheye")
    cfg.aim.laser_offset_pan_deg = centre_deg                                          # 겨냥 기준 방향을 가장자리로 옮겨 본다
    cam = CameraModel(cfg.camera)
    r_px = 2.0
    nominal = math.degrees(cam.angular_extent(*cam.project(math.radians(centre_deg), 0.0), cfg.aim.sensor_radius_frac * r_px))
    assert cfg.aim.sensor_min_tol_deg < nominal < cfg.aim.tol_deg_far                  # 센서 쪽 허용치가 실제로 결정한다

    def aligned_at(offset_deg):
        u, v = cam.project(math.radians(centre_deg + offset_deg), 0.0)
        det = Detection("UGV", 0.9, (u - 10, v - 10, u + 10, v + 10), 0.0, None, {"sensors": [(u, v, r_px)]})
        _, _, res = AimController(cfg).update(det, 0.0, 0.0, 1.0, 2.0)
        assert res.err_pan == pytest.approx(offset_deg, abs=1e-6)
        return res.aligned
    assert aligned_at(0.8 * nominal) and aligned_at(-0.8 * nominal)
    assert not aligned_at(1.25 * nominal) and not aligned_at(-1.25 * nominal)


# ====================================================================== 크기 → 거리
@pytest.mark.parametrize("b_deg", [0.0, 25.0, 50.0, 65.0])
def test_fisheye_range_from_size_recovers_the_true_distance(b_deg):
    cm = fish()
    r_true, real = 1.6, 0.40
    b = math.radians(b_deg)
    top = cm.project_point(r_true * math.cos(b), r_true * math.sin(b), real / 2)
    bot = cm.project_point(r_true * math.cos(b), r_true * math.sin(b), -real / 2)
    assert top and bot
    h_px = bot[1] - top[1]
    cx_px, cy_px = 0.5 * (top[0] + bot[0]), 0.5 * (top[1] + bot[1])
    det = Detection("TANK", 0.9, (cx_px - 20, top[1], cx_px + 20, bot[1]), 0.0)
    r, how = estimate_range(cm, det, real, None, 0.0)
    assert how == "size" and r == pytest.approx(r_true, rel=0.02)


def test_estimate_range_passes_v_to_the_model():
    cm = fish()
    det = Detection("TANK", 0.9, (500, 40, 540, 100), 0.0)
    r1, _ = estimate_range(cm, det, 0.4, None, 0.0)
    expected_bearing = cm.bearing(det.cx, det.cy)
    assert math.isfinite(r1) and expected_bearing != cm.bearing(det.cx)


# ====================================================================== 커버리지
def test_coverage_fov_follows_usable_hfov_and_model():
    cfg = Config()
    cfg.camera = fisheye_cfg(usable_hfov_deg=110.0)
    assert CoverageMap(cfg).fov_half == pytest.approx(math.radians(55.0))
    cfg2 = Config()
    assert CoverageMap(cfg2).fov_half == math.atan2(320.0, CameraModel(cfg2.camera).f)       # 기본은 지금과 같다


def test_project_point_fisheye_returns_none_behind_the_camera_and_matches_project():
    cm = fish()
    assert cm.theta_max < math.pi and cm.project_point(-1.0, 0.0, 0.0) is None        # 정반대 방향은 모델 영역 밖
    f, l, u = 2.0, 0.8, -0.1
    b, e = math.atan2(l, f), math.atan2(u, math.hypot(f, l))
    assert cm.project_point(f, l, u) == pytest.approx(cm.project(b, e), abs=1e-9)


# ====================================================================== 성능
def test_project_10000_calls_under_50ms_and_angles_1000_calls_under_50ms():
    cm = fish()
    t0 = time.perf_counter()
    for i in range(10000):
        cm.project(0.3 + (i % 7) * 0.01, 0.1)
    t_project = time.perf_counter() - t0
    t0 = time.perf_counter()
    for i in range(1000):
        cm.angles(500.0 + (i % 9), 100.0)
    t_angles = time.perf_counter() - t0
    assert t_project < 0.050 and t_angles < 0.050, (t_project, t_angles)


# ====================================================================== 보정 결과 → 설정 (calib/generate.py)
def profile_with(width=640, height=480, K=None, D=None, model="fisheye", usable=None):
    p = Profile()
    if K is None:
        K = [170.0 * width / 640, 0.0, 320.0 * width / 640, 0.0, 168.0 * height / 480, 243.0 * height / 480, 0.0, 0.0, 1.0]
    p.set("sensors.camera.model", model, "measured")
    p.set("sensors.camera.K", K, "measured")
    p.set("sensors.camera.D", list(D if D is not None else D_FISH), "measured")
    p.set("sensors.camera.width", width, "measured")
    p.set("sensors.camera.height", height, "measured")
    p.set("sensors.camera.calibrated", True, "measured")
    if usable is not None:
        p.set("sensors.camera.usable_hfov_deg", usable, "measured")
    return p


def load_cfg(tmp_path, overlay):
    f = tmp_path / "ov.json"
    f.write_text(json.dumps(overlay))
    return Config.load(str(f))


def test_uncalibrated_profile_overlay_is_unchanged():
    p = Profile()
    cam = gen_config_overlay(p)["camera"]
    assert set(cam) == {"width", "height", "hfov_deg"}


def test_calibrated_pinhole_profile_writes_k_only():
    p = Profile()
    p.set("sensors.camera.K", [300.0, 0.0, 321.0, 0.0, 301.0, 241.0, 0.0, 0.0, 1.0], "measured")
    p.set("sensors.camera.calibrated", True, "measured")
    cam = gen_config_overlay(p)["camera"]
    assert cam["K"] == [300.0, 0.0, 321.0, 0.0, 301.0, 241.0, 0.0, 0.0, 1.0] and "model" not in cam and "D" not in cam


def test_fisheye_profile_fills_model_k_and_d_and_the_runtime_uses_them(tmp_path):
    cam = gen_config_overlay(profile_with(usable=120.0))["camera"]
    assert cam["model"] == "fisheye" and cam["D"] == list(D_FISH) and cam["K"][0] == 170.0 and cam["usable_hfov_deg"] == 120.0
    cfg = load_cfg(tmp_path, {"camera": cam})
    cm = CameraModel(cfg.camera)
    assert cm.model == "fisheye" and cm.D == D_FISH and cm.half_fov_h() == pytest.approx(math.radians(60.0))


def test_other_calibration_size_with_same_aspect_scales_k_proportionally(tmp_path):
    p = profile_with(width=1280, height=960)
    with warnings.catch_warnings():
        warnings.simplefilter("error")                                            # 비례 조정은 경고가 아니다
        cam = gen_config_overlay(p)["camera"]
    assert (cam["width"], cam["height"]) == RUNTIME_IMAGE_SIZE
    assert cam["K"] == pytest.approx([170.0, 0.0, 320.0, 0.0, 168.0, 243.0, 0.0, 0.0, 1.0])
    big = CameraModel(CameraCfg(width=1280, height=960, K=tuple(p.get("sensors.camera.K")), model="fisheye", D=D_FISH))
    small = CameraModel(load_cfg(tmp_path, {"camera": cam}).camera)
    for b, e in ((0.3, 0.1), (0.9, -0.2), (-1.1, 0.4)):
        ub, vb = big.project(b, e)
        us, vs = small.project(b, e)
        assert (us, vs) == pytest.approx((ub / 2, vb / 2), abs=1e-9)             # 같은 방향이 절반 크기 영상의 절반 위치에 찍힌다


def test_calibration_with_a_different_aspect_ratio_is_rejected_with_a_warning(tmp_path):
    p = profile_with(width=1280, height=720, K=[330.0, 0.0, 640.0, 0.0, 330.0, 360.0, 0.0, 0.0, 1.0])
    with pytest.warns(UserWarning, match="비례 조정"):
        cam = gen_config_overlay(p)["camera"]
    assert (cam["width"], cam["height"]) == RUNTIME_IMAGE_SIZE and "K" not in cam and "model" not in cam and "D" not in cam
    cm = CameraModel(load_cfg(tmp_path, {"camera": cam}).camera)
    assert cm.model == "pinhole"                                                  # hfov 핀홀로 둔다


def test_fisheye_profile_with_wrong_d_length_is_written_as_pinhole_k_only():
    cam = gen_config_overlay(profile_with(D=(0.1, 0.0, 0.0, 0.0, 0.0)))["camera"]
    assert "model" not in cam and "D" not in cam and "K" in cam


def test_apply_to_profile_then_overlay_end_to_end(tmp_path):
    from maicon26.calib import camera as calib_camera
    res = {"model": "fisheye", "K": list(K_FISH), "D": list(D_FISH), "rms_px": 0.2, "per_view_rms_px": [0.2], "size": [640, 480], "n_views": 12}
    p = Profile()
    calib_camera.apply_to_profile(p, res, "test")
    cfg = load_cfg(tmp_path, gen_config_overlay(p))
    assert CameraModel(cfg.camera).model == "fisheye"


def test_real_robot_config_and_simulation_profile_stay_pinhole():
    real = Config.load(str(ROOT / "config" / "waveshare_real.json"))
    assert real.camera.model == "pinhole" and real.camera.D is None and CameraModel(real.camera).model == "pinhole"
    assert Config().camera.model == "pinhole"
    from maicon26.hal.wingbot_calibration import configure_wingbot_simulation
    cfg = Config()
    configure_wingbot_simulation(cfg, ROOT / "ros2/wingbot_description/config/frames.yaml")
    assert cfg.camera.model == "pinhole" and CameraModel(cfg.camera).model == "pinhole"        # 시뮬 계약 프로파일은 핀홀 카메라다(Fortress 가 왜곡을 안 만든다)


def test_default_camera_config_has_the_three_new_keys():
    c = CameraCfg()
    assert (c.model, c.D, c.usable_hfov_deg) == ("pinhole", None, None)
