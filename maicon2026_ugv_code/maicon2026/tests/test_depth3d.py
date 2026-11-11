"""깊이 카메라 3D 지도: 기하(TF·보정값) → 복셀 지도 → 시뮬 깊이 → 임무 사용(항법·사거리·사선·높이) → 실기 연결부.

시뮬 깊이는 광선으로 만든 '진짜 기하' + 스테레오 잡음 모델(시뮬 가정)이다. 실기 깊이 영상으로는 시험하지 못했다.
"""
import math
import os
import tempfile
import warnings

import numpy as np
import pytest

from maicon26.calib.depth import apply_to_profile, calibrate_mount_from_floor, median_depth
from maicon26.calib.generate import depth_intrinsics, gen_config_overlay, gen_depth_camera_info
from maicon26.calib.profile import Profile
from maicon26.config import Config, _merge
from maicon26.geom3d import Intrinsics, RigModel, backproject
from maicon26.hal.depth_io import camera_info_K, decode_depth_image
from maicon26.mission.aim import AimController
from maicon26.perception.camera import CameraModel
from maicon26.perception.depth_fusion import DepthFusion
from maicon26.perception.voxel_map import VoxelMap3D
from maicon26.sim.robot import SimRobot
from maicon26.sim.run import build_match, run, score
from maicon26.sim.world import World
from maicon26.types import Detection, Pose2D

warnings.filterwarnings("ignore", category=RuntimeWarning)


def _clean(cfg: Config) -> Config:
    dc = cfg.depth
    dc.sim_noise_k, dc.sim_dropout, dc.sim_flying_pixels = 0.0, 0.0, 0.0
    return cfg


# ====================================================================== 1. 기하 / 보정값
def test_rig_axes_and_urdf_numbers():
    rig = RigModel(Config().rig)
    T = rig.optical_in_world(0, 0, 0, 0, 0, "depth")
    assert np.allclose(T.t, [0.10, 0.0, 0.14])                    # 깊이: base_link 기준 (0.10, 0, 0.10) + base_z 0.04 — 팬/틸트 아래 고정
    assert np.allclose(T.R[:, 2], [1, 0, 0])                      # 광축(z_opt) = 앞
    assert np.allclose(T.R[:, 0], [0, -1, 0])                     # x_opt = 오른쪽
    assert np.allclose(T.R[:, 1], [0, 0, -1])                     # y_opt = 아래
    Tc = rig.optical_in_world(0, 0, 0, 0, 0, "camera")
    assert np.allclose(Tc.t, [0.05, 0.0, 0.27])                   # URDF: camera_link = tilt_link + (0.05, 0, 0.03)


def test_pan_left_tilt_up_and_yaw():
    rig = RigModel(Config().rig)
    p, t = math.radians(30), math.radians(10)
    T = rig.optical_in_world(0, 0, 0, 30, 10, "camera")             # RGB 카메라는 팬/틸트 위: 왼쪽(+y)·위(+z)를 본다
    assert np.allclose(T.R[:, 2], [math.cos(t) * math.cos(p), math.cos(t) * math.sin(p), math.sin(t)], atol=1e-9)
    ox, oz = 0.05, 0.03                                             # tilt_link 기준 카메라 위치
    assert np.allclose(T.t, [(ox * math.cos(t) - oz * math.sin(t)) * math.cos(p), (ox * math.cos(t) - oz * math.sin(t)) * math.sin(p),
                             0.24 + ox * math.sin(t) + oz * math.cos(t)], atol=1e-9)                                 # 팬·틸트 축 둘레로 돈다
    D0 = rig.optical_in_world(0, 0, 0, 0, 0, "depth")
    D1 = rig.optical_in_world(0, 0, 0, 30, 10, "depth")             # 깊이 카메라는 팬/틸트 아래 고정 → 팬/틸트와 무관
    assert np.allclose(D0.R, D1.R) and np.allclose(D0.t, D1.t)
    T2 = rig.optical_in_world(1.0, 2.0, math.pi / 2, 0, 0, "depth")
    assert np.allclose(T2.R[:, 2], [0, 1, 0], atol=1e-9) and np.allclose(T2.t, [1.0, 2.10, 0.14], atol=1e-9)        # 차체가 +y 를 보면 센서도


def test_intrinsics_from_K_scaled_and_roundtrip():
    I = Intrinsics.from_K([300, 0, 330, 0, 310, 200, 0, 0, 1], 640, 400)
    s = I.scaled(320, 200)
    assert (s.fx, s.fy, s.cx, s.cy) == (150, 155, 165, 100)
    p = np.array([[0.3, -0.2, 1.7]])
    u, v, z = I.project(p)
    r = I.rays(u, v)[0] * z[0]
    assert np.allclose(r, p[0], atol=1e-9)
    with pytest.raises(ValueError):
        Intrinsics.from_K([0, 0, 0, 0, 0, 0, 0, 0, 1], 640, 400)


def test_depth_cfg_uses_K_and_hfov_fallback():
    c = Config()
    assert abs(c.depth.intrinsics(640, 400).hfov_deg - c.depth.hfov_deg) < 1e-6
    c.depth.K, c.depth.K_width, c.depth.K_height = (500, 0, 330, 0, 505, 190, 0, 0, 1), 640, 400
    assert c.depth.intrinsics(640, 400).cx == 330
    s = c.depth.intrinsics(96, 60)
    assert abs(s.fx - 500 * 96 / 640) < 1e-9 and abs(s.cy - 190 * 60 / 400) < 1e-9


def test_rgb_camera_calibration_K_is_used_for_bearing_and_aim():
    cfg = Config()
    base = CameraModel(cfg.camera)
    assert abs(base.bearing(320)) < 1e-9
    cfg.camera.K = (400, 0, 300, 0, 380, 260, 0, 0, 1)            # 주점이 영상 중심이 아님
    cm = CameraModel(cfg.camera)
    assert abs(cm.bearing(300)) < 1e-9 and abs(cm.elevation(260)) < 1e-9 and cm.bearing(320) < 0
    ac = AimController(cfg)
    det = Detection("RADAR_FACILITY", 0.9, (280.0, 200.0, 330.0, 300.0), extra={"sensors": [(300.0, 260.0, 5.0, 0.9)]})
    _, _, res = ac.update(det, 0.0, 0.0, 0.0, 1.2)
    assert abs(res.err_pan) < 0.05 and abs(res.err_tilt) < 0.05    # 주점 위의 센서는 이미 '정중앙' → 보정할 오차 없음
    cfg2 = Config()                                               # K 를 무시하던 때(hfov, 중심 320x240)였다면 1.5°+ 의 오차가 남는다
    _, _, res2 = AimController(cfg2).update(det, 0.0, 0.0, 0.0, 1.2)
    assert abs(res2.err_pan) > 1.0


def test_config_overlay_roundtrip_with_rig_and_depth():
    p = Profile()
    p.set("sensors.depth.K", [520.0, 0, 322.0, 0, 521.0, 201.0, 0, 0, 1], "measured")
    p.set("sensors.camera.K", [410.0, 0, 318.0, 0, 409.0, 242.0, 0, 0, 1], "measured")
    p.set("sensors.camera.calibrated", True, "measured")
    ov = gen_config_overlay(p)
    cfg = Config()
    _merge(cfg, ov)                                               # 알 수 없는 키가 있으면 KeyError
    assert cfg.rig.base_z == p.get("model.wheels.radius") and tuple(cfg.rig.pan_xyz) == tuple(p.get("model.pan_tilt.xyz"))
    assert cfg.depth.K[0] == 520.0 and cfg.depth.K_width == 640 and cfg.camera.K[2] == 318.0
    di = depth_intrinsics(p)
    assert di["calibrated"] and gen_depth_camera_info(p)["camera_matrix"]["data"][0] == 520.0
    assert not depth_intrinsics(Profile())["calibrated"]          # 기본은 hfov 근사 = 미보정


# ====================================================================== 2. 복셀 지도 (합성)
def _wall_points(x=2.0, y0=2.2, y1=2.8, z0=0.08, z1=0.38, step=0.02):
    ys, zs = np.meshgrid(np.arange(y0, y1, step), np.arange(z0, z1, step))
    return np.stack([np.full(ys.size, x), ys.ravel(), zs.ravel()], axis=1)


def test_voxel_integrate_free_occupied_floor():
    cfg = Config()
    vm = VoxelMap3D(cfg.arena, cfg.map3d)
    org = np.array([1.0, 2.5, 0.24])
    wall = _wall_points()
    floor = np.stack([np.full(40, 1.6), np.linspace(2.3, 2.7, 40), np.full(40, 0.01)], axis=1)
    for _ in range(3):
        vm.integrate(org, np.vstack([wall, floor]))
    occ = vm.occupied()
    i = lambda x, y, z: (int(z / vm.res), int((y - vm.oy) / vm.res), int((x - vm.ox) / vm.res))
    assert occ[i(2.0, 2.5, 0.2)] and occ[i(2.0, 2.5, 0.3)]         # 벽 점유
    assert not occ[i(1.5, 2.5, 0.24)] and vm.seen[i(1.5, 2.5, 0.24)]   # 사이 공간: 비움으로 '본' 것
    assert not occ[i(1.6, 2.5, 0.0)]                               # 바닥 점은 점유가 아니다
    assert not vm.seen[i(2.4, 2.5, 0.24)]                          # 벽 뒤는 본 적 없다


def test_voxel_queries_segment_ray_clearance():
    cfg = Config()
    vm = VoxelMap3D(cfg.arena, cfg.map3d)
    org = np.array([1.0, 2.5, 0.24])
    for _ in range(3):
        vm.integrate(org, _wall_points())
    assert vm.segment_blocked(org, [3.0, 2.5, 0.24], min_neighbors=3) >= 1     # 한 겹짜리 면도 막는다(면 복셀은 이웃이 4개)
    assert vm.segment_blocked(org, [1.8, 2.5, 0.24]) == 0          # 벽 앞에서 끝나는 선
    assert vm.segment_blocked(org, [2.03, 2.5, 0.24]) == 0         # 벽 '자신'의 면에 닿는 사선은 막힘이 아니다(끝 10 cm 제외)
    h = vm.ray_first_hit(org, [1, 0, 0], 3.0)
    assert h is not None and abs(h - 1.0) < 0.08
    assert abs(vm.forward_clearance(1.0, 2.5, 0.0, 0.15, 0.06, 0.34, 3.0) - 1.0) < 0.1
    assert vm.forward_clearance(1.0, 2.5, math.pi, 0.15, 0.06, 0.34, 3.0) == 3.0   # 뒤쪽은 비어 있음
    # 광선 방향으로 늘어선 한 줄(스테레오 허상)은 막힘으로 세지 않는다
    for k in range(6):
        vm.L[int(0.24 / vm.res), int((2.5 - vm.oy) / vm.res), int((1.3 + 0.05 * k - vm.ox) / vm.res)] = 4.0
    assert vm.segment_blocked(org, [1.9, 2.5, 0.24]) >= 1                         # 이웃 조건이 없으면 한 줄도 막힘으로 센다
    assert vm.segment_blocked(org, [1.9, 2.5, 0.24], min_neighbors=3) == 0        # 이웃 3개 조건이면 허상 한 줄은 걸러진다


def test_blind_columns_keep_low_block_drop_box_and_floating_ghost():
    cfg = Config()
    vm = VoxelMap3D(cfg.arena, cfg.map3d)

    def fill(x0, y0, x1, y1, z0, z1):
        a = lambda v, o: int((v - o) / vm.res)
        vm.L[int(z0 / vm.res):int(z1 / vm.res), a(y0, vm.oy):a(y1, vm.oy), a(x0, vm.ox):a(x1, vm.ox)] = 4.0
    fill(2.0, 2.0, 2.4, 2.4, 0.0, 0.40)              # 박스 40 cm: LiDAR 가 보는 것
    fill(3.0, 3.0, 3.2, 3.2, 0.05, 0.15)             # 낮은 블록 12 cm: LiDAR 가 못 보는 것
    fill(4.0, 1.0, 4.4, 1.4, 0.30, 0.40)             # 박스 윗부분만 남은 잔재(바닥과 떨어짐) — 구조물이 바뀐 뒤 흔히 남는다
    fill(1.0, 1.0, 1.2, 1.2, 0.10, 0.20)             # 박스 옆 15 cm 안의 낮은 번짐은 LiDAR 영역으로 본다
    layer = vm.lidar_blind_columns(0.06, 0.16, 0.24, 0.05, 1.8, 3, 1.0, 3)
    cell = lambda x, y: layer[int((y - vm.oy) / vm.res), int((x - vm.ox) / vm.res)]
    assert cell(3.1, 3.1) and not cell(2.2, 2.2) and not cell(4.2, 1.2)
    n_block = int(layer[int((3.0 - vm.oy) / vm.res):int((3.2 - vm.oy) / vm.res), int((3.0 - vm.ox) / vm.res):int((3.2 - vm.ox) / vm.res)].sum())
    assert n_block >= 12 and int(layer.sum()) == n_block + int(layer[int((1.0 - vm.oy) / vm.res):int((1.2 - vm.oy) / vm.res),
                                                                      int((1.0 - vm.ox) / vm.res):int((1.2 - vm.ox) / vm.res)].sum())


def test_export_ply_and_npz():
    cfg = Config()
    vm = VoxelMap3D(cfg.arena, cfg.map3d)
    org = np.array([1.0, 2.5, 0.24])
    for _ in range(3):
        vm.integrate(org, _wall_points())
    with tempfile.TemporaryDirectory() as d:
        n = vm.save_ply(os.path.join(d, "m.ply"))
        txt = open(os.path.join(d, "m.ply"), encoding="utf-8").read().splitlines()
        assert n == int(vm.occupied().sum()) > 50 and f"element vertex {n}" in txt and len(txt) == n + 10
        vm.save_npz(os.path.join(d, "m.npz"))
        with np.load(os.path.join(d, "m.npz")) as z:                # 닫아야 윈도우에서 임시 폴더가 지워진다
            assert z["L"].shape == vm.L.shape and float(z["res"]) == vm.res


# ====================================================================== 3. 시뮬 깊이 ↔ 지도 정합
def test_depth_render_geometry_fixed_to_body_not_pan_tilt():
    cfg = _clean(Config())
    w = World(_nov(cfg), seed=3)
    w.cubes.clear()
    u = w.add_ugv("BLUE", 2)
    u.pose.x, u.pose.y, u.pose.th = 1.5, 2.5, 0.0
    w.add_blank_cube(2.7, 2.5)                                     # 앞쪽 면 x = 2.5
    dc = cfg.depth
    img = w.depth_image(u, dc)
    assert abs(img[dc.height // 2, dc.width // 2] - 0.90) < 0.005   # 센서 x = 1.5 + 0.10
    u.pan, u.tilt = 30.0, 12.0                                      # 팬/틸트 아래 고정이라 영상이 그대로여야 한다
    assert np.array_equal(w.depth_image(u, dc), img)
    u.pose.th = math.radians(20)                                    # 차체가 돌면 따라 돈다: 광축 20° 왼쪽 → x = 2.5 면까지 (2.5 - ox)/cos20
    w.add_blank_cube(2.7, 3.0)                                      # 그 광축이 닿는 곳(y ≈ 2.5 + 0.9·tan20 ≈ 2.9..3.0)
    ox = 1.5 + 0.10 * math.cos(math.radians(20))
    img2 = w.depth_image(u, dc)
    assert abs(img2[dc.height // 2, dc.width // 2] - (2.5 - ox) / math.cos(math.radians(20))) < 0.01


def test_registered_points_land_on_true_surfaces():
    """깊이 → 월드 점(알려진 자세·팬·틸트)이 정말 경기장 표면(바닥·벽·박스) 위에 놓이는가. 팬/틸트가 있을 때도."""
    cfg = _clean(Config())
    w = World(_nov(cfg), seed=3)
    u = w.add_ugv("BLUE", 2)
    u.pose.x, u.pose.y, u.pose.th = 1.7, 2.4, 0.4
    u.pan, u.tilt = -28.0, 9.0
    dc = cfg.depth
    img = w.depth_image(u, dc)
    T = RigModel(cfg.rig).optical_in_world(u.pose.x, u.pose.y, u.pose.th, u.pan, u.tilt, "depth")
    pts, _ = backproject(img, dc.intrinsics(), T, 1, 0.1, 7.5)
    g = w.geo
    def surf_err(p):
        e = [abs(p[2])]                                              # 바닥
        e += [abs(p[0]), abs(p[0] - g.W), abs(p[1]), abs(p[1] - g.H)] if p[2] <= g.wall_h else []   # 외벽
        for c in w.cubes:                                            # 박스 표면까지 거리
            x0, y0, x1, y1 = c.rect()
            dx = max(x0 - p[0], 0, p[0] - x1)
            dy = max(y0 - p[1], 0, p[1] - y1)
            dz = max(0 - p[2], 0, p[2] - g.box_size)
            e.append(math.sqrt(dx * dx + dy * dy + dz * dz))
        return min(e)
    errs = np.array([surf_err(p) for p in pts[::7]])
    assert len(pts) > 1500 and np.percentile(errs, 99) < 0.02       # 99 % 가 2 cm 안(이산화·가장자리 포함)


def test_map_from_spin_recovers_boxes_without_ghosts():
    cfg = Config()
    w = World(_nov(cfg), seed=3)
    u = w.add_ugv("BLUE", 2)
    r = SimRobot(w, u)
    fu = DepthFusion(cfg)
    u.pose.x, u.pose.y = 1.4, 2.5
    for k in range(400):                                           # 참 자세(SLAM 오차 없음)로 제자리에서 몸체를 돌린다(깊이 카메라는 전방 고정)
        w.t = k * 0.2
        u.pose.th = 2 * math.pi * (k / 80.0)
        fu.ingest(r.get_depth(), w.t, u.pose)
    truth = [(c.cx, c.cy) for c in w.cubes]
    full = [c for c in fu.map.box_clusters() if c["w"] >= 0.35 and c["d"] >= 0.35]       # 4면이 다 보인 박스
    assert len(full) >= 3
    for c in full:
        d = min(math.hypot(c["x"] - tx, c["y"] - ty) for tx, ty in truth)
        assert d < 0.08 and 0.30 <= c["height"] <= 0.50, c          # 중심 8 cm 안, 높이 약 40 cm
    assert all(min(math.hypot(c["x"] - tx, c["y"] - ty) for tx, ty in truth) < 0.30 for c in fu.map.box_clusters())   # 유령 덩어리 없음
    assert fu.report()["coverage"] > 0.3


def test_locate_pixel_accuracy_and_occlusion():
    cfg = _clean(Config())
    w = World(_nov(cfg), seed=3)
    w.cubes.clear()
    u = w.add_ugv("BLUE", 2)
    r = SimRobot(w, u)
    u.pose.x, u.pose.y, u.pose.th = 1.2, 2.5, 0.0
    w.add_blank_cube(2.6, 2.5)                                     # 앞면 x = 2.4
    fu = DepthFusion(cfg)
    w.t = 1.0
    fu.ingest(r.get_depth(), w.t, u.pose)
    # 표적 점(앞면 위, 높이 0.33)을 RGB 카메라(리그 위치)에서 본 픽셀
    tgt = np.array([2.4, 2.58, 0.33])
    T = RigModel(cfg.rig).optical_in_world(u.pose.x, u.pose.y, u.pose.th, 0, 0, "camera")
    pc = T.inv().apply(tgt)
    cm = fu.rgb
    uu, vv = cm.cx + cm.f * pc[0] / pc[2], cm.cy + cm.fy * pc[1] / pc[2]
    pt = fu.locate_pixel(uu, vv, w.t, u.pose, 0.0, 0.0)
    assert pt is not None and np.linalg.norm(pt - tgt) < 0.03
    # 깊이와 RGB 의 시각이 크게 어긋나면(0.25 s 초과) 쓰지 않는다
    assert fu.locate_pixel(uu, vv, w.t + 0.6, u.pose, 0.0, 0.0) is None
    # 가림: 표적 앞(1.8)에 선반(높이 0.30~0.42)을 두면 같은 픽셀이 선반 면을 가리킨다
    w.add_low_obstacle(1.75, 2.3, 1.85, 2.9, 0.42, z0=0.30)
    w.t = 2.0
    fu.ingest(r.get_depth(), w.t, u.pose)
    pt2 = fu.locate_pixel(uu, vv, w.t, u.pose, 0.0, 0.0)
    assert pt2 is not None and pt2[0] < 1.85 + 0.05


# ====================================================================== 4. 임무에서의 사용
def test_low_obstacle_invisible_to_lidar_enters_nav_layer_and_guard():
    cfg = Config()
    w = World(_nov(cfg), seed=3)
    w.cubes.clear()
    u = w.add_ugv("BLUE", 2)
    r = SimRobot(w, u)
    u.pose.x, u.pose.y, u.pose.th = 1.2, 2.55, 0.0
    w.add_low_obstacle(2.2, 2.35, 2.5, 2.75, 0.12)                 # 12 cm 낮은 블록
    sc = w.lidar(u)
    assert sc.min_in_sector(0.0, math.radians(10)) > 3.5           # LiDAR 는 블록을 못 본다(뒤쪽 벽만)
    fu = DepthFusion(cfg)
    for k in range(60):                                            # 깊이 카메라는 전방 고정 → 차체를 좌우로 흔들어 본다
        w.t = k * 0.2
        u.pose.th = 0.4 * math.sin(0.9 * w.t)
        fu.ingest(r.get_depth(), w.t, u.pose, sc, u.pose)
    u.pose.th = 0.0
    layer = fu.nav_layer()
    ys, xs = np.nonzero(layer)
    cx, cy = fu.map.ox + (xs + 0.5) * fu.map.res, fu.map.oy + (ys + 0.5) * fu.map.res
    assert len(xs) >= 4
    assert np.all((cx > 2.1) & (cx < 2.6) & (cy > 2.25) & (cy < 2.85))     # 블록 둘레에만, 다른 곳엔 허상 없음
    assert abs(fu.front_clearance(u.pose, 2.0) - 1.0) < 0.12       # 블록 앞면(2.2)까지 약 1.0 m


def test_shelf_blocks_laser_beam_but_not_lidar():
    cfg = Config()
    w = World(_nov(cfg), seed=3)
    w.cubes.clear()
    u = w.add_ugv("BLUE", 2)
    r = SimRobot(w, u)
    u.pose.x, u.pose.y, u.pose.th = 1.2, 2.5, 0.0
    w.add_low_obstacle(2.0, 2.3, 2.2, 2.7, 0.42, z0=0.30)          # 공중 선반(LiDAR 평면 위)
    sc = w.lidar(u)
    assert sc.min_in_sector(0.0, math.radians(10)) > 3.5
    fu = DepthFusion(cfg)
    for k in range(40):
        w.t = k * 0.2
        u.pose.th = 0.3 * math.sin(0.9 * w.t)
        fu.ingest(r.get_depth(), w.t, u.pose, sc, u.pose)
    u.pose.th = 0.0
    org = fu.laser_origin(u.pose, 0.0, 0.0, 0.0, 0.03)             # 높이 ≈ 0.30
    assert fu.beam_blocked(org, np.array([3.5, 2.5, 0.33])) >= cfg.map3d.beam_min_blocked
    assert fu.beam_blocked(org, np.array([3.5, 3.6, 0.33])) == 0   # 선반을 비켜 가는 사선은 열려 있다
    assert fu.beam_blocked(org, np.array([1.9, 2.5, 0.33])) == 0   # 선반 앞에서 끝나는 선


def _nov(cfg):
    """이 시험들은 특정 시드의 박스 배치(깊이 지도 복원·접촉 감지)를 가정한다 — 경기장 유효성 재생성으로 배치가 바뀌지 않게 끈다."""
    cfg.arena.validate_layout = False
    return cfg


def _solo(cfg_mut=None, seed=2, t=12.0):
    cfg = Config()
    cfg.nav.backend = "builtin"
    cfg.mission.spin_w, cfg.mission.tour_nx, cfg.mission.tour_ny = 1.8, 3, 3
    if cfg_mut:
        cfg_mut(cfg)
    m = build_match(seed=seed, cfg=_nov(cfg), teams=["B2"])
    run(m, max_t=t)
    return m, m.agents[0]


def test_agent_range_uses_depth_only_when_lidar_cannot():
    m, a = _solo()
    pose = a.stack.pose
    th = pose.th
    pt = np.array([pose.x + 1.3 * math.cos(th), pose.y + 1.3 * math.sin(th), 0.33])
    a._det_ctx = (a.robot.now(), pose, 0.0, 0.0)
    a.fusion.locate_pixel = lambda *args, **kw: pt
    det = Detection("RADAR_FACILITY", 0.9, (300.0, 150.0, 340.0, 230.0))
    a._scan = None                                                 # LiDAR 없음 → 크기 기반 추정으로 떨어짐
    n0 = a.stats["range_from_depth"]                               # 12 s 동안 사격 접근에서 이미 쓰였을 수 있다(탐색 방식에 따라)
    r, how = a._range(det, 0.40, 0.0)
    assert how == "depth" and abs(r - 1.3) < 1e-6 and a.stats["range_from_depth"] == n0 + 1
    a.cfg.map3d.use_range = False
    assert a._range(det, 0.40, 0.0)[1] == "size"                   # 끄면 기존 동작
    a.cfg.map3d.use_range = True
    a._scan = m.robots[0].get_scan()
    calls = []
    a.fusion.locate_pixel = lambda *args, **kw: calls.append(1) or None            # 깊이 호출을 기록만 한다
    cm = a.cam
    wall_det = Detection("RADAR_FACILITY", 0.9, (cm.cx - 20, cm.cy - 40, cm.cx + 20, cm.cy + 40))
    how = a._range(wall_det, 0.40, 0.0)[1]
    if how == "lidar":                                                # LiDAR 가 거리를 줬다면 깊이는 불리지 않아야 한다(탐색 방식에 따라 정면에 반사가 없을 수도 있어 조건부)
        assert not calls

def test_shot_geometry_height_check_and_beam_timeout():
    m, a = _solo()
    pose = a.stack.pose
    a._det_ctx = (a.robot.now(), pose, 0.0, 0.0)
    det = Detection("RADAR_FACILITY", 0.9, (280.0, 200.0, 360.0, 280.0), extra={"sensors": [(320.0, 240.0, 5.0, 0.9)]})
    mc = a.cfg.map3d
    mk = lambda z: np.array([pose.x + 1.2, pose.y, z])
    a.fusion.beam_blocked = lambda *args: 0
    a.fusion.locate_pixel = lambda *args, **kw: mk(0.33)
    assert a._shot_geometry_ok(det, 10.0, 0.0, 0.0)                 # 흰 원판 높이(0.33): 통과
    a.fusion.locate_pixel = lambda *args, **kw: mk(0.90)
    assert not a._shot_geometry_ok(det, 10.0, 0.0, 0.0) and a.stats["height_reject"] == 1     # 0.9 m 높이의 '하얀 점' = 오검출
    a.fusion.locate_pixel = lambda *args, **kw: None
    assert a._shot_geometry_ok(det, 10.0, 0.0, 0.0)                 # 깊이로 못 재면 '나쁨'이 아니라 '모름' → 통과
    a.fusion.locate_pixel = lambda *args, **kw: mk(0.33)
    a.fusion.beam_blocked = lambda *args: 5                         # 사선 막힘
    t0 = 20.0
    assert not a._shot_geometry_ok(det, t0, 0.0, 0.0)
    assert not a._shot_geometry_ok(det, t0 + mc.beam_timeout_s - 0.1, 0.0, 0.0)
    assert a._shot_geometry_ok(det, t0 + mc.beam_timeout_s + 0.1, 0.0, 0.0) and a.stats["beam_override"] == 1   # 계속 막힘 = 허상으로 보고 통과
    a.fusion.beam_blocked = lambda *args: 0
    assert a._shot_geometry_ok(det, t0 + 10, 0.0, 0.0) and a._beam_since is None                           # 열리면 초기화


def test_depth_guard_slows_and_stops():
    m, a = _solo()
    mc = a.cfg.map3d
    a.fusion.front_clearance = lambda pose, max_d=1.0: 1.0
    assert a._depth_guard(0.3) == 0.3
    a.fusion.front_clearance = lambda pose, max_d=1.0: 0.5 * (mc.guard_dist + mc.guard_slow)
    assert 0.1 < a._depth_guard(0.3) < 0.2                          # 중간: 비례 감속
    a.fusion.front_clearance = lambda pose, max_d=1.0: mc.guard_dist - 0.01
    assert a._depth_guard(0.3) == 0.0 and a.stats["depth_guard_ticks"] >= 2


def test_agent_works_without_depth_and_with_depth_disabled():
    m, a = _solo(lambda c: setattr(c.depth, "enable", False))
    assert a.fusion is None and a.stats["depth_frames"] == 0 and a.map3d_report() == {}
    m, a = _solo(lambda c: setattr(c.depth, "sim_hz", 0.0))        # 센서가 없는 로봇(get_depth → None)
    assert a.fusion is not None and a.stats["depth_frames"] == 0 and a.stack.grid.layer3d is None


def _barrier_trial(depth_on, backend="builtin", t=120.0):
    cfg = Config()
    cfg.nav.backend = backend
    cfg.mission.spin_w, cfg.mission.tour_nx, cfg.mission.tour_ny = 1.8, 3, 3
    cfg.depth.enable = depth_on
    cfg.map3d.bump_enable = False                                  # 깊이 3D 층의 기여만 본다(접촉 감지 단독 효과는 아래 별도 시험)
    m = build_match(seed=2, cfg=_nov(cfg), teams=["B2"])
    m.world.add_low_obstacle(1.55, 0.55, 1.75, 4.45, 0.12)         # BLUE 시작과 중앙 사이를 가르는 12 cm 낮은 장벽(위·아래 0.55 m 틈)
    run(m, max_t=t)
    v = score(m)["agents"]["B-UGV-2"]
    return v, m.agents[0]


@pytest.mark.parametrize("backend", ["builtin", "nav2-fake"])
def test_low_barrier_is_avoided_only_with_depth(backend):
    """LiDAR 에 안 보이는 낮은 장벽: 깊이를 끄면 계속 들이받고, 켜면 돌아간다."""
    off, _ = _barrier_trial(False, backend)
    on, a = _barrier_trial(True, backend)
    # 끄면 장벽에 계속 부딪힘(충돌 판정 틱 수: builtin 810, nav2-fake 505). 켜도 0 은 아니다 — 깊이 카메라는 전방 고정 시야·최소 거리 0.30 m 라
    # 장벽 가장자리를 비스듬히 스칠 때는 못 본다(builtin 82, nav2-fake 0).
    # 직사각형 충돌 모델로 바꾼 뒤 끈 쪽 충돌 틱이 예전(810/505)보다 크게 줄었다(정지 0.192 m·복구 방향 등) — 절대값이 아니라 상대 비교로 본다
    assert off["collisions"] > 40, off
    assert on["collisions"] * 2 < off["collisions"], (on, off)
    assert on["distance"] > off["distance"]                         # 켜면 그만큼 더 멀리 간다(들이받으며 제자리걸음하지 않음)
    assert a.stats["nav_layer_cells"] > 0


# ====================================================================== 4b. 접촉(정체) 감지
class _Frozen:
    """stack 을 감싸 pose 만 고정한다(제자리에서 막힌 로봇을 흉내)."""
    def __init__(self, st, pose):
        self._st, self.pose = st, pose

    def __getattr__(self, k):
        return getattr(self._st, k)


def _frozen_agent(scan_range):
    from maicon26.mission.manager import Mode
    from maicon26.types import Scan
    m, a = _solo()
    a.stack = _Frozen(a.stack, Pose2D(2.0, 2.5, 0.0))
    a.mode = Mode.EXPLORE
    a._scan = Scan(-math.pi, 2 * math.pi / 360, np.full(360, scan_range), 0.0, 8.0)
    a._stall.reset()
    a.stats["bump_events"] = 0
    return a


def _drive(a, v, t0=100.0, secs=2.5):
    a._v_cmd = v
    t = t0
    while t < t0 + secs:
        a._stall_check(t, 0.05)
        t += 0.05
    return t


def test_bump_detects_blocked_forward_command_and_remembers_the_spot():
    a = _frozen_agent(3.0)                                         # LiDAR 앞은 3 m 비어 있음
    _drive(a, 0.25)                                                # 전진 명령인데 SLAM 자세는 그대로
    assert a.stats["bump_events"] == 1 and a.mode.value == "RECOVER"
    assert any("BUMP" in msg for _, msg in a.log)
    g = a.stack.grid
    iy, ix = g.to_cell(2.0 + a.cfg.map3d.bump_ahead_m, 2.5)
    assert g.layer3d is not None and g.layer3d[iy, ix]             # 막힌 자리가 계획 격자의 장애물 층에 들어감(Nav2 점군도 같은 층)
    iy2, ix2 = g.to_cell(2.0 - 0.3, 2.5)
    assert not g.layer3d[iy2, ix2]                                 # 뒤쪽은 아님
    # 기억은 bump_ttl_s 뒤에 사라진다(움직이는 물체였을 수도)
    a._stall_check(100.0 + a.cfg.map3d.bump_ttl_s + 5.0, 0.05)
    assert not a._bumps and not (a.stack.grid.layer3d is not None and a.stack.grid.layer3d[iy, ix])


def test_bump_ignores_lidar_visible_obstacle_turning_and_slow_commands():
    a = _frozen_agent(0.3)                                         # LiDAR 가 앞 0.3 m 에서 본다 → 기존 안전 제한·RECOVER 몫
    _drive(a, 0.25)
    assert a.stats["bump_events"] == 0
    a = _frozen_agent(3.0)
    _drive(a, 0.0)                                                 # 제자리 회전·정지(전진 명령 없음)
    _drive(a, 0.05)                                                # 느린 명령(bump_v_min 미만)
    assert a.stats["bump_events"] == 0
    a.cfg.map3d.bump_enable = False
    _drive(a, 0.25)
    assert a.stats["bump_events"] == 0


def test_bump_not_triggered_when_actually_moving():
    a = _frozen_agent(3.0)
    t, x = 100.0, 2.0
    a._v_cmd = 0.25
    while t < 103.0:
        x += 0.25 * 0.05
        a.stack.pose = Pose2D(x, 2.5, 0.0)
        a._stall_check(t, 0.05)
        t += 0.05
    assert a.stats["bump_events"] == 0


def test_contact_detection_alone_cuts_collisions_into_invisible_barrier():
    """깊이를 꺼도(LiDAR·깊이 모두 못 보는 상황) 접촉 감지만으로 장벽에 계속 들이받는 것이 크게 줄어든다."""
    def trial(bump):
        cfg = Config()
        cfg.nav.backend = "builtin"
        cfg.mission.spin_w, cfg.mission.tour_nx, cfg.mission.tour_ny = 1.8, 3, 3
        cfg.depth.enable = False
        cfg.map3d.bump_enable = bump
        m = build_match(seed=2, cfg=_nov(cfg), teams=["B2"])
        m.world.add_low_obstacle(1.55, 0.55, 1.75, 4.45, 0.12)
        run(m, max_t=120.0)
        return score(m)["agents"]["B-UGV-2"]["collisions"], m.agents[0].stats["bump_events"]
    off, _ = trial(False)
    on, n = trial(True)
    assert off > 40 and n >= 1 and on * 2 < off, (on, off, n)


# ====================================================================== 5. 카메라 자세 감시·보정
def _spin_fusion(cfg, n=70):
    w = World(_nov(cfg), seed=3)
    u = w.add_ugv("BLUE", 2)
    r = SimRobot(w, u)
    fu = DepthFusion(cfg)
    u.pose.x, u.pose.y = 1.4, 2.5
    for k in range(n):
        w.t = k * 0.2
        u.pose.th = (k / n) * 2 * math.pi
        sc = w.lidar(u)
        fu.ingest(r.get_depth(), w.t, u.pose, sc, u.pose)
    return fu


def test_depth_lidar_check_flags_wrong_camera_pose():
    ok = _spin_fusion(Config())
    assert ok.stats["lidar_checks"] > 20 and abs(ok.stats["lidar_resid_m"]) < 0.03 and not ok.lidar_misaligned
    cfg = Config()
    cfg.depth.sim_mount_err_xyz = (0.12, 0.0, 0.0)                  # 실제 카메라가 모델보다 12 cm 앞에 달림
    bad = _spin_fusion(cfg)
    assert bad.stats["lidar_resid_m"] < -0.08 and bad.lidar_misaligned and bad.stats["lidar_bias_alarm"] >= 1


def test_floor_plane_calibrates_mount_pitch_roll_height():
    cfg = Config()
    cfg.depth.sim_mount_err_rpy = (0.02, 0.05, 0.0)                 # 실제 설치: roll 1.15°, pitch 2.9°
    cfg.depth.sim_mount_err_xyz = (0.0, 0.0, -0.01)
    w = World(_nov(cfg), seed=3)
    u = w.add_ugv("BLUE", 2)
    u.pose.x, u.pose.y, u.pose.th = 1.6, 2.5, 0.0
    frames = []
    for k in range(16):                                            # 16장 중앙값 — 카메라가 낮아(13 cm) 바닥이 비스듬히 보여 잡음에 민감하다
        w.t = 0.3 * k
        frames.append(w.depth_image(u, cfg.depth))
    res = calibrate_mount_from_floor(median_depth(frames), cfg.depth.intrinsics(), max_range=1.2, tol=0.008)
    assert abs(res["pitch"] - 0.05) < math.radians(1.0) and abs(res["roll"] - 0.02) < math.radians(1.0), res
    assert abs(res["height"] - 0.13) < 0.015                        # 바닥 위 0.04 + 0.10 − 0.01
    p = Profile()
    lines = apply_to_profile(p, res)
    assert p.src("sensors.depth.rpy") == "measured" and abs(p.get("sensors.depth.rpy")[1] - 0.05) < 0.012 and lines
    assert abs(p.get("sensors.depth.xyz")[2] - 0.09) < 0.02         # base_link 기준 높이 = 바닥 위 높이 − base_z


# ====================================================================== 6. 실기 연결부
class _Img:
    def __init__(self, arr, enc, big=False, pad=0):
        h, w = arr.shape
        raw = arr.astype((">" if big else "<") + ("u2" if enc == "16UC1" else "f4"))
        rows = [r.tobytes() + b"\x00" * pad for r in raw]
        self.height, self.width, self.encoding, self.is_bigendian = h, w, enc, int(big)
        self.step = w * raw.dtype.itemsize + pad
        self.data = b"".join(rows)


def test_decode_depth_image_encodings_padding_and_invalid():
    mm = np.array([[500, 0, 1250], [2000, 65535, 750]], dtype=np.uint16)
    d = decode_depth_image(_Img(mm, "16UC1", pad=6), 0.001)
    assert d.dtype == np.float32 and np.allclose(d[0], [0.5, 0.0, 1.25]) and abs(d[1, 1] - 65.535) < 1e-3
    d = decode_depth_image(_Img(mm, "16UC1", big=True), 0.001)
    assert np.allclose(d[1, [0, 2]], [2.0, 0.75])
    f = np.array([[0.8, np.nan, np.inf], [-1.0, 3.0, 0.0]], dtype=np.float32)
    d = decode_depth_image(_Img(f, "32FC1"))
    assert d[0, 0] == np.float32(0.8) and d[0, 1] == 0 and d[0, 2] == 0 and d[1, 0] == 0 and d[1, 1] == 3.0   # nan/inf/음수/0 은 무효(0)
    with pytest.raises(ValueError):
        decode_depth_image(_Img(mm, "16UC1").__class__.__new__(_Img), 0.001) if False else decode_depth_image(type("X", (), {"height": 1, "width": 1, "encoding": "rgb8", "data": b"abc", "step": 3, "is_bigendian": 0})())


def test_camera_info_prefers_projection_matrix():
    info = type("I", (), {"k": [500, 0, 320, 0, 500, 200, 0, 0, 1], "p": [510, 0, 322, 0, 0, 511, 201, 0, 0, 0, 1, 0],
                          "width": 640, "height": 400})()
    K, w, h = camera_info_K(info)
    assert K[0] == 510 and K[2] == 322 and K[5] == 201 and (w, h) == (640, 400)
    info.p = []
    assert camera_info_K(info)[0][0] == 500
    info.k = [0] * 9
    with pytest.raises(ValueError):
        camera_info_K(info)


def test_nav2_stack_forwards_blind_obstacles_in_map_frame():
    from maicon26.nav.nav2_stack import Nav2Stack
    from maicon26.sim.fake_nav2 import FakeNav2Client
    cfg = Config()
    cfg.nav.backend = "nav2-fake"
    w = World(_nov(cfg), seed=3)
    u = w.add_ugv("BLUE", 2)
    r = SimRobot(w, u)
    cl = FakeNav2Client(r, cfg)
    got = {}
    orig = cl.set_depth_obstacles
    cl.set_depth_obstacles = lambda pts, height=0.1: (got.setdefault("pts", np.asarray(pts)), orig(pts, height))[1]
    st = Nav2Stack(cfg, cl)
    st.start(r.get_scan(), r.get_odom(), (u.pose.x, u.pose.y))
    g = st.grid
    mask = np.zeros(g.L.shape, dtype=bool)
    iy, ix = g.to_cell(2.4, 2.6)
    mask[iy:iy + 3, ix:ix + 3] = True
    st.set_obstacle_layer(mask)
    pts_map = got["pts"]
    assert pts_map.shape == (9, 2)
    ax = np.array([cl._map_to_arena(x, y) for x, y in pts_map])    # map → 경기장 좌표로 되돌리면 원래 셀
    assert abs(ax[:, 0].mean() - (g.ox + (ix + 1.5) * g.res)) < 1e-3 and abs(ax[:, 1].mean() - (g.oy + (iy + 1.5) * g.res)) < 1e-3
    assert cl._nav.grid.layer3d is not None and cl._nav.grid.layer3d[iy, ix]     # Nav2 대역의 플래너 격자에도 반영
    st.set_obstacle_layer(None)
    assert cl._nav.grid.layer3d is None and st.grid.layer3d is None



