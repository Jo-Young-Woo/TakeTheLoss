"""보정 도구 시험. 시뮬의 '숨은 정답'(sim/drive.py truth_for_tests)을 측정→분석이 복원하는지 본다.
ROS 불필요. (실기 측정 자체는 검증할 수 없다 — 분석 코드가 정답을 복원함을 확인하는 것이 목적)"""
import json
import math
import os
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from maicon26.calib.analysis import fit_run
from maicon26.calib.experiments import EXPERIMENTS, run_experiments
from maicon26.calib.generate import (derive_nav, gen_bt_xml, gen_camera_info, gen_config_overlay,
                                     gen_sim_drive, gen_urdf, generate_all, stop_distance)
from maicon26.calib.profile import Profile
from maicon26.calib.records import load_run
from maicon26.calib.robot_io import SimIO
from maicon26.config import Config
from maicon26.sim.drive import SimDrive, truth_for_tests

ROBOT = dict(v_max=0.52, a_max=0.9, w_max=2.6)


@pytest.fixture(scope="module")
def fitted(tmp_path_factory):
    out = str(tmp_path_factory.mktemp("rec"))
    trs = []
    for surf in ("default", "tile", "carpet"):
        io = SimIO(truth_for_tests(), robot=ROBOT, surface=surf)
        which = list(EXPERIMENTS) if surf == "default" else ["straight", "brake", "speed_steps"]
        trs += run_experiments(io, out, which, surface=surf, log=lambda s: None)
    P, lines = fit_run(Profile(), load_run(out), source="sim")        # 디스크 왕복(CSV/JSON)까지 포함
    return P, lines, out


def test_profile_roundtrip_and_provenance(tmp_path):
    P = Profile()
    assert P.src("model.mass") == "pdf" and P.src("drive.v_max") == "assumed"
    P.set("drive.v_max", 0.51, "measured")
    f = tmp_path / "p.yaml"
    P.save(str(f))
    Q = Profile.load(str(f))
    assert Q.get("drive.v_max") == 0.51 and Q.src("drive.v_max") == "measured"
    assert ("drive.v_max", 0.51) not in Q.assumed()
    assert any(p == "sensors.lidar.xyz" for p, _ in Q.assumed())


def test_recovers_drive_table(fitted):
    P, _, _ = fitted
    g = P.get
    assert g("drive.v_max") == pytest.approx(0.52, abs=0.01)
    assert g("drive.a_accel") == pytest.approx(0.9, rel=0.08)
    assert g("drive.a_brake") == pytest.approx(1.9, rel=0.08)
    assert g("drive.cmd_latency_s") == pytest.approx(0.12, abs=0.04)          # 표본 주기(20 ms) 1~2개 분해능
    assert g("drive.lr_bias_rad_per_m") == pytest.approx(0.025, rel=0.06)
    assert g("drive.w_max") == pytest.approx(2.6 * 0.94, rel=0.03)          # 실제 도달 각속도 = 명령 × 회전 효율
    assert g("drive.pivot.yaw_efficiency") == pytest.approx(0.94, abs=0.02)
    assert g("drive.pivot.drift_m_per_rad") == pytest.approx(0.02, abs=0.006)
    st = g("drive.stop_distance")
    assert set(st) == {"0.15", "0.30", "0.45"} and st["0.45"] > st["0.30"] > st["0.15"]
    assert st["0.45"] == pytest.approx(0.12 * 0.45 + 0.45 ** 2 / (2 * 1.9), abs=0.015)
    assert P.src("drive.v_max") == "measured"


def test_recovers_surfaces_and_odometry(fitted):
    P, _, _ = fitted
    assert P.get("sensors.odom.lin_scale") == pytest.approx(1.03, abs=0.005)
    assert P.get("drive.surfaces.tile.slip") == pytest.approx(0.01, abs=0.004)
    assert P.get("drive.surfaces.carpet.slip") == pytest.approx(0.06, abs=0.008)
    assert P.get("drive.surfaces.carpet.a_brake") < P.get("drive.surfaces.default.a_brake")      # 카펫은 덜 선다
    assert P.get("drive.surfaces.carpet.mu_lower_bound") < P.get("drive.surfaces.default.mu_lower_bound")
    assert P.get("sensors.odom.yaw_scale") == pytest.approx(0.96 / 0.94, abs=0.01)
    assert P.get("sensors.odom.gyro_bias_z") == pytest.approx(0.004, abs=0.001)
    assert P.get("sensors.odom.noise_v") == pytest.approx(0.004, abs=0.001)
    assert P.get("sensors.odom.latency_s") < 0.02


def test_recovers_sensors(fitted):
    P, _, _ = fitted
    assert P.get("sensors.imu.gyro_bias_z") == pytest.approx(-0.006, abs=0.001)
    assert P.get("sensors.imu.gyro_noise_z") == pytest.approx(0.002, abs=0.0007)
    assert P.get("sensors.imu.latency_s") == pytest.approx(0.03, abs=0.025)
    x, y, _ = P.get("sensors.lidar.xyz")
    assert (x, y) == pytest.approx((0.04, -0.01), abs=0.006)
    assert P.get("sensors.lidar.rpy")[2] == pytest.approx(0.07, abs=0.005)


def test_avoid_recorded(fitted):
    _, lines, out = fitted
    assert any("avoid__default" in l and "succeeded" in l for l in lines)
    assert any(f.endswith("avoid__default.csv") for f in os.listdir(out))
    assert os.path.exists(os.path.join(out, "manifest.json"))


def test_truth_end_mode_straight_matches_continuous(tmp_path):
    """참값이 연속으로 없고 줄자로 잰 끝값만 있는 경우(실기 흔한 상황)에도 스케일/편차를 구한다."""
    io = SimIO(truth_for_tests(), robot=ROBOT)
    trs = run_experiments(io, str(tmp_path), ["straight"], log=lambda s: None)
    for t in trs:
        et = t.meta["truth_end"]                                     # SimIO 가 준 끝 자세 → 줄자 값으로 변환
        dx, dy = et["x"] - 0.8, et["y"] - 2.5
        t.meta["truth_end"] = {"distance_m": dx, "lateral_m": dy, "heading_deg": math.degrees(et["th"])}
        for c in ("truth_x", "truth_y", "truth_th"):                 # 연속 참값 삭제
            i = ["t", "cmd_v", "cmd_w", "odom_x", "odom_y", "odom_th", "gyro_z", "truth_x", "truth_y", "truth_th"].index(c)
            for r in t.rows:
                r[i] = float("nan")
        t._arr = None
    P, _ = fit_run(Profile(), trs)
    assert P.get("sensors.odom.lin_scale") == pytest.approx(1.03, abs=0.01)
    assert P.get("drive.lr_bias_rad_per_m") == pytest.approx(0.025, rel=0.15)


# ---------------------------------------------------------------- 시뮬 구동 노브: 기본은 중립(기존 시뮬 불변)
def test_default_sim_drive_is_neutral():
    from maicon26.sim.run import build_match
    m = build_match(seed=1, teams=["B2"])
    u = m.world.ugvs[0]
    assert u.drive.to_dict() == SimDrive().to_dict() and u.drive.lag_s == 0 and u.drive.lidar_yaw == 0


# ---------------------------------------------------------------- 생성기
def _fitted_profile(fitted):
    return fitted[0]


def test_urdf_valid_and_physical(fitted):
    P = _fitted_profile(fitted)
    root = ET.fromstring(gen_urdf(P))
    links = {l.get("name") for l in root.findall("link")}
    {j.get("name") for j in root.findall("joint")}
    assert {"base_footprint", "base_link", "laser", "imu_link", "camera_link", "camera_optical"} <= links
    assert sum(1 for l in links if l.startswith("wheel_")) == 6
    # 모든 조인트의 parent/child 가 존재하고 트리(루트 하나)
    children = set()
    for j in root.findall("joint"):
        assert j.find("parent").get("link") in links and j.find("child").get("link") in links
        children.add(j.find("child").get("link"))
    assert links - children == {"base_footprint"}
    total = sum(float(l.find("inertial/mass").get("value")) for l in root.findall("link") if l.find("inertial") is not None)
    assert total == pytest.approx(P.get("model.mass"), abs=1e-3)
    lid = [j for j in root.findall("joint") if j.get("name") == "laser_joint"][0]
    assert float(lid.find("origin").get("rpy").split()[2]) == pytest.approx(0.07, abs=0.005)      # 보정된 LiDAR yaw 가 TF 로
    box = root.find("link[@name='base_link']/collision/geometry/box").get("size").split()
    assert float(box[0]) == pytest.approx(0.25) and float(box[1]) == pytest.approx(0.23)


def test_nav_derivation_follows_drive_table(fitted):
    P = _fitted_profile(fitted)
    n = derive_nav(P)
    assert stop_distance(P, 0.30) == pytest.approx(P.get("drive.stop_distance")["0.30"], abs=1e-6)       # 표 사용
    assert stop_distance(P, n["desired_linear_vel"]) <= 0.25 + 1e-3                                       # 정지거리 예산 이내
    assert n["desired_linear_vel"] <= 0.75 * P.get("drive.v_max") + 1e-9
    assert n["inflation_radius"] <= P.get("nav.clearance.corridor_min") / 2 + 0.10 + 1e-9
    slow = P.copy()
    slow.set("drive.a_brake", 0.3, "measured")
    slow.set("drive.stop_distance", {}, "measured")                                                       # 표 없이 모델로
    slow.set("drive.cmd_latency_s", 0.3, "measured")
    assert derive_nav(slow)["desired_linear_vel"] < n["desired_linear_vel"]                                # 제동이 약하면 더 느리게
    ns = derive_nav(slow)
    # 정지 *트리거* 거리는 센서 오차+지연으로 정하고(footprint.ClearanceBudget.stop_front), 제동거리는 감속 구간(slow_dist−stop_dist)이 맡는다 — 같은 거리를 두 번 더하지 않는다.
    assert ns["slow_dist"] - ns["stop_dist"] >= ns["d_stop_at_desired"] - 1e-9
    assert ns["stop_dist"] <= n["stop_dist"] + 1e-9           # 더 느리게 달리면 지연 동안 가는 거리(v·지연)가 줄어 트리거 거리도 같거나 작다


def test_config_overlay_loads_and_changes_sim(fitted, tmp_path):
    P = _fitted_profile(fitted)
    f = tmp_path / "o.json"
    f.write_text(json.dumps(gen_config_overlay(P)))
    cfg = Config.load(str(f))                                                # 알 수 없는 키가 있으면 KeyError
    assert cfg.robot.v_max == pytest.approx(P.get("drive.v_max"), abs=1e-3)
    assert cfg.nav.stop_dist > 0.15 and cfg.nav.slow_dist > cfg.nav.stop_dist


def test_sim_drive_roundtrip_reproduces_measured_robot(fitted):
    """측정값으로 만든 SimDrive 로 같은 시험을 다시 돌리면 같은 값이 나온다(시뮬이 실기를 흉내)."""
    P = _fitted_profile(fitted)
    d = SimDrive.from_dict(gen_sim_drive(P))
    cfg = json.loads(json.dumps(gen_config_overlay(P)))
    io = SimIO(d, robot=dict(v_max=cfg["robot"]["v_max"], a_max=cfg["robot"]["a_max"], w_max=cfg["robot"]["w_max"]))
    # w_max 는 '효율 반영 후' 값이라 시뮬 명령 한계에는 1/효율을 곱해 되돌린다
    io.cfg.robot.w_max = cfg["robot"]["w_max"] / P.get("drive.pivot.yaw_efficiency")
    trs = run_experiments(io, str(os.path.join(os.path.dirname(fitted[2]), "re")), ["speed_steps", "brake", "straight"], log=lambda s: None)
    Q, _ = fit_run(Profile(), trs)
    assert Q.get("drive.v_max") == pytest.approx(P.get("drive.v_max"), abs=0.015)
    assert Q.get("drive.a_brake") == pytest.approx(P.get("drive.a_brake"), rel=0.1)
    assert Q.get("drive.lr_bias_rad_per_m") == pytest.approx(P.get("drive.lr_bias_rad_per_m"), rel=0.1)


def test_generate_all_writes_every_deliverable(fitted, tmp_path):
    import shutil
    src = os.path.join(os.path.dirname(__file__), "..")
    root = tmp_path / "proj"
    for rel in ("ros2/config/nav2_params.yaml", "ros2/behavior_trees/navigate_replan_fast.xml"):
        os.makedirs(os.path.dirname(root / rel), exist_ok=True)
        shutil.copy(os.path.join(src, rel), root / rel)
    files = generate_all(_fitted_profile(fitted), str(root), log=lambda s: None)
    for must in ("ros2/urdf/wingbot_ugv.urdf", "config/sim_drive.json", "calib/outputs/drive_calibration.yaml",
                 "calib/outputs/drive_calibration.md", "calib/outputs/sensor_calibration.yaml", "ros2/config/camera_info.yaml",
                 "ros2/config/nav2_params.calibrated.yaml", "ros2/behavior_trees/navigate_replan_fast.calibrated.xml",
                 "config/robot_calibrated.json"):
        assert must in files and (root / must).stat().st_size > 50
    import yaml
    y = yaml.safe_load(open(root / "ros2/config/nav2_params.calibrated.yaml", encoding="utf-8"))
    lc = y["local_costmap"]["local_costmap"]["ros__parameters"]
    assert "robot_radius" not in lc and lc["footprint_padding"] == 0.0           # 원형 근사 대신 직사각형 footprint(+계획 여유), 패딩 이중 가산 없음
    import ast
    poly = ast.literal_eval(lc["footprint"])
    assert max(abs(px) for px, _ in poly) > 0.125 and max(abs(py) for _, py in poly) > 0.115   # 차체(250×230 mm)보다 크다(= 여유 포함)
    ET.parse(root / "ros2/behavior_trees/navigate_replan_fast.calibrated.xml")
    assert "| 최대 속도 (전진)" in (root / "calib/outputs/drive_calibration.md").read_text(encoding="utf-8")


def test_bt_recovery_values_substituted():
    P = Profile()
    P.set("nav.recovery.backup_dist", 0.33, "tuned")
    xml = gen_bt_xml(P, '<Spin spin_dist="1.57"/><Wait wait_duration="2"/><BackUp backup_dist="0.25" backup_speed="0.05"/>')
    assert 'backup_dist="0.33"' in xml


# ---------------------------------------------------------------- 카메라 (OpenCV 있을 때)
def test_camera_calibration_recovers_fisheye():
    cv2 = pytest.importorskip("cv2")
    if cv2.__version__.startswith("5."):
        pytest.skip("OpenCV 5.0 의 cv2.fisheye.calibrate 는 수렴하지 않음 — 4.x(WSL/Humble)에서 검증")
    from maicon26.calib import camera
    rng = np.random.default_rng(0)
    size = (640, 480)
    K = np.array([[210.0, 0, 322.0], [0, 208.0, 238.0], [0, 0, 1.0]])
    D = np.array([[0.03], [-0.01], [0.004], [-0.001]])
    objp = camera.board_points(9, 6, 0.025)
    objs, imgs = [], []
    while len(objs) < 25:
        rv = rng.normal(0, 0.35, 3)
        tv = np.array([rng.uniform(-0.08, 0.04), rng.uniform(-0.06, 0.03), rng.uniform(0.25, 0.5)])
        pr, _ = cv2.fisheye.projectPoints(objp.reshape(1, -1, 3), rv, tv, K, D)
        pr = pr.reshape(-1, 2)
        if (pr[:, 0].min() < 5 or pr[:, 0].max() > size[0] - 5 or pr[:, 1].min() < 5 or pr[:, 1].max() > size[1] - 5):
            continue
        objs.append(objp.copy()); imgs.append(pr + rng.normal(0, 0.15, pr.shape))
    res = camera.calibrate(objs, imgs, size, "fisheye")
    assert res["rms_px"] < 0.5
    assert res["K"][0] == pytest.approx(210.0, rel=0.02) and res["K"][4] == pytest.approx(208.0, rel=0.02)
    P = Profile()
    camera.apply_to_profile(P, res, "test")
    assert P.get("sensors.camera.calibrated") is True and P.src("sensors.camera.K") == "measured"
    ci = gen_camera_info(P)
    assert ci["camera_matrix"]["data"][0] == pytest.approx(210.0, rel=0.02)


def test_camera_corner_detection_on_rendered_board(tmp_path):
    cv2 = pytest.importorskip("cv2")
    from maicon26.calib import camera
    sq, cols, rows = 40, 10, 7                                             # 내부 코너 9x6
    img = np.full(((rows + 1) * sq, (cols + 1) * sq), 255, np.uint8)
    for r in range(rows):
        for c in range(cols):
            if (r + c) % 2 == 0:
                img[sq // 2 + r * sq: sq // 2 + (r + 1) * sq, sq // 2 + c * sq: sq // 2 + (c + 1) * sq] = 0
    paths = []
    for k, (ax, ay) in enumerate([(0, 0), (12, 5), (-10, 8), (6, -9), (15, 12), (-14, -6), (8, 10), (-6, 14)]):
        src = np.float32([[0, 0], [img.shape[1], 0], [img.shape[1], img.shape[0]], [0, img.shape[0]]])
        dst = src * 0.8 + np.float32([60 + ax, 50 + ay]) + np.float32([[0, 0], [-6 * k % 17, 0], [0, 0], [0, 5 * k % 13]])
        warped = cv2.warpPerspective(img, cv2.getPerspectiveTransform(src, dst), (640, 480), borderValue=255)
        p = str(tmp_path / f"b{k}.png")
        cv2.imwrite(p, warped)
        paths.append(p)
    objs, imgs, size = camera.detect_corners(paths, (9, 6), 0.025, log=lambda s: None)
    assert size == (640, 480) and len(objs) >= 6 and imgs[0].shape == (54, 2)
