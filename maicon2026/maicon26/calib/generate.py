"""프로파일 → 산출물.

  1. 로봇 모델        ros2/urdf/wingbot_ugv.urdf            URDF (크기·질량·무게중심·관성·바퀴·충돌형상·센서 프레임·마찰)
                     config/sim_drive.json                  2D 시뮬 구동 설정 (maicon26.sim.drive.SimDrive)
  2. 구동 보정표      calib/outputs/drive_calibration.yaml/.md
  3. 센서 보정        calib/outputs/sensor_calibration.yaml  (TF, 카메라 내부 파라미터, 오도메트리 오차, 지연)
                     ros2/config/camera_info.yaml           (sensor_msgs/CameraInfo 형식)
  4. 항법 설정        ros2/config/nav2_params.calibrated.yaml, ros2/behavior_trees/navigate_replan_fast.calibrated.xml,
                     config/robot_calibrated.json           (maicon26 Config 덮어쓰기)
미측정(assumed) 값은 그대로 쓰되 산출물 머리말에 '가정 N개'를 적어 속지 않게 한다.
"""
from __future__ import annotations

import copy
import json
import math
import os
import re
import warnings
from typing import Any, Dict, List, Tuple

from .profile import Profile

G = 9.81
HEADER = "# 자동 생성 파일 — 직접 고치지 말고 calib/robot_profile.yaml 을 고친 뒤 scripts/calib_generate.py 를 다시 실행\n"


def _banner(p: Profile) -> str:
    n = len(p.assumed())
    return HEADER + f"# 프로파일에 아직 '가정(assumed)'인 값 {n}개 (scripts/calib_report.py 로 목록 확인)\n"


# ====================================================================== 정지거리 모델
def stop_distance(p: Profile, v: float) -> float:
    """속도 v 에서의 정지거리: 표가 있으면 보간, 없으면 지연·감속 모델 d = L·v + v²/(2a)."""
    tbl = p.get("drive.stop_distance") or {}
    if len(tbl) >= 2:
        xs = sorted((float(k), float(d)) for k, d in tbl.items())
        if v <= xs[0][0]:
            return xs[0][1] * v / xs[0][0]
        for (x0, d0), (x1, d1) in zip(xs[:-1], xs[1:]):
            if v <= x1:
                return d0 + (d1 - d0) * (v - x0) / (x1 - x0)
        (x0, d0), (x1, d1) = xs[-2], xs[-1]                      # 외삽
        return d1 + (d1 - d0) * (v - x1) / (x1 - x0)
    a = max(p.get("drive.a_brake"), 0.1)
    return p.get("drive.cmd_latency_s") * v + v * v / (2 * a)


def max_speed_for_stop(p: Profile, budget: float) -> float:
    """정지거리가 budget(m) 이내인 최대 속도 (이분 탐색)."""
    lo, hi = 0.0, p.get("drive.v_max")
    if stop_distance(p, hi) <= budget:
        return hi
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if stop_distance(p, mid) <= budget else (lo, mid)
    return lo


# ====================================================================== 항법 파생값
def profile_budget(p: Profile, v_ref: float):
    """프로파일의 차체 치수(model.size)와 LiDAR 오차(sensors.lidar.error)에서 ClearanceBudget 을 만든다 — Config 와 같은 함수(footprint.budget_from)."""
    from types import SimpleNamespace as NS
    from ..footprint import budget_from
    robot = NS(length=p.get("model.size.x"), width=p.get("model.size.y"), height=p.get("model.size.z"))
    e = dict(p.get("sensors.lidar.error"))
    prov = {k: p.src(f"sensors.lidar.error.{k}") for k in
            ("range_bias_m", "range_sigma_m", "angle_sigma_deg", "mount_xy_m", "mount_yaw_deg", "latency_s", "pose_sigma_m")}
    lerr = NS(range_bias_m=e["range_bias_m"], range_sigma_m=e["range_sigma_m"], angle_sigma_deg=e["angle_sigma_deg"], mount_xy_m=e["mount_xy_m"],
              mount_yaw_deg=e["mount_yaw_deg"], latency_s=e["latency_s"], pose_sigma_m=e["pose_sigma_m"], k_sigma=e["k_sigma"],
              ref_range_m=e["ref_range_m"], source=prov)
    return budget_from(robot, lerr, v_ref)


def derive_nav(p: Profile) -> Dict[str, Any]:
    v_max, w_max = p.get("drive.v_max"), p.get("drive.w_max")
    budget = 0.25                                               # 장애물 앞에서 멈추는 데 쓸 수 있는 거리 (이격 0.40 m 통로 기준)
    v_safe = max_speed_for_stop(p, budget)
    desired = p.get("nav.follow.desired_linear_vel") or min(0.75 * v_max, v_safe)
    look = p.get("nav.follow.lookahead_dist") or max(0.25, 1.1 * desired)
    rot_w = p.get("nav.follow.rotate_to_heading_w") or 0.7 * w_max
    corridor = p.get("nav.clearance.corridor_min")
    bud = profile_budget(p, desired)
    # 비용 기울기 반경: 외접 + 계획 여유 이상(회전 공간까지 비용이 번지게), 통로 반폭 + 0.10 이하
    infl = min(max(p.get("nav.clearance.inflation_radius"), bud.plan_inflate_rotate + 0.01), corridor / 2 + 0.10)
    inscribed = p.inscribed_radius()
    d_stop = stop_distance(p, desired)
    stop_dist = bud.stop_front                                   # 앞 반길이 + 정지 여유(센서 오차 + 지연). 제동거리는 slow_dist 램프가 맡는다(중복 가산 안 함)
    return {
        "budget": bud, "footprint": bud.nav2_footprint, "plan_margin": bud.plan_margin, "stop_margin": bud.stop_margin,
        "inflate_hard": bud.plan_inflate_straight, "rotate_clear": bud.plan_inflate_rotate, "side_lim": bud.stop_side,
        "v_safe_for_stop_budget": v_safe, "desired_linear_vel": desired, "lookahead_dist": look,
        "min_lookahead_dist": 0.25, "max_lookahead_dist": max(0.7, look),
        "rotate_to_heading_w": rot_w, "max_angular_accel": p.get("drive.alpha_max"),
        "min_approach_linear_velocity": max(0.05, 0.2 * desired),
        "regulated_min_speed": max(0.08, 0.3 * desired),
        "inflation_radius": infl, "cost_scaling": p.get("nav.clearance.cost_scaling"),
        "xy_goal_tolerance": p.get("nav.goal.xy_tol"),
        "progress_time_allowance": 8.0, "d_stop_at_desired": d_stop, "stop_dist": stop_dist,
        "slow_dist": stop_dist + max(0.28, d_stop), "cmd_timeout": max(0.5, 3 * p.get("drive.cmd_latency_s")),
        "inscribed_radius": inscribed, "circumscribed_radius": p.circumscribed_radius(),
    }


# ====================================================================== 1. URDF
def _inertial(m: float, com: Tuple[float, float, float], I: Dict[str, float]) -> str:
    return (f'<inertial><origin xyz="{com[0]:.4f} {com[1]:.4f} {com[2]:.4f}" rpy="0 0 0"/><mass value="{m:.4f}"/>'
            f'<inertia ixx="{I["ixx"]:.6f}" ixy="0" ixz="0" iyy="{I["iyy"]:.6f}" iyz="0" izz="{I["izz"]:.6f}"/></inertial>')


def _rpy(v) -> str:
    return f"{v[0]:.5f} {v[1]:.5f} {v[2]:.5f}"


def _xyz(v) -> str:
    return f"{v[0]:.5f} {v[1]:.5f} {v[2]:.5f}"


def gen_urdf(p: Profile) -> str:
    sx, sy, sz = p.get("model.size.x"), p.get("model.size.y"), p.get("model.size.z")
    mass = p.get("model.mass")
    h = p.get("model.chassis_height")
    com = (p.get("model.com.x"), p.get("model.com.y"), p.get("model.com.z"))
    W = p.get("model.wheels")
    wr, ww, wxs, track = W["radius"], W["width"], W["x"], W["track"]
    wheel_m = 0.06                                               # 바퀴 하나 질량 [assumed]
    body_m = max(mass - W["count"] * wheel_m, 0.1)
    clearance = 0.02
    zc = -wr + clearance + h / 2                                 # base_link(차축 높이) 기준 섀시 중심
    pad = p.get("model.collision.padding")
    mu = p.get("drive.surfaces.default.mu_lower_bound") or 0.8   # 측정 전엔 0.8 가정(시뮬 마찰 설정)
    I_body = p.inertia()
    I_wheel = {"ixx": wheel_m * (3 * wr * wr + ww * ww) / 12, "iyy": wheel_m * wr * wr / 2, "izz": wheel_m * (3 * wr * wr + ww * ww) / 12}
    mast_h = max(sz - (h + clearance), 0.02)                     # 섀시 위로 솟은 부분 (카메라/팬틸트/안테나)
    pt = p.get("model.pan_tilt.xyz")
    lidar, cam, depth, imu = (p.get(f"sensors.{k}") for k in ("lidar", "camera", "depth", "imu"))
    n_assumed = len(p.assumed())
    L: List[str] = []
    A = L.append
    A('<?xml version="1.0"?>')
    A(f"<!-- 자동 생성: WINGBOT-UGV. 프로파일의 가정 값 {n_assumed}개 포함 — scripts/calib_report.py. 직접 고치지 말 것 -->")
    A('<robot name="wingbot_ugv">')
    A('  <material name="body"><color rgba="0.2 0.25 0.3 1"/></material>')
    A('  <material name="wheel"><color rgba="0.05 0.05 0.05 1"/></material>')
    A('  <material name="sensor"><color rgba="0.9 0.5 0.1 1"/></material>')
    A('  <link name="base_footprint"/>')
    A(f'  <joint name="base_joint" type="fixed"><parent link="base_footprint"/><child link="base_link"/><origin xyz="0 0 {wr:.4f}" rpy="0 0 0"/></joint>')
    # 본체: 섀시 + 마스트를 한 링크로 (충돌 형상 두 개)
    A('  <link name="base_link">')
    A("    " + _inertial(body_m, com, I_body))
    A(f'    <visual><origin xyz="0 0 {zc:.4f}"/><geometry><box size="{sx:.4f} {sy:.4f} {h:.4f}"/></geometry><material name="body"/></visual>')
    A(f'    <collision><origin xyz="0 0 {zc:.4f}"/><geometry><box size="{sx + 2 * pad:.4f} {sy + 2 * pad:.4f} {h:.4f}"/></geometry></collision>')
    A(f'    <visual><origin xyz="0 0 {zc + h / 2 + mast_h / 2:.4f}"/><geometry><box size="{0.12:.3f} {0.12:.3f} {mast_h:.4f}"/></geometry><material name="body"/></visual>')
    A(f'    <collision><origin xyz="0 0 {zc + h / 2 + mast_h / 2:.4f}"/><geometry><box size="{0.12:.3f} {0.12:.3f} {mast_h:.4f}"/></geometry></collision>')
    A("  </link>")
    # 바퀴 6개 (6WD 스키드 조향)
    for i, x in enumerate(wxs):
        for side, y in (("left", track / 2), ("right", -track / 2)):
            n = f"wheel_{['front', 'mid', 'rear'][i]}_{side}"
            A(f'  <link name="{n}">')
            A("    " + _inertial(wheel_m, (0, 0, 0), I_wheel))
            A(f'    <visual><origin rpy="{math.pi / 2:.5f} 0 0"/><geometry><cylinder radius="{wr:.4f}" length="{ww:.4f}"/></geometry><material name="wheel"/></visual>')
            A(f'    <collision><origin rpy="{math.pi / 2:.5f} 0 0"/><geometry><cylinder radius="{wr:.4f}" length="{ww:.4f}"/></geometry></collision>')
            A("  </link>")
            A(f'  <joint name="{n}_joint" type="continuous"><parent link="base_link"/><child link="{n}"/>'
              f'<origin xyz="{x:.4f} {y:.4f} 0" rpy="0 0 0"/><axis xyz="0 1 0"/><dynamics damping="0.01" friction="0.01"/></joint>')
            A(f'  <gazebo reference="{n}"><mu1>{mu:.3f}</mu1><mu2>{mu:.3f}</mu2><kp>1e5</kp><kd>1.0</kd></gazebo>')
    # 센서 프레임
    def fixed(name: str, parent: str, xyz, rpy) -> None:
        A(f'  <link name="{name}"/>')
        A(f'  <joint name="{name}_joint" type="fixed"><parent link="{parent}"/><child link="{name}"/><origin xyz="{_xyz(xyz)}" rpy="{_rpy(rpy)}"/></joint>')
    fixed(lidar["frame"], "base_link", lidar["xyz"], lidar["rpy"])
    fixed(imu["frame"], "base_link", imu["xyz"], imu["rpy"])
    fixed("pan_link", "base_link", pt, (0, 0, 0))               # 팬/틸트는 고정(0°)으로 둔다 — 동적 각도는 Agent가 따로 알고 있다
    fixed("tilt_link", "pan_link", (0, 0, 0), (0, 0, 0))
    fixed("camera_link", cam["parent"], cam["xyz"], cam["rpy"])
    A(f'  <link name="{cam["frame"]}"/>')                       # 광학 프레임: z 앞, x 오른쪽, y 아래
    A(f'  <joint name="{cam["frame"]}_joint" type="fixed"><parent link="camera_link"/><child link="{cam["frame"]}"/>'
      f'<origin xyz="0 0 0" rpy="{-math.pi / 2:.5f} 0 {-math.pi / 2:.5f}"/></joint>')
    fixed("depth_link", depth["parent"], depth["xyz"], depth["rpy"])
    A(f'  <link name="{depth["frame"]}"/>')
    A(f'  <joint name="{depth["frame"]}_joint" type="fixed"><parent link="depth_link"/><child link="{depth["frame"]}"/>'
      f'<origin xyz="0 0 0" rpy="{-math.pi / 2:.5f} 0 {-math.pi / 2:.5f}"/></joint>')
    A("</robot>")
    return "\n".join(L) + "\n"


# ====================================================================== 3. 센서
def camera_intrinsics(p: Profile) -> Dict[str, Any]:
    cam = p.get("sensors.camera")
    w, h = cam["width"], cam["height"]
    if cam.get("K"):
        K, D, calibrated = cam["K"], cam.get("D") or [0.0] * 5, True
    else:                                                        # 화각에서 핀홀 근사 (보정 전)
        f = (w / 2) / math.tan(math.radians(cam["hfov_deg"]) / 2)
        K, D, calibrated = [f, 0.0, w / 2, 0.0, f, h / 2, 0.0, 0.0, 1.0], [0.0] * 5, False
    return {"width": w, "height": h, "model": cam["model"], "K": [float(x) for x in K], "D": [float(x) for x in D], "calibrated": calibrated}


def gen_sensor_calibration(p: Profile) -> Dict[str, Any]:
    ci = camera_intrinsics(p)
    prov = {k: v for k, v in p.data.get("provenance", {}).items() if k.startswith("sensors.")}
    return {
        "tf": {"base_link→" + p.get("sensors.lidar.frame"): {"xyz": p.get("sensors.lidar.xyz"), "rpy": p.get("sensors.lidar.rpy")},
               "base_link→" + p.get("sensors.imu.frame"): {"xyz": p.get("sensors.imu.xyz"), "rpy": p.get("sensors.imu.rpy")},
               "base_link→pan_link": {"xyz": p.get("model.pan_tilt.xyz")},
               p.get("sensors.camera.parent") + "→camera_link": {"xyz": p.get("sensors.camera.xyz"), "rpy": p.get("sensors.camera.rpy")},
               p.get("sensors.depth.parent") + "→depth_link": {"xyz": p.get("sensors.depth.xyz"), "rpy": p.get("sensors.depth.rpy")}},
        "camera": {"frame": p.get("sensors.camera.frame"), **ci, "hfov_deg": p.get("sensors.camera.hfov_deg"),
                   "latency_s": p.get("sensors.camera.latency_s")},
        "depth": {"frame": p.get("sensors.depth.frame"), **depth_intrinsics(p), "hfov_deg": p.get("sensors.depth.hfov_deg"),
                  "unit_scale": p.get("sensors.depth.unit_scale"), "min_range": p.get("sensors.depth.min_range"),
                  "max_range": p.get("sensors.depth.max_range"), "latency_s": p.get("sensors.depth.latency_s")},
        "lidar": {k: p.get(f"sensors.lidar.{k}") for k in ("model", "frame", "rate_hz", "range_min", "range_max", "latency_s")},
        "imu": {k: p.get(f"sensors.imu.{k}") for k in ("frame", "gyro_bias_z", "gyro_noise_z", "gyro_scale", "latency_s")},
        "odometry": p.get("sensors.odom"),
        "provenance": prov,
    }


def depth_intrinsics(p: Profile) -> Dict[str, Any]:
    d = p.get("sensors.depth")
    w, h = d["width"], d["height"]
    if d.get("K"):
        K, calibrated = d["K"], True
    else:
        f = (w / 2) / math.tan(math.radians(d["hfov_deg"]) / 2)
        K, calibrated = [f, 0.0, w / 2, 0.0, f, h / 2, 0.0, 0.0, 1.0], False
    return {"width": w, "height": h, "K": [float(x) for x in K], "calibrated": calibrated}


def gen_depth_camera_info(p: Profile) -> Dict[str, Any]:
    """깊이 camera_info (정류된 깊이 영상이라 왜곡 0). depthai-ros 가 스스로 내는 것이 있으면 그쪽이 우선 — 이건 보정 전 대체값이다."""
    di = depth_intrinsics(p)
    K = di["K"]
    return {"image_width": di["width"], "image_height": di["height"], "camera_name": p.get("sensors.depth.frame"),
            "camera_matrix": {"rows": 3, "cols": 3, "data": K},
            "distortion_model": "plumb_bob", "distortion_coefficients": {"rows": 1, "cols": 5, "data": [0.0] * 5},
            "rectification_matrix": {"rows": 3, "cols": 3, "data": [1, 0, 0, 0, 1, 0, 0, 0, 1]},
            "projection_matrix": {"rows": 3, "cols": 4, "data": [K[0], 0.0, K[2], 0.0, 0.0, K[4], K[5], 0.0, 0.0, 0.0, 1.0, 0.0]}}


def gen_camera_info(p: Profile) -> Dict[str, Any]:
    ci = camera_intrinsics(p)
    K = ci["K"]
    model = {"fisheye": "equidistant", "plumb_bob": "plumb_bob", "rational_polynomial": "rational_polynomial"}.get(ci["model"], ci["model"])
    P = [K[0], 0.0, K[2], 0.0, 0.0, K[4], K[5], 0.0, 0.0, 0.0, 1.0, 0.0]
    return {"image_width": ci["width"], "image_height": ci["height"], "camera_name": p.get("sensors.camera.frame"),
            "camera_matrix": {"rows": 3, "cols": 3, "data": K},
            "distortion_model": model, "distortion_coefficients": {"rows": 1, "cols": len(ci["D"]), "data": ci["D"]},
            "rectification_matrix": {"rows": 3, "cols": 3, "data": [1, 0, 0, 0, 1, 0, 0, 0, 1]},
            "projection_matrix": {"rows": 3, "cols": 4, "data": P}}


# ====================================================================== 2. 구동 보정표
def gen_drive_table(p: Profile) -> Dict[str, Any]:
    d = copy.deepcopy(p.get("drive"))
    d["mu_note"] = "mu_lower_bound = 제동 감속 ÷ g (하한값). 정확한 마찰계수는 견인 시험: μ = 끌 때 힘[N] ÷ (질량·g)"
    m, g = p.get("model.mass"), G
    for s, row in d["surfaces"].items():
        pull = row.get("mu_pull_test_N")
        if pull:
            row["mu_pull_test"] = pull / (m * g)
    d["provenance"] = {k: v for k, v in p.data.get("provenance", {}).items() if k.startswith("drive.")}
    return d


def drive_table_md(p: Profile) -> str:
    d = p.get("drive")
    src = lambda path: p.src(path)
    rows = [("최대 속도 (전진)", "m/s", d["v_max"], "drive.v_max"), ("최대 속도 (후진)", "m/s", d["v_max_reverse"], "drive.v_max_reverse"),
            ("가속도", "m/s²", d["a_accel"], "drive.a_accel"), ("제동 감속도", "m/s²", d["a_brake"], "drive.a_brake"),
            ("최대 각속도", "rad/s", d["w_max"], "drive.w_max"), ("최대 각가속도", "rad/s²", d["alpha_max"], "drive.alpha_max"),
            ("명령→움직임 지연", "s", d["cmd_latency_s"], "drive.cmd_latency_s"), ("상승시간 10→90 %", "s", d["rise_time_s"], "drive.rise_time_s"),
            ("직진 좌우 편차 (+좌)", "rad/m", d["lr_bias_rad_per_m"], "drive.lr_bias_rad_per_m"),
            ("제자리 회전 효율", "-", d["pivot"]["yaw_efficiency"], "drive.pivot.yaw_efficiency"),
            ("제자리 회전 중 밀림", "m/rad", d["pivot"]["drift_m_per_rad"], "drive.pivot.drift_m_per_rad"),
            ("정지 후 회전 오버슈트", "deg", d["pivot"]["overshoot_deg"], "drive.pivot.overshoot_deg")]
    out = ["| 항목 | 단위 | 값 | 출처 |", "|---|---|---|---|"]
    for n, u, v, path in rows:
        out.append(f"| {n} | {u} | {v:.4g} | {src(path)} |")
    out += ["", "**정지거리** (명령 0 → 정지)", "", "| 속도 m/s | 정지거리 m |", "|---|---|"]
    st = d.get("stop_distance") or {}
    out += [f"| {k} | {v:.3f} |" for k, v in sorted(st.items(), key=lambda kv: float(kv[0]))] or ["| (미측정) | |"]
    out += ["", "**주행면별**", "", "| 주행면 | 슬립 | 가속 m/s² | 제동 m/s² | μ 하한 | μ(견인) |", "|---|---|---|---|---|---|"]
    for s, r in d["surfaces"].items():
        f = lambda x: "-" if x is None else f"{x:.3g}"
        pull = r.get("mu_pull_test_N")
        out.append(f"| {s} | {f(r.get('slip'))} | {f(r.get('a_accel'))} | {f(r.get('a_brake'))} | {f(r.get('mu_lower_bound'))} | "
                   f"{f(pull / (p.get('model.mass') * G) if pull else None)} |")
    return "\n".join(out) + "\n"


# ====================================================================== 4. 항법 설정
def gen_nav2_params(p: Profile, base: Dict[str, Any]) -> Dict[str, Any]:
    n = derive_nav(p)
    y = copy.deepcopy(base)
    fp = y["controller_server"]["ros__parameters"]["FollowPath"]
    fp.update(desired_linear_vel=round(n["desired_linear_vel"], 3), lookahead_dist=round(n["lookahead_dist"], 3),
              min_lookahead_dist=n["min_lookahead_dist"], max_lookahead_dist=round(n["max_lookahead_dist"], 3),
              rotate_to_heading_angular_vel=round(n["rotate_to_heading_w"], 3), max_angular_accel=round(n["max_angular_accel"], 3),
              min_approach_linear_velocity=round(n["min_approach_linear_velocity"], 3),
              regulated_linear_scaling_min_speed=round(n["regulated_min_speed"], 3))
    cs = y["controller_server"]["ros__parameters"]
    cs["general_goal_checker"]["xy_goal_tolerance"] = n["xy_goal_tolerance"]
    cs["progress_checker"]["movement_time_allowance"] = n["progress_time_allowance"]
    for cm in ("local_costmap", "global_costmap"):
        r = y[cm][cm]["ros__parameters"]
        r.pop("robot_radius", None)
        r["footprint"] = "[" + ", ".join(f"[{x:.3f}, {y:.3f}]" for x, y in n["footprint"]) + "]"     # 직사각형 + 계획 여유(원형 근사로 모서리를 가리지 않는다)
        r["footprint_padding"] = 0.0                                                             # 여유는 footprint 에 이미 들어 있다(두 번 더하지 않는다)
        r["inflation_layer"]["inflation_radius"] = round(n["inflation_radius"], 3)
        r["inflation_layer"]["cost_scaling_factor"] = n["cost_scaling"]
    bs = y["behavior_server"]["ros__parameters"]
    bs["max_rotational_vel"] = round(min(1.2, 0.6 * p.get("drive.w_max")), 3)
    bs["rotational_acc_lim"] = round(min(3.2, 0.5 * p.get("drive.alpha_max")), 3)
    return y


def gen_bt_xml(p: Profile, xml: str) -> str:
    rc = p.get("nav.recovery")
    xml = re.sub(r'<Spin spin_dist="[^"]*"', f'<Spin spin_dist="{rc["spin_dist"]}"', xml)
    xml = re.sub(r'<Wait wait_duration="[^"]*"', f'<Wait wait_duration="{rc["wait_s"]}"', xml)
    xml = re.sub(r'<BackUp backup_dist="[^"]*" backup_speed="[^"]*"',
                 f'<BackUp backup_dist="{rc["backup_dist"]}" backup_speed="{rc["backup_speed"]}"', xml)
    return xml


RUNTIME_IMAGE_SIZE = (640, 480)      # /image_raw 크기(2차 API p17). Config.camera 의 width/height/K 는 이 크기 기준이다


def camera_overlay(p: Profile, ci: Dict[str, Any]) -> Dict[str, Any]:
    """프로파일의 카메라 → Config.camera 덮어쓰기(7장). 보정 전이면 기존과 같다(width, height, hfov_deg).
    보정했으면 K 를 쓰되, 보정 해상도가 640×480 이 아니면 K 를 (가로·세로 각각) 비례 조정해 640×480 기준으로 낸다.
    가로세로 비가 달라 비례 조정이 안 되면 거부하고 경고한다 — K·D 없이 hfov 핀홀로 둔다(실제 영상과 어긋난 K 를 쓰지 않는다).
    어안(model=fisheye)이고 D 가 4개면 model·D 를 같이 낸다. 유효 화각 sensors.camera.usable_hfov_deg 가 있으면 그것도."""
    rw, rh = RUNTIME_IMAGE_SIZE
    w, h = int(ci["width"]), int(ci["height"])
    cam_cfg: Dict[str, Any] = {"width": w, "height": h, "hfov_deg": p.get("sensors.camera.hfov_deg")}
    if not ci["calibrated"]:
        return cam_cfg
    K = [float(x) for x in ci["K"]]
    if (w, h) != (rw, rh):
        sx, sy = rw / w, rh / h
        if abs(sx / sy - 1.0) > 0.01:
            warnings.warn(f"카메라 보정 해상도 {w}×{h} 는 {rw}×{rh} 와 가로세로 비가 달라 K 를 비례 조정할 수 없다 — K·D 를 쓰지 않고 hfov 핀홀로 둔다. "
                          f"{rw}×{rh} 로 다시 보정할 것", stacklevel=2)
            cam_cfg.update(width=rw, height=rh)
            return cam_cfg
        K = [K[0] * sx, 0.0, K[2] * sx, 0.0, K[4] * sy, K[5] * sy, 0.0, 0.0, 1.0]
        cam_cfg.update(width=rw, height=rh)
    cam_cfg["K"] = K                                           # 체커보드로 잰 K: CameraModel 이 hfov 대신 fx, fy, 주점을 쓴다
    D = ci.get("D") or []
    if ci.get("model") == "fisheye" and len(D) == 4:
        cam_cfg["model"], cam_cfg["D"] = "fisheye", [float(x) for x in D]
    usable = p.get("sensors.camera.usable_hfov_deg")
    if usable is not None:
        cam_cfg["usable_hfov_deg"] = float(usable)
    return cam_cfg


def gen_config_overlay(p: Profile) -> Dict[str, Any]:
    n = derive_nav(p)
    ci = camera_intrinsics(p)
    di = depth_intrinsics(p)
    cam_cfg = camera_overlay(p, ci)
    depth_cfg = {"hfov_deg": p.get("sensors.depth.hfov_deg"), "K_width": di["width"], "K_height": di["height"],
                 "unit_scale": p.get("sensors.depth.unit_scale"), "min_range": p.get("sensors.depth.min_range"),
                 "max_range": p.get("sensors.depth.max_range")}
    if di["calibrated"]:
        depth_cfg["K"] = di["K"]
    return {
        # 차체→팬→틸트→센서 기구학 = URDF/TF (깊이 3D 지도가 카메라 자세를 구하는 데 쓴다)
        "rig": {"base_z": p.get("model.wheels.radius"), "pan_xyz": p.get("model.pan_tilt.xyz"),
                "camera_xyz": p.get("sensors.camera.xyz"), "camera_rpy": p.get("sensors.camera.rpy"),
                "depth_xyz": p.get("sensors.depth.xyz"), "depth_rpy": p.get("sensors.depth.rpy"),
                "lidar_xyz": p.get("sensors.lidar.xyz")},
        "depth": depth_cfg,
        "robot": {"v_max": round(p.get("drive.v_max"), 4), "w_max": round(p.get("drive.w_max"), 4),
                  "a_max": round(p.get("drive.a_accel"), 4), "radius": round(p.circumscribed_radius(), 4),
                  "length": p.get("model.size.x"), "width": p.get("model.size.y"), "height": p.get("model.size.z"),
                  # 차동 구동 변환(hal/diffdrive.py): 바퀴 간격·반경, 슬립 효율, 직진 편차 → Agent 가 지령에서 미리 보상한다
                  "track_width": p.get("model.wheels.track"), "wheel_radius": p.get("model.wheels.radius"),
                  "yaw_eff": round(p.get("drive.pivot.yaw_efficiency"), 4),
                  "lr_bias": round(p.get("drive.lr_bias_rad_per_m"), 5),
                  "pan_limit": p.get("model.pan_tilt.pan_limit_deg"), "tilt_limit": p.get("model.pan_tilt.tilt_limit_deg")},
        "camera": cam_cfg,
        "nav": {"v_cruise": round(n["desired_linear_vel"], 4), "stop_dist": round(n["stop_dist"], 4), "slow_dist": round(n["slow_dist"], 4),
                "inflate_hard": round(n["inflate_hard"], 3), "rotate_clear": round(n["rotate_clear"], 3), "side_lim": round(n["side_lim"], 3)},
        "lidar_err": {**{k: v for k, v in p.get("sensors.lidar.error").items()},
                      "status": "measured" if all(p.src(f"sensors.lidar.error.{k}") == "measured" for k in
                                                  ("range_bias_m", "range_sigma_m", "angle_sigma_deg", "mount_xy_m", "mount_yaw_deg", "latency_s", "pose_sigma_m")) else "assumed",
                      "source": {k: p.src(f"sensors.lidar.error.{k}") for k in
                                 ("range_bias_m", "range_sigma_m", "angle_sigma_deg", "mount_xy_m", "mount_yaw_deg", "latency_s", "pose_sigma_m")}},
        "nav2": {"cmd_timeout": round(n["cmd_timeout"], 3)},
    }


def gen_sim_drive(p: Profile) -> Dict[str, Any]:
    """측정한 값으로 2D 시뮬을 실기에 가깝게: SimDrive 필드 (config robot.* 는 robot_calibrated.json)."""
    d, s = p.get("drive"), p.get("sensors")
    loss = max(0.0, 1.0 - d["pivot"]["yaw_efficiency"])
    d["surfaces"].get("default", {})
    surfaces = {}
    for name, r in d["surfaces"].items():
        acc = (r.get("a_accel") or d["a_accel"]) / d["a_accel"]
        brk = (r.get("a_brake") or d["a_brake"]) / d["a_brake"]
        surfaces[name] = [float(r.get("slip") or 0.0), float(acc), float(brk)]
    surfaces.setdefault("default", [0.0, 1.0, 1.0])
    return {"a_brake": d["a_brake"], "alpha": d["alpha_max"], "lag_s": d["cmd_latency_s"], "lr_bias": d["lr_bias_rad_per_m"],
            "pivot_loss": loss, "pivot_drift": d["pivot"]["drift_m_per_rad"],
            "odom_scale": s["odom"]["lin_scale"], "odom_yaw_scale": s["odom"]["yaw_scale"] * (1.0 - loss),
            "gyro_bias": s["odom"]["gyro_bias_z"], "imu_bias": s["imu"]["gyro_bias_z"], "imu_noise": s["imu"]["gyro_noise_z"],
            "imu_lag_s": s["imu"]["latency_s"], "lidar_xy": [s["lidar"]["xyz"][0], s["lidar"]["xyz"][1]], "lidar_yaw": s["lidar"]["rpy"][2],
            "surface": "default", "surfaces": surfaces}


# ====================================================================== 일괄
def generate_all(p: Profile, root: str, log=print) -> List[str]:
    """root = 프로젝트 루트(maicon2026). 쓴 파일 경로 목록."""
    import yaml  # type: ignore
    written: List[str] = []

    def w(rel: str, text: str) -> None:
        path = os.path.join(root, rel)
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w", encoding="utf-8", newline="\n") as f:
            f.write(text)
        written.append(rel)
        log(f"[generate] {rel}")

    dump = lambda d: yaml.safe_dump(d, allow_unicode=True, sort_keys=False, default_flow_style=None)
    w("ros2/urdf/wingbot_ugv.urdf", gen_urdf(p))
    w("config/sim_drive.json", json.dumps(gen_sim_drive(p), ensure_ascii=False, indent=2) + "\n")
    w("calib/outputs/drive_calibration.yaml", _banner(p) + dump(gen_drive_table(p)))
    w("calib/outputs/drive_calibration.md", "# 구동 보정표\n\n" + drive_table_md(p))
    w("calib/outputs/sensor_calibration.yaml", _banner(p) + dump(gen_sensor_calibration(p)))
    w("ros2/config/camera_info.yaml", _banner(p) + dump(gen_camera_info(p)))
    w("ros2/config/depth_camera_info.yaml", _banner(p) + dump(gen_depth_camera_info(p)))
    with open(os.path.join(root, "ros2/config/nav2_params.yaml"), "r", encoding="utf-8") as f:
        base = yaml.safe_load(f)
    w("ros2/config/nav2_params.calibrated.yaml", _banner(p) + dump(gen_nav2_params(p, base)))
    with open(os.path.join(root, "ros2/behavior_trees/navigate_replan_fast.xml"), "r", encoding="utf-8") as f:
        w("ros2/behavior_trees/navigate_replan_fast.calibrated.xml", gen_bt_xml(p, f.read()))
    w("config/robot_calibrated.json", json.dumps(gen_config_overlay(p), ensure_ascii=False, indent=2) + "\n")
    return written

