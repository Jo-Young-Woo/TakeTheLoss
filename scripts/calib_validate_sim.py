#!/usr/bin/env python3
"""시뮬 기록(숨은 정답이 있는)으로 분석 정확도를 검증하고, 같은 기록으로 산출물 일체를 만들어 본다.

  python scripts/calib_validate_sim.py --records calib/records/demo_sim_ros
결과: <records>/fitted_profile.yaml, fit_report.md, truth_comparison.md, outputs/ (URDF·보정표·센서·Nav2·Config)
실기 기록에는 쓰지 않는다(정답이 없다). 분석 코드가 정답을 어느 정도 오차로 복원하는지 보여주는 용도.
"""
import argparse, os, shutil, sys, tempfile
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from maicon26.calib.analysis import fit_run
from maicon26.calib.generate import generate_all
from maicon26.calib.profile import Profile
from maicon26.calib.records import load_run
from maicon26.sim.drive import truth_for_tests

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))

# (프로파일 경로, 숨은 정답, 허용 상대오차, 설명)
D = truth_for_tests()
ROBOT = dict(v_max=0.52, a_max=0.9, w_max=2.6)
CHECKS = [
    ("drive.v_max", ROBOT["v_max"], 0.03, "최대 속도"),
    ("drive.a_accel", ROBOT["a_max"], 0.10, "가속도"),
    ("drive.a_brake", D.a_brake, 0.25, "제동 감속도 (0.3 s 미만 과도 → 표본 주기 한계로 오차 큼; 정지거리 표를 우선)"),
    ("drive.cmd_latency_s", D.lag_s, 0.35, "명령 지연 (분해능 ≈ 20 ms)"),
    ("drive.w_max", ROBOT["w_max"] * (1 - D.pivot_loss), 0.05, "최대 각속도(실제 회전 = 명령 × 효율)"),
    ("drive.alpha_max", D.alpha * (1 - D.pivot_loss), 0.30, "각가속도 (0.3 s 미만 과도 → 오차 큼)"),
    ("drive.lr_bias_rad_per_m", D.lr_bias, 0.05, "직진 좌우 편차"),
    ("drive.pivot.yaw_efficiency", 1 - D.pivot_loss, 0.05, "제자리 회전 효율"),
    ("drive.pivot.drift_m_per_rad", D.pivot_drift, 0.20, "회전 중 밀림"),
    ("sensors.odom.lin_scale", D.odom_scale, 0.01, "오도메트리 선형 스케일"),
    ("sensors.odom.yaw_scale", D.odom_yaw_scale / (1 - D.pivot_loss), 0.02, "오도메트리 회전 스케일(참 대비)"),
    ("sensors.odom.gyro_bias_z", D.gyro_bias, 0.15, "오도메트리 yaw 편향"),
    ("sensors.imu.gyro_bias_z", D.imu_bias, 0.05, "IMU 자이로 편향"),
    ("sensors.imu.gyro_noise_z", D.imu_noise, 0.30, "IMU 자이로 잡음"),
    ("sensors.imu.latency_s", D.imu_lag_s, 0.60, "IMU 지연 (표본 단위 편향 ≈ 20 ms 포함)"),
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--records", required=True)
    a = ap.parse_args()
    rec = os.path.abspath(a.records)
    P, lines = fit_run(Profile(), load_run(rec), source=os.path.basename(rec))
    P.save(os.path.join(rec, "fitted_profile.yaml"))
    with open(os.path.join(rec, "fit_report.md"), "w", encoding="utf-8") as f:
        f.write("# 분석 결과 (시뮬 기록)\n\n" + "\n".join(lines) + "\n")
    rows = ["# 분석 결과 vs 숨은 정답 (시뮬)\n", "| 항목 | 정답 | 측정 | 오차 | 허용 | 판정 | 비고 |", "|---|---|---|---|---|---|---|"]
    ok = 0
    for path, truth, tol, note in CHECKS:
        m = P.get(path)
        err = abs(m - truth) / abs(truth)
        good = err <= tol
        ok += good
        rows.append(f"| `{path}` | {truth:.4g} | {m:.4g} | {err * 100:.1f} % | {tol * 100:.0f} % | {'✅' if good else '❌'} | {note} |")
    x, y, _ = P.get("sensors.lidar.xyz")
    yaw = P.get("sensors.lidar.rpy")[2]
    for name, m, truth, tol in (("LiDAR x [m]", x, D.lidar_xy[0], 0.006), ("LiDAR y [m]", y, D.lidar_xy[1], 0.006), ("LiDAR yaw [rad]", yaw, D.lidar_yaw, 0.005)):
        good = abs(m - truth) <= tol
        ok += good
        rows.append(f"| {name} | {truth:.4g} | {m:.4g} | {abs(m - truth):.4f} (절대) | {tol} | {'✅' if good else '❌'} | lidar_square |")
    rows.append(f"\n**{ok}/{len(CHECKS) + 3} 항목 허용 범위 안.**")
    with open(os.path.join(rec, "truth_comparison.md"), "w", encoding="utf-8") as f:
        f.write("\n".join(rows) + "\n")
    # 산출물 일체를 임시 루트에서 만들어 records/outputs 로 복사
    with tempfile.TemporaryDirectory() as tmp:
        for rel in ("ros2/config/nav2_params.yaml", "ros2/behavior_trees/navigate_replan_fast.xml"):
            os.makedirs(os.path.dirname(os.path.join(tmp, rel)), exist_ok=True)
            shutil.copy(os.path.join(ROOT, rel), os.path.join(tmp, rel))
        generate_all(P, tmp, log=lambda s: None)
        out = os.path.join(rec, "outputs")
        shutil.rmtree(out, ignore_errors=True)
        for dp, _, fs in os.walk(tmp):
            for fn in fs:
                rel = os.path.relpath(os.path.join(dp, fn), tmp)
                if rel in ("ros2/config/nav2_params.yaml", "ros2/behavior_trees/navigate_replan_fast.xml"):
                    continue
                os.makedirs(os.path.dirname(os.path.join(out, rel)), exist_ok=True)
                shutil.copy(os.path.join(dp, fn), os.path.join(out, rel))
    print("\n".join(rows))
    print(f"\n저장: {rec} (fitted_profile.yaml, fit_report.md, truth_comparison.md, outputs/)")


if __name__ == "__main__":
    main()
