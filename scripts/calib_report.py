#!/usr/bin/env python3
"""프로파일의 출처별 현황과 '아직 가정인 값' 목록(= 실기에서 측정할 것)을 보여준다.

  python scripts/calib_report.py [--profile calib/robot_profile.yaml]
"""
import argparse, collections, os, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from maicon26.calib.profile import Profile

# 가정 값 → 어떤 시험으로 채우는가
HOW = [("drive.v_max", "speed_steps"), ("drive.v_max_reverse", "speed_steps"), ("drive.a_accel", "speed_steps"),
       ("drive.a_brake", "brake"), ("drive.stop_distance", "brake"), ("drive.w_max", "turn_steps"), ("drive.alpha_max", "turn_steps"),
       ("drive.cmd_latency_s", "speed_steps"), ("drive.rise_time_s", "speed_steps"), ("drive.lr_bias", "straight"),
       ("drive.pivot", "rotate"), ("drive.surfaces", "straight/brake/speed_steps (주행면별) + 견인 시험"),
       ("sensors.lidar.xyz", "lidar_square (x,y) + 줄자 (z)"), ("sensors.lidar.rpy", "lidar_square"),
       ("sensors.lidar.latency_s", "calib_run 이 scan_lat 기록"), ("sensors.camera.K", "calib_fit.py --camera-images (체커보드)"),
       ("sensors.camera.D", "calib_fit.py --camera-images"), ("sensors.camera.xyz", "줄자/CAD"), ("sensors.camera.rpy", "체커보드 외부 파라미터/CAD"),
       ("sensors.imu", "static + turn_steps (xyz/rpy 는 CAD)"), ("sensors.odom", "static/speed_steps/straight/rotate"),
       ("sensors.depth", "CAD"), ("model.com", "매달아 균형점 측정(또는 CAD)"), ("model.wheels", "캘리퍼/CAD"),
       ("model.chassis_height", "줄자"), ("model.inertia_override", "이중 진자 시험(또는 CAD) — 없으면 직육면체 근사")]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--profile", default=os.path.join(os.path.dirname(__file__), "..", "calib", "robot_profile.yaml"))
    a = ap.parse_args()
    p = Profile.load(a.profile) if os.path.exists(a.profile) else Profile()
    cnt = collections.Counter(p.src(path) for path, _ in p.leaves())
    print("출처별 값 개수:", dict(cnt))
    asm = p.assumed()
    print(f"\n아직 가정인 값 {len(asm)}개 — 실기 측정으로 교체:")
    for path, v in asm:
        how = next((h for k, h in HOW if path.startswith(k)), "")
        print(f"  {path:42s} = {str(v)[:40]:40s}  ← {how}")


if __name__ == "__main__":
    main()
