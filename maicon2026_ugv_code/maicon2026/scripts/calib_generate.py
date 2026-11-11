#!/usr/bin/env python3
"""프로파일 → 산출물 (URDF, 시뮬 구동, 구동 보정표, 센서 보정, camera_info, Nav2 파라미터·BT, Config 덮어쓰기).

  python scripts/calib_generate.py            # calib/robot_profile.yaml 사용 (없으면 PDF+가정 기본값으로 먼저 만든다)
적용:
  python3 scripts/run_real.py ... --config config/robot_calibrated.json
  ros2 launch ros2/launch/maicon_nav.launch.py params_file:=$PWD/ros2/config/nav2_params.calibrated.yaml \\
       bt_xml:=$PWD/ros2/behavior_trees/navigate_replan_fast.calibrated.xml
  python scripts/run_sim.py --config config/robot_calibrated.json      (시뮬에서 측정한 구동 특성까지 쓰려면 sim_drive.json 을 SimDrive.load)
"""
import argparse, os, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from maicon26.calib.generate import generate_all
from maicon26.calib.profile import Profile

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--profile", default=os.path.join(ROOT, "calib", "robot_profile.yaml"))
    a = ap.parse_args()
    if os.path.exists(a.profile):
        prof = Profile.load(a.profile)
    else:
        prof = Profile()
        os.makedirs(os.path.dirname(a.profile), exist_ok=True)
        prof.save(a.profile)
        print(f"[generate] 프로파일이 없어 기본값(PDF + 가정)으로 만들었다: {a.profile}")
    files = generate_all(prof, ROOT)
    print(f"\n{len(files)}개 파일 생성. 가정(assumed) 값 {len(prof.assumed())}개 남음 → scripts/calib_report.py")


if __name__ == "__main__":
    main()
