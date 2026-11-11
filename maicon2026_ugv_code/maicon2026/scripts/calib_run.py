#!/usr/bin/env python3
"""보정 시험 실행 + 기록(CSV/JSON, ROS 모드는 rosbag 도).

실기(ROS2, 로봇 드라이버가 /cmd_vel /odom /imu /scan 을 주고받는 상태):
  python3 scripts/calib_run.py --io ros --surface tile --experiments static speed_steps turn_steps straight rotate brake
  python3 scripts/calib_run.py --io ros --surface tile --experiments lidar_square --d-front 1.20 --d-left 0.80
  python3 scripts/calib_run.py --io ros --surface carpet --experiments straight brake speed_steps      # 주행면별 반복
  (참값: 모캡/외부 추적이 있으면 --truth-topic /ground_truth. 없으면 줄자·각도기로 잰 값을 시험 끝마다 입력하거나 --manual-truth 파일)
시뮬(ROS 불필요, 분석 코드 검증용 — 숨은 정답은 sim/drive.py truth_for_tests):
  python scripts/calib_run.py --io sim --surface default carpet --experiments all

시험별 필요 공간·목적은 maicon26/calib/experiments.py 머리말.
"""
import argparse, datetime, os, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from maicon26.calib.experiments import EXPERIMENTS, run_experiments
from maicon26.calib.records import RosbagRecorder


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--io", choices=["sim", "ros"], default="ros")
    ap.add_argument("--experiments", nargs="+", default=["all"], help="all 또는 " + " ".join(EXPERIMENTS))
    ap.add_argument("--surface", nargs="+", default=["default"], help="주행면 이름(바닥 재질). 여러 개면 면마다 반복 — 면을 바꿀 때 Enter 대기")
    ap.add_argument("--out", default=None, help="기록 폴더 (기본 calib/records/<시각>_<io>)")
    ap.add_argument("--no-bag", action="store_true", help="rosbag 기록 끄기")
    ap.add_argument("--truth-topic", default=None, help="참값 nav_msgs/Odometry 토픽 (모캡 등)")
    ap.add_argument("--manual-truth", default=None, help="시험 이름→{distance_m,lateral_m,heading_deg} JSON")
    ap.add_argument("--teleport-topic", default=None, help="시뮬 브리지 전용(/sim/teleport): 시작 자세로 자동 이동")
    ap.add_argument("--non-interactive", action="store_true")
    ap.add_argument("--use-sim-time", action="store_true")
    ap.add_argument("--cmd-topic", default="/cmd_vel"); ap.add_argument("--odom-topic", default="/odom")
    ap.add_argument("--imu-topic", default="/imu"); ap.add_argument("--scan-topic", default="/scan")
    ap.add_argument("--d-front", type=float, default=None, help="lidar_square: 정면 벽까지 거리(차체 중심, m)")
    ap.add_argument("--d-left", type=float, default=None, help="lidar_square: 왼쪽 벽까지 거리(m)")
    ap.add_argument("--sim-drive", default=None, help="시뮬 숨은 정답 JSON (기본: sim/drive.py truth_for_tests)")
    a = ap.parse_args()

    which = list(EXPERIMENTS) if "all" in a.experiments else a.experiments
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    root = os.path.join(os.path.dirname(__file__), "..")
    out = a.out or os.path.join(root, "calib", "records", f"{stamp}_{a.io}")
    kwargs = {"lidar_square": {"d_front": a.d_front, "d_left": a.d_left}}
    for i, surf in enumerate(a.surface):
        if a.io == "sim":
            from maicon26.calib.robot_io import SimIO
            from maicon26.sim.drive import SimDrive, truth_for_tests
            drive = SimDrive.load(a.sim_drive) if a.sim_drive else truth_for_tests()
            io = SimIO(drive, robot=dict(v_max=0.52, a_max=0.9, w_max=2.6), surface=surf)
            rec = None
        else:
            from maicon26.calib.robot_io import RosIO
            io = RosIO(a.cmd_topic, a.odom_topic, a.imu_topic, a.scan_topic, a.truth_topic, a.use_sim_time,
                       a.manual_truth, not a.non_interactive, teleport_topic=a.teleport_topic)
            rec = RosbagRecorder(out, enabled=not a.no_bag)
            if i > 0 and not a.non_interactive and sys.stdin.isatty():
                input(f"\n주행면을 '{surf}' 로 바꾸고 Enter: ")
        try:
            run_experiments(io, out, which, surface=surf, recorder=rec, kwargs=kwargs,
                            info={"tool": "calib_run", "date": stamp, "surfaces": a.surface})
        finally:
            io.close()
    if a.io == "ros":
        from maicon26.nav.ros_runtime import shutdown
        shutdown()
    print(f"\n기록 저장: {os.path.abspath(out)}")
    print(f"다음: python scripts/calib_fit.py --records {os.path.relpath(out)}")


if __name__ == "__main__":
    main()
