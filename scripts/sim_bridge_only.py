#!/usr/bin/env python3
"""시뮬 로봇만 ROS 로 노출(Agent 없음) — 보정 시험(scripts/calib_run.py --io ros)을 PC 에서 리허설/검증할 때.

  source /opt/ros/humble/setup.bash
  python3 scripts/sim_bridge_only.py                       # /scan /odom /imu /ground_truth 발행, /cmd_vel 수신, 벽시계 실시간
  python3 scripts/calib_run.py --io ros --truth-topic /ground_truth --teleport-topic /sim/teleport --non-interactive --experiments all
숨은 정답은 sim/drive.py truth_for_tests (또는 --drive JSON). 시험 결과가 이 정답을 복원하는지 보는 용도.
"""
import argparse, os, sys, time
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from maicon26.config import Config
from maicon26.sim.drive import SimDrive, truth_for_tests
from maicon26.sim.robot import SimRobot
from maicon26.sim.world import World


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--drive", default=None, help="SimDrive JSON (기본: 시험용 숨은 정답)")
    ap.add_argument("--surface", default="default")
    ap.add_argument("--seconds", type=float, default=1e9)
    ap.add_argument("--dt", type=float, default=0.02)
    a = ap.parse_args()
    from maicon26.sim.ros_bridge import RosSimBridge
    from maicon26.nav.ros_runtime import shutdown
    cfg = Config()
    cfg.mission.t_total = 1e9
    cfg.robot.v_max, cfg.robot.a_max, cfg.robot.w_max = 0.52, 0.9, 2.6
    w = World(cfg, seed=0)
    w.cubes.clear()
    u = w.add_ugv("BLUE", 2, cfg)
    u.drive = SimDrive.load(a.drive) if a.drive else truth_for_tests()
    u.drive.surface = a.surface
    br = RosSimBridge(w, SimRobot(w, u), sim_clock=False, wall_stamps=True, redirect_agent_cmd=False)
    print("[bridge] 시작: /scan /odom /imu /ground_truth 발행, /cmd_vel /sim/teleport /sim/add_box 수신", flush=True)
    nxt = time.monotonic()
    try:
        while w.t < a.seconds:
            br.publish()
            w.step(a.dt)
            nxt += a.dt
            slack = nxt - time.monotonic()
            if slack > 0:
                time.sleep(slack)
            else:
                nxt = time.monotonic()
    except KeyboardInterrupt:
        pass
    finally:
        br.close()
        shutdown()


if __name__ == "__main__":
    main()
