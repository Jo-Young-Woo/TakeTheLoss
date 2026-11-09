#!/usr/bin/env python3
"""2D 시뮬 + 실제 ROS2 SLAM/Nav2 (WSL Ubuntu 22.04, Humble). 한 대의 UGV 를 ROS 로 구동한다.

터미널 1:  source /opt/ros/humble/setup.bash
           ros2 launch ros2/launch/maicon_nav.launch.py slam:=cartographer use_sim_time:=true
터미널 2:  source /opt/ros/humble/setup.bash
           python3 scripts/run_sim_ros.py --robot B2 --seed 1 [--t-base 40] [--shuffle 20 35]
(또는 scripts/ros_sim_demo.sh 가 둘을 한 번에 실행한다.)

--t-base: 목표거점 명령 시각(기본 240 s). 시험을 빨리 보려면 40 처럼 줄인다(탐색 시간이 줄 뿐 나머지 규칙은 같다).
"""
import argparse, math, os, sys, time
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import numpy as np
from maicon26.config import Config
from maicon26.sim.run import build_match, score
from maicon26.sim.dynamic import shuffle_structures


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot", default="B2", help="B1..B3 / R1..R3 (한 대만)")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--t-base", type=float, default=None, help="목표거점 명령 시각(s)")
    ap.add_argument("--max-t", type=float, default=340.0)
    ap.add_argument("--shuffle", type=float, nargs="*", default=None, help="구조물 재배치 시각(s)")
    ap.add_argument("--speed", type=float, default=1.0, help="벽시계 대비 시뮬 배속 상한(CPU 가 못 따라가면 느려질 뿐 일관성은 유지)")
    ap.add_argument("--grid-source", choices=["scan", "slam_map"], default="scan")
    ap.add_argument("--low-barrier", action="store_true",
                    help="LiDAR 평면 아래의 낮은 장벽(12 cm)을 BLUE 시작과 중앙 사이에 둔다 — 깊이 3D 지도가 Nav2 비용지도(/depth_obstacles_cloud)로 전달되는지 시험")
    ap.add_argument("--no-depth", action="store_true", help="깊이 카메라/3D 지도 끄기")
    a = ap.parse_args()

    from maicon26.sim.ros_bridge import RosSimBridge
    cfg = Config()
    cfg.mission.spin_w, cfg.mission.tour_nx, cfg.mission.tour_ny = 1.8, 3, 3
    cfg.nav.backend = "nav2"
    cfg.nav2.use_sim_time = True
    cfg.nav2.grid_source = a.grid_source
    if a.t_base:
        cfg.mission.t_base_cmd = a.t_base
        cfg.mission.prestage_at = max(5.0, a.t_base - 18.0)
    if a.no_depth:
        cfg.depth.enable = False
    m = build_match(seed=a.seed, cfg=cfg, teams=[a.robot])
    w, agent, robot = m.world, m.agents[0], m.robots[0]
    if a.low_barrier:
        w.add_low_obstacle(1.55, 0.55, 1.75, 4.45, 0.12)
    bridge = RosSimBridge(w, robot)
    pend = sorted(a.shuffle or [])
    dt, err, last_print = 0.05, [], -10.0
    wall0 = time.time()
    print(f"[ros-sim] {agent.name} nav2 backend, t_base={cfg.mission.t_base_cmd}", flush=True)
    try:
        while not w.finished and w.t < a.max_t:
            t_wall = time.time()
            while pend and w.t >= pend[0]:
                pend.pop(0)
                shuffle_structures(w)
                print(f"[ros-sim] t={w.t:.1f} 구조물 재배치 (박스 {len(w.cubes)}개)", flush=True)
            bridge.publish()
            agent.tick()
            if agent.stack.initialized:
                p, q = agent.stack.pose, robot.u.pose
                err.append(math.hypot(p.x - q.x, p.y - q.y))
            w.step(dt)
            if w.t - last_print >= 10.0:
                last_print = w.t
                p = robot.u.pose
                print(f"[ros-sim] t={w.t:6.1f} mode={agent.mode.value:7s} pose=({p.x:.2f},{p.y:.2f}) "
                      f"est_err={(err[-1]*100 if err else float('nan')):.1f}cm cmd_rx={bridge.n_cmd} "
                      f"{agent.stack.diagnostics()}", flush=True)
            if agent.mode.value == "DONE":
                break
            slack = dt / a.speed - (time.time() - t_wall)
            if slack > 0:
                time.sleep(slack)
    finally:
        agent.stack.close()
        bridge.close()
        from maicon26.nav.ros_runtime import shutdown as ros_shutdown
        ros_shutdown()
    v = score(m)["agents"][agent.name]
    e = np.array(err) if err else np.array([float("nan")])
    print("=" * 70)
    print(f"[ros-sim] 종료 t={w.t:.1f}  벽시계 {time.time()-wall0:.0f}s")
    print(f"[ros-sim] M5 거점 진입: {v['m5_reached']}   충돌: {v['collisions']}   주행거리: {v['distance']} m")
    print(f"[ros-sim] 위치오차(SLAM+앵커 vs 참값) 중앙값 {np.nanmedian(e)*100:.1f} cm  p95 {np.nanpercentile(e,95)*100:.1f} cm  최대 {np.nanmax(e)*100:.1f} cm")
    print(f"[ros-sim] 진단: {agent.stack.diagnostics()}")
    if agent.fusion is not None:
        r, s = agent.map3d_report(), agent.stats
        pts = getattr(agent.stack.client, "_depth_pts", None)
        print(f"[ros-sim] 3D 지도: 깊이 {s['depth_frames']}장, 점유 복셀 {r['occupied_voxels']}, 항법 층 {s['nav_layer_cells']}셀, "
              f"Nav2 로 발행한 점 {0 if pts is None else len(pts)}개, 전방 가드 {s['depth_guard_ticks']}틱, 깊이↔LiDAR 잔차 {r['lidar_resid_m']} m")
    for t, msg in agent.log[-12:]:
        print("   log", t, msg)
    print(f"[ros-sim] 접촉 감지 {agent.stats['bump_events']}회, RECOVER {agent.stats['recoveries']}회")
    for t, msg in [x for x in agent.log if any(k in x[1] for k in ("BUMP", "RECOVER", "SAFE_STOP", "mode "))][:40]:
        print("   evt", t, msg[:150])


if __name__ == "__main__":
    main()
