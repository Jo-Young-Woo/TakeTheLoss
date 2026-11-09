#!/usr/bin/env python3
"""Run the real mission Agent against Gazebo driving, LiDAR and Nav2.

Use CONTROL=agent with gazebo_nav_demo.sh. The local match link exercises
START, command parsing, submissions and STOP without contacting a match server.
"""
from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
import time
from dataclasses import replace
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from maicon26.config import Config  # noqa: E402
from maicon26.hal.gazebo import GazeboRobot  # noqa: E402
from maicon26.link.base import ServerLink, ServerMsg  # noqa: E402
from maicon26.mission.manager import Agent  # noqa: E402
from maicon26.nav.ros_runtime import shutdown  # noqa: E402


def gazebo_truth():
    """Read Gazebo's model pose for evaluation only; Agent never sees it."""
    try:
        proc = subprocess.run(
            ["ign", "topic", "-e", "-n", "1", "--json-output", "-t",
             "/world/maicon_arena/dynamic_pose/info"],
            capture_output=True, text=True, timeout=5, check=True)
        poses = json.loads(proc.stdout)["pose"]
        p = next(p for p in poses if p["name"] == "wingbot_ugv")
        pos, q = p["position"], p["orientation"]
        x, y, z, w = (float(q.get(k, 0.0)) for k in ("x", "y", "z", "w"))
        th = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        return {"x": float(pos.get("x", 0.0)), "y": float(pos.get("y", 0.0)), "th": th}
    except (OSError, ValueError, KeyError, StopIteration, subprocess.SubprocessError):
        return None


class TrialServer(ServerLink):
    def __init__(self, robot: GazeboRobot, team: str, slot: int, duration: float) -> None:
        self.robot, self.team, self.slot, self.duration = robot, team, slot, duration
        self.started = self.stopped = False
        self.start_t = 0.0
        self.submissions = {}

    def identity(self):
        return self.team, self.slot

    def poll(self):
        now = self.robot.now()
        if not self.started:
            self.started, self.start_t = True, now
            return [ServerMsg("start", stamp=now),
                    ServerMsg("command", stamp=now, text=(
                        "작전 명령. 우선 타격 표적은 레이더시설. ALPHA, BRAVO 섹터를 정찰하라. "
                        "이후 정비소로 이동 준비."))]
        if not self.stopped and now - self.start_t >= self.duration:
            self.stopped = True
            return [ServerMsg("stop", stamp=now)]
        return []

    def submit(self, kind, payload):
        self.submissions[kind] = payload
        return True


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--robot", choices=[f"{s}{n}" for s in "BR" for n in (1, 2, 3)], default="B2")
    ap.add_argument("--duration", type=float, default=45.0, help="simulated mission seconds before STOP")
    ap.add_argument("--report", type=Path, default=ROOT / "ros2/gazebo/agent_trial.json")
    args = ap.parse_args()
    if args.duration <= 0:
        ap.error("--duration must be positive")
    team, slot = ("BLUE" if args.robot[0] == "B" else "RED"), int(args.robot[1])
    cfg = Config()
    cfg.team, cfg.slot = team, slot
    cfg.arena = replace(cfg.arena, base_depth=.4, base_height=1.2)
    cfg.nav.backend = "nav2"
    cfg.nav2.use_sim_time = True
    bx, by = cfg.arena.base_center(f"{args.robot[0]}-{slot}")
    cfg.nav2.start_pose_override = (bx + (.15 if team == "BLUE" else -.15), by,
                                    0.0 if team == "BLUE" else math.pi)
    cfg.nav2.grid_source = "scan"
    cfg.nav2.publish_depth_obstacles = False
    cfg.depth.enable = cfg.map3d.enable = False
    robot = GazeboRobot()
    agent = None
    wall0 = time.monotonic()
    try:
        while not robot.ready() and time.monotonic() - wall0 < 25:
            time.sleep(.1)
        if not robot.ready():
            raise RuntimeError("Gazebo /clock, /scan, /odom not ready; launch CONTROL=agent first")
        server = TrialServer(robot, team, slot, args.duration)
        agent = Agent(robot, server, cfg=cfg, name=args.robot)
        start = robot.now()
        first_odom = robot.get_odom()
        prev_odom = first_odom
        path_m = 0.0
        next_print = start
        last_log = 0
        wall_limit = max(45.0, args.duration * 3)
        while time.monotonic() - wall0 < wall_limit:
            agent.tick()
            odom = robot.get_odom()
            step = math.hypot(odom.x - prev_odom.x, odom.y - prev_odom.y)
            if step < .2:
                path_m += step
            prev_odom = odom
            now = robot.now()
            if now >= next_print:
                p = agent.stack.pose
                print(f"[gazebo-agent] t={now-start:5.1f}s mode={agent.mode.value:9s} "
                      f"pose=({p.x:.2f},{p.y:.2f},{p.th:.2f}) "
                      f"cmd=({robot.last_cmd[0]:+.2f},{robot.last_cmd[1]:+.2f}) "
                      f"path={path_m:.2f}m init={agent.stack.initialized}", flush=True)
                next_print = now + 5
            for t, msg in agent.log[last_log:]:
                print(f"  [{t:6.2f}] {msg}", flush=True)
            last_log = len(agent.log)
            if agent.mode.value == "DONE":
                break
            time.sleep(1.0 / cfg.loop_hz)
        robot.stop()
        scan = robot.get_scan()
        valid = scan.valid() if scan is not None else []
        truth = gazebo_truth()
        estimate = agent.stack.pose
        result = {
            "robot": args.robot, "requested_duration_s": args.duration,
            "elapsed_sim_s": round(robot.now() - start, 2),
            "elapsed_wall_s": round(time.monotonic() - wall0, 2),
            "mode": agent.mode.value, "initialized": agent.stack.initialized,
            "path_m": round(path_m, 3),
            "start_odom": vars(first_odom), "end_odom": vars(robot.get_odom()),
            "estimated_pose": vars(agent.stack.pose),
            "gazebo_truth_pose": truth,
            "pose_error_m": round(math.hypot(estimate.x - truth["x"], estimate.y - truth["y"]), 3)
            if truth is not None else None,
            "yaw_error_rad": round(math.atan2(math.sin(estimate.th - truth["th"]),
                                               math.cos(estimate.th - truth["th"])), 3)
            if truth is not None else None,
            "cmd_count": robot.cmd_count, "nonzero_cmd_count": robot.nonzero_count,
            "max_cmd_v": robot.max_cmd_v, "last_cmd": robot.last_cmd,
            "scan_valid_count": int(sum(valid)),
            "scan_min_m": float(scan.ranges[valid].min()) if len(valid) and any(valid) else None,
            "nav": agent.stack.diagnostics(),
            "mode_time": agent.stats["mode_time"],
            "safe_stops": agent.stats["safe_stops"],
            "recoveries": agent.stats["recoveries"],
            "tick_ms_max": round(agent.stats["tick_ms_max"], 1),
            "tick_over_budget": agent.stats["tick_over_budget"],
            "submissions": server.submissions,
            "log": agent.log,
        }
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
        print(f"[gazebo-agent] report={args.report} path={path_m:.2f}m "
              f"nonzero_cmd={robot.nonzero_count} mode={agent.mode.value}", flush=True)
        return 0 if agent.mode.value == "DONE" and robot.nonzero_count > 0 else 1
    finally:
        if agent is not None:
            agent.stack.close()
        robot.close()
        shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
