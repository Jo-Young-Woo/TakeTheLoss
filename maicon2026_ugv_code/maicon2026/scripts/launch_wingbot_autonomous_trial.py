#!/usr/bin/env python3
"""Launch a fresh, isolated one-WINGBOT arena and record one autonomous match.

Source the ROS Humble and current model workspace installation first.
"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/"ros2/wingbot_description/scripts"))
from drive_gate import stop_children


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--ros-domain-id", type=int, default=77)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--no-uav", action="store_true")
    args = p.parse_args()
    if args.output.exists():
        p.error("Use a new output directory; recordings are preserved")
    if not 0 <= args.ros_domain_id <= 232:
        p.error("Invalid ROS domain")
    args.output.mkdir(parents=True)
    from ament_index_python.packages import get_package_share_directory
    arena = Path(get_package_share_directory("maicon_arena_gz"))
    env = dict(os.environ, ROS_DOMAIN_ID=str(args.ros_domain_id),
               IGN_PARTITION="wingbot_autonomy_"+str(os.getpid()), LIBGL_ALWAYS_SOFTWARE="1",
               FASTRTPS_DEFAULT_PROFILES_FILE=str(arena/"config/fastdds_shm64.xml"),
               FASTDDS_DEFAULT_PROFILES_FILE=str(arena/"config/fastdds_shm64.xml"),
               RMW_IMPLEMENTATION="rmw_fastrtps_cpp")
    launch = ["ros2","launch","maicon_arena_gz","arena.launch.py","robot_model:=wingbot",
              "ugvs:=1","ego_slot:=B-2","profile:=wingbot_recording","gui:=false",
              "render_engine:=ogre2","image_transport_profile:=shm64",
              "validation_camera:=true","uav_cam:="+str(not args.no_uav).lower()]
    runner = [sys.executable,str(ROOT/"scripts/run_wingbot_autonomous_trial.py"),
              "--output-dir",str(args.output/"trial"),"--seed",str(args.seed)]
    if args.no_uav:
        runner.append("--no-uav")
    logs = [(args.output/"launch.log").open("w"), (args.output/"agent.log").open("w")]
    children = []
    opened = time.monotonic()
    result = {"launch":launch,"runner":runner,"renderer":"Fortress CPU Ogre2 sensors",
              "scope":"one B-2; actual RGB/LiDAR/odometry; no layout truth input to Agent"}
    try:
        children.append(subprocess.Popen(launch,env=env,stdout=logs[0],stderr=subprocess.STDOUT,start_new_session=True))
        children.append(subprocess.Popen(runner,env=env,stdout=logs[1],stderr=subprocess.STDOUT,start_new_session=True,cwd=ROOT))
        result["returncode"] = children[1].wait(timeout=720)
        print(json.dumps({"output":str(args.output),"returncode":result["returncode"]}),flush=True)
    finally:
        stop_children(children,logs)
        result["wall_seconds"] = time.monotonic()-opened
        (args.output/"run_manifest.json").write_text(json.dumps(result,indent=2)+"\n")
    return result["returncode"]


if __name__ == "__main__":
    raise SystemExit(main())
