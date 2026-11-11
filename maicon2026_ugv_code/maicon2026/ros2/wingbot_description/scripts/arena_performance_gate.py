#!/usr/bin/env python3
"""Gate D: actual six-model arena, one full ego and five sensorless dummies.

Run from a shell sourced with the matching three-package colcon installation.
No mission controller or network game server is started.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

from drive_gate import stop_children


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output",type=Path,required=True)
    p.add_argument("--profile",choices=("standard","lite"),required=True)
    p.add_argument("--ros-domain-id",type=int,required=True)
    args=p.parse_args()
    if not 0<=args.ros_domain_id<=232:
        p.error("ROS domain must be 0..232")
    from ament_index_python.packages import get_package_share_directory
    arena=Path(get_package_share_directory("maicon_arena_gz"))
    model=Path(get_package_share_directory("wingbot_description"))
    args.output.mkdir(parents=True,exist_ok=True)
    os.environ.update(IGN_PARTITION="wingbot_arena_gate_"+str(os.getpid()),
        ROS_DOMAIN_ID=str(args.ros_domain_id),LIBGL_ALWAYS_SOFTWARE="1",
        FASTRTPS_DEFAULT_PROFILES_FILE=str(arena/"config/fastdds_shm64.xml"),
        FASTDDS_DEFAULT_PROFILES_FILE=str(arena/"config/fastdds_shm64.xml"),
        RMW_IMPLEMENTATION="rmw_fastrtps_cpp")
    command=["ros2","launch","maicon_arena_gz","arena.launch.py",
             "robot_model:=wingbot","ugvs:=6","profile:="+args.profile,
             "gui:=false","render_engine:=ogre2","uav_cam:=true",
             "image_transport_profile:=shm64","validation_camera:=false"]
    launch_log=(args.output/"launch.log").open("w",encoding="utf-8")
    children,logs=[],[launch_log]
    started=time.monotonic()
    manifest=dict(command=command,profile=args.profile,model_count=6,full_ego=1,sensorless_dummies=5,
                  renderer="CPU Ogre2; no GUI; UAV camera enabled",ros_domain=args.ros_domain_id,
                  model_sha256={str(path.relative_to(model)):hashlib.sha256(path.read_bytes()).hexdigest()
                                for path in (model/"urdf/wingbot_gz.xacro",model/"urdf/wingbot_props.xacro",
                                             model/"config/frames.yaml",model/"config/wingbot_mods.yaml")})
    try:
        children.append(subprocess.Popen(command,stdout=launch_log,stderr=subprocess.STDOUT,start_new_session=True))
        # This probe waits for clock, excludes five seconds of warmup, and runs
        # sixty seconds. No other tests/render sessions may run concurrently.
        threshold=".98" if args.profile=="standard" else ".90"
        perf_path=args.output/"performance.json"
        perf=subprocess.run([sys.executable,str(arena/"scripts/perf_probe.py"),"--duration","60","--warmup","5",
            "--clock-timeout","60","--image-topic","/image_raw","--output",str(perf_path),
            "--minimum-mean",threshold,"--minimum-window",".85",
            "--launch-wall-time",str(started)],capture_output=True,text=True,timeout=150)
        (args.output/"performance.log").write_text(perf.stdout+perf.stderr,encoding="utf-8")
        runtime_path=args.output/"runtime.json"
        runtime=subprocess.run([sys.executable,str(arena/"scripts/runtime_probe.py"),"--duration","8",
            "--output",str(runtime_path)],capture_output=True,text=True,timeout=30)
        (args.output/"runtime.log").write_text(runtime.stdout+runtime.stderr,encoding="utf-8")
        manifest["performance"]=json.loads(perf_path.read_text()) if perf_path.exists() else {}
        manifest["runtime"]=json.loads(runtime_path.read_text()) if runtime_path.exists() else {}
        runtime_data=manifest["runtime"]
        manifest["passed"]=(perf.returncode==0 and runtime.returncode==0 and
            len(runtime_data.get("models",{}))==6 and
            manifest["performance"].get("full_sixty_second_probe",False))
        manifest["error"]=None if manifest["passed"] else "Performance, sensors, or actual model count failed; inspect logs."
        print(json.dumps({k:manifest[k] for k in ("profile","passed","performance","error")}),flush=True)
        return 0 if manifest["passed"] else 1
    except Exception as exc:
        manifest.update(passed=False,error=str(exc))
        raise
    finally:
        stop_children(children,logs)
        (args.output/"gate_d_arena.json").write_text(json.dumps(manifest,indent=2)+"\n",encoding="utf-8")


if __name__=="__main__":
    raise SystemExit(main())
