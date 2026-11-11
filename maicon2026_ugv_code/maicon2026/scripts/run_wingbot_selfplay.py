#!/usr/bin/env python3
"""Six independent sensor-driven Agents in one actual Gazebo arena.

The coordinator starts/stops the local referee and records evidence only. It
does not issue velocity, aiming, route or fire commands to any robot. World
poses are isolated to evaluation and video annotation, never sent to Agents.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import zipfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
if __name__ == '__main__':
    from selected_runtime import activate
    activate(ROOT)
sys.path.insert(0, str(ROOT/"ros2/maicon_arena_gz/scripts"))
sys.path.insert(0, str(ROOT/"ros2/wingbot_description/scripts"))
from drive_gate import stop_children
from multi_agent_wiring import SLOTS
from run_wingbot_autonomous_trial import CameraRecorder, write_json, write_preview_json, ARTIFACT_IO_STATS


SCENARIOS = ("selfplay","vs_evaders","vs_hunters","vs_blockers","mixed_allies","goal_race")


def policies_for(scenario):
    if scenario not in SCENARIOS:
        raise ValueError("Unknown physical trial scenario")
    policies = dict.fromkeys(SLOTS,"agent")
    opponent = {"vs_evaders":"evader","vs_hunters":"hunter","vs_blockers":"blocker"}.get(scenario)
    if opponent:
        policies.update({slot:opponent for slot in SLOTS if slot.startswith("R")})
    if scenario == "mixed_allies":
        policies.update({"B-1":"idle","B-3":"patrol_own","R-1":"hunter","R-2":"blocker","R-3":"evader"})
    return policies


def read_json(path):
    try:
        return json.loads(path.read_text(encoding="utf8"))
    except (OSError, ValueError):
        return None


def source_manifest(root, installed_share=None):
    """Hash source and installed arena inputs before launching any process."""
    suffixes={".py",".yaml",".yml",".json",".sdf",".urdf",".xacro",".xml"}
    roots={"repo/"+folder:root/folder for folder in
           ("maicon26","scripts","config","ros2/maicon_arena_gz","ros2/wingbot_description")}
    from selected_runtime import selection
    active_root,_=selection(root)
    roots['repo/maicon26']=active_root/'maicon26'
    if installed_share is not None:
        roots["installed/share"]=installed_share
        roots["installed/lib"]=installed_share.parents[1]/"lib/maicon_arena_gz"
    hashes={}
    for label,directory in roots.items():
        if not directory.exists():
            continue
        for path in sorted(directory.rglob("*")):
            if path.is_file() and path.suffix in suffixes and "__pycache__" not in path.parts:
                hashes[label+"/"+path.relative_to(directory).as_posix()]=hashlib.sha256(path.read_bytes()).hexdigest()
    return dict(captured_utc=datetime.now(timezone.utc).isoformat(),sha256=hashes,
                source_scope="Pre-launch working-tree and installed arena inputs, not an isolated checkout. Keep source frozen while children import.")


class Tail:
    def __init__(self, path):
        self.path, self.position, self.buffer, self.last = path, 0, "", {}

    def poll(self):
        try:
            with self.path.open(encoding="utf8") as handle:
                handle.seek(self.position)
                self.buffer += handle.read()
                self.position = handle.tell()
        except OSError:
            return self.last
        lines = self.buffer.split("\n")
        self.buffer = lines.pop()
        for line in lines:
            try:
                self.last = json.loads(line)
            except ValueError:
                pass
        return self.last


def overlay(frame, poses, traces, elapsed, risk_snapshot=None, scenario="selfplay"):
    """Projected truth rings are evaluation labels; all decisions are Agent logs."""
    import cv2
    import numpy as np
    image = frame.copy()
    height, width = image.shape[:2]
    colours = {"AVOID": (255, 160, 50), "ENGAGE_OK": (30, 160, 255), "COVER": (40, 40, 255)}
    panel = image.copy()
    # The recorder paints its large timestamp above y=90 after this overlay.
    # Leave that band clear so all six status rows remain readable.
    cv2.rectangle(panel, (8, 95), (365, 278), (15, 15, 15), -1)
    image = cv2.addWeighted(panel, .72, image, .28, 0)
    for i, slot in enumerate(SLOTS):
        trace = traces.get(slot, {})
        decision = trace.get("decision", {})
        posture = decision.get("posture", "WAIT")
        colour = colours.get(posture, (180, 180, 180))
        text = f"{slot} {posture} L{decision.get('level',0)} S{decision.get('stage',0)} {trace.get('mode','WAIT')}"
        cv2.putText(image, text, (17, 117+i*28), cv2.FONT_HERSHEY_SIMPLEX, .46, colour, 1, cv2.LINE_AA)
        # Recording-camera projection: x-right; forward aims at z=.2 from
        # (0,-6.2,5.5). These circles are labels, not rendered robot geometry.
        if slot in poses:
            x, y, z, yaw = poses[slot]
            pitch = math.atan2(5.3, 6.2)
            forward = (y+6.2)*math.cos(pitch) - (z+.14-5.5)*math.sin(pitch)
            up = (y+6.2)*math.sin(pitch) + (z+.14-5.5)*math.cos(pitch)
            focal = width/(2*math.tan(1.2/2))
            if forward > .01:
                point = (int(width/2+focal*x/forward), int(height/2-focal*up/forward))
                cv2.circle(image, point, 18, colour, 2, cv2.LINE_AA)
                cv2.putText(image, slot, (point[0]+19,point[1]), cv2.FONT_HERSHEY_SIMPLEX,.4,colour,1,cv2.LINE_AA)
    # A separate view of B2's own estimated tracks: no true positions here.
    x0, y0, mw, mh = width-260, 70, 240, 192
    cv2.rectangle(image, (x0,y0), (x0+mw,y0+mh), (30,30,30), -1)
    if risk_snapshot is not None:
        risk = np.asarray(risk_snapshot["risk"])
        maximum = max(1., float(np.max(risk)))
        colours_risk = cv2.applyColorMap(np.uint8(np.clip(risk/maximum,0,1)*255),cv2.COLORMAP_TURBO)
        heat = cv2.resize(np.flipud(colours_risk),(mw,mh),interpolation=cv2.INTER_NEAREST)
        image[y0:y0+mh,x0:x0+mw] = cv2.addWeighted(image[y0:y0+mh,x0:x0+mw],.3,heat,.7,0)
    cv2.putText(image, "B2 measured risk + tracks", (x0,y0-8), cv2.FONT_HERSHEY_SIMPLEX,.4,(220,220,220),1,cv2.LINE_AA)
    decision = traces.get("B-2", {}).get("decision", {})
    for track in decision.get("tracks", []):
        point = (int(x0+track['x']/6*mw), int(y0+mh-track['y']/4.8*mh))
        colour = {"enemy": (50,50,240), "ally": (240,140,50), "unknown": (120,220,220)}.get(track.get('team'),(200,200,200))
        if x0<=point[0]<=x0+mw and y0<=point[1]<=y0+mh:
            end = (int(point[0]+track.get('vx',0)*mw/6), int(point[1]-track.get('vy',0)*mh/4.8))
            cv2.circle(image, point, 4, colour, -1)
            cv2.arrowedLine(image, point, end, colour, 1, cv2.LINE_AA)
    cv2.putText(image, f"Actual Gazebo RGB | sensor-only {scenario} | match {elapsed:.1f}s", (12,height-14),
                cv2.FONT_HERSHEY_SIMPLEX,.48,(240,240,240),1,cv2.LINE_AA)
    return image


def run(args):
    from ament_index_python.packages import get_package_share_directory
    import rclpy
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import Image
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage
    from maicon26.hal.fpv_recording import decode_bgr

    out = args.output.resolve()
    if out.exists():
        raise ValueError("Use a new output directory to preserve existing recordings")
    out.mkdir(parents=True)
    share = Path(get_package_share_directory("maicon_arena_gz"))
    write_json(out/"source_manifest.json",source_manifest(ROOT,share))
    os.environ.update(ROS_DOMAIN_ID=str(args.ros_domain_id), IGN_PARTITION="wingbot_selfplay_"+str(os.getpid()),
                      LIBGL_ALWAYS_SOFTWARE="0" if args.gui else "1", RMW_IMPLEMENTATION="rmw_fastrtps_cpp",
                      FASTRTPS_DEFAULT_PROFILES_FILE=str(share/"config/fastdds_shm256.xml"),
                      FASTDDS_DEFAULT_PROFILES_FILE=str(share/"config/fastdds_shm256.xml"),
                      OPENBLAS_NUM_THREADS="1", OMP_NUM_THREADS="1", MKL_NUM_THREADS="1")
    command = ["ros2", "launch", "maicon_arena_gz", "arena.launch.py", "robot_model:=wingbot",
               "ugvs:=6", "full_sensors:=true", "ego_slot:=B-2", "profile:=wingbot_recording",
               "gui:="+str(args.gui).lower(), "render_engine:=ogre2", "validation_camera:=true",
               "uav_cam:="+str(not args.no_uav).lower(), "image_transport_profile:=shm256"]
    children, logs, agents, events, traces, poses, state = [], [], {}, [], {}, {}, {}
    tails = {slot: Tail(out/slot/"agent_trace.jsonl") for slot in SLOTS}
    clock, clock_wall = [None], [time.monotonic()]
    node = recorder = None
    quit_requested = [False]
    handlers = {sig: signal.signal(sig, lambda *_: quit_requested.__setitem__(0, True)) for sig in (signal.SIGINT, signal.SIGTERM)}
    policies = policies_for(args.scenario)
    result = dict(scope="six physical WINGBOT models, isolated sensor-driven mission/policy processes, RGB/LiDAR/odom/IMU",
                  truth_used_by_agents=False, coordinator_issues_motion_commands=False,
                  variant=args.variant, seed=args.seed, duration_requested_s=args.duration,
                  source_manifest="source_manifest.json",
                  scenario=args.scenario, policies=policies, command=command, ros_domain_id=args.ros_domain_id,
                  simulation_impact=args.simulate_impact,
                  ign_partition=os.environ["IGN_PARTITION"], limitations=["Reference-photo image recognizer, not a trained real-world detector"], errors=[])
    opened = time.monotonic()
    truth = (out/"evaluation_truth_all.jsonl").open("w", encoding="utf8")
    last_truth, last_tail, start_sim = [-1e9], 0., None
    risk_snapshot = [None]
    try:
        logs.append((out/"launch.log").open("w", encoding="utf8"))
        children.append(subprocess.Popen(command, cwd=ROOT, stdout=logs[-1], stderr=subprocess.STDOUT, start_new_session=True))
        # Keep our bounded cleanup handler; the default rclpy SIGINT handler
        # shuts down the context before finally can report/stop child groups.
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        node = rclpy.create_node("wingbot_selfplay_observer")
        control = node.create_publisher(String, "/arena/control", 10)
        recorder = CameraRecorder(out/"wingbot_selfplay_third_person.mp4", 10., "WINGBOT actual sensors | "+args.scenario)
        add_frame = recorder.writer.add
        recorder.writer.add = lambda frame, stamp: add_frame(overlay(frame, dict(poses), dict(traces), float(state.get("elapsed") or 0.),risk_snapshot[0],args.scenario),stamp)
        recording = [False]
        def on_clock(message):
            stamp = message.clock.sec+message.clock.nanosec*1e-9
            if clock[0] is None or stamp>clock[0]: clock_wall[0] = time.monotonic()
            clock[0] = stamp
        def on_state(message):
            value = json.loads(message.data)
            state.clear(); state.update(value)
        def on_pose(message):
            for transform in message.transforms:
                name = transform.child_frame_id
                if "::" in name or not name.startswith("ugv_"): continue
                slot = name[4]+"-"+name[5:]
                if slot not in SLOTS: continue
                p,q = transform.transform.translation, transform.transform.rotation
                poses[slot] = (p.x,p.y,p.z,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
            if recording[0] and clock[0] is not None and clock[0]-last_truth[0]>=.045:
                truth.write(json.dumps(dict(sim_s=clock[0],elapsed=state.get("elapsed"),poses=poses))+"\n")
                last_truth[0] = clock[0]
        def on_uav(message):
            path = out/"ready_uav_rgb.png"
            if not path.exists():
                import cv2
                cv2.imwrite(str(path), decode_bgr(message))
                write_json(out/"ready_uav_image.json", dict(stamp=message.header.stamp.sec+message.header.stamp.nanosec*1e-9,
                                                          width=message.width,height=message.height,source="actual Gazebo UAV RGB"))
        node.create_subscription(Clock,"/clock",on_clock,qos_profile_sensor_data)
        node.create_subscription(String,"/arena/state",on_state,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        node.create_subscription(TFMessage,"/arena/dynamic_pose",on_pose,qos_profile_sensor_data)
        node.create_subscription(String,"/arena/events",lambda msg:events.append(json.loads(msg.data)),100)
        node.create_subscription(Image,"/arena/validation/image_raw",lambda msg:recorder.receive(msg) if recording[0] else None,qos_profile_sensor_data)
        if not args.no_uav: node.create_subscription(Image,"/uav/image_raw",on_uav,qos_profile_sensor_data)
        for slot in SLOTS:
            logs.append((out/(slot+".log")).open("w",encoding="utf8"))
            child_command = [sys.executable,str(ROOT/"scripts/run_wingbot_autonomous_trial.py"),
                "--multi-agent","--slot",slot,"--await-start","--no-record","--output-dir",str(out/slot),
                "--variant",args.variant,"--seed",str(args.seed),"--ready-timeout",str(args.ready_timeout),
                "--wall-timeout",str(args.wall_timeout),"--policy",policies[slot]]
            if args.no_uav: child_command.append("--no-uav")
            if args.simulate_impact: child_command.append("--simulate-impact")
            agents[slot] = subprocess.Popen(child_command,cwd=ROOT,stdout=logs[-1],stderr=subprocess.STDOUT,start_new_session=True)
            children.append(agents[slot])
        ready_end = time.monotonic()+args.ready_timeout
        last_status = 0.
        while time.monotonic()<ready_end:
            rclpy.spin_once(node,timeout_sec=.025)
            if quit_requested[0]: raise InterruptedError("Coordinator interrupted")
            failed = {s:p.returncode for s,p in agents.items() if p.poll() is not None}
            if failed: raise RuntimeError("Agent exited before ready: "+str(failed))
            ready = [s for s in SLOTS if (out/s/"ready.json").is_file()]
            if time.monotonic()-last_status>10:
                print(json.dumps(dict(phase="waiting_for_six_sensor_stacks",ready=ready,models=sorted(poses),sim_s=clock[0])),flush=True)
                last_status=time.monotonic()
            if len(ready)==6 and len(poses)==6 and state.get("phase")=="ready": break
        else: raise TimeoutError("All six independent Agents did not become sensor-ready")
        result["ready"] = {slot: read_json(out/slot/"ready.json") for slot in SLOTS}
        result["models"] = sorted(poses)
        recording[0] = True
        control.publish(String(data=json.dumps({"cmd":"start","objectives":{"BLUE":"R-2","RED":"B-2"}})))
        report_sent = None
        last_status = 0.
        while time.monotonic()-opened<args.wall_timeout:
            rclpy.spin_once(node,timeout_sec=.02)
            wall = time.monotonic()
            if quit_requested[0]: raise InterruptedError("Coordinator interrupted")
            if children[0].poll() is not None: raise RuntimeError("Gazebo launch exited")
            if wall-clock_wall[0]>20.: raise RuntimeError("Gazebo clock stalled")
            if recorder.error: raise RuntimeError(recorder.error)
            if wall-last_tail>=.25:
                traces.update({slot: tails[slot].poll() for slot in SLOTS}); last_tail=wall
                try:
                    import numpy as np
                    with np.load(out/"B-2"/"risk_overlay.npz") as payload:
                        risk_snapshot[0] = {key:payload[key].copy() for key in payload.files}
                except (OSError,ValueError,EOFError,zipfile.BadZipFile):
                    pass
            if state.get("phase")=="running" and start_sim is None:
                start_sim=clock[0]-float(state.get("elapsed") or 0.)
            elapsed=float(state.get("elapsed") or 0.)
            if start_sim is not None and elapsed>=args.duration and state.get("phase")=="running" and report_sent is None:
                control.publish(String(data=json.dumps({"cmd":"report"}))); report_sent=wall
            if state.get("phase") in ("reporting","finished") and report_sent is None: report_sent=wall
            failed={s:p.returncode for s,p in agents.items() if p.poll() not in (None,0)}
            if failed: raise RuntimeError("Agent failure: "+str(failed))
            if all(p.poll() is not None for p in agents.values()): break
            if report_sent is not None and wall-report_sent>120.: raise TimeoutError("Agent report/cleanup exceeded 120 wall seconds")
            if wall-last_status>10:
                status=dict(elapsed=elapsed,phase=state.get("phase"),modes={s:traces.get(s,{}).get("mode") for s in SLOTS},frames=recorder.accepted)
                print(json.dumps(status),flush=True); write_preview_json(out/"progress.json",status); last_status=wall
        else: raise TimeoutError("Trial wall timeout")
        result["completed"] = all(p.returncode==0 for p in agents.values())
    except Exception as error:
        result["errors"].append(type(error).__name__+": "+str(error))
        result["completed"] = False
        print(result["errors"][-1],file=sys.stderr,flush=True)
    finally:
        if node is not None and rclpy.ok():
            if state.get("phase")=="running":
                control.publish(String(data=json.dumps({"cmd":"report"})))
            # Give child finally blocks a chance to save reports before SIGTERM.
            end=time.monotonic()+5
            while any(p.poll() is None for p in agents.values()) and time.monotonic()<end:
                rclpy.spin_once(node,timeout_sec=.05)
        stop_children(children,logs)
        if recorder is not None: result["recording"] = recorder.close()
        if node is not None: node.destroy_node(); rclpy.try_shutdown()
        truth.close()
        result.update(wall_s=time.monotonic()-opened,match_start_sim_s=start_sim,end_sim_s=clock[0],
                      artifact_io_stats=dict(ARTIFACT_IO_STATS),
                      referee_state=state,referee_events=events,
                      agent_returncodes={s:p.returncode for s,p in agents.items()},
                      agent_reports={slot:read_json(out/slot/"mission_report.json") for slot in SLOTS})
        write_json(out/"selfplay_report.json",result)
        for sig,handler in handlers.items(): signal.signal(sig,handler)
    return 0 if result["completed"] else 1


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--duration",type=float,default=300.)
    parser.add_argument("--seed",type=int,default=0)
    parser.add_argument("--variant",choices=("progress","v2","blocker_fire","dwell_zero"),default="progress")
    parser.add_argument("--scenario",choices=SCENARIOS,default="selfplay")
    parser.add_argument("--no-uav",action="store_true")
    parser.add_argument("--simulate-impact",action="store_true",help="Enable uncalibrated Gazebo IMU impact handling")
    parser.add_argument("--gui",action="store_true",help="Show the live Gazebo window (uses GPU rendering)")
    parser.add_argument("--ros-domain-id",type=int,default=81)
    parser.add_argument("--ready-timeout",type=float,default=180.)
    parser.add_argument("--wall-timeout",type=float,default=3600.)
    args=parser.parse_args()
    if not 0<args.duration<=300 or args.wall_timeout<=300 or args.ready_timeout<=0:
        parser.error("Invalid duration/timeouts")
    return run(args)


if __name__=="__main__": raise SystemExit(main())
