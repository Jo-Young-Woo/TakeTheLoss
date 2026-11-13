#!/usr/bin/env python3
"""One or six independent sensor-driven Agents in an actual Gazebo arena.

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
from run_wingbot_autonomous_trial import CameraRecorder, draw_face_ledger, write_json, write_preview_json, ARTIFACT_IO_STATS


SCENARIOS = ("selfplay","vs_evaders","vs_hunters","vs_blockers","mixed_allies","goal_race")


def prepare_observer_recording_profile(output, share, fps=30.):
    """Raise the real oblique recording camera rate; leave Agent sensors alone."""
    import yaml
    if not math.isfinite(fps) or not 2 <= fps <= 30:
        raise ValueError('Observer recording rate must be between 2 and 30 Hz')
    profile = yaml.safe_load((Path(share)/'config/profiles/wingbot_recording.yaml').read_text())
    profile['name'] = 'oblique_observer_video'
    profile['validation_camera']['update_rate'] = float(fps)
    path = Path(output)/'observer_recording_profile.yaml'
    path.write_text(yaml.safe_dump(profile, sort_keys=False), encoding='utf8')
    return path


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
    suffixes={".py",".yaml",".yml",".json",".sdf",".urdf",".xacro",".xml",".lua"}
    roots={"repo/"+folder:root/folder for folder in
           ("maicon26","scripts","config","ros2/maicon_arena_gz","ros2/wingbot_description",
            "ros2/launch","ros2/config","ros2/behavior_trees")}
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


def overlay(frame, poses, traces, elapsed, risk_snapshot=None, scenario="selfplay", arena_size=(6.,4.8)):
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
    aw, ah = arena_size
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
        point = (int(x0+track['x']/aw*mw), int(y0+mh-track['y']/ah*mh))
        colour = {"enemy": (50,50,240), "ally": (240,140,50), "unknown": (120,220,220)}.get(track.get('team'),(200,200,200))
        if x0<=point[0]<=x0+mw and y0<=point[1]<=y0+mh:
            end = (int(point[0]+track.get('vx',0)*mw/aw), int(point[1]-track.get('vy',0)*mh/ah))
            cv2.circle(image, point, 4, colour, -1)
            cv2.arrowedLine(image, point, end, colour, 1, cv2.LINE_AA)
    cv2.putText(image, f"Actual Gazebo RGB | sensor-only {scenario} | match {elapsed:.1f}s", (12,height-14),
                cv2.FONT_HERSHEY_SIMPLEX,.48,(240,240,240),1,cv2.LINE_AA)
    return image


def draw_iff_panel(image, ledger):
    """Display only B-2's own identification evidence, never referee identities."""
    import cv2
    x,y=1712,482
    iff={} if ledger is None else ledger.get("iff",{})
    cv2.putText(image,"B-2 IFF | sensor tracks",(x,y),cv2.FONT_HERSHEY_SIMPLEX,.5,(80,220,255),1,cv2.LINE_AA)
    yy=y+21
    for tr in sorted(iff.get("tracks",[]),key=lambda item:item["age_s"])[:5]:
        relation=tr["relation"] if tr["age_s"]<=3. else "stale"
        color={"ally":(240,170,70),"enemy":(60,90,255),"unknown":(160,220,220),"stale":(120,120,120)}[relation]
        label=f"#{tr['id']} {tr['ident'] or '?'} {relation} age {tr['age_s']:.1f}s"
        cv2.putText(image,label,(x,yy),cv2.FONT_HERSHEY_SIMPLEX,.42,color,1,cv2.LINE_AA)
        yy+=19
    if not iff.get("tracks"):
        cv2.putText(image,"No identified contacts",(x,yy),cv2.FONT_HERSHEY_SIMPLEX,.42,(160,160,160),1,cv2.LINE_AA)
        yy+=19
    friends=iff.get("friends",[])
    if friends:
        yy+=4
        for friend in friends[:2]:
            label=f"{friend['ident']}: {friend['state']}"
            cv2.putText(image,label,(x,yy),cv2.FONT_HERSHEY_SIMPLEX,.42,(210,180,100),1,cv2.LINE_AA)
            yy+=19
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
    slots = ("B-2",) if args.ugvs == 1 else SLOTS
    share = Path(get_package_share_directory("maicon_arena_gz"))
    # Installed camera assets and the Agent must describe the same arena.
    import yaml
    package = ROOT/"ros2/maicon_arena_gz"
    for relative in ("config/arena_spec.yaml", "config/uav_grid_calibration.json", "config/layouts/layout_default.yaml"):
        if (package/relative).read_bytes() != (share/relative).read_bytes():
            raise ValueError("Installed arena differs from source; rebuild maicon_arena_gz before running: "+relative)
    spec_path = out/"arena_spec.yaml"
    spec_path.write_bytes((package/"config/arena_spec.yaml").read_bytes())
    (out/"layout_default.yaml").write_bytes((package/"config/layouts/layout_default.yaml").read_bytes())
    arena_size = yaml.safe_load(spec_path.read_text())["floor"]["size"]
    write_json(out/"source_manifest.json",source_manifest(ROOT,share))
    recording_profile = None
    if args.record_view == 'uav':
        from uav_view_recording import prepare_uav_recording_profile
        recording_profile = prepare_uav_recording_profile(out,share,args.uav_video_fps)
    else:
        recording_profile = prepare_observer_recording_profile(out,share,args.observer_video_fps)
    gpu = args.gui or args.render_device == 'gpu'
    os.environ.update(ROS_DOMAIN_ID=str(args.ros_domain_id), IGN_PARTITION="wingbot_selfplay_"+str(os.getpid()),
                      LIBGL_ALWAYS_SOFTWARE="0" if gpu else "1", RMW_IMPLEMENTATION="rmw_fastrtps_cpp",
                      FASTRTPS_DEFAULT_PROFILES_FILE=str(share/"config/fastdds_shm256.xml"),
                      FASTDDS_DEFAULT_PROFILES_FILE=str(share/"config/fastdds_shm256.xml"),
                      OPENBLAS_NUM_THREADS="1", OMP_NUM_THREADS="1", MKL_NUM_THREADS="1")
    command = ["ros2", "launch", "maicon_arena_gz", "arena.launch.py", "robot_model:=wingbot",
               "arena_spec:="+str(spec_path),
               "ugvs:="+str(args.ugvs), "full_sensors:="+str(args.ugvs==6).lower(),
               "ego_slot:=B-2", "profile:="+str(recording_profile or "wingbot_recording"),
               "gui:="+str(args.gui).lower(), "render_engine:=ogre2", "validation_camera:=true",
               "uav_cam:="+str(not args.no_uav).lower(), "image_transport_profile:=shm256"]
    nav_command = None
    if args.nav_backend == 'nav2':
        from wingbot_nav2_trial import prepare
        nav_command = prepare(out, spec_path)
    if gpu:
        command.append("wsl_gpu_compat:=true")
    children, logs, agents, events, traces, poses, state = [], [], {}, [], {}, {}, {}
    tails = {slot: Tail(out/slot/"agent_trace.jsonl") for slot in slots}
    clock, clock_wall = [None], [time.monotonic()]
    node = recorder = None
    quit_requested = [False]
    handlers = {sig: signal.signal(sig, lambda *_: quit_requested.__setitem__(0, True)) for sig in (signal.SIGINT, signal.SIGTERM)}
    policies = {slot:policy for slot,policy in policies_for(args.scenario).items() if slot in slots}
    result = dict(scope=f"{len(slots)} physical WINGBOT models, isolated sensor-driven mission/policy processes, RGB/LiDAR/odom/IMU",
                  active_slots=list(slots),
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
    ledger_snapshot = [None]
    try:
        logs.append((out/"launch.log").open("w", encoding="utf8"))
        children.append(subprocess.Popen(command, cwd=ROOT, stdout=logs[-1], stderr=subprocess.STDOUT, start_new_session=True))
        if nav_command:
            logs.append((out/'nav2_cartographer.log').open('w',encoding='utf8'))
            children.append(subprocess.Popen(nav_command,cwd=ROOT,stdout=logs[-1],stderr=subprocess.STDOUT,start_new_session=True))
            result['nav2_command'] = nav_command
            result['navigation_backend'] = 'nav2_cartographer_astar'
        # Keep our bounded cleanup handler; the default rclpy SIGINT handler
        # shuts down the context before finally can report/stop child groups.
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        node = rclpy.create_node("wingbot_selfplay_observer")
        control = node.create_publisher(String, "/arena/control", 10)
        if args.record_view == 'uav':
            from uav_view_recording import UavViewRecording
            def ground_for_uav():
                snap=ledger_snapshot[0]
                return snap, (start_sim+snap['elapsed'] if start_sim is not None and snap is not None else None)
            recorder=UavViewRecording(out,share,ground_for_uav,recording_profile)
        else:
            recorder = CameraRecorder(out/"wingbot_selfplay_third_person.mp4", args.observer_video_fps,
                                      "WINGBOT oblique view | B-2 face ledger | "+args.scenario,decorate=False)
            add_frame = recorder.writer.add
            recorder.writer.add = lambda frame, stamp: add_frame(draw_iff_panel(
                draw_face_ledger(overlay(frame, dict(poses), dict(traces),
                                         float(state.get("elapsed") or 0.),risk_snapshot[0],args.scenario,arena_size),
                                 ledger_snapshot[0]),ledger_snapshot[0]),stamp)
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
        for slot in slots:
            logs.append((out/(slot+".log")).open("w",encoding="utf8"))
            child_command = [sys.executable,str(ROOT/"scripts/run_wingbot_autonomous_trial.py"),
                "--arena-spec",str(spec_path),
                "--slot",slot,"--await-start","--no-record","--output-dir",str(out/slot),
                "--variant",args.variant,"--seed",str(args.seed),"--ready-timeout",str(args.ready_timeout),
                "--wall-timeout",str(args.wall_timeout),"--policy",policies[slot],
                "--nav-backend",args.nav_backend]
            if args.ugvs == 6: child_command.append("--multi-agent")
            if args.no_uav: child_command.append("--no-uav")
            if args.simulate_impact: child_command.append("--simulate-impact")
            agents[slot] = subprocess.Popen(child_command,cwd=ROOT,stdout=logs[-1],stderr=subprocess.STDOUT,start_new_session=True)
            children.append(agents[slot])
        ready_end = time.monotonic()+args.ready_timeout
        last_status = 0.
        while time.monotonic()<ready_end:
            rclpy.spin_once(node,timeout_sec=.025)
            if quit_requested[0]: raise InterruptedError("Coordinator interrupted")
            if nav_command and children[1].poll() is not None: raise RuntimeError('Nav2/Cartographer launch exited before ready')
            failed = {s:p.returncode for s,p in agents.items() if p.poll() is not None}
            if failed: raise RuntimeError("Agent exited before ready: "+str(failed))
            ready = [s for s in slots if (out/s/"ready.json").is_file()]
            if time.monotonic()-last_status>10:
                print(json.dumps(dict(phase="waiting_for_sensor_stacks",expected=list(slots),ready=ready,models=sorted(poses),sim_s=clock[0])),flush=True)
                last_status=time.monotonic()
            if len(ready)==len(slots) and set(poses)==set(slots) and state.get("phase")=="ready": break
        else: raise TimeoutError("Requested Agents did not become sensor-ready: "+str(slots))
        result["ready"] = {slot: read_json(out/slot/"ready.json") for slot in slots}
        result["models"] = sorted(poses)
        recording[0] = True
        control.publish(String(data=json.dumps({"cmd":"start","objectives":{"BLUE":"R-2","RED":"B-2"}})))
        report_sent = None
        mission_finished = False
        last_status = 0.
        while time.monotonic()-opened<args.wall_timeout:
            rclpy.spin_once(node,timeout_sec=.02)
            wall = time.monotonic()
            if quit_requested[0]: raise InterruptedError("Coordinator interrupted")
            if children[0].poll() is not None: raise RuntimeError("Gazebo launch exited")
            if nav_command and children[1].poll() is not None: raise RuntimeError('Nav2/Cartographer launch exited')
            if wall-clock_wall[0]>20.: raise RuntimeError("Gazebo clock stalled")
            if recorder.error: raise RuntimeError(recorder.error)
            if wall-last_tail>=.25:
                traces.update({slot: tails[slot].poll() for slot in slots}); last_tail=wall
                latest_ledger=read_json(out/"B-2"/"face_ledger.json")
                if latest_ledger is not None: ledger_snapshot[0]=latest_ledger
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
            all_done = all(p.poll() is not None for p in agents.values())
            if all_done and not mission_finished:
                mission_finished = True
                result["mission_finished_elapsed_s"] = elapsed
                latest_ledger=read_json(out/"B-2"/"face_ledger.json")
                if latest_ledger is not None: ledger_snapshot[0]=latest_ledger
                if hasattr(recorder,"finish_mission"): recorder.finish_mission()
            # Capture may legitimately end the mission before 5:00. Keep the
            # real camera/world running until the requested recording duration.
            if all_done and start_sim is not None and clock[0]-start_sim>=args.duration: break
            if not all_done and report_sent is not None and wall-report_sent>120.:
                raise TimeoutError("Agent report/cleanup exceeded 120 wall seconds")
            if wall-last_status>10:
                status=dict(elapsed=elapsed,phase=state.get("phase"),modes={s:traces.get(s,{}).get("mode") for s in slots},frames=recorder.accepted)
                print(json.dumps(status),flush=True); write_preview_json(out/"progress.json",status); last_status=wall
        else: raise TimeoutError("Trial wall timeout")
        result["completed"] = (all(p.returncode==0 for p in agents.values())
                               and start_sim is not None and clock[0]-start_sim>=args.duration)
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
                      agent_reports={slot:read_json(out/slot/"mission_report.json") for slot in slots})
        write_json(out/"selfplay_report.json",result)
        for sig,handler in handlers.items(): signal.signal(sig,handler)
    return 0 if result["completed"] else 1


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--nav-backend",choices=('builtin','nav2'),default='builtin')
    parser.add_argument("--duration",type=float,default=300.)
    parser.add_argument("--ugvs",type=int,choices=(1,6),default=6,
                        help="One B-2 or six physical WINGBOTs; same recording and mission duration")
    parser.add_argument("--seed",type=int,default=0)
    parser.add_argument("--variant",choices=("progress","v2","blocker_fire","dwell_zero"),default="progress")
    parser.add_argument("--scenario",choices=SCENARIOS,default="selfplay")
    parser.add_argument("--no-uav",action="store_true")
    parser.add_argument("--record-view",choices=('observer','uav'),default='observer',
                        help='Record the observer camera or native UAV RGB with image-derived face inventory')
    parser.add_argument("--uav-video-fps",type=float,default=30.,
                        help='Real co-located UAV recording camera Hz; Agent UAV input stays at 1 Hz')
    parser.add_argument("--simulate-impact",action="store_true",help="Enable uncalibrated Gazebo IMU impact handling")
    parser.add_argument("--observer-video-fps",type=float,default=30.,
                        help='Real oblique observer camera and video rate; Agent sensors stay unchanged')
    parser.add_argument("--gui",action="store_true",help="Show the live Gazebo window (uses GPU rendering)")
    parser.add_argument("--render-device",choices=('cpu','gpu'),default='cpu',
                        help='CPU is verified for textured Ogre2 worlds on this WSL driver')
    parser.add_argument("--ros-domain-id",type=int,default=81)
    parser.add_argument("--ready-timeout",type=float,default=180.)
    parser.add_argument("--wall-timeout",type=float,default=3600.)
    args=parser.parse_args()
    if args.nav_backend == 'nav2' and (args.ugvs != 1 or args.scenario != 'selfplay'):
        parser.error('Nav2/Cartographer wiring supports one autonomous B-2 only')
    if not 0<args.duration<=300 or args.wall_timeout<=300 or args.ready_timeout<=0:
        parser.error("Invalid duration/timeouts")
    if args.no_uav and args.record_view == 'uav':
        parser.error('--record-view uav requires the UAV camera')
    if args.ugvs == 1 and args.scenario != 'selfplay':
        parser.error('--ugvs 1 supports the autonomous selfplay policy only')
    if not math.isfinite(args.uav_video_fps) or not 2 <= args.uav_video_fps <= 30:
        parser.error('--uav-video-fps must be between 2 and 30')
    if not math.isfinite(args.observer_video_fps) or not 2 <= args.observer_video_fps <= 30:
        parser.error('--observer-video-fps must be between 2 and 30')
    return run(args)


if __name__=="__main__": raise SystemExit(main())
