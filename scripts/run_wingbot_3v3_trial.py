#!/usr/bin/env python3
"""Six physical WINGBOTs, referee-resolved combat and actual third-person film.

Training bot navigation reads Gazebo model poses; this is a controlled combat
drill, not six independent vision agents. B2 uses the MIL movement and gimbal
services, while lightweight opponents use their physical DiffDrive/head joints.
"""
import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
sys.path.insert(0,str(ROOT/"ros2/maicon_arena_gz/scripts"))
sys.path.insert(0,str(ROOT/"ros2/wingbot_description/scripts"))
from drive_gate import stop_children
from wingbot_runtime import WingbotFrames
from run_wingbot_autonomous_trial import CameraRecorder

PAIRS={"B-1":"R-1","B-2":"R-2","B-3":"R-3"}
BLUE_TARGETS={"B-1":(2.07,.97),"B-2":(2.07,-.63),"B-3":(2.07,-.97)}
ROUTES={
 "B-1":[(-2.15,2.18),(-.55,2.18),(-.40,1.72),(.40,1.72),(.65,2.18),(1.95,2.18),(2.07,.97)],
 "B-2":[(-2.05,.65),(-.55,.65),(1.95,.65),(2.07,-.63)],
 "B-3":[(-2.15,-2.18),(-.55,-2.18),(-.40,-1.72),(.40,-1.72),(.65,-2.18),(1.95,-2.18),(2.07,-.97)],
}
SLOTS=tuple(PAIRS)+tuple(PAIRS.values())

def wrap(a):return math.atan2(math.sin(a),math.cos(a))
def limit(x,a):return max(-a,min(a,x))
def orientation(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

def aiming(frames, own, other, facing):
    """Model-frame boresight solver, with visible receiver hemisphere check."""
    ox,oy,oz,yaw=own
    tx,ty,tz,tyaw=other
    candidates=[]
    for center,radius,normal in frames.receivers():
        c,s=math.cos(tyaw),math.sin(tyaw)
        rx,ry=tx+c*center[0]-s*center[1],ty+s*center[0]+c*center[1]
        rz=tz+center[2]
        nx,ny=c*normal[0]-s*normal[1],s*normal[0]+c*normal[1]
        visibility=(ox-rx)*nx+(oy-ry)*ny
        if visibility<=.05:continue
        physical_pan=0.;tilt=0.
        for _ in range(4):
            local,_=frames.laser(physical_pan,tilt)
            cl,sl=math.cos(yaw),math.sin(yaw)
            lx,ly=ox+cl*local[0]-sl*local[1],oy+sl*local[0]+cl*local[1]
            lz=oz+local[2]
            bearing=wrap(math.atan2(ry-ly,rx-lx)-yaw)
            physical_pan=-bearing
            tilt=math.atan2(rz-lz,math.hypot(rx-lx,ry-ly))
        if abs(physical_pan)>math.radians(49) or abs(tilt)>math.radians(14.5):
            continue
        candidates.append((math.hypot(rx-ox,ry-oy),physical_pan,tilt,visibility))
    return min(candidates) if candidates else None

def run(args):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import Image
    from std_msgs.msg import String,Float64
    from tf2_msgs.msg import TFMessage
    from rosgraph_msgs.msg import Clock
    from mil_interfaces_sim.srv import Gimbal,Fire
    from ament_index_python.packages import get_package_share_directory

    out=args.output
    if out.exists():raise ValueError("Use a new output directory")
    out.mkdir(parents=True)
    share=Path(get_package_share_directory("maicon_arena_gz"))
    model=Path(get_package_share_directory("wingbot_description"))
    frames=WingbotFrames(model/"config/frames.yaml")
    os.environ.update(ROS_DOMAIN_ID=str(args.ros_domain_id),IGN_PARTITION="wingbot_3v3_"+str(os.getpid()),
        LIBGL_ALWAYS_SOFTWARE="1",FASTRTPS_DEFAULT_PROFILES_FILE=str(share/"config/fastdds_shm64.xml"),
        FASTDDS_DEFAULT_PROFILES_FILE=str(share/"config/fastdds_shm64.xml"),
        RMW_IMPLEMENTATION="rmw_fastrtps_cpp")
    command=["ros2","launch","maicon_arena_gz","arena.launch.py","robot_model:=wingbot",
             "ugvs:=6","ego_slot:=B-2","profile:=wingbot_recording","gui:=false",
             "validation_camera:=true","uav_cam:=true","image_transport_profile:=shm64"]
    logs=[(out/"launch.log").open("w",encoding="utf-8")]
    children=[];node=None;recorder=None;errors=[];events=[]
    results={"scope":"Six physical WINGBOT models; B2 full sensors; five sensorless bot opponents",
             "combat_ai":"central training controller reads Gazebo model poses",
             "referee":"actual geometric laser/receiver and physical occlusion",
             "command":command,"slot_teams":{s:("BLUE" if s[0]=="B" else "RED") for s in SLOTS}}
    try:
        children.append(subprocess.Popen(command,stdout=logs[0],stderr=subprocess.STDOUT,start_new_session=True))
        rclpy.init()
        node=rclpy.create_node("wingbot_six_combat")
        recorder=CameraRecorder(out/"wingbot_3v3_third_person.mp4",10.,"WINGBOT 3v3 | actual Gazebo combat")
        state={};poses={};clock=[None];last_control=0.;origin=None
        pubs={s:node.create_publisher(Twist,"/cmd_vel" if s=="B-2" else "/ugv_"+s.replace("-","")+"/cmd_vel",10) for s in SLOTS}
        heads={s:{axis:node.create_publisher(Float64,"/ugv_"+s.replace("-","")+"/"+axis+"_cmd",10)
                  for axis in ("pan","tilt")} for s in SLOTS if s!="B-2"}
        control=node.create_publisher(String,"/arena/control",10)
        fire_pub=node.create_publisher(String,"/arena/fire",10)
        gimbal_cli=node.create_client(Gimbal,"/mil/gimbal")
        fire_cli=node.create_client(Fire,"/mil/fire")
        def on_clock(msg):clock[0]=msg.clock.sec+msg.clock.nanosec*1e-9
        def on_state(msg):state.clear();state.update(json.loads(msg.data))
        def on_pose(msg):
            for t in msg.transforms:
                if "::" in t.child_frame_id:continue
                name=t.child_frame_id
                if name.startswith("ugv_"):
                    key=name[4]+("-"+name[5:])
                    if key in SLOTS:
                        p,q=t.transform.translation,t.transform.rotation
                        poses[key]=(p.x,p.y,p.z,orientation(q))
        node.create_subscription(Clock,"/clock",on_clock,qos_profile_sensor_data)
        node.create_subscription(String,"/arena/state",on_state,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        node.create_subscription(TFMessage,"/arena/dynamic_pose",on_pose,qos_profile_sensor_data)
        node.create_subscription(Image,"/arena/validation/image_raw",recorder.receive,qos_profile_sensor_data)
        node.create_subscription(String,"/arena/events",lambda m:events.append(json.loads(m.data)),100)
        def command_vel(slot,v=0.,w=0.):
            msg=Twist();msg.linear.x=float(v);msg.angular.z=float(w);pubs[slot].publish(msg)
        def stop_all():
            for slot in SLOTS:command_vel(slot)
        def spin_for(seconds):
            until=time.monotonic()+seconds
            while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.02)
        ready_until=time.monotonic()+65
        while time.monotonic()<ready_until:
            rclpy.spin_once(node,timeout_sec=.04)
            if state.get("phase")=="ready" and len(poses)==6 and recorder.accepted>=2 and gimbal_cli.service_is_ready() and fire_cli.service_is_ready():break
        else:raise TimeoutError("Six models, referee, B2 MIL and recording camera did not become ready")
        results["initial_models"]=sorted(poses)
        control.publish(String(data=json.dumps({"cmd":"start"})))
        start_wait=time.monotonic()+5
        while state.get("phase")!="running" and time.monotonic()<start_wait:rclpy.spin_once(node,timeout_sec=.02)
        if state.get("phase")!="running":raise TimeoutError("Referee did not start")
        origin=clock[0]
        routes={s:list(path) for s,path in ROUTES.items()}
        heading_target={s:(math.radians(75) if s!="B-3" else math.radians(-75)) for s in PAIRS}
        heading_target.update({s:(math.radians(170) if s=="R-3" else math.radians(190)) for s in PAIRS.values()})
        last_fire={s:-100. for s in SLOTS};last_head={s:-100. for s in SLOTS};aim_since={s:None for s in SLOTS}
        last_b2_gimbal=(None,None);last_report=-1.;walk=[]
        wallstart=time.monotonic()
        while time.monotonic()-wallstart<args.wall_timeout:
            rclpy.spin_once(node,timeout_sec=.01)
            if clock[0] is None:continue
            elapsed=clock[0]-origin
            if state.get("phase") in ("reporting","finished"):break
            if elapsed>=args.duration:
                control.publish(String(data=json.dumps({"cmd":"report"})))
                spin_for(.2);break
            if clock[0]-last_control<.05:continue
            last_control=clock[0]
            for slot in SLOTS:
                if slot not in poses:continue
                x,y,z,yaw=poses[slot]
                if slot in ROUTES:
                    points=routes[slot]
                    if points and math.hypot(points[0][0]-x,points[0][1]-y)<.13:points.pop(0)
                    if points:
                        gx,gy=points[0]
                        dist=math.hypot(gx-x,gy-y);err=wrap(math.atan2(gy-y,gx-x)-yaw)
                        command_vel(slot, min(.44,.85*dist) if abs(err)<.4 else 0.,limit(2.6*err,1.5))
                        aim_since[slot]=None
                        continue
                err=wrap(heading_target[slot]-yaw)
                command_vel(slot,0.,limit(2.5*err,.8))
                if abs(err)>.08:
                    aim_since[slot]=None
                    continue
                other=PAIRS[slot] if slot in PAIRS else next(b for b,r in PAIRS.items() if r==slot)
                if other not in poses:continue
                solution=aiming(frames,poses[slot],poses[other],yaw)
                if solution is None or solution[0]>1.35:
                    aim_since[slot]=None;continue
                _,pan,tilt,_=solution
                if slot=="B-2":
                    if clock[0]-last_head[slot]>.5 and (last_b2_gimbal[0] is None or abs(pan-last_b2_gimbal[0])>.005 or abs(tilt-last_b2_gimbal[1])>.005):
                        req=Gimbal.Request();req.pan=math.degrees(pan)/.5;req.tilt=math.degrees(tilt)/.15;req.speed=180.;req.acc=60.
                        gimbal_cli.call_async(req);last_b2_gimbal=(pan,tilt);last_head[slot]=clock[0]
                else:
                    heads[slot]["pan"].publish(Float64(data=float(pan)))
                    heads[slot]["tilt"].publish(Float64(data=float(tilt)))
                if aim_since[slot] is None:aim_since[slot]=clock[0]
                if clock[0]-aim_since[slot]>1.5 and clock[0]-last_fire[slot]>10.5:
                    if slot=="B-2":
                        fire_cli.call_async(Fire.Request())
                    else:
                        fire_pub.publish(String(data=json.dumps({"slot":slot,"pan_rad":pan,"tilt_rad":tilt})))
                    last_fire[slot]=clock[0]
            if elapsed-last_report>=2.:
                blue_hits=sum(e.get("type")=="ugv_hit" and e.get("shooter","").startswith("B") for e in events)
                red_hits=sum(e.get("type")=="ugv_hit" and e.get("shooter","").startswith("R") for e in events)
                recorder.writer.label=f"WINGBOT 3v3 | BLUE {blue_hits} : RED {red_hits} | referee hits"
                compact={"sim_s":round(clock[0],2),"elapsed":round(elapsed,2),"poses":{s:[round(v,3) for v in poses[s]] for s in SLOTS if s in poses},
                         "route_steps":{s:len(routes[s]) for s in ROUTES},"shots":{s:sum(e.get("type")=="laser_fired" and e.get("slot")==s for e in events) for s in SLOTS},
                         "hits":sum(e.get("type")=="ugv_hit" for e in events)}
                walk.append(compact)
                (out/"progress.json").write_text(json.dumps(compact,indent=2),encoding="utf-8")
                print(json.dumps({"elapsed":round(elapsed,1),"steps":compact["route_steps"],"shots":compact["shots"],"hits":compact["hits"]}),flush=True)
                last_report=elapsed
        stop_all();spin_for(.2)
        results.update(sim_duration_s=clock[0]-origin if origin is not None else 0.,
                       ended_phase=state.get("phase"),state=state,events=events,
                       model_pose_evaluation_only=False,control_trace=walk,
                       laser_fired=sum(e.get("type")=="laser_fired" for e in events),
                       ugv_hits=[e for e in events if e.get("type")=="ugv_hit"],
                       friendly_fire=[e for e in events if e.get("type")=="ugv_hit" and e.get("friendly_fire")],
                       team_shots={team:sum(e.get("type")=="laser_fired" and e.get("slot","").startswith(team[0]) for e in events) for team in ("BLUE","RED")})
        shot_slots={e.get("slot") for e in events if e.get("type")=="laser_fired"}
        hit_lanes={e.get("shooter","")[-1] for e in results["ugv_hits"] if not e.get("friendly_fire")}
        results["shot_slots"]=sorted(shot_slots)
        results["hit_lanes"]=sorted(hit_lanes)
        results["passed"]=len(poses)==6 and shot_slots==set(SLOTS) and hit_lanes=={"1","2","3"}
    except Exception as exc:
        errors.append(type(exc).__name__+": "+str(exc))
        results["passed"]=False
    finally:
        if node is not None:
            try:stop_all()
            except Exception:pass
            node.destroy_node()
            rclpy.try_shutdown()
        if recorder is not None:
            results["recording"]=recorder.close()
        stop_children(children,logs)
        results["errors"]=errors
        (out/"combat_report.json").write_text(json.dumps(results,ensure_ascii=False,indent=2,default=str),encoding="utf-8")
    print(json.dumps({"passed":results["passed"],"shots":results.get("team_shots"),"hits":len(results.get("ugv_hits",[])),"errors":errors}),flush=True)
    return 0 if results["passed"] else 1

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--output",type=Path,required=True)
    ap.add_argument("--duration",type=float,default=300.)
    ap.add_argument("--wall-timeout",type=float,default=540.)
    ap.add_argument("--ros-domain-id",type=int,default=79)
    args=ap.parse_args()
    if not 0<args.duration<=300 or args.wall_timeout<args.duration:ap.error("Invalid time bounds")
    return run(args)
if __name__=="__main__":raise SystemExit(main())
