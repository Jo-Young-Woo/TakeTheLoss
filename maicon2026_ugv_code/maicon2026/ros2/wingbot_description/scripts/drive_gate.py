#!/usr/bin/env python3
"""Run Gate C and solo Gate D in an isolated, empty Fortress world.

Ground truth is measured independently from SceneBroadcaster; it is used only
for validation. This does not run an Agent, mission strategy, or a game server.
Uses a private Ignition partition/ROS domain and stops all children on exit.
"""
import argparse
from bisect import bisect_left
from collections import deque
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import xml.etree.ElementTree as ET

import numpy as np
import yaml


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def summarize(rows, command, target_duration):
    if len(rows) < 10:
        raise ValueError("Too few independent pose/odometry samples")
    result = {"command": command, "command_sim_seconds": target_duration, "samples": len(rows)}
    for key in ("truth", "odom"):
        values = np.asarray([r[key] for r in rows])
        delta = values[-1,:2]-values[0,:2]
        heading = values[0,2]
        result[key+"_distance_m"] = float(np.linalg.norm(delta))
        result[key+"_forward_m"] = float(delta @ [math.cos(heading), math.sin(heading)])
        result[key+"_lateral_m"] = float(delta @ [-math.sin(heading), math.cos(heading)])
        result[key+"_yaw_rad"] = float(np.unwrap(values[:,2])[-1]-np.unwrap(values[:,2])[0])
        stamps = np.asarray([r["odom_stamp"] for r in rows])
        intervals = np.diff(stamps)
        if np.any(intervals <= 0):
            raise ValueError("Measurement timestamps must strictly increase")
        result[key+"_sampled_peak_linear_mps"] = float(np.max(np.linalg.norm(np.diff(values[:,:2], axis=0),axis=1)/intervals))
        result[key+"_sampled_peak_angular_rps"] = float(np.max(np.abs(np.diff(np.unwrap(values[:,2])))/intervals))
    result["odom_twist_peak_linear_mps"] = max(abs(r["odom_twist"][0]) for r in rows)
    result["odom_twist_peak_angular_rps"] = max(abs(r["odom_twist"][1]) for r in rows)
    v, w = command
    if v:
        expected = math.copysign(min(abs(v), .6)*target_duration,v)
        actual = result["truth_forward_m"]
        result.update(expected_forward_m=expected, expected_distance_m=abs(expected),
                      truth_error_ratio=abs(actual-expected)/abs(expected),
                      odom_error_ratio=abs(result["odom_forward_m"]-actual)/max(abs(actual),1e-9))
    elif w:
        expected = math.copysign(min(abs(w),3)*target_duration,w)
        actual = result["truth_yaw_rad"]
        result.update(expected_yaw_rad=expected, truth_error_ratio=abs(actual-expected)/abs(expected),
                      odom_error_ratio=abs(result["odom_yaw_rad"]-actual)/max(abs(actual),1e-9))
    return result


def stamp_seconds(stamp):
    return float(stamp.sec)+float(stamp.nanosec)*1e-9


def assess_segment(segment):
    """End-to-end motion and sampled speed limits must both pass."""
    tolerance=.05 if segment["command"][0] else .10
    segment["velocity_caps_passed"]=(segment["truth_sampled_peak_linear_mps"]<=.6*1.05
        and segment["truth_sampled_peak_angular_rps"]<=3*1.10
        and segment["odom_twist_peak_linear_mps"]<=.6+1e-6
        and segment["odom_twist_peak_angular_rps"]<=3+1e-6)
    segment["passed"]=(segment["truth_error_ratio"]<=tolerance and segment["odom_error_ratio"]<=.05
                        and segment["velocity_caps_passed"])
    return segment["passed"]


def align_truth(history, stamp, max_bracket_s=.05):
    """Interpolate independent stamped truth to an odometry timestamp.

    Never extrapolate or substitute wall receipt time. Interpolation brackets
    are retained in the samples so time alignment can be audited afterwards.
    """
    if not history:
        return None
    index = bisect_left([sample[0] for sample in history], stamp)
    if index < len(history) and abs(history[index][0]-stamp) <= 1e-9:
        sample = history[index]
        return list(sample[1]), sample[2], list(sample[3]), (sample[0],sample[0])
    if index == 0 or index == len(history):
        return None
    first, last = history[index-1], history[index]
    interval = last[0]-first[0]
    if interval <= 0 or interval > max_bracket_s:
        return None
    ratio = (stamp-first[0])/interval
    pose = [first[1][i]+ratio*(last[1][i]-first[1][i]) for i in (0,1)]
    angular = math.atan2(math.sin(last[1][2]-first[1][2]),math.cos(last[1][2]-first[1][2]))
    pose.append(first[1][2]+ratio*angular)
    z = first[2]+ratio*(last[2]-first[2])
    # Full quaternion is diagnostic only; yaw used for all scoring is aligned.
    return pose,z,list(first[3]),(first[0],last[0])


def stop_children(children, logs):
    """Best-effort process-group cleanup cannot abandon later children."""
    for child in reversed(children):
        if child.poll() is None:
            try:
                os.killpg(child.pid,signal.SIGINT)
            except ProcessLookupError:
                pass
    for child in reversed(children):
        for sig,timeout in ((None,8),(signal.SIGTERM,5),(signal.SIGKILL,3)):
            if child.poll() is not None:
                break
            try:
                if sig is not None:
                    os.killpg(child.pid,sig)
                child.wait(timeout=timeout)
                break
            except (ProcessLookupError,subprocess.TimeoutExpired):
                continue
    for log in logs:
        try:
            log.close()
        except OSError:
            pass


def make_world(urdf, destination):
    document = ET.parse(urdf).getroot()
    drive = next(p for p in document.findall(".//plugin") if "DiffDrive" in p.get("name",""))
    topics = {"cmd":drive.findtext("topic"), "odom":drive.findtext("odom_topic")}
    sensors = {}
    for sensor in document.findall(".//sensor"):
        sensors[sensor.get("type")] = sensor.findtext("topic")
    converted = subprocess.run(["ign","sdf","-p",str(urdf)],check=True,capture_output=True,text=True)
    model = ET.fromstring(converted.stdout).find("model")
    model.set("name","wingbot")
    pose = model.find("pose")
    if pose is None:
        pose = ET.SubElement(model,"pose")
    pose.text="0 0 0.003 0 0 0"
    sdf = ET.Element("sdf",version="1.9")
    world = ET.SubElement(sdf,"world",name="wingbot_drive_gate")
    gravity=ET.SubElement(world,"gravity");gravity.text="0 0 -9.8"
    physics=ET.SubElement(world,"physics",name="dart",type="dart")
    for key,value in (("max_step_size",".004"),("real_time_factor","1")):
        ET.SubElement(physics,key).text=value
    for name,file in (("Physics","physics"),("UserCommands","user-commands"),
                      ("SceneBroadcaster","scene-broadcaster"),("Sensors","sensors"),("Imu","imu")):
        p=ET.SubElement(world,"plugin",filename="ignition-gazebo-"+file+"-system",name="ignition::gazebo::systems::"+name)
        if name=="Sensors":
            ET.SubElement(p,"render_engine").text="ogre2"
    scene=ET.SubElement(world,"scene")
    for key,value in (("ambient",".65 .65 .65 1"),("background",".1 .1 .1 1"),("shadows","false")):
        ET.SubElement(scene,key).text=value
    floor=ET.SubElement(world,"model",name="ground")
    ET.SubElement(floor,"static").text="true"
    link=ET.SubElement(floor,"link",name="ground")
    for tag in ("collision","visual"):
        e=ET.SubElement(link,tag,name="floor")
        ET.SubElement(e,"pose").text="0 0 -.05 0 0 0"
        ET.SubElement(ET.SubElement(ET.SubElement(e,"geometry"),"box"),"size").text="20 20 .1"
        if tag=="collision":
            ode=ET.SubElement(ET.SubElement(ET.SubElement(e,"surface"),"friction"),"ode")
            ET.SubElement(ode,"mu").text="1.0"; ET.SubElement(ode,"mu2").text="1.0"
    # Single Pose messages preserve the simulator timestamp. Fortress's
    # Pose_V -> TFMessage bridge drops the outer header on individual poses.
    publisher=ET.SubElement(model,"plugin",filename="ignition-gazebo-pose-publisher-system",
                            name="ignition::gazebo::systems::PosePublisher")
    for key,value in (("publish_model_pose","true"),("publish_link_pose","false"),
                      ("publish_collision_pose","false"),("publish_visual_pose","false"),
                      # Fortress implementation gates the root model on both
                      # model and nested-model flags. There are no nested models.
                      ("publish_nested_model_pose","true"),("use_pose_vector_msg","false"),
                      ("update_frequency","100")):
        ET.SubElement(publisher,key).text=value
    world.append(model)
    ET.indent(sdf)
    destination.write_text(ET.tostring(sdf,encoding="unicode"),encoding="utf-8")
    return topics,sensors


def run(children, logs):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--urdf",type=Path,required=True)
    p.add_argument("--package-root",type=Path,required=True)
    p.add_argument("--output",type=Path,required=True)
    p.add_argument("--perf-seconds",type=float,default=60)
    p.add_argument("--skip-performance",action="store_true")
    p.add_argument("--performance-only",action="store_true")
    p.add_argument("--timeout",type=float,default=240)
    p.add_argument("--ros-domain-id",type=int,required=True,
                   help="Explicit unused ROS domain for this validation run (0..232)")
    args=p.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    # Environment changes are process-local, never persisted to the user shell.
    os.environ["IGN_PARTITION"]="wingbot_drive_gate_"+str(os.getpid())
    if not 0 <= args.ros_domain_id <= 232:
        raise ValueError("ROS domain ID must be 0..232")
    os.environ["ROS_DOMAIN_ID"]=str(args.ros_domain_id)
    os.environ["LIBGL_ALWAYS_SOFTWARE"]="1"
    os.environ["IGN_GAZEBO_RESOURCE_PATH"]=str(args.package_root.resolve().parent)+os.pathsep+os.environ.get("IGN_GAZEBO_RESOURCE_PATH","")
    os.environ["GZ_SIM_RESOURCE_PATH"]=os.environ["IGN_GAZEBO_RESOURCE_PATH"]
    world=args.output/"drive_world.sdf"
    topics,sensors=make_world(args.urdf,world)
    specs=[
        ("/clock","/clock","rosgraph_msgs/msg/Clock","ignition.msgs.Clock","GZ_TO_ROS"),
        ("/wingbot_gate/truth","/model/wingbot/pose","geometry_msgs/msg/PoseStamped","ignition.msgs.Pose","GZ_TO_ROS"),
        ("/wingbot_gate/odom",topics["odom"],"nav_msgs/msg/Odometry","ignition.msgs.Odometry","GZ_TO_ROS"),
        ("/wingbot_gate/cmd",topics["cmd"],"geometry_msgs/msg/Twist","ignition.msgs.Twist","ROS_TO_GZ")]
    for kind,ros_type,gz_type in (("gpu_lidar","sensor_msgs/msg/LaserScan","ignition.msgs.LaserScan"),("imu","sensor_msgs/msg/Imu","ignition.msgs.IMU")):
        if kind in sensors:
            specs.append(("/wingbot_gate/"+kind,sensors[kind],ros_type,gz_type,"GZ_TO_ROS"))
    bridge=[dict(zip(("ros_topic_name","gz_topic_name","ros_type_name","gz_type_name","direction"),item)) for item in specs]
    bridge_path=args.output/"bridge.yaml"
    bridge_path.write_text(yaml.safe_dump(bridge),encoding="utf-8")
    def start(command,name):
        log=(args.output/(name+".log")).open("w",encoding="utf-8")
        logs.append(log)
        child=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        children.append(child)
        return child
    start(["ign","gazebo","-r","-s",str(world),"--render-engine","ogre2"],"server")
    start(["ros2","run","ros_gz_bridge","parameter_bridge","--ros-args","-p","config_file:="+str(bridge_path)],"bridge")
    if "camera" in sensors:
        start(["ros2","run","ros_gz_image","image_bridge",sensors["camera"]],"images")
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import Twist, PoseStamped
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import Image,Imu,LaserScan
    rclpy.init(args=[])
    node=Node("wingbot_drive_gate")
    latest,counts,wall_received={}, {}, {}
    truth_history,odom_history=deque(maxlen=5000),deque(maxlen=1000)
    def received(key,msg):
        latest[key]=msg
        counts[key]=counts.get(key,0)+1
        wall_received[key]=time.monotonic()
        if key=="truth":
            stamp=stamp_seconds(msg.header.stamp)
            pos,rot=msg.pose.position,msg.pose.orientation
            if not truth_history or stamp>truth_history[-1][0]:
                truth_history.append((stamp,(pos.x,pos.y,yaw(rot)),pos.z,(rot.x,rot.y,rot.z,rot.w)))
        elif key=="odom":
            stamp=stamp_seconds(msg.header.stamp)
            if not odom_history or stamp>odom_history[-1][0]:
                odom_history.append((stamp,msg))
    for key,topic,cls in (("clock","/clock",Clock),("odom","/wingbot_gate/odom",Odometry),
                          ("truth","/wingbot_gate/truth",PoseStamped),
                          ("gpu_lidar","/wingbot_gate/gpu_lidar",LaserScan),
                          ("imu","/wingbot_gate/imu",Imu)):
        node.create_subscription(cls,topic,lambda msg,key=key: received(key,msg),qos_profile_sensor_data)
    if "camera" in sensors:
        node.create_subscription(Image,sensors["camera"],lambda msg: received("camera",msg),qos_profile_sensor_data)
    pub=node.create_publisher(Twist,"/wingbot_gate/cmd",10)
    opened=time.monotonic()
    samples_path=args.output/"drive_samples.jsonl"
    sample_file=samples_path.open("w",encoding="utf-8")
    def pump():
        rclpy.spin_once(node,timeout_sec=.005)
        if time.monotonic()-opened>args.timeout:
            raise TimeoutError("Gate wall-time budget exceeded")
        if any(child.poll() is not None for child in children):
            raise RuntimeError("A Gazebo/bridge process exited; inspect logs")
    def command(v,w):
        msg=Twist();msg.linear.x=float(v);msg.angular.z=float(w);pub.publish(msg)
    def clock():
        msg=latest["clock"].clock
        return msg.sec+msg.nanosec*1e-9
    def row():
        for key in ("clock","odom","truth"):
            if time.monotonic()-wall_received.get(key,-1e9)>1:
                raise RuntimeError("Stale "+key)
        for stamp,message in reversed(odom_history):
            if clock()-stamp>.15:
                break
            aligned=align_truth(truth_history,stamp)
            if aligned is None:
                continue
            truth,z,quaternion,bracket=aligned
            od=message.pose.pose
            return dict(clock=clock(),odom_stamp=stamp,truth_stamp=stamp,truth_bracket=bracket,
                        truth=truth,odom=[od.position.x,od.position.y,yaw(od.orientation)],
                        odom_twist=[message.twist.twist.linear.x,message.twist.twist.angular.z],
                        z=z, quaternion=quaternion)
        raise RuntimeError("No fresh truth bracket for odometry timestamp; refusing unsynchronized measurements")
    def phase(name,v,w,duration):
        start_t=clock()
        rows=[row()]
        last_stamp=rows[-1]["odom_stamp"]
        last_sent=-1e9
        while clock()-start_t<duration:
            if clock()-last_sent>=.02:
                command(v,w);last_sent=clock()
            pump()
            current=row()
            if current["odom_stamp"]>last_stamp:
                last_stamp=current["odom_stamp"];rows.append(current)
        command(0,0)
        actual_command_seconds=clock()-start_t
        # Include only physical response while stopping, not an invented distance.
        stop_t=clock()
        while clock()-stop_t<.3:
            pump();command(0,0)
            current=row()
            if current["odom_stamp"]>last_stamp:
                last_stamp=current["odom_stamp"];rows.append(current)
        for current in rows:
            current["phase"]=name
            sample_file.write(json.dumps(current)+"\n")
        sample_file.flush()
        result=summarize(rows,[v,w],actual_command_seconds) if v or w else {}
        result["name"]=name
        result.update(command_start_stamp=start_t,command_stop_stamp=start_t+actual_command_seconds,
                      observation_start_stamp=rows[0]["odom_stamp"],observation_end_stamp=rows[-1]["odom_stamp"],
                      stop_settling_sim_seconds=clock()-stop_t,
                      maximum_truth_bracket_s=max(b-a for a,b in (r["truth_bracket"] for r in rows)))
        print(json.dumps(result),flush=True)
        return result
    report={"gate":"C","passed":False,"segments":[],"rendering":"CPU Ogre2; no GUI",
            "urdf_sha256":hashlib.sha256(args.urdf.read_bytes()).hexdigest(),
            "world_sha256":hashlib.sha256(world.read_bytes()).hexdigest()}
    performance_passed = True
    try:
        while not all(k in latest for k in ("clock","odom","truth")) or pub.get_subscription_count()==0:
            pump()
            if time.monotonic()-opened>45:
                raise TimeoutError("No clock/odom/truth/command subscriber")
        while True:
            pump()
            try:
                row()
                break
            except RuntimeError:
                if time.monotonic()-opened>45:
                    raise TimeoutError("No synchronized truth/odometry samples")
        phase("settle",0,0,2)
        if not args.performance_only:
            for name,v,w,duration in (("straight",.3,0,5),("spin",0,1,3),
                                      ("clamp_linear",2,0,1.5),("clamp_angular",0,8,1),
                                      ("clamp_linear_reverse",-2,0,1.5),("clamp_angular_reverse",0,-8,1)):
                report["segments"].append(phase(name,v,w,duration))
                phase(name+"_settle",0,0,.5)
            for segment in report["segments"]:
                assess_segment(segment)
            report["passed"]=all(s["passed"] for s in report["segments"])
            (args.output/"gate_c.json").write_text(json.dumps(report,indent=2)+"\n",encoding="utf-8")
        if not args.skip_performance:
            phase("performance_warmup",0,0,3)
            wall0,sim0=time.monotonic(),clock()
            count0=dict(counts); perf_rows=[(wall0,sim0)]
            while time.monotonic()-wall0<args.perf_seconds:
                pump()
                now=time.monotonic()
                if now-perf_rows[-1][0]>=.01:
                    perf_rows.append((now,clock()))
            elapsed=perf_rows[-1][0]-wall0
            rtf=(perf_rows[-1][1]-sim0)/elapsed
            rates={k:(counts.get(k,0)-count0.get(k,0))/elapsed for k in ("camera","gpu_lidar","imu")}
            intervals=[];anchor=perf_rows[0]
            for sample in perf_rows[1:]:
                if sample[0]-anchor[0]>=1:
                    intervals.append((sample[1]-anchor[1])/(sample[0]-anchor[0]));anchor=sample
            result=dict(gate="D_solo_full",passed=rtf>=.98 and all(rates[k]>=r for k,r in (("camera",8),("gpu_lidar",8),("imu",16))),
                        rtf_mean=rtf,rtf_min_1s=min(intervals),wall_seconds=elapsed,sensor_hz=rates,
                        thresholds=dict(rtf_mean=.98,camera_hz=8,lidar_hz=8,imu_hz=16),
                        urdf_sha256=report["urdf_sha256"],rendering=report["rendering"])
            performance_passed = result["passed"]
            scan = latest.get("gpu_lidar")
            if scan is not None:
                ranges = np.array(scan.ranges)
                result["scan_diagnostic"] = dict(samples=len(ranges),
                    near_self_returns=int(np.sum(np.isfinite(ranges) & (ranges < .08))),
                    infinite=int(np.sum(np.isinf(ranges))), minimum=scan.range_min, maximum=scan.range_max)
            (args.output/"gate_d_solo.json").write_text(json.dumps(result,indent=2)+"\n",encoding="utf-8")
            print(json.dumps(result),flush=True)
        return 0 if (args.performance_only or report["passed"]) and performance_passed else 1
    except Exception as exc:
        report["error"]=str(exc)
        (args.output/"gate_c_error.json").write_text(json.dumps(report,indent=2)+"\n",encoding="utf-8")
        raise
    finally:
        try:
            if rclpy.ok():
                for _ in range(3):
                    command(0,0)
        finally:
            sample_file.close()
            try:
                node.destroy_node()
            finally:
                rclpy.try_shutdown()


def main():
    children,logs=[],[]
    try:
        return run(children,logs)
    finally:
        # Covers launch, import and ROS initialization failures as well as the
        # measured test body. ROS shutdown exceptions cannot skip child cleanup.
        stop_children(children,logs)


if __name__=="__main__":
    raise SystemExit(main())
