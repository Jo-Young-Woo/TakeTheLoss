#!/usr/bin/env python3
"""Read-only sensor yaw audit. Ground truth is used ONLY by offline evaluation."""
import argparse
import bisect
import json
import math
from pathlib import Path
import time


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


def wrapped(value):
    return math.atan2(math.sin(value),math.cos(value))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--slot",default="B3")
    parser.add_argument("--seconds",type=float,default=20.)
    parser.add_argument("--output",type=Path,required=True)
    args=parser.parse_args()
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu
    from tf2_msgs.msg import TFMessage
    from rosgraph_msgs.msg import Clock
    import numpy as np
    rclpy.init();node=Node("maicon_readonly_heading_audit")
    data={"odom":[],"imu":[],"evaluation_truth":[]}
    clock_value=[None]
    def clock(message):
        clock_value[0]=message.clock.sec+message.clock.nanosec*1e-9
    def stamp(message):
        return message.header.stamp.sec+message.header.stamp.nanosec*1e-9
    def odom(message):
        data["odom"].append([stamp(message),yaw(message.pose.pose.orientation)])
    def imu(message):
        data["imu"].append([stamp(message),yaw(message.orientation),message.angular_velocity.z])
    def poses(message):
        for tr in message.transforms:
            if tr.child_frame_id=="ugv_"+args.slot:
                observed=stamp(tr) or clock_value[0]
                if observed is not None:
                    data["evaluation_truth"].append([observed,yaw(tr.transform.rotation)])
    node.create_subscription(Clock,"/clock",clock,qos_profile_sensor_data)
    node.create_subscription(Odometry,"/robots/"+args.slot+"/odom",odom,qos_profile_sensor_data)
    node.create_subscription(Imu,"/robots/"+args.slot+"/imu/data",imu,qos_profile_sensor_data)
    node.create_subscription(TFMessage,"/arena/dynamic_pose",poses,qos_profile_sensor_data)
    started=time.monotonic()
    try:
        while rclpy.ok() and time.monotonic()-started<args.seconds:
            rclpy.spin_once(node,timeout_sec=.1)
    finally:
        node.destroy_node();rclpy.shutdown()
    truth=sorted(data["evaluation_truth"])
    times=[row[0] for row in truth]
    results={}
    for source in ("odom","imu"):
        paired=[]
        for row in data[source]:
            i=bisect.bisect_left(times,row[0])
            candidates=[truth[j] for j in (i-1,i) if 0<=j<len(truth)]
            if not candidates:continue
            best=min(candidates,key=lambda tr:abs(tr[0]-row[0]))
            if abs(best[0]-row[0])<=.05:
                paired.append([row[0],row[1],best[1]])
        if paired:
            first=paired[0]
            error=np.asarray([wrapped((r[1]-first[1])-(r[2]-first[2])) for r in paired])
            absolute=np.degrees(np.abs(error))
            results[source]=dict(pairs=len(paired),span_s=paired[-1][0]-first[0],
                absolute_initial_offset_deg=math.degrees(wrapped(first[1]-first[2])),
                yaw_delta_error_p50_deg=float(np.percentile(absolute,50)),
                yaw_delta_error_p95_deg=float(np.percentile(absolute,95)),
                yaw_delta_error_max_deg=float(absolute.max()),
                final_yaw_delta_error_deg=math.degrees(float(error[-1])))
        else:
            results[source]={"pairs":0,"status":"no timestamp-matched samples"}
    report=dict(scope="Read-only independent evaluator; truth never fed to Agent",
                slot=args.slot,wall_seconds=time.monotonic()-started,
                sample_counts={key:len(value)for key,value in data.items()},results=results,
                pose_timestamp_source="TF header if nonzero, otherwise most recently received simulation clock (arrival-time approximation)",
                raw=data,warning="Short observation window; not a completed mission localization validation")
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2),encoding="utf-8")
    print(json.dumps({k:v for k,v in report.items()if k!="raw"},indent=2))


if __name__=="__main__":
    main()
