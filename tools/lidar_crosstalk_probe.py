#!/usr/bin/env python3
"""Stationary real-hardware /scan recorder and transient-return study.

Record with the other LiDAR OFF for baseline, then ON at measured positions.
Gazebo has no interference model; this tool never enables a mission feature.
"""
from __future__ import annotations
import argparse
import json
import math
from pathlib import Path
import time
import numpy as np


def record(args):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    rclpy.init()
    node=Node("maicon_lidar_crosstalk_probe")
    start=time.monotonic()
    path=Path(args.output); path.parent.mkdir(parents=True,exist_ok=True)
    with path.open("w",encoding="utf-8") as stream:
        def on_scan(msg):
            row={"t":time.monotonic()-start,
                 "stamp":msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
                 "angle_min":msg.angle_min,"angle_increment":msg.angle_increment,
                 "range_max":msg.range_max,
                 "ranges":[float(x) if math.isfinite(x) else None for x in msg.ranges],
                 "intensities":list(msg.intensities),
                 "opponent_present":args.opponent_on,
                 "opponent_bearing_deg":args.bearing_deg,
                 "opponent_distance_m":args.distance_m,
                 "hardware_only":True}
            stream.write(json.dumps(row,allow_nan=False)+"\n")
        node.create_subscription(LaserScan,args.topic,on_scan,qos_profile_sensor_data)
        try:
            while rclpy.ok() and time.monotonic()-start<args.seconds:
                rclpy.spin_once(node,timeout_sec=.1)
        finally:
            node.destroy_node(); rclpy.shutdown()


def analyse(rows, baseline_rows, range_tol=.10):
    if len(baseline_rows)<3:
        raise ValueError("At least three stationary baseline scans are required")
    n=len(baseline_rows[0]["ranges"])
    if any(len(r["ranges"])!=n for r in rows+baseline_rows):
        raise ValueError("Do not mix scan resolutions")
    base=np.nanmedian(np.asarray([r["ranges"] for r in baseline_rows],dtype=float),axis=0)
    values=np.asarray([r["ranges"] for r in rows],dtype=float)
    unexplained=np.isfinite(values)&(np.abs(values-base)>range_tol)
    # No baseline return is unknown, not an interference observation.
    unexplained &= np.isfinite(base)
    times=np.asarray([r["t"] for r in rows],dtype=float)
    if len(times)<3 or not np.all(np.diff(times)>0):
        raise ValueError("Need at least three scans with increasing receive time")
    for i,row in enumerate(rows):
        angles=row["angle_min"]+np.arange(n)*row["angle_increment"]
        # Optional independent 6.1 tracker export prevents a real moving UGV
        # from being counted as cross-talk. Angles are scan-frame radians.
        for tr in row.get("tracked_ranges",[]):
            mask=(angles>=tr["angle_min"])&(angles<=tr["angle_max"])
            mask &= (values[i]>=tr["range_min"])&(values[i]<=tr["range_max"])
            unexplained[i,mask]=False
    transient=np.zeros_like(unexplained)
    # One/two-scan anomalies with clear preceding and following observations.
    for j in range(n):
        starts=np.flatnonzero(np.diff(np.r_[False,unexplained[:,j],False].astype(int))==1)
        ends=np.flatnonzero(np.diff(np.r_[False,unexplained[:,j],False].astype(int))==-1)
        for start,end in zip(starts,ends):
            if 0<start and end<len(rows) and end-start<=2:
                transient[start:end,j]=True
    events=[]
    for i in np.flatnonzero(transient.any(axis=1)):
        bins=np.flatnonzero(transient[i]); row=rows[i]
        angles=row["angle_min"]+bins*row["angle_increment"]
        intensities=row.get("intensities",[])
        bearing=row.get("opponent_bearing_deg")
        nearest=None if bearing is None else float(np.min(np.abs(np.angle(np.exp(1j*(angles-math.radians(bearing)))))))
        events.append({"t":float(times[i]),"bins":bins.tolist(),
            "bearing_error_deg":None if nearest is None else math.degrees(nearest),
            "measured_opponent_distance_m":row.get("opponent_distance_m"),
            "intensity":[float(intensities[b]) for b in bins if b<len(intensities)]})
    presence=np.asarray([float(bool(r.get("opponent_present"))) for r in rows])
    rate=transient.sum(axis=1)
    correlation=float(np.corrcoef(presence,rate)[0,1]) if np.std(presence)>0 and np.std(rate)>0 else None
    return {"status":"unverified_hardware_hypothesis", "feature_enabled":False,
        "scan_count":len(rows),"duration_s":float(times[-1]-times[0]),
        "transient_scan_count":int(transient.any(axis=1).sum()),
        "transient_point_count":int(transient.sum()),"events":events,
        "opponent_presence_correlation":correlation,
        "acceptance":{"required_detection_rate":.8,"required_false_events_per_minute":1.,
            "must_beat_lidar_tracker":True,"measured_detection_rate":None,
            "measured_false_events_per_minute":None,"measured_latency_advantage_s":None,
            "accepted":False},
        "limitations":["Unexplained transients are candidates, not proof of LiDAR interference.",
            "Stationary ego and independent target/event labels required.",
            "No acceptance claim without paired OFF/ON runs and 6.1 tracker timestamps."]}


def labelled_metrics(events, labels):
    """Independent ON/OFF windows and matched 6.1 timestamps, same clock.

    labels=[{start_s,end_s,present,lidar_detected_s?}]. Adjacent anomalous
    frames (<0.3 s) count as one event, avoiding FPS-dependent false alarms.
    """
    starts=[]; previous=None
    for event in sorted(events,key=lambda x:x["t"]):
        t=float(event["t"])
        if previous is None or t-previous>.3:
            starts.append(t)
        previous=t
    positive=[x for x in labels if x.get("present")]
    negative=[x for x in labels if not x.get("present")]
    hits=0; advantages=[]
    for label in positive:
        matches=[t for t in starts if label["start_s"]<=t<label["end_s"]]
        if matches:
            hits+=1
            if label.get("lidar_detected_s") is not None:
                advantages.append(float(label["lidar_detected_s"])-matches[0])
    negative_s=sum(max(0.,x["end_s"]-x["start_s"]) for x in negative)
    false_count=sum(any(x["start_s"]<=t<x["end_s"] for x in negative) for t in starts)
    rate=hits/len(positive) if positive else None
    false_rate=false_count*60./negative_s if negative_s>0 else None
    advantage=float(np.percentile(advantages,5)) if len(advantages)==hits and hits else None
    return {"required_detection_rate":.8,"required_false_events_per_minute":1.,
        "must_beat_lidar_tracker":True,"positive_windows":len(positive),
        "negative_duration_s":negative_s,"matched_detections":hits,
        "measured_detection_rate":rate,"measured_false_events_per_minute":false_rate,
        "measured_latency_advantage_s":advantage,
        "accepted":bool(rate is not None and rate>=.8 and false_rate is not None
                        and false_rate<=1. and advantage is not None and advantage>0.)}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    sub=parser.add_subparsers(dest="command",required=True)
    rec=sub.add_parser("record"); rec.add_argument("--topic",default="/scan")
    rec.add_argument("--seconds",type=float,default=60.); rec.add_argument("--output",required=True)
    rec.add_argument("--opponent-on",action="store_true")
    rec.add_argument("--bearing-deg",type=float); rec.add_argument("--distance-m",type=float)
    ana=sub.add_parser("analyse"); ana.add_argument("input"); ana.add_argument("--baseline",required=True)
    ana.add_argument("--output",required=True); ana.add_argument("--range-tol",type=float,default=.10)
    ana.add_argument("--labels",help="Independent ON/OFF windows and 6.1 detection times in JSON")
    args=parser.parse_args()
    if args.command=="record":
        record(args); return
    read=lambda p:[json.loads(line) for line in Path(p).read_text(encoding="utf-8").splitlines() if line.strip()]
    report=analyse(read(args.input),read(args.baseline),args.range_tol)
    if args.labels:
        report["acceptance"]=labelled_metrics(report["events"],json.loads(Path(args.labels).read_text(encoding="utf-8")))
    Path(args.output).write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding="utf-8")


if __name__=="__main__":
    main()
