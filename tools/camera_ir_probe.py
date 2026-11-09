#!/usr/bin/env python3
"""Record raw camera brightness and analyse transient flashes on real hardware.

Max brightness alone saturates on ordinary white objects. Both a bright pixel
and a positive temporal change are required; this is still not IR identification.
"""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import time
import numpy as np


def brightness(frame, previous=None):
    gray=np.max(frame,axis=2) if frame.ndim==3 else frame
    result={"max_brightness":int(gray.max()),"p999_brightness":float(np.percentile(gray,99.9)),
            "flash_pixels":0,"max_rise":0}
    if previous is not None and previous.shape==gray.shape:
        rise=gray.astype(np.int16)-previous.astype(np.int16)
        result["flash_pixels"]=int(np.count_nonzero((gray>=240)&(rise>=50)))
        result["max_rise"]=int(max(0,rise.max()))
    return result,gray.copy()


def record(args):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    rclpy.init(); node=Node("maicon_camera_ir_probe")
    started=time.monotonic(); previous=None
    path=Path(args.output); path.parent.mkdir(parents=True,exist_ok=True)
    with path.open("w",encoding="utf-8") as out:
        def on_image(msg):
            nonlocal previous
            encoding=msg.encoding.lower()
            channels={"rgb8":3,"bgr8":3,"rgba8":4,"bgra8":4,"mono8":1}.get(encoding)
            if channels is None:
                node.get_logger().warning("Unsupported image encoding "+encoding); return
            raw=np.frombuffer(bytes(msg.data),dtype=np.uint8).reshape(msg.height,msg.step)
            frame=raw[:,:msg.width*channels].reshape(msg.height,msg.width,channels)
            if channels==4: frame=frame[:,:,:3]
            row,previous=brightness(frame,previous)
            row.update(t=time.monotonic()-started,stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
                distance_m=args.distance_m,opponent_present=args.opponent_on,hardware_only=True)
            out.write(json.dumps(row)+"\n")
        node.create_subscription(Image,args.topic,on_image,qos_profile_sensor_data)
        try:
            while rclpy.ok() and time.monotonic()-started<args.seconds:
                rclpy.spin_once(node,timeout_sec=.1)
        finally:
            node.destroy_node(); rclpy.shutdown()


def analyse(rows,min_pixels=1):
    events=[{"t":float(r["t"]),"flash_pixels":r["flash_pixels"],"max_rise":r["max_rise"]}
            for r in rows if r.get("flash_pixels",0)>=min_pixels]
    return {"status":"unverified_hardware_hypothesis","feature_enabled":False,
        "frame_count":len(rows),"events":events,
        "max_brightness":max((r["max_brightness"] for r in rows),default=None),
        "acceptance":None,
        "limitations":["Brightness transients cannot establish 905 nm response or opponent aiming.",
                        "Need stationary scene, OFF/ON controls, independent labels and 6.1 timestamps."]}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    sub=parser.add_subparsers(dest="command",required=True)
    rec=sub.add_parser("record"); rec.add_argument("--topic",default="/image_raw")
    rec.add_argument("--seconds",type=float,default=60.); rec.add_argument("--output",required=True)
    rec.add_argument("--distance-m",type=float,required=True)
    rec.add_argument("--opponent-on",action="store_true")
    ana=sub.add_parser("analyse"); ana.add_argument("input"); ana.add_argument("--output",required=True)
    ana.add_argument("--min-pixels",type=int,default=1); ana.add_argument("--labels")
    args=parser.parse_args()
    if args.command=="record":
        if not .5<=args.distance_m<=2.: parser.error("Use measured distance between 0.5 and 2 m")
        record(args); return
    rows=[json.loads(line) for line in Path(args.input).read_text(encoding="utf-8").splitlines() if line.strip()]
    report=analyse(rows,args.min_pixels)
    if args.labels:
        from lidar_crosstalk_probe import labelled_metrics
        report["acceptance"]=labelled_metrics(report["events"],json.loads(Path(args.labels).read_text(encoding="utf-8")))
    Path(args.output).write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding="utf-8")


if __name__=="__main__":
    main()
