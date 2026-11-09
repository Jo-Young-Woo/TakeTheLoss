#!/usr/bin/env python3
"""Observed LOS-to-field delays in a controlled 2D sensor model (not hardware).

Uses actual BlobDetector/UgvTracker/ThreatBoard/build_field. Camera detections
are projected synthetic observations delayed by an explicitly assumed Jetson
inference delay. UAV uses the simulator's noisy image-detection model. Sensor
phase is randomized. No Agent or mission receives fixture truth.
"""
import argparse
from collections import deque
import json
import math
from pathlib import Path
import sys
import time
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from maicon26.config import Config
from maicon26.sim.world import World
from maicon26.sim.robot import SimRobot, SimServer
from maicon26.nav.grid import OccupancyGrid
from maicon26.perception.lidar_blobs import BlobDetector
from maicon26.perception.tracks import UgvTracker
from maicon26.perception.threat_board import ThreatBoard
from maicon26.perception.detector import SimDetector
from maicon26.mission.threat_field import build_field
from maicon26.types import Frame, Pose2D


def one(source,seed,inference_s=.15):
    cfg=Config();cfg.progress.enabled=True;cfg.risk.threaded=False
    cfg.depth.enable=False;cfg.arena.validate_layout=False
    w=World(cfg,seed=seed,m5_ends_match=False)
    w.cubes=[]
    ego=w.add_ugv('BLUE',2)
    ego.pose=Pose2D(.5,cfg.arena.H/2,0.)
    server=SimServer(w,ego)
    grid=OccupancyGrid(cfg.arena,cfg.nav.res)
    blobs=BlobDetector(cfg);tracker=UgvTracker(cfg,'BLUE');board=ThreatBoard(cfg,'BLUE')
    # Survey genuinely empty space before placing a new stationary object.
    for j in range(20):
        w.t=-3+j*.1
        scan=w.lidar(ego)
        grid.update(scan,ego.pose)
        blobs.update(scan,ego.pose,grid,w.t)
    target=w.add_ugv('RED',2)
    target.pose=Pose2D(2.5,cfg.arena.H/2,math.pi)
    # LOS begins here. Fixture setup/metric positions never enter track inputs.
    w.t=0.
    rng=np.random.default_rng(seed+150)
    next_scan=float(rng.uniform(0,.1));next_camera=float(rng.uniform(0,.1))
    period=float(source.split('_')[-1]) if source.startswith('uav_') else 1.
    w.uav_period=period;w.uav_latency=.4
    server._last_uav_stamp=float(rng.uniform(0,period))-period
    server._hist.clear();server._last_uav_delivered=-1e9
    next_field=float(rng.uniform(0,.2))
    camera_queue=deque();latest_blobs=[];last_scan=None
    raw_received=None;scan_cpu_ms=[]
    for step in range(int((period+2.)/.01)):
        w.t=step*.01
        urgent=False
        cpu_start=None
        if w.t+1e-8>=next_scan:
            scan=w.lidar(ego);last_scan=scan;next_scan+=.1
            if source=='lidar':
                raw_received=w.t
                cpu_start=time.perf_counter()
                latest_blobs=blobs.update(scan,ego.pose,grid,w.t)
                urgent=True
        if source=='camera' and w.t+1e-8>=next_camera:
            next_camera+=.1
            frame=Frame(None,w.t,ego.pan,ego.tilt,cfg.camera.width,cfg.camera.height,
                        {'sim_detections':w.camera_detections(ego)})
            camera_queue.append((w.t+inference_s,frame,last_scan))
        while camera_queue and camera_queue[0][0]<=w.t+1e-8:
            _,frame,capture_scan=camera_queue.popleft()
            tracker.update_camera(SimDetector().detect(frame),ego.pose,frame,capture_scan)
        if source.startswith('uav_'):
            frame=server.get_uav()
            if frame:
                tracker.update_uav(frame,ego.pose,self_ident=ego.id)
        tracks=board.update(tracker,latest_blobs,w.t,ego.pose,use_risk_weights=True)
        if urgent or w.t+1e-8>=next_field:
            field=build_field(tracks,grid,cfg.avoid,cfg.laser.max_range,w.t,cfg.nav.inflate_hard,
                              risk_cfg=cfg.risk,min_clearance_m=cfg.engage.min_clearance_m)
            next_field=w.t+.2
            cpu_ms=(time.perf_counter()-cpu_start)*1000 if cpu_start is not None else None
            if cpu_ms is not None:scan_cpu_ms.append(cpu_ms)
            if field and any(math.hypot(t.x-target.pose.x,t.y-target.pose.y)<.4 for t in field.tracks):
                return dict(source=source,seed=seed,detected=True,los_to_field_s=w.t,
                            dwell_margin_s=2.-w.t,scan_received_to_field_cpu_ms=cpu_ms,
                            field_time=w.t,raw_scan_received=raw_received)
    return dict(source=source,seed=seed,detected=False,los_to_field_s=None,
                censored_at_s=w.t,scan_cpu_ms=scan_cpu_ms)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--seeds',type=int,default=20);p.add_argument('--inference-s',type=float,default=.15)
    p.add_argument('--output',default='recordings/deadlock_ab_20261109/threat_latency.json')
    args=p.parse_args()
    rows=[one(source,seed,args.inference_s) for source in ('lidar','camera','uav_1','uav_2','uav_5') for seed in range(args.seeds)]
    result=dict(backend='controlled 2D synthetic sensor bench',
                jetson_inference_s_assumption=args.inference_s,hardware_validated=False,
                known_pose='fixed sensor bench fixture, no Agent receives truth',
                field_schedule='5 Hz; incoming LiDAR requests immediate publication',rows=rows,summary={})
    for source in ('lidar','camera','uav_1','uav_2','uav_5'):
        subset=[r for r in rows if r['source']==source]
        values=[r['los_to_field_s'] for r in subset if r['detected']]
        cpu=[r['scan_received_to_field_cpu_ms'] for r in subset if r.get('scan_received_to_field_cpu_ms') is not None]
        result['summary'][source]=dict(detected=len(values),trials=len(subset),
              p50_s=float(np.percentile(values,50)) if values else None,
              p95_s=float(np.percentile(values,95)) if values else None,
              cpu_p95_ms=float(np.percentile(cpu,95)) if cpu else None)
    out=Path(args.output);out.parent.mkdir(parents=True,exist_ok=True)
    out.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(result['summary'],indent=2))


if __name__=='__main__':main()
