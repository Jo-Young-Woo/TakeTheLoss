#!/usr/bin/env python3
"""CPU scan-to-risk benchmark with synthetic sensor input, not a hardware claim."""
import argparse
import json
import math
from pathlib import Path
import platform
import sys
import time
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
import numpy as np
from maicon26.config import Config
from maicon26.nav.grid import OccupancyGrid
from maicon26.perception.lidar_blobs import BlobDetector
from maicon26.perception.threat_board import ThreatBoard
from maicon26.mission.risk_worker import RiskFieldWorker
from maicon26.types import Pose2D, Scan


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--samples",type=int,default=100)
    parser.add_argument("--output",type=Path,default=ROOT/"recordings/deadlock_fast_threat_20261109/latency.json")
    args=parser.parse_args()
    cfg=Config(); cfg.progress.enabled=True
    grid=OccupancyGrid(cfg.arena); grid.L[:]=grid.L_MIN
    pose=Pose2D(1.,2.,0.)
    detector=BlobDetector(cfg); board=ThreatBoard(cfg,"BLUE"); worker=RiskFieldWorker(cfg)
    angles=np.linspace(-math.pi,math.pi,640,endpoint=False)
    def scan(stamp,bodies=()):
        ranges=np.full(640,8.)
        for x,y in bodies:
            px,py=x-pose.x,y-pose.y
            b=px*np.cos(angles)+py*np.sin(angles)
            disc=b*b-(px*px+py*py-.13**2); hit=(disc>=0)&(b>0)
            ranges[hit]=np.minimum(ranges[hit],b[hit]-np.sqrt(disc[hit]))
        return Scan(-math.pi,2*math.pi/640,ranges,stamp,8.)
    detector.update(scan(0.),pose,grid,0.)
    samples=[]; tracks=[]
    try:
        for i in range(args.samples):
            started=time.perf_counter(); now=(i+1)*.1
            data=scan(now,[(2.,1.),(2.2,2.),(2.6,3.),(1.5,3.6),(3.8,1.1)])
            blobs=detector.update(data,pose,grid,now)
            tracks=board.update(None,blobs,now,pose,use_risk_weights=True)
            worker.submit(tracks,grid,now,i,urgent=True)
            while worker.latest()[0]!=i and time.perf_counter()-started<2.:
                time.sleep(.0005)
            if worker.latest()[0]!=i:
                raise RuntimeError("Risk worker publication timed out")
            samples.append((time.perf_counter()-started)*1000.)
    finally:
        worker.close()
    def stats(values):
        return dict(p50_ms=float(np.percentile(values,50)),p95_ms=float(np.percentile(values,95)),max_ms=float(max(values)))
    result=dict(source="Synthetic sensor CPU benchmark, not Gazebo LOS latency or Jetson hardware",
        samples=args.samples,scan_points=640,tracks=len(tracks),platform=platform.platform(),
        lidar_compute=stats(detector.latency_ms),risk_worker_receive_to_publish=stats(worker.latency_ms),
        scan_processing_through_risk_publication=stats(samples),
        budget_100ms_pass=bool(np.percentile(samples,95)<=100),errors=list(worker.errors))
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2),encoding="utf-8")
    print(json.dumps(result,indent=2))


if __name__=="__main__":
    main()
