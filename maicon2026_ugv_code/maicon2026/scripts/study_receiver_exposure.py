#!/usr/bin/env python3
"""Controlled referee ray study; not a dwell/game/hardware hit rate."""
import os
os.environ.setdefault("OPENBLAS_NUM_THREADS","1")
import argparse
from functools import lru_cache
import json
import math
from pathlib import Path
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/"ros2/maicon_arena_gz/scripts"),str(ROOT/"ros2/wingbot_description/scripts")]
import numpy as np
import yaml
from arena_referee import ArenaReferee
from wingbot_runtime import WingbotFrames


def evaluate(pan_samples=41,tilt_samples=31):
    load=lambda p:yaml.safe_load(p.read_text(encoding="utf-8"))
    arena=ROOT/"ros2/maicon_arena_gz"
    frames=WingbotFrames(ROOT/"ros2/wingbot_description/config/frames.yaml")
    # Cache exactly identical generated geometry. Referee intersections,
    # self-occlusion and dome acceptance remain the production implementation.
    frames.laser=lru_cache(maxsize=8192)(frames.laser)
    original_receivers=frames.receivers
    receiver_geometry=tuple(original_receivers())
    frames.receivers=lambda: receiver_geometry
    original_collisions=frames.collision_transforms
    frames.collision_transforms=lru_cache(maxsize=16)(lambda pan=0.,tilt=0.: tuple(original_collisions(pan,tilt)))
    referee=ArenaReferee(load(arena/"config/arena_spec.yaml"),{"boxes":[]},
        load(arena/"config/simulation_assumptions.yaml"),frames=frames)
    referee.static_bounds=[]  # Explicitly controlled open-space experiment.
    referee.targets=[]
    headings={"front_toward_shooter":math.pi,"rear_toward_shooter":0.,
              "left_dome_toward_shooter":math.pi/2,"right_dome_toward_shooter":-math.pi/2}
    rows=[]
    started=time.perf_counter()
    for distance in (.6,1.,1.5):
        half_pan=math.degrees(math.atan2(.18,distance))+1.
        centre_tilt=math.degrees(math.atan2(.122-.250,max(.1,distance-.048)))
        half_tilt=math.degrees(math.atan2(.05,distance))+1.
        pans=np.linspace(-half_pan,half_pan,pan_samples)
        tilts=np.linspace(max(-15.,centre_tilt-half_tilt),min(15.,centre_tilt+half_tilt),tilt_samples)
        for label,heading in headings.items():
            referee.poses={"B-2":(0.,0.,0.,0.),"R-2":(distance,0.,0.,heading)}
            referee.gimbals={"R-2":(0.,0.)}
            count=0; pixels=[]
            for pi,pan in enumerate(pans):
                for ti,tilt in enumerate(tilts):
                    referee.gimbals["B-2"]=(math.radians(float(pan)),math.radians(float(tilt)))
                    if referee._first_receiver("B-2")==("ugv","R-2"):
                        count+=1; pixels.append([pi,ti])
            rows.append(dict(range_m=distance,receiver_heading=label,rays=len(pans)*len(tilts),
                receiver_intersections=count,ray_fraction=count/(len(pans)*len(tilts)),
                pan_bounds_deg=[float(pans[0]),float(pans[-1])],
                tilt_bounds_deg=[float(tilts[0]),float(tilts[-1])],hit_grid_indices=pixels))
    return dict(study="Production ArenaReferee._first_receiver geometry sweep",
        source_frames_sha256=frames.data["source_sha256"],wall_seconds=time.perf_counter()-started,
        conditions=dict(static_obstacles=False,model_body_occlusion=True,
            receiver_hemispheres=True,physical_calibration_verified=False,continuous_dwell_simulated=False),
        rows=rows,orientation_cost_promoted=False,
        interpretation="Ray fractions describe this bounded angular sweep only, not probability of a valid 2-second hit. Do not change the mission cost without matched gameplay A/B and hardware confirmation.")


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output",type=Path,default=ROOT/"recordings/deadlock_fast_threat_20261109/receiver_exposure.json")
    args=parser.parse_args(); result=evaluate()
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2),encoding="utf-8")
    print(json.dumps({**{k:v for k,v in result.items() if k!="rows"},
        "rows":[{k:v for k,v in row.items() if k!="hit_grid_indices"} for row in result["rows"]]},indent=2))


if __name__=="__main__":
    main()
