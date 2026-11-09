#!/usr/bin/env python3
"""Reproduce a stale waypoint retained inside a newly observed obstacle.

Uses the production Agent routing method and a deterministic candidate path.
No controller/model changes or full mission run; exit1 exposes the defect.
"""
import argparse
import json
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
sys.path.insert(0,str(ROOT/'tests'))
from test_deadlock_integration import agent
from maicon26.nav import risk_astar


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',default='recordings/deadlock_ab_20261109/stale_risk_carrot_repro.json')
    args=parser.parse_args()
    a=agent()
    try:
        grid=a.stack.grid
        grid.L[:]=grid.L_MIN
        a.stack.pose.x,a.stack.pose.y=1.,1.
        canonical=(3.,1.)
        old=(1.8,1.)
        grid.add_static_box((1.75,.95,1.85,1.05))
        blocked,cost=grid.cost_maps(a.cfg.nav.inflate_hard,a.cfg.nav.inflate_soft,a.cfg.nav.soft_weight)
        a._av_ctx=SimpleNamespace(blocked=blocked,cost=cost,grid=grid,slack_s=100.)
        a._av_tracks=[]
        a._field=None
        a._rc={'goal':canonical,'t':0.,'carrot':old}
        path=[(1.,1.),(1.,1.4),(2.,1.4),canonical]
        result=risk_astar.PlanResult(path,2.5,0.,0.,0.)
        with patch.object(risk_astar,'plan',return_value=result):
            selected=a._risk_carrot(10.,canonical,1.)
        ix=grid.to_cell(*selected)
        record=dict(selected=selected,previous_waypoint=old,new_planned_path=path,
                    retained_observed_blocked_waypoint=bool(blocked[ix]),
                    production_modified_by_probe=False,thresholds_modified=False)
        out=Path(args.output);out.parent.mkdir(parents=True,exist_ok=True)
        out.write_text(json.dumps(record,indent=2),encoding='utf-8')
        print(json.dumps(record,indent=2))
        return int(record['retained_observed_blocked_waypoint'])
    finally:
        a.close_deadlock()


if __name__=='__main__':raise SystemExit(main())
