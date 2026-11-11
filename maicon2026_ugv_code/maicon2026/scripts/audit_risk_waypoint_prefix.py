#!/usr/bin/env python3
"""Replay a frozen 2D prefix with read-only intermediate navigation evidence."""
import argparse
import gzip
import hashlib
import json
import math
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-root',required=True)
    parser.add_argument('--reference',required=True)
    parser.add_argument('--output',required=True)
    parser.add_argument('--duration',type=float,default=25.)
    args=parser.parse_args()
    source=Path(args.source_root).resolve()
    if not Path(args.reference).is_file():
        parser.error('Reference trace does not exist: '+args.reference)
    sys.path.insert(0,str(source))
    sys.path.insert(0,str(ROOT/'scripts'))
    from compare_sim import build_scenario,clean
    from maicon26.config import Config
    from maicon26.mission.manager import Agent
    from maicon26.nav.navigator import Navigator
    from maicon26.sim.run import run
    cfg=Config();cfg.depth.enable=False;cfg.progress.enabled=True;cfg.risk.threaded=False
    m,_=build_scenario('selfplay',0,cfg)
    names={id(a.stack.navigator):a.name for a in m.agents}
    latest={};rows=[];blocked=[]
    original_carrot=Agent._risk_carrot
    original_step=Navigator.step
    def observed_carrot(a,now,goal,speed):
        result=original_carrot(a,now,goal,speed)
        ctx=a._av_ctx
        if result is not None and ctx is not None:
            cell=ctx.grid.to_cell(*result)
            occupied=not ctx.grid.in_bounds(*cell) or bool(ctx.blocked[cell])
            if occupied:
                blocked.append(dict(t=now,robot=a.name,waypoint=result,mission_goal=goal))
        return result
    def observed_step(nav,pose,scan,now,dt):
        result=original_step(nav,pose,scan,now,dt)
        latest[names[id(nav)]]=dict(status=result.status,raw_vw=[result.v,result.w],
            goal=nav.goal,effective_goal=nav.goal_eff,path=nav.path)
        return result
    Agent._risk_carrot=observed_carrot;Navigator.step=observed_step
    def probe(mm):
        for a in mm.agents:
            pose=a.stack.pose
            rows.append(clean(dict(t=mm.world.t,robot=a.name,pose=[pose.x,pose.y,pose.th],
                vw=[a._v_cmd,a._w_cmd],navigation=latest.get(a.name),risk_route=a._rc,
                mission_goal=a._mission_destination())))
    try:
        run(m,dt=.1,max_t=args.duration,probe=probe)
    finally:
        Agent._risk_carrot=original_carrot;Navigator.step=original_step
        for a in m.agents:a.close_deadlock()
    original={}
    with gzip.open(args.reference,'rt',encoding='utf-8') as stream:
        for line in stream:
            value=json.loads(line)
            if value['t']>args.duration+.01:break
            original[(value['robot'],round(value['t'],4))]=value
    discrepancies=[]
    for row in rows:
        ref=original.get((row['robot'],round(row['t'],4)))
        if ref is None:continue
        error=max(abs(x-y) for x,y in zip(row['pose']+row['vw'],ref['pose']+[ref['v'],ref['w']]))
        if error>1e-9:discrepancies.append(dict(robot=row['robot'],t=row['t'],error=error))
    out=Path(args.output);out.mkdir(parents=True,exist_ok=True)
    with gzip.open(out/'navigation_trace.jsonl.gz','wt',encoding='utf-8') as stream:
        for row in rows:stream.write(json.dumps(row)+'\n')
    record=dict(frozen_source=str(source),duration_s=m.world.t,
        reference_sha256=hashlib.sha256(Path(args.reference).read_bytes()).hexdigest(),
        replay_matches_reference=bool(original) and not discrepancies,
        discrepancy_count=len(discrepancies),first_discrepancies=discrepancies[:5],
        blocked_waypoint_samples=len(blocked),blocked_waypoints=blocked,
        production_modified_by_probe=False)
    (out/'result.json').write_text(json.dumps(record,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in record.items() if k!='blocked_waypoints'},indent=2))
    return 0 if record['replay_matches_reference'] else 1


if __name__=='__main__':raise SystemExit(main())
