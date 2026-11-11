#!/usr/bin/env python3
"""Replay frozen 2D source for read-only per-face M3 failure attribution.

Simulator detection provenance (_gt) is read by this evaluator only. It never
changes observations, adds detector calls/RNG draws, or provides truth to Agent.
"""
import argparse
from collections import Counter,defaultdict
from dataclasses import fields,is_dataclass
import json
import math
from pathlib import Path
import sys
import time

ROOT=Path(__file__).resolve().parents[1]


def record(value):
    """Preserve defaultdict votes without dataclasses.asdict constructor calls."""
    if is_dataclass(value):return {f.name:record(getattr(value,f.name)) for f in fields(value)}
    if isinstance(value,dict):return {str(k):record(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)):return [record(v) for v in value]
    return value


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source-root',required=True)
    p.add_argument('--reference',required=True)
    p.add_argument('--output',required=True)
    p.add_argument('--seed',type=int,default=0)
    args=p.parse_args()
    sys.path.insert(0,str(Path(args.source_root).resolve()))
    from maicon26.config import Config
    from maicon26.sim.run import build_match,run
    from maicon26.sim.world import FACE_N
    from maicon26.link.schema import scoring_sectors,server_sector
    from compare_sim import face_coverage,clean
    cfg=Config();cfg.depth.enable=False
    if hasattr(cfg,'progress'):cfg.progress.enabled=False
    m=build_match(args.seed,cfg,teams=['B1','B2','B3','R1','R2','R3'],m5_ends_match=False,style='sector',layout_seed=args.seed)
    out=Path(args.output);out.mkdir(parents=True,exist_ok=True)
    last={};observations=defaultdict(lambda:defaultdict(list));snapshot={};started=time.perf_counter();ticks=0
    cube_id={(round(c.cx,6),round(c.cy,6)):i for i,c in enumerate(m.world.cubes)}
    def probe(mm):
        nonlocal ticks
        ticks+=1
        if ticks%100==0:
            (out/'progress.json').write_text(json.dumps(dict(t=mm.world.t,wall_s=time.perf_counter()-started)),encoding='utf-8')
        for a in mm.agents:
            stamp=a._det_consumed
            if last.get(a.name)==stamp:continue
            last[a.name]=stamp
            # SimDetector deliberately strips _gt from Agent detections. Read
            # the already-generated, consumed source frame on evaluator side;
            # never call get_frame()/camera_detections() again or draw RNG.
            frame=a.robot._frame
            raw=frame.meta.get('sim_detections',[]) if frame is not None and frame.stamp==stamp else []
            for det in raw:
                gt=det.extra.get('_gt')
                if not isinstance(gt,(tuple,list)) or len(gt)!=3:continue
                ci=cube_id.get((round(gt[0],6),round(gt[1],6)))
                if ci is None:continue
                observations[a.name][(ci,int(gt[2]))].append(dict(t=mm.world.t,cls=det.cls,conf=det.conf))
        if mm.world.t>=210 and not snapshot:
            snapshot['t']=mm.world.t
            snapshot['coverage']=face_coverage(mm)
            snapshot['agents']={a.name:dict(objects=[clean(record(f)) for f in a.omap.faces],
                confirmed=[dict(clean(record(f)),cls=f.cls,mean_conf=f.mean_conf) for f in a.omap.confirmed_faces()],
                boxes=[clean(record(b)) for b in a.coverage.boxes.values()],
                association_stats=dict(getattr(a.omap,'association_stats',{})),
                pose=[a.stack.pose.x,a.stack.pose.y,a.stack.pose.th]) for a in mm.agents}
    run(m,dt=.1,max_t=210.2,probe=probe)
    reference=json.loads(Path(args.reference).read_text(encoding='utf-8'))
    reproducible=snapshot['coverage']==reference['coverage_210']
    rows=[]
    for a in m.agents:
        state=snapshot['agents'][a.name];allowed=scoring_sectors(a.team)
        matched={tuple(v) for v in snapshot['coverage'][a.name]['faces']}
        for ci,c in enumerate(m.world.cubes):
            sector=server_sector(m.world.geo.sector_of(c.cx,c.cy))
            if sector not in allowed:continue
            for fi,item in c.items.items():
                if item.kind!='object' or (ci,fi) in matched:continue
                nx,ny=FACE_N[fi];x,y=c.cx+nx*c.sx/2,c.cy+ny*c.sy/2
                obs=observations[a.name][(ci,fi)]
                correct=[v for v in obs if v['cls']==item.cls]
                near=[]
                for f in state['objects']:
                    distance=math.hypot(f['x']-x,f['y']-y)
                    dot=f['vx']*nx+f['vy']*ny
                    if distance<=.40 and dot>.3:
                        kind=max(f['votes'],key=f['votes'].get) if f['votes'] else None
                        predicted=server_sector(m.world.geo.sector_of(f['cx'],f['cy']))
                        near.append(dict(distance_m=distance,normal_dot=dot,cls=kind,n=f['n'],
                                         predicted_sector=predicted,geometry_key=f['geometry_key']))
                near.sort(key=lambda z:z['distance_m'])
                recs=[]
                for b in state['boxes']:
                    for f in b['faces']:
                        if math.hypot(f['cx']-x,f['cy']-y)<=.22 and f['nx']*nx+f['ny']*ny>.3:
                            recs.append(dict(views=f['views'],det_n=f['det_n'],votes=f['det_votes'],
                                             occluded_views=f['occluded_views'],arrive_attempts=f['arrive_attempts']))
                if not obs:
                    category='viewed_but_no_received_detection' if any(f['views'] for f in recs) else 'no_received_detection_or_verified_view'
                elif not correct:
                    category='only_wrong_class_detections'
                elif any(f['distance_m']<=.22 and f['cls']==item.cls and f['n']<cfg.perception.min_votes for f in near):
                    category='correct_class_insufficient_confirmation_votes'
                elif any(f['distance_m']<=.22 and f['cls']!=item.cls for f in near):
                    category='map_class_or_identity_mismatch'
                else:
                    category='correct_detection_without_matching_confirmed_geometry'
                rows.append(dict(robot=a.name,cube=ci,face=fi,truth_class=item.cls,truth_sector=sector,
                    face_xy=[x,y],category=category,received_detections=len(obs),correct_detections=len(correct),
                    first_received_s=obs[0]['t'] if obs else None,last_received_s=obs[-1]['t'] if obs else None,
                    nearby_object_map=near[:4],estimated_view_records=recs,
                    wrong_sector_candidates=[f for f in near if f['predicted_sector']!=sector]))
    result=dict(reproduced_saved_coverage_exactly=reproducible,reference=args.reference,
        source_root=str(Path(args.source_root).resolve()),time_s=snapshot['t'],
        attribution_valid=reproducible,backend='2D synthetic detector; not real image classifier validation',
        evidence_limit='No received detection is not proof of never visible: view records are estimated geometry and detector has stochastic dropout.',
        counts=dict(Counter(r['category'] for r in rows)),missing_faces=rows,
        coverage=snapshot['coverage'],agent_truth_feedback=False)
    (out/'m3_face_audit.json').write_text(json.dumps(clean(result),ensure_ascii=False,indent=2),encoding='utf-8')
    (out/'snapshot210.json').write_text(json.dumps(clean(snapshot),ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({k:result[k] for k in ('reproduced_saved_coverage_exactly','counts')},indent=2))
    return 0 if reproducible else 1


if __name__=='__main__':raise SystemExit(main())
