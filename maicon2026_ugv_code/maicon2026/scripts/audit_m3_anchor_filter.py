#!/usr/bin/env python3
"""Post-hoc unanchored-fallback filter proposal using saved210s map evidence.

No policy edits or new simulation outcomes. Uses frozen source's actual ledger
counter, not a simplified count of ObjectMap records.
"""
import argparse
import copy
import json
import math
from pathlib import Path
import sys


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--source-root',required=True)
    p.add_argument('audit_folder');args=p.parse_args()
    sys.path.insert(0,str(Path(args.source_root).resolve()))
    from maicon26.config import Config
    from maicon26.sim.world import World,FACE_N
    from maicon26.sim.run import score_m3
    from maicon26.perception.object_map import ObjectMap,Face
    from maicon26.perception.coverage import CoverageMap,BoxModel,FaceRec
    from maicon26.link.schema import scoring_sectors,server_sector,OBJECT_CODES
    cfg=Config();cfg.depth.enable=False
    w=World(cfg,seed=0,m5_ends_match=False,layout_seed=0)
    out=Path(args.audit_folder);snapshot=json.loads((out/'snapshot210.json').read_text(encoding='utf-8'))
    result=[]
    for uid,state in snapshot['agents'].items():
        team='BLUE' if uid.startswith('B') else 'RED';allowed=scoring_sectors(team)
        omap=ObjectMap(cfg)
        for data in state['objects']:
            data=dict(data)
            if data.get('geometry_key') is not None:data['geometry_key']=tuple(data['geometry_key'])
            omap.faces.append(Face(**data))
        coverage=CoverageMap(cfg)
        for data in state['boxes']:
            data=dict(data);data['faces']=[FaceRec(**f) for f in data['faces']]
            box=BoxModel(**data);coverage.boxes[box.bid]=box
        def payload(source):
            counts=coverage.m3_counts(source)
            # Same explicit four-sector replacement semantics as Agent.
            complete={s:{kind:0 for kind in OBJECT_CODES} for s in allowed}
            for sector,classes in counts.items():
                sector=server_sector(sector)
                if sector in complete:
                    for kind,n in classes.items():
                        if kind in OBJECT_CODES:complete[sector][kind]+=n
            return {'sectors':complete}
        original=payload(omap)
        filtered=copy.deepcopy(omap)
        filtered.faces=[f for f in filtered.faces if f.geometry_key is not None]
        proposed=payload(filtered)
        before=score_m3(w.objects_truth(),original,team);after=score_m3(w.objects_truth(),proposed,team)
        valid_unanchored=[]
        for f in omap.confirmed_faces():
            if f.kind!='object' or f.geometry_key is not None:continue
            if server_sector(w.geo.sector_of(f.cx,f.cy)) not in allowed:continue
            for ci,c in enumerate(w.cubes):
                if server_sector(w.geo.sector_of(c.cx,c.cy)) not in allowed:continue
                for fi,item in c.items.items():
                    nx,ny=FACE_N[fi];x,y=c.cx+nx*c.sx/2,c.cy+ny*c.sy/2
                    if item.kind=='object' and f.cls==item.cls and math.hypot(f.x-x,f.y-y)<=.22 and f.vx*nx+f.vy*ny>.3:
                        valid_unanchored.append(dict(cube=ci,face=fi,cls=f.cls,votes=f.n,xy=[f.x,f.y]))
        def metrics(score):
            return dict(tp=score['n_correct'],fp=score['n_sub']-score['n_correct'],fn=score['n_truth']-score['n_correct'],
                        points=score['points'],sectors=score['sector_results'])
        result.append(dict(robot=uid,before=metrics(before),after=metrics(after),
            unanchored_valid_records=len(valid_unanchored),valid_unanchored=valid_unanchored,
            original_payload=original,proposed_payload=proposed))
    report=dict(method='offline what-if only; remove ObjectMap unanchored fallback, retain observed ledger faces',
        time_s=snapshot['t'],source_root=args.source_root,policy_modified=False,new_run=False,
        caution='TP/FP/FN use official per-sector class counts; individual valid unanchored evidence is listed separately.',rows=result)
    (out/'anchor_filter_whatif210.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    for r in result:print(json.dumps({k:r[k] for k in ['robot','before','after','unanchored_valid_records']}))


if __name__=='__main__':main()
