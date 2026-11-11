#!/usr/bin/env python3
"""Read-only STAGE stalls: static geometry, observed tracks and evaluator poses."""
import argparse
from collections import Counter,defaultdict
import gzip
import json
import math
from pathlib import Path
import sys
from compare_sim import Monitor


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('case');p.add_argument('--source-root',required=True)
    args=p.parse_args();sys.path.insert(0,str(Path(args.source_root).resolve()))
    from maicon26.config import Config
    from maicon26.sim.world import World
    cfg=Config();cfg.depth.enable=False
    folder=Path(args.case);result=json.loads((folder/'result.json').read_text(encoding='utf-8'))
    world=World(cfg,seed=result['seed'],layout_seed=result['seed'],m5_ends_match=True)
    frames=defaultdict(list)
    for line in gzip.open(folder/'ticks.jsonl.gz','rt',encoding='utf-8'):
        r=json.loads(line);frames[r['t']].append(r)
    monitor=Monitor();rows=[];totals=Counter();per_robot=defaultdict(Counter);first_wait={}
    corner=math.hypot(cfg.robot.length,cfg.robot.width)/2
    exclusion=corner+max(corner,cfg.avoid.r_body)+cfg.engage.min_clearance_m
    for t,frame in frames.items():
        for r in frame:
            uid=r['robot'];dt=t-monitor.states.get(uid,{}).get('last',t)
            monitor.update(r,r['evaluation_only'])
            if r['mode']!='STAGE':continue
            if r['normal_wait']:first_wait.setdefault(uid,t)
            if r['normal_wait'] or monitor.states[uid]['stalled']<3:continue
            goal=r.get('goal')
            if not goal:continue
            static=[]
            for i,cube in enumerate(world.cubes):
                x0,y0,x1,y1=cube.rect()
                distance=math.hypot(max(x0-goal[0],0.,goal[0]-x1),max(y0-goal[1],0.,goal[1]-y1))
                static.append((distance,i))
            static.sort();actual=[]
            for other in frame:
                if other['robot']==uid:continue
                pos=other['evaluation_only']['pose']
                actual.append(dict(robot=other['robot'],team='ally' if other['robot'][0]==uid[0] else 'enemy',
                    to_goal_m=math.dist(pos[:2],goal),to_ego_m=math.dist(pos[:2],r['evaluation_only']['pose'][:2]),
                    normal_wait=other['normal_wait'],mode=other['mode']))
            actual.sort(key=lambda v:v['to_goal_m'])
            observed=[dict(track=tr['id'],team=tr['team'],age=tr.get('age'),
                           to_goal_m=math.dist((tr['x'],tr['y']),goal)) for tr in r.get('tracks',[])]
            observed.sort(key=lambda v:v['to_goal_m'])
            flags=dict(static_point_within_body_inscribed_radius=static[0][0]<min(cfg.robot.length,cfg.robot.width)/2,
                actual_ally_within_required_center_gap=any(v['team']=='ally' and v['to_goal_m']<exclusion for v in actual),
                actual_enemy_within_required_center_gap=any(v['team']=='enemy' and v['to_goal_m']<exclusion for v in actual),
                observed_fresh_ally_within_required_center_gap=any(v['team']=='ally' and v['age'] is not None and v['age']<=1 and v['to_goal_m']<exclusion for v in observed),
                observed_fresh_unknown_within_required_center_gap=any(v['team']=='unknown' and v['age'] is not None and v['age']<=1 and v['to_goal_m']<exclusion for v in observed))
            totals['stage_stall_s']+=dt;per_robot[uid]['stage_stall_s']+=dt
            for key,value in flags.items():
                if value:totals[key]+=dt;per_robot[uid][key]+=dt
            rows.append(dict(t=t,robot=uid,goal=goal,flags=flags,nearest_static=static[0],
                nearest_actual=actual[:3],nearest_observed=observed[:3],goal_distance_m=math.dist(r['pose'][:2],goal)))
    report=dict(source='Offline evaluation; actual poses/static objects never fed to controller',
        required_center_separation_m=exclusion,stage_accept_radius_m=.3,
        flags_overlap=True,totals_s=dict(totals),by_robot_s={k:dict(v) for k,v in per_robot.items()},
        first_stage_normal_wait=first_wait,rows=rows)
    (folder/'stage_occupancy_audit.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps({k:report[k] for k in ['required_center_separation_m','totals_s','by_robot_s','first_stage_normal_wait']},indent=2))


if __name__=='__main__':main()
