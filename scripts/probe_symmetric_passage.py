#!/usr/bin/env python3
"""Two identical sensor-driven Agents approach opposite ends of one corridor.

Mission destination is a known opposite start base. Navigation, perception,
posture/progress and final safety commands are the production Agent methods.
Truth is used only by the video/evaluator, never for driving.
"""
import argparse
import gzip
import hashlib
import json
import math
from pathlib import Path
import sys
import time
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'scripts'))
from compare_sim import Renderer,telemetry,clean,Monitor
from maicon26.config import Config
from maicon26.mission.manager import Agent,Mode
from maicon26.sim.run import Match,run
from maicon26.sim.world import World,Cube
from maicon26.sim.robot import SimRobot,SimServer


def debug_snapshot(a,u,t,out):
    import cv2
    import numpy as np
    nav=a.stack.navigator;grid=a.stack.grid
    blocked,cost=grid.cost_maps(a.cfg.nav.inflate_hard,a.cfg.nav.inflate_soft,a.cfg.nav.soft_weight)
    name=f'{u.id}_{t:g}s'
    arrays=dict(L=grid.L.copy(),static=grid.static.copy(),blocked=blocked,cost=cost)
    if a._field is not None:
        arrays.update(risk=a._field.risk.copy(),body=a._field.body.copy(),close=a._field.close.copy())
    if a._av_maps[1] is not None:
        arrays.update(av_blocked=a._av_maps[1],av_cost=a._av_maps[2])
    np.savez_compressed(out/(name+'.npz'),**arrays)
    meta=dict(t=t,estimated=[a.stack.pose.x,a.stack.pose.y,a.stack.pose.th],
              evaluation_actual=[u.pose.x,u.pose.y,u.pose.th],
              destination=a.destination,risk_cache=a._rc,nav_path=nav.path,
              nav_goal=nav.goal,nav_goal_eff=nav.goal_eff,cur_goal=a._cur_goal,
              nav_plan_fail=nav._plan_fail,nav_last_plan=nav._last_plan,
              nav_recover_until=nav._recover_until,nav_extra_block=nav._extra_block,
              nav_turn_pref=nav.turn_pref,grid=dict(ox=grid.ox,oy=grid.oy,res=grid.res),
              cfg=dict(hard=a.cfg.nav.inflate_hard,soft=a.cfg.nav.inflate_soft,
                       stop=a.cfg.nav.stop_dist,lookahead=a.cfg.nav.lookahead),
              decision=a.deadlock_telemetry())
    (out/(name+'.json')).write_text(json.dumps(clean(meta),indent=2),encoding='utf-8')
    canvas=np.full((*grid.L.shape,3),150,np.uint8)
    canvas[grid.L<-.4]=(230,230,230)
    canvas[blocked]=(40,40,40)
    def cell(p):
        iy,ix=grid.to_cell(*p);return (ix,iy)
    for path,color in ((a._rc.get('path',[]),(0,190,220)),(nav.path,(0,200,0))):
        pts=np.array([cell(p) for p in path],np.int32)
        if len(pts)>1:cv2.polylines(canvas,[pts],False,color,1)
    cv2.circle(canvas,cell((a.stack.pose.x,a.stack.pose.y)),2,(255,50,50),-1)
    cv2.circle(canvas,cell(a.destination),2,(255,0,255),1)
    if nav.goal:cv2.circle(canvas,cell(nav.goal),2,(0,0,255),1)
    for tr in a._av_tracks:
        cv2.circle(canvas,cell((tr.x,tr.y)),2,(0,140,255),1)
    canvas=cv2.flip(cv2.resize(canvas,None,fx=6,fy=6,interpolation=cv2.INTER_NEAREST),0)
    cv2.putText(canvas,f'{name}: cyan risk, green local, red local goal',(10,20),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,0,0),1)
    cv2.imwrite(str(out/(name+'.png')),canvas)


class TransitAgent(Agent):
    def __init__(self,*args,destination,**kwargs):
        self.destination=destination
        super().__init__(*args,**kwargs)
    def _arbitrate(self,*args):return Mode.EXPLORE
    def _mission_destination(self):return self.destination
    def _mode_explore(self,now,dt,t):
        self._nav_to(now,dt,self.destination,.15)


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--output',default='recordings/deadlock_ab_20261109/symmetric_passage')
    ap.add_argument('--width',type=float,default=1.4)
    ap.add_argument('--duration',type=float,default=30.)
    args=ap.parse_args()
    cfg=Config();cfg.progress.enabled=True;cfg.risk.threaded=False;cfg.depth.enable=False
    cfg.arena.validate_layout=False
    w=World(cfg,seed=7,m5_ends_match=False)
    h=cfg.arena.H;top=h/2+args.width/2;bottom=h/2-args.width/2
    w.cubes=[Cube(cfg.arena.W/2,top+(h-top)/2,{},sx=cfg.arena.W-1.6,sy=h-top),
             Cube(cfg.arena.W/2,bottom/2,{},sx=cfg.arena.W-1.6,sy=bottom)]
    agents=[];servers=[];robots=[]
    for team,slot in [('BLUE',1),('RED',3)]:
        # Different slot seeds but symmetric physical starts/known base guesses.
        u=w.add_ugv(team,2)
        r=SimRobot(w,u);sv=SimServer(w,u,sector_style='sector')
        target=cfg.arena.base_center('R-2' if team=='BLUE' else 'B-2')
        a=TransitAgent(r,sv,cfg=cfg,seed=slot,name=f'{team[0]}-UGV-{slot}',destination=target)
        # Evaluator uses stable actual UGV id; seed symmetry stays slot-specific.
        a.name=u.id
        agents.append(a);servers.append(sv);robots.append(r)
    m=Match(w,agents,servers,robots)
    out=Path(args.output);out.mkdir(parents=True,exist_ok=True)
    fingerprints={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest()
                  for p in sorted((ROOT/'maicon26').rglob('*.py'))}
    fingerprints['scripts/probe_symmetric_passage.py']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    (out/'source_hashes.json').write_text(json.dumps(fingerprints,indent=2),encoding='utf-8')
    (out/'configuration.json').write_text(json.dumps(cfg.to_dict(),indent=2),encoding='utf-8')
    video=Renderer(out/'symmetric_passage.mp4',w,5)
    trace=gzip.open(out/'ticks.jsonl.gz','wt',encoding='utf-8')
    nextvideo=0.;arrived={};minimum=99.;saved=set()
    def probe(mm):
        nonlocal nextvideo,minimum
        rows=[telemetry(a,w.t) for a in agents]
        for a,u,row in zip(agents,w.ugvs,rows):
            distance=math.hypot(u.pose.x-a.destination[0],u.pose.y-a.destination[1])
            if distance<.3 and u.id not in arrived:arrived[u.id]=w.t
            row['evaluation_only']={'pose':[u.pose.x,u.pose.y,u.pose.th],'distance':distance}
            row['route_debug']={'risk_path':a._rc.get('path',[]),'risk_carrot':a._rc.get('carrot'),
                                'nav_path':a.stack.navigator.path,'nav_goal':a.stack.navigator.goal,
                                'nav_goal_eff':a.stack.navigator.goal_eff,'cur_goal':a._cur_goal}
            trace.write(json.dumps(clean(row))+'\n')
            for stamp in (1.5,2.,3.,5.,10.,15.,20.,25.):
                if w.t>=stamp and (u.id,stamp) not in saved:
                    debug_snapshot(a,u,stamp,out);saved.add((u.id,stamp))
        minimum=min(minimum,math.dist((w.ugvs[0].pose.x,w.ugvs[0].pose.y),(w.ugvs[1].pose.x,w.ugvs[1].pose.y)))
        if w.t>=nextvideo:
            video.draw(m,rows,'symmetric physical passage');nextvideo+=.2
    started=time.perf_counter()
    try:
        run(m,dt=.05,max_t=args.duration,probe=probe)
    finally:
        trace.close();videometa=video.close()
        for a in agents:a.close_deadlock()
    report=dict(duration_s=w.t,pass_under_30_s=len(arrived)==2 and max(arrived.values())<=30.,
                arrivals=arrived,min_center_separation_m=minimum,wall_s=time.perf_counter()-started,
                corridor_width_m=args.width,video=videometa,
                deterministic_seeds={'B2':{'nav':1,'progress_slot':'B-UGV-1'},
                                     'R2':{'nav':3,'progress_slot':'R-UGV-3'}},
                algorithm='same production Agent methods; destination-only test mission; sensor-only control',
                final=[dict(id=u.id,pose=[u.pose.x,u.pose.y,u.pose.th],collisions=u.collisions) for u in w.ugvs])
    (out/'result.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report,indent=2))
    return 0 if report['pass_under_30_s'] else 1


if __name__=='__main__':raise SystemExit(main())
