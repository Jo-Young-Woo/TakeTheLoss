#!/usr/bin/env python3
"""Seeded six-robot Agent regression matrix; evaluator truth never enters Agent.

The 2D backend supplies noisy simulated LiDAR/odom and synthetic camera/UAV
detections. Its videos are diagnostic 2D renderings, never Gazebo recordings.
The baseline can import an immutable source snapshot with --source-root.
"""
from __future__ import annotations

import argparse
from collections import Counter, deque
import concurrent.futures
import gzip
import hashlib
import html
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
SCENARIOS = ('selfplay', 'vs_evaders', 'vs_hunters', 'vs_blockers', 'mixed_allies', 'goal_race')
VARIANTS = ('v2', 'progress', 'blocker_fire', 'dwell_zero')
METRIC_VERSION = 7


def source_fingerprints(source_root):
    package=Path(source_root)/'maicon26'
    if not package.is_dir():raise ValueError(f'Missing source package: {package}')
    return {str(p.relative_to(package)).replace('\\','/'):hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(package.rglob('*.py'))}


def read_case_result(folder):
    """Only scoring-face progress counters can enter strict acceptance."""
    folder=Path(folder);path=folder/'result.json'
    if not path.exists():return None,'no completed result'
    result=json.loads(path.read_text(encoding='utf-8'))
    if result.get('metric_version')==METRIC_VERSION:return result,None
    if result.get('metric_version') in (5,6):
        return None,'historical partial scope: progress counted unscored or unavailable structure faces'
    return None,'superseded metric version'


def read_stage_corrected_result(folder):
    """Read preserved metric6 diagnostics, never strict matrix acceptance."""
    folder=Path(folder);path=folder/'result.json';derived=folder/'analysis_v6.json'
    if not path.exists():return None,'no completed result'
    result=json.loads(path.read_text(encoding='utf-8'))
    if result.get('metric_version')==5 and derived.exists():
        analysis=json.loads(derived.read_text(encoding='utf-8'))
        if analysis.get('analysis_version')!=6 or analysis.get('original_metric_version')!=5:
            return None,'derived analysis version mismatch'
        if not analysis.get('stage_wait_correction_verified'):return None,'derived analysis has unresolved wait causes'
        if analysis.get('original_result_sha256')!=hashlib.sha256(path.read_bytes()).hexdigest():
            return None,'derived analysis result identity mismatch'
        ticks=folder/'ticks.jsonl.gz'
        if not ticks.exists() or analysis.get('original_ticks_sha256')!=hashlib.sha256(ticks.read_bytes()).hexdigest():
            return None,'derived analysis trace identity mismatch'
        configuration=folder/'configuration.json'
        if not configuration.exists() or analysis.get('original_configuration_sha256')!=hashlib.sha256(configuration.read_bytes()).hexdigest():
            return None,'derived analysis configuration identity mismatch'
        result=dict(result,metric_version=6,diagnostics=analysis['diagnostics'],
                    analysis_origin='analysis_v6.json derived from preserved complete metric5 trace',
                    original_metric_version=5,strict_acceptance_eligible=False,
                    limitation='Progress face counter includes unscored or unavailable structure faces')
        return result,None
    return None,'no verified historical stage correction'


def reusable_case(folder,expected_hashes,termination_policy):
    """A completed process is reusable only for this exact policy source."""
    folder=Path(folder)
    result,reason=read_case_result(folder)
    if result is None:return False,reason
    if result.get('termination_policy')!=termination_policy:return False,'different termination policy'
    hashes=folder/'source_hashes.json'
    if not hashes.exists():return False,'missing source identity'
    old=json.loads(hashes.read_text(encoding='utf-8'))
    if old!=expected_hashes:return False,'superseded source revision'
    return True,'same metric, termination policy and source'


def clean(value):
    if isinstance(value, dict):
        return {str(k): clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean(v) for v in value]
    if hasattr(value, 'item'):
        return clean(value.item())
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if hasattr(value, 'value'):
        return value.value
    return value


def norm_id(uid):
    return uid.replace('-UGV-', '')


def build_scenario(name, seed, cfg, *, continue_after_capture=False):
    from maicon26.sim.run import build_match
    from maicon26.sim.hostile import Hostile, Chaser, Patroller, Blocker
    from maicon26.types import Pose2D
    from maicon26.util import clamp, wrap

    class Evader(Hostile):
        kind = 'pure_evader'
        def step(self, match):
            if not self.active(match):
                return
            others = [o for o in self.world.ugvs if o.team != self.u.team]
            if not others:
                return
            p = self.u.pose
            target = min(others, key=lambda o: math.hypot(o.pose.x-p.x, o.pose.y-p.y))
            d = math.hypot(target.pose.x-p.x, target.pose.y-p.y)
            if d < 2.0:
                yaw = math.atan2(p.y-target.pose.y, p.x-target.pose.x)
                self._drive_to(p.x+math.cos(yaw), p.y+math.sin(yaw), self.v_max)
            else:
                self.u.cmd_v = self.u.cmd_w = 0.0

    teams = ['B1', 'B2', 'B3', 'R1', 'R2', 'R3'] if name in ('selfplay', 'goal_race') else (
        ['B2'] if name == 'mixed_allies' else ['B1', 'B2', 'B3'])
    m = build_match(seed, cfg, teams=teams, m5_ends_match=not continue_after_capture, style='sector', layout_seed=seed)
    w = m.world
    dummies = []
    if name == 'mixed_allies':
        x, y = w.geo.base_center('B-1')
        dummies.append(Blocker(w, 'BLUE', 1, Pose2D(x, y, 0), 'R-UGV-1', shoot=False))
        x, y = w.geo.base_center('B-3')
        dummies.append(Patroller(w, 'BLUE', 3, Pose2D(x, y, 0), 'R-UGV-3',
                                points=[(x+.45, y), (x+.45, y+.45)], aim=False, shoot=False))
    if name not in ('selfplay', 'goal_race'):
        for slot in (1, 2, 3):
            x, y = w.geo.base_center(f'R-{slot}')
            target = f'B-UGV-{slot}' if name != 'mixed_allies' else 'B-UGV-2'
            typ = {'vs_evaders': Evader, 'vs_hunters': Chaser, 'vs_blockers': Blocker}.get(name,
                      (Chaser, Blocker, Evader)[slot-1])
            if typ is Blocker:
                # Fixed CM entrances: no teleport into the tested Agent's route.
                x = w.geo.W/2 + (.40 if slot == 2 else -.40)
                y = w.geo.H * (.18, .50, .82)[slot-1]
                # Keep dummy starts outside physical boxes, chosen by evaluator only.
                for dx, dy in ((0, 0), (.5, 0), (-.5, 0), (0, .5), (0, -.5)):
                    if all(not (x0-.18 < x+dx < x1+.18 and y0-.18 < y+dy < y1+.18)
                           for x0, y0, x1, y1 in w.obstacles()):
                        x, y = x+dx, y+dy
                        break
            dummies.append(typ(w, 'RED', slot, Pose2D(x, y, math.pi), target,
                                shoot=typ is not Evader, v_max=.35))
    return m, dummies


def telemetry(a, now):
    if hasattr(a, 'deadlock_telemetry') and getattr(getattr(a.cfg, 'progress', None), 'enabled', False):
        out = clean(a.deadlock_telemetry())
    elif hasattr(a, 'deadlock_diagnostics'):
        out = clean(a.deadlock_diagnostics(now))
    else:
        pose = a.stack.pose
        mode=a.mode.value
        candidate=getattr(a,'_ex_cand',None)
        goal = (a._goal_xy() if mode=='GOAL' else a._stage_point() if mode=='STAGE' else
                candidate.vp if candidate is not None and mode=='EXPLORE' else a._cur_goal)
        nav = getattr(a.stack, 'navigator', getattr(a.stack, 'nav', None))
        path = a._rc.get('path') or (candidate.path if candidate is not None else None) or getattr(nav,'path',[])
        remaining = None
        if path and goal and math.dist(path[-1],goal)<.35:
            j = min(range(len(path)), key=lambda i: math.hypot(path[i][0]-pose.x, path[i][1]-pose.y))
            pts = [(pose.x, pose.y)]+list(path[j:])
            remaining = sum(math.dist(p, q) for p, q in zip(pts, pts[1:]))
        elif goal:
            remaining = math.hypot(goal[0]-pose.x, goal[1]-pose.y)
        level = int(getattr(a.avoid, 'effective', 0))
        out = {'posture': 'COVER' if level >= 2 and getattr(a.avoid, 'owner', False) else 'AVOID',
               'level': level, 'goal': goal, 'remaining_m': remaining, 'weight': a.cfg.avoid.w_risk,
               'stage': 0, 'stalled_s': None, 'reason': 'v2 baseline', 'exposure': None,
               'tracks': [dict(id=t.tid, x=t.x, y=t.y, vx=t.vx, vy=t.vy,
                              team='ally' if t.friend else ('enemy' if t.p_enemy >= .8 else 'unknown'),
                              confidence=t.p_enemy, los=None, sources=t.t_src)
                          for t in a._av_tracks]}
    pose = a.stack.pose
    for source, dest in (('risk_weight','weight'),('remaining_path_m','remaining_m'),
                         ('no_progress_s','stalled_s'),('transition_reason','reason')):
        if source in out:
            out[dest] = out[source]
    all_faces=sum(f.views > 0 for b in a.coverage.boxes.values() for f in b.faces)
    if 'observed_faces' not in out:
        # Use exactly the same sensor-derived eligibility as the production
        # monitor, including for the frozen v2 source that lacks its helper.
        from maicon26.link.schema import scoring_sectors,server_sector
        allowed=set(scoring_sectors(a.team))
        out['observed_faces']=sum(f.views>0 for b in a.coverage.boxes.values()
            if a.coverage.box_available(b) and a.coverage.m3_candidate_box(b)
            and server_sector(a.coverage.box_sector(b)) in allowed for f in b.faces)
    out.update(t=now, robot=a.name, mode=a.mode.value, pose=[pose.x, pose.y, pose.th],
               v=a._v_cmd, w=a._w_cmd, hits=a.robot.status().hits_taken,
               shots=a.laser.shots, confirmed_faces=len(a.omap.confirmed_faces()),
               all_observed_faces=all_faces,observed_faces_scope='available M3 candidate boxes in scoring sectors',
               plan_failures=a.stats.get('risk_plan_fail', 0))
    # Read-only attribution of the existing distance estimate. Do not replace
    # Euclidean fallbacks or change progress acceptance in the evaluator.
    candidate=getattr(a,'_ex_cand',None)
    nav=getattr(a.stack,'navigator',None)
    options=(('risk_path',a._rc.get('path')),
             ('explore_path',getattr(candidate,'path',None)),
             ('local_nav_path',getattr(nav,'path',None)))
    goal=out.get('goal')
    selector=getattr(a,'_remaining_path_route',None)
    if callable(selector):
        selected_source,selected_path=selector(goal)
        labels={'risk':'risk_path','exploration':'explore_path','navigator':'local_nav_path'}
        selected=(labels.get(selected_source,selected_source),selected_path) if selected_path else None
    else:
        # Frozen v2/02/03 used the first nonempty route even when stale.
        selected=next(((name,path) for name,path in options if path),None)
    endpoint_distance=math.dist(selected[1][-1],goal) if selected and goal else None
    out['remaining_path_source']=(selected[0] if endpoint_distance is not None and endpoint_distance<.35
                                  else 'euclidean_fallback' if goal else 'no_goal')
    out['selected_path_endpoint_error_m']=endpoint_distance
    # Planned observations and active emission are excluded, never all low speeds.
    elapsed = now-a.t_start if a.t_start is not None else -1
    stage_distance=math.dist((pose.x,pose.y),goal) if goal else None
    out['native_normal_wait']=out.get('normal_wait')
    out['observation_phase']=getattr(a,'_ex_phase',None)
    obs_start=getattr(a,'_ex_obs_t0',None)
    out['observation_age_s']=now-obs_start if obs_start is not None and out['observation_phase']=='observe' else None
    wait_reasons=[]
    if out.get('normal_wait',False):wait_reasons.append('agent_declared')
    if a.mode.value in ('WAIT','DONE','HALT','REPORT'):wait_reasons.append('server_phase')
    if a.mode.value=='GOAL' and getattr(a,'m5_entered_local',None) is not None:wait_reasons.append('goal_entered')
    if (a.mode.value=='STAGE' and 225<=elapsed<240 and abs(a._v_cmd)<.01
            and stage_distance is not None and stage_distance<=.3):wait_reasons.append('stage_arrived')
    if (a.mode.value=='EXPLORE' and out['observation_phase']=='observe' and candidate is not None
            and out['observation_age_s'] is not None and 0<=out['observation_age_s']<=12.
            and abs(a._v_cmd)<.01):wait_reasons.append('planned_observation')
    out['laser_active']=bool(a.laser.firing())
    out['drive_locked']=bool(a.drive_locked(now))
    if out['laser_active']:wait_reasons.append('laser_active')
    if out['drive_locked']:wait_reasons.append('drive_locked')
    out['normal_wait']=bool(wait_reasons)
    out['normal_wait_reasons']=wait_reasons
    out['stage_goal_distance_m']=stage_distance if a.mode.value=='STAGE' else None
    return clean(out)


class Monitor:
    """Independent 12-second acceptance monitor; does not reset on goal churn."""
    def __init__(self):
        self.states = {}
        self.events = []
        self.flips = []

    def update(self, row, truth):
        t, uid = row['t'], row['robot']
        s = self.states.setdefault(uid, dict(last=t, stalled=0., active=0., stall_time=0.,
                                             goal=None, rem=None, faces=0, shots=0,
                                             incident=False, transitions=deque(), prev=None,
                                             flip_latch=False, direction=0, reversals=deque(), pose=truth['pose']))
        dt = max(0., t-s['last'])
        s['last'] = t
        if row['mode'] in ('WAIT','HALT','REPORT','DONE'):
            # Server phase/initialization resets are not avoidance decisions.
            s['prev']=None
            s['direction']=0
            s['transitions'].clear()
            s['reversals'].clear()
            s['flip_latch']=False
            return
        current = (row.get('posture'), row.get('level'))
        if s['prev'] is not None and current != s['prev']:
            s['transitions'].append(t)
        s['prev'] = current
        direction = 1 if row.get('v',0) > .02 else -1 if row.get('v',0) < -.02 else 0
        if direction and s['direction'] and direction != s['direction']:
            s['reversals'].append(t)
        if direction:
            s['direction'] = direction
        while s['reversals'] and t-s['reversals'][0] > 10:
            s['reversals'].popleft()
        while s['transitions'] and t-s['transitions'][0] > 10:
            s['transitions'].popleft()
        oscillating = len(s['transitions']) > 3 or len(s['reversals']) > 2
        if oscillating and not s['flip_latch']:
            self.flips.append(dict(robot=uid, t=t, kind='B', reason='posture/level >3 or direction reversals >2 in 10 s'))
        s['flip_latch'] = oscillating
        goal = tuple(round(v, 2) for v in row['goal']) if row.get('goal') else None
        rem = row.get('remaining_m')
        face_count = row.get('observed_faces',row['confirmed_faces'])
        progressed = face_count > s['faces'] or row['shots'] > s['shots']
        if goal == s['goal'] and rem is not None and s['rem'] is not None:
            if s['rem']-rem >= .15:
                progressed = True
                s['rem'] = rem
        elif goal != s['goal']:
            s['goal'], s['rem'] = goal, rem
        if rem is not None and rem <= .12:
            progressed = True
        s['faces'], s['shots'] = face_count, row['shots']
        if row['normal_wait']:
            # Pause, not reset: repeatedly entering a look must not erase a stall.
            return
        s['active'] += dt
        if progressed:
            s['stalled'], s['incident'] = 0., False
        else:
            s['stalled'] += dt
        if s['stalled'] >= 3:
            s['stall_time'] += dt
        if s['stalled'] > 12.0 and not s['incident']:
            neighbours = truth['nearest_ugv_m']
            moving = math.dist(truth['pose'][:2], s['pose'][:2])
            kind = ('C' if row.get('plan_failures', 0) > 0 and rem is None else
                    'A' if neighbours < 1.2 and row.get('posture') == 'COVER' else
                    'D' if neighbours < .70 and moving < .1 else 'E')
            self.events.append(dict(robot=uid, t=t, kind=kind, stalled_s=s['stalled'],
                                    mode=row['mode'], goal=goal, nearest_ugv_m=neighbours,
                                    reason='12 s without qualifying path/face/shot/arrival progress'))
            s['incident'] = True
        if progressed:
            s['pose'] = truth['pose']

    def result(self):
        return dict(deadlock_events=self.events, oscillation_events=self.flips,
                    categories=dict(Counter(e['kind'] for e in self.events+self.flips)),
                    active_s=sum(s['active'] for s in self.states.values()),
                    stall_s=sum(s['stall_time'] for s in self.states.values()),
                    stall_ratio=sum(s['stall_time'] for s in self.states.values())/
                                max(1., sum(s['active'] for s in self.states.values())))


def face_coverage(m):
    from maicon26.link.schema import scoring_sectors, server_sector
    from maicon26.sim.world import FACE_N
    result = {}
    for a in m.agents:
        allowed = scoring_sectors(a.team)
        truth = []
        for ci, c in enumerate(m.world.cubes):
            if server_sector(m.world.geo.sector_of(c.cx, c.cy)) not in allowed:
                continue
            for fi, item in c.items.items():
                if item.kind != 'object':
                    continue
                nx, ny = FACE_N[fi]
                truth.append((ci, fi, c.cx+nx*c.sx/2, c.cy+ny*c.sy/2, nx, ny, item.cls))
        confirmed = list(a.omap.confirmed_faces())
        used, matches = set(), []
        for ci, fi, x, y, nx, ny, kind in truth:
            candidates = [(math.hypot(f.x-x, f.y-y), j) for j, f in enumerate(confirmed)
                          if j not in used and f.cls == kind and math.hypot(f.x-x, f.y-y) <= .22
                          and f.vx*nx+f.vy*ny > .3]
            if candidates:
                _, j = min(candidates)
                used.add(j)
                matches.append([ci, fi])
        result[a.name] = dict(expected=len(truth), matched=len(matches), faces=matches,
                              coverage=len(matches)/max(1, len(truth)))
    return result


class Renderer:
    def __init__(self, path, world, fps=2):
        import cv2
        self.cv, self.w, self.fps = cv2, world, fps
        self.width, self.height = 1280, 800
        self.writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*'avc1'), fps, (self.width, self.height))
        if not self.writer.isOpened():
            raise RuntimeError('H264 video encoder unavailable')
        self.frames = 0

    def xy(self, x, y):
        return int(40+x/self.w.geo.W*860), int(735-y/self.w.geo.H*640)

    def draw(self, m, rows, label):
        import numpy as np
        cv, w = self.cv, self.w
        img = np.full((self.height, self.width, 3), (28, 30, 34), np.uint8)
        cv.putText(img, f'2D synthetic sensors | {label} | t={w.t:6.1f}s', (30, 35), cv.FONT_HERSHEY_SIMPLEX, .65, (235,235,235), 1)
        cv.putText(img, 'Evaluator-only true poses; Agent receives sensor model outputs', (30, 62), cv.FONT_HERSHEY_SIMPLEX, .5, (180,180,180), 1)
        byid = {r['robot']: r for r in rows}
        ego = next((a for a in m.agents if a.name == 'B-UGV-2'), m.agents[0])
        fld = getattr(ego, '_field', None)
        if fld is not None:
            heat = np.clip(fld.risk[0]*60, 0, 255).astype(np.uint8)
            heat = cv.applyColorMap(heat, cv.COLORMAP_TURBO)
            heat = cv.flip(cv.resize(heat, (860,640)), 0)
            img[95:735,40:900] = cv.addWeighted(img[95:735,40:900], .68, heat, .32, 0)
        cv.rectangle(img, (40,95),(900,735), (180,180,180),2)
        for x0,y0,x1,y1 in w.obstacles():
            cv.rectangle(img,self.xy(x0,y1),self.xy(x1,y0),(90,90,90),-1)
        colors = {'AVOID': (255,130,35), 'ENGAGE_OK': (0,155,255), 'COVER': (40,40,250)}
        for i,u in enumerate(w.ugvs):
            x,y = self.xy(u.pose.x,u.pose.y)
            r = byid.get(u.id)
            color = colors.get(r.get('posture'), (130,130,130)) if r else (130,130,130)
            cv.circle(img,(x,y),18,color,2)
            cv.circle(img,(x,y),10,(255,100,50) if u.team=='BLUE' else (70,70,255),-1)
            cv.line(img,(x,y),(int(x+20*math.cos(u.pose.th)),int(y-20*math.sin(u.pose.th))),(255,255,255),2)
            cv.putText(img,norm_id(u.id)+(f' L{r.get("level",0)}' if r else ' dummy'),(x-20,y-23),cv.FONT_HERSHEY_SIMPLEX,.4,(240,240,240),1)
            yy = 110+i*105
            cv.putText(img, norm_id(u.id)+(f' {r.get("posture")} L{r.get("level")}' if r else ' scripted dummy'),(930,yy),cv.FONT_HERSHEY_SIMPLEX,.43,color,1)
            if r:
                for dy,text in enumerate([r['mode'],f'ladder={r.get("stage",0)} hits={u.hits_taken}',f'w={r.get("weight")} faces={r["confirmed_faces"]}']):
                    cv.putText(img,text,(930,yy+22*(dy+1)),cv.FONT_HERSHEY_SIMPLEX,.42,(210,210,210),1)
                if r.get('goal'):
                    cv.line(img,(x,y),self.xy(*r['goal']),(120,120,160),1)
        for t in getattr(ego, '_av_tracks', []):
            x,y=self.xy(t.x,t.y)
            color=(255,150,0) if t.friend else ((0,0,255) if t.p_enemy>=.8 else (0,230,230))
            cv.rectangle(img,(x-5,y-5),(x+5,y+5),color,1)
            cv.arrowedLine(img,(x,y),self.xy(t.x+t.vx,t.y+t.vy),color,1)
        cv.putText(img,'Risk / track overlay: B2 estimated map | blue=AVOID orange=ENGAGE_OK red=COVER',(30,775),cv.FONT_HERSHEY_SIMPLEX,.5,(230,230,230),1)
        self.writer.write(img); self.frames += 1

    def close(self):
        self.writer.release()
        return dict(frames=self.frames, fps=self.fps, duration_s=self.frames/self.fps,
                    source='2D diagnostic render, not Gazebo')


def trial(args):
    sys.path.insert(0, str(Path(args.source_root).resolve()) if args.source_root else str(ROOT))
    from maicon26.config import Config
    from maicon26.sim.run import run, score
    cfg = Config()
    cfg.depth.enable = False  # Official API has no verified depth topic.
    if hasattr(cfg, 'progress'):
        cfg.progress.enabled = args.variant != 'v2'
        cfg.risk.threaded = False  # Deterministic virtual-time sensor pipeline.
    elif args.variant != 'v2':
        raise RuntimeError('new progress configuration has not been installed')
    if args.variant == 'blocker_fire':
        cfg.engage.fire_on_blocker = True
    if args.variant == 'dwell_zero':
        cfg.risk.dwell_cap_from_hits = 0
    out = Path(args.output)/args.variant/args.scenario/f'seed_{args.seed:02d}'
    out.mkdir(parents=True, exist_ok=True)
    import maicon26
    source_package=Path(maicon26.__file__).parent
    fingerprints=source_fingerprints(source_package.parent)
    (out/'source_hashes.json').write_text(json.dumps(fingerprints,indent=2),encoding='utf-8')
    harness_hashes={name:hashlib.sha256((ROOT/'scripts'/name).read_bytes()).hexdigest()
                    for name in ('compare_sim.py','audit_m3_faces.py')}
    (out/'harness_hashes.json').write_text(json.dumps(harness_hashes,indent=2),encoding='utf-8')
    (out/'configuration.json').write_text(json.dumps(cfg.to_dict(),ensure_ascii=False,indent=2),encoding='utf-8')
    m, dummies = build_scenario(args.scenario, args.seed, cfg,continue_after_capture=args.continue_after_capture)
    monitor = Monitor()
    video = Renderer(out/'diagnostic.mp4',m.world,args.video_fps) if args.record else None
    next_video = 0.
    cov210 = None
    ticks = 0
    t_wall = time.perf_counter()
    log = gzip.open(out/'ticks.jsonl.gz','wt',encoding='utf-8')
    def probe(mm):
        nonlocal next_video, cov210, ticks
        for dummy in dummies:
            dummy.step(mm)
        rows=[]
        for agent in mm.agents:
            row=telemetry(agent,mm.world.t)
            u=next(u for u in mm.world.ugvs if u.id==agent.name)
            truth=dict(pose=[u.pose.x,u.pose.y,u.pose.th],
                       nearest_ugv_m=min((math.hypot(u.pose.x-o.pose.x,u.pose.y-o.pose.y) for o in mm.world.ugvs if o is not u),default=99.))
            monitor.update(row,truth)
            row['evaluation_only']=truth
            log.write(json.dumps(clean(row),ensure_ascii=False,separators=(',',':'))+'\n')
            rows.append(row)
        ticks += 1
        if video and mm.world.t+1e-8 >= next_video:
            video.draw(mm,rows,f'{args.variant}/{args.scenario}/seed{args.seed}')
            next_video += 1/args.video_fps
        if cov210 is None and mm.world.t >= 210:
            cov210=face_coverage(mm)
            from audit_m3_faces import record
            maps=dict(t=mm.world.t,coverage=cov210,
                scope='read-only Agent estimated evidence; evaluator truth only in coverage match',
                agents={a.name:dict(objects=[clean(record(f)) for f in a.omap.faces],
                    confirmed=[dict(clean(record(f)),cls=f.cls,mean_conf=f.mean_conf) for f in a.omap.confirmed_faces()],
                    boxes=[clean(record(b)) for b in a.coverage.boxes.values()],
                    association_stats=dict(getattr(a.omap,'association_stats',{})),
                    pose=[a.stack.pose.x,a.stack.pose.y,a.stack.pose.th]) for a in mm.agents})
            (out/'snapshot210.json').write_text(json.dumps(clean(maps),ensure_ascii=False,indent=2),encoding='utf-8')
        if ticks%100 == 0:
            (out/'progress.json').write_text(json.dumps(dict(t=mm.world.t,wall_s=time.perf_counter()-t_wall)),encoding='utf-8')
    try:
        run(m,dt=args.dt,max_t=args.duration,probe=probe)
        report=dict(variant=args.variant,scenario=args.scenario,seed=args.seed,
                    metric_version=METRIC_VERSION,
                    backend='2D synthetic sensor model', agents=[a.name for a in m.agents],
                    termination_policy=('diagnostic_continue_after_capture' if args.continue_after_capture else 'official_first_capture_or_300s'),
                    rule_source='docs/API_SOURCE_EXTRACT.txt:387,454',
                    first_capture=next((dict(event) for event in m.world.events if event.get('type')=='base_reached'),None),
                    dummy_controllers=[dict(id=d.u.id,kind=d.kind) for d in dummies],
                    agent_truth_access=False, fixture_truth_used_for_evaluation_and_dummy_control=True,
                    simulated_t=m.world.t,wall_s=time.perf_counter()-t_wall,
                    score=score(m), diagnostics=monitor.result(), coverage_210=cov210,
                    final_face_coverage=face_coverage(m),source_root=str(Path(args.source_root).resolve()) if args.source_root else str(ROOT))
        if video:
            report['video']=video.close();video=None
        (out/'result.json').write_text(json.dumps(clean(report),ensure_ascii=False,indent=2),encoding='utf-8')
        return report
    finally:
        log.close()
        if video:
            video.close()
        for agent in m.agents:
            close = getattr(agent, 'close', None)
            if close:
                close()
            worker = getattr(agent, '_risk_worker', None)
            if worker:
                worker.close()
            agent.stack.close()


def summarize(output):
    root=Path(output)
    rows=[];excluded=[]
    for path in sorted(root.glob('*/*/seed_*/result.json')):
        if not re.fullmatch(r'seed_\d{2,}',path.parent.name):
            excluded.append(dict(path=str(path.relative_to(root)),reason='archived result; never an active case'))
            continue
        result,reason=read_case_result(path.parent)
        if result is None:
            original=json.loads(path.read_text(encoding='utf-8'))
            excluded.append(dict(path=str(path.relative_to(root)),reason=reason,
                                 metric_version=original.get('metric_version',0),controller_score_still_valid=True))
            continue  # Early harness pilot instrumentation is not comparable.
        d=result['diagnostics']; score=result['score']['agents']; n=max(1,len(score))
        cover=result.get('coverage_210') or {}
        row=dict(variant=result['variant'],scenario=result['scenario'],seed=result['seed'],
                 analysis_origin=result.get('analysis_origin','original recorded metrics'),
                 deadlocks=len(d['deadlock_events']),flips=len(d['oscillation_events']),stall_ratio=d['stall_ratio'],
                 hits=sum(v['hits_taken'] for v in score.values())/n,
                 m2_m3=sum(v['m2_points']+(v.get('m3') or {}).get('points',0) for v in score.values())/n,
                 m5_times=[v['m5_reached'] for v in score.values() if v['m5_reached'] is not None],
                 first_capture=result.get('first_capture'),termination_policy=result.get('termination_policy'),
                 coverage_210=sum(v['coverage'] for v in cover.values())/max(1,len(cover)),
                 coverage_210_min=min((v['coverage'] for v in cover.values()),default=0.),
                 wall_s=result['wall_s'],result=str(path.relative_to(root)),
                 events=d['deadlock_events']+d['oscillation_events'])
        rows.append(row)
    (root/'summary.json').write_text(json.dumps(rows,ensure_ascii=False,indent=2),encoding='utf-8')
    manifest=dict(metric_version=METRIC_VERSION,requested_seeds=list(range(20)),
                  requested_scenarios=list(SCENARIOS),variants=list(VARIANTS),
                  expected_games_per_variant=20*len(SCENARIOS),
                  completed_valid=dict(Counter(r['variant'] for r in rows)),
                  excluded_diagnostics=excluded,
                  acceptance='not established until all requested runs and thresholds pass',
                  backend='2D sensor-model benchmark; separate Gazebo results required')
    (root/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8')
    body=['<!doctype html><meta charset="utf-8"><title>MAICON deadlock A/B</title>',
          '<style>body{font:15px sans-serif;margin:30px;background:#15171c;color:#eee}table{border-collapse:collapse}td,th{padding:8px;border:1px solid #555}a{color:#87c9ff}</style>',
          '<h1>MAICON 교착 A/B — 2D 합성 센서 시험</h1><p>Gazebo 또는 실제 카메라 성능의 증거가 아닙니다. 센서 모델의 관측을 Agent에 제공하고 진실값은 평가와 더미 제어에서만 사용합니다. 기준: 12초 초과 무진척, 5% 이하 정체, 3:30 모든 개별 그림 면 100%. 기본 경기 종료는 첫 M5 점령 또는 5:00입니다. 표는 실행 완료 결과이며 전체 행렬 합격이 아닙니다.</p>',
          '<table><tr><th>변형/시나리오/시드</th><th>교착</th><th>진동</th><th>정체%</th><th>M2+M3</th><th>피격</th><th>3:30 면 평균/최소%</th><th>첫 M5 점령</th><th>영상·사건</th></tr>']
    for r in rows:
        folder=Path(r['result']).parent.as_posix(); video=folder+'/diagnostic.mp4'
        links=' '.join(f'<a href="{video}#t={max(0,e["t"]-3):.1f}">{e["kind"]} {e["t"]:.1f}s {e["robot"]}</a>' for e in r['events'])
        capture=r.get('first_capture')
        label=f'{capture["t"]:.1f}s {html.escape(capture.get("camp","?"))}' if capture else '없음'
        body.append(f'<tr><td>{r["variant"]}/{r["scenario"]}/{r["seed"]}</td><td>{r["deadlocks"]}</td><td>{r["flips"]}</td><td>{100*r["stall_ratio"]:.1f}</td><td>{r["m2_m3"]:.1f}</td><td>{r["hits"]:.2f}</td><td>{100*r["coverage_210"]:.1f} / {100*r["coverage_210_min"]:.1f}</td><td>{label}</td><td><a href="{video}">영상</a> {links}</td></tr>')
    body.append('</table>')
    (root/'report.html').write_text('\n'.join(body),encoding='utf-8')
    return rows


def matrix(args):
    Path(args.output).mkdir(parents=True,exist_ok=True)
    jobs=[]
    expected_by_variant={}
    for variant in args.variants:
        source=(args.baseline_source if variant=='v2' else args.candidate_source) or ROOT
        expected_by_variant[variant]=source_fingerprints(source)
        for scenario in args.scenarios:
            for seed in args.seeds:
                result_path=Path(args.output)/variant/scenario/f'seed_{seed:02d}'/'result.json'
                if result_path.exists() and not args.resume:
                    raise FileExistsError(f'Completed case exists; use --resume or a new output folder: {result_path}')
                if (result_path.exists() or (result_path.parent/'source_hashes.json').exists()) and args.resume:
                    old=json.loads(result_path.read_text(encoding='utf-8')) if result_path.exists() else {}
                    policy='diagnostic_continue_after_capture' if args.continue_after_capture else 'official_first_capture_or_300s'
                    reusable,reason=reusable_case(result_path.parent,expected_by_variant[variant],policy)
                    if reusable:
                        continue
                    # Preserve the original controller run and video; replace
                    # only the case's active slot with a fresh diagnostic run.
                    source=result_path.parent.resolve()
                    suffix=('_superseded_source' if reason=='superseded source revision' else
                            '_incomplete_attempt' if reason=='no completed result' else '_legacy_metric_'+str(old.get('metric_version',0)))
                    target=source.with_name(source.name+suffix)
                    index=0
                    while target.exists():
                        index+=1
                        target=source.with_name(source.name+suffix+'_'+str(index))
                    root=Path(args.output).resolve()
                    if root not in source.parents or root not in target.parents:
                        raise ValueError('Diagnostic archive must remain inside matrix output')
                    source.rename(target)
                    (target/'superseded.json').write_text(json.dumps(dict(reason=reason,
                        previous_metric_version=old.get('metric_version'),replacement_metric_version=METRIC_VERSION,
                        preserved_source_hashes='source_hashes.json',preserved_video='diagnostic.mp4'),indent=2),encoding='utf-8')
                cmd=[sys.executable,str(Path(__file__).resolve()),'--single','--variant',variant,'--scenario',scenario,
                     '--seed',str(seed),'--output',str(Path(args.output).resolve()),'--duration',str(args.duration),
                     '--dt',str(args.dt),'--video-fps',str(args.video_fps)]
                if args.record:
                    cmd.append('--record')
                if args.continue_after_capture:
                    cmd.append('--continue-after-capture')
                if variant=='v2' and args.baseline_source:
                    cmd+=['--source-root',str(Path(args.baseline_source).resolve())]
                elif variant!='v2' and args.candidate_source:
                    cmd+=['--source-root',str(Path(args.candidate_source).resolve())]
                jobs.append((variant,scenario,seed,cmd))
    def execute(job):
        while (Path(args.output)/'PAUSE_SCHEDULING').exists():
            time.sleep(1.)  # Operator may free the GPU/CPU for a Gazebo trial.
        variant,scenario,seed,cmd=job
        p=Path(args.output)/variant/scenario/f'seed_{seed:02d}';p.mkdir(parents=True,exist_ok=True)
        env=dict(os.environ,OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1')
        with (p/'console.log').open('w',encoding='utf-8') as f:
            done=subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT,env=env)
        return dict(variant=variant,scenario=scenario,seed=seed,returncode=done.returncode)
    summarize(args.output)  # Archives stop counting before replacement starts.
    with concurrent.futures.ThreadPoolExecutor(max_workers=args.workers) as pool:
        for done in pool.map(execute,jobs):
            print(json.dumps(done),flush=True)
            summarize(args.output)
    summarize(args.output)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output')
    p.add_argument('--single',action='store_true');p.add_argument('--summarize',action='store_true')
    p.add_argument('--variant',choices=VARIANTS,default='progress');p.add_argument('--scenario',choices=SCENARIOS,default='selfplay')
    p.add_argument('--seed',type=int,default=0);p.add_argument('--seeds',type=int,nargs='+',default=list(range(20)))
    p.add_argument('--variants',choices=VARIANTS,nargs='+',default=['v2','progress'])
    p.add_argument('--scenarios',choices=SCENARIOS,nargs='+',default=list(SCENARIOS))
    p.add_argument('--duration',type=float,default=330);p.add_argument('--dt',type=float,default=.1)
    p.add_argument('--record',action='store_true');p.add_argument('--video-fps',type=float,default=2)
    p.add_argument('--workers',type=int,default=6);p.add_argument('--resume',action='store_true')
    p.add_argument('--continue-after-capture',action='store_true',help='Explicit nonofficial300s completion diagnostic; official first M5 capture ends play by default.')
    p.add_argument('--source-root');p.add_argument('--baseline-source');p.add_argument('--candidate-source')
    args=p.parse_args()
    if args.output is None:
        args.output=('recordings/deadlock_continuation_20261109' if args.continue_after_capture else 'recordings/deadlock_official_ab_20261109')
    if args.summarize:
        summarize(args.output)
    elif args.single:
        trial(args)
    else:
        matrix(args)


if __name__=='__main__':
    main()
