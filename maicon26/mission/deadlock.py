"""Sensor-only progress/posture integration; no simulator state is accepted here.
This mixin keeps mission arbitration separate from threat severity and posture.
"""
from __future__ import annotations

import math
from collections import deque
import numpy as np

from ..nav.progress import ProgressMonitor
from ..nav.grid import line_free
from ..util import clamp, wrap
from .posture import PostureController, blocker_fire_allowed


class DeadlockMixin:
    def _init_deadlock(self):
        self.progress = ProgressMonitor(self.cfg, slot=self.name)
        self.posture_control = PostureController(self.cfg)
        self.posture_state = None
        self.progress_state = None
        self._decision_snapshot = {}
        self._progress_goal = None
        self._progress_remaining = None
        self._progress_arrived = False
        self._progress_backoff = None
        self._progress_actions = ()
        self._mandatory_recovery = None
        self._decision_events = deque(maxlen=300)
        self._hit_history = deque(maxlen=32)
        self._seen_hit_count = 0
        self._exposure = 0.
        self._dwell_start = None
        self._measured_speed = 0.
        self._blocker_last_fire = {}
        self._blocker_active = None
        self._risk_worker = None
        self._risk_scan_stamp = None

    def close_deadlock(self):
        if self._risk_worker is not None:
            self._risk_worker.close()
            self._risk_worker = None

    def _deadlock_enabled(self):
        return self.cfg.progress.enabled

    @staticmethod
    def _goal_key(goal):
        return None if goal is None else (round(float(goal[0]), 1), round(float(goal[1]), 1))

    def _mission_destination(self):
        recovery = self._mandatory_recovery
        if recovery is not None and recovery['mode']==self.mode.value and self.mode.value in ('STAGE','GOAL'):
            canonical = self._goal_xy() if self.mode.value=='GOAL' else self._stage_point()
            if recovery['canonical']==self._goal_key(canonical):
                return recovery['goal']
        if self.mode.value == 'GOAL':
            return self._goal_xy()
        if self.mode.value == 'STAGE':
            return self._stage_point()
        if self._ex_cand is not None and self.mode.value == 'EXPLORE':
            return self._ex_cand.vp
        return self._progress_goal or self._cur_goal

    def _scoring_faces(self):
        from ..link.schema import scoring_sectors, server_sector
        allowed=set(scoring_sectors(self.team))
        return [f for b in self.coverage.boxes.values()
                if self.coverage.box_available(b) and self.coverage.m3_candidate_box(b)
                and server_sector(self.coverage.box_sector(b)) in allowed for f in b.faces]

    def _alternate_mandatory_goal(self, canonical, now, visited):
        """Choose an observed-free approach; never change the assigned M5 base.

        A flood fill checks observed map connectivity once. Unknown cells and
        occupied/inflated cells cannot become an invented bypass. Future motion
        still goes through the normal risk planner and final LiDAR guard.
        """
        grid,p=self.stack.grid,self.stack.pose
        blocked,_=grid.cost_maps(self.cfg.nav.inflate_hard,self.cfg.nav.inflate_soft,0.)
        free=(grid.L<-.2)&~blocked
        start=grid.to_cell(p.x,p.y)
        if not grid.in_bounds(*start) or not free[start]:
            choices=np.argwhere(free)
            if len(choices)==0: return None
            choices=sorted(choices,key=lambda cell:(cell[0]-start[0])**2+(cell[1]-start[1])**2)
            start=tuple(map(int,choices[0]))
            if math.dist(grid.to_world(*start),(p.x,p.y))>.25: return None
        seen=np.zeros(free.shape,dtype=bool);seen[start]=True
        queue=deque([start])
        while queue:
            y,x=queue.popleft()
            for dy,dx in ((-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)):
                yy,xx=y+dy,x+dx
                if (not grid.in_bounds(yy,xx) or seen[yy,xx] or not free[yy,xx]
                        or (dy and dx and (not free[y,xx] or not free[yy,x]))):
                    continue
                seen[yy,xx]=True;queue.append((yy,xx))
        candidates=[]
        for y,x in np.argwhere(seen):
            point=grid.to_world(int(y),int(x));key=self._goal_key(point)
            separation=math.dist(point,canonical)
            if (not .25<=separation<=1.2 or math.dist(point,(p.x,p.y))<.35
                    or key in visited or not self.progress.eligible(key,now)):
                continue
            # A staging point must remain in the arena, not become another
            # team's base. GOAL entry candidates stay inside the assigned base.
            inside = bool(self.mode.value=='GOAL' and self.m5_base and
                          self.geo.in_base(*point,self.m5_base,margin=-.12))
            if not inside and self.geo.sector_of(*point) is None:
                continue
            own_radius=math.hypot(self.cfg.robot.length,self.cfg.robot.width)/2
            radius=own_radius+max(self.cfg.avoid.r_body,own_radius)
            if any(tr.age(now)<=1. and math.dist(point,(tr.x,tr.y))<radius+self.cfg.engage.min_clearance_m
                   for tr in self._av_tracks):
                continue
            score=3.*separation+math.dist(point,(p.x,p.y))
            candidates.append((0 if inside else 1,score,self.progress.tie_break(key),point,inside))
        if not candidates: return None
        chosen=min(candidates)
        return chosen[3],chosen[4]

    def _mandatory_navigation_goal(self, now, canonical):
        """Honor failed approach cooldown/exclusion without abandoning M5's ID."""
        if not self._deadlock_enabled(): return canonical
        context=(self.mode.value,self._goal_key(canonical),self.m5_base)
        state=self._mandatory_recovery
        if state is not None and state['context']!=context:
            state=self._mandatory_recovery=None
        if state is not None and state['goal'] is not None:
            goal=state['goal']
            if self.progress.eligible(self._goal_key(goal),now):
                reached=math.dist(goal,(self.stack.pose.x,self.stack.pose.y))<=.18
                if not reached or state['entry'] or self.mode.value=='STAGE': return goal
                state['visited'].add(self._goal_key(goal))
                state['goal']=None;state['next_try']=now
                self.stack.clear_goal()
            else:
                state['goal']=None;state['next_try']=now
        if self.progress.eligible(self._goal_key(canonical),now):
            self._mandatory_recovery=None
            return canonical
        if state is None:
            state=dict(context=context,mode=self.mode.value,canonical=self._goal_key(canonical),
                       goal=None,entry=False,visited=set(),next_try=now)
            self._mandatory_recovery=state
        if now<state['next_try']: return None
        state['next_try']=now+1.
        selected=self._alternate_mandatory_goal(canonical,now,state['visited'])
        if selected is None:
            self.stats['mandatory_approach_unavailable']=self.stats.get('mandatory_approach_unavailable',0)+1
            return None
        state['goal'],state['entry']=selected
        self._progress_goal=state['goal']
        self.stats['mandatory_approach_changes']=self.stats.get('mandatory_approach_changes',0)+1
        return state['goal']

    def _normal_progress_wait(self, now):
        mode = self.mode.value
        if mode in ('WAIT', 'HALT', 'REPORT', 'DONE') or self.drive_locked(now):
            return True
        if self.laser.firing():
            return True
        if self._ex_phase == 'observe' and self._ex_cand is not None:
            return now - self._ex_obs_t0 <= 12.
        if mode == 'STAGE':
            p, g = self.stack.pose, self._mission_destination()
            if g is None: return False
            return math.hypot(g[0]-p.x, g[1]-p.y) <= .3
        if mode == 'GOAL' and self.m5_entered_local is not None:
            return True
        return False

    def _remaining_path(self, goal):
        if goal is None:
            return None
        p = self.stack.pose
        path = self._rc.get('path')
        if not path and self._ex_cand is not None:
            path = self._ex_cand.path
        if not path:
            nav = getattr(self.stack, 'navigator', None)
            path = getattr(nav, 'path', None)
        if path and math.hypot(path[-1][0]-goal[0], path[-1][1]-goal[1]) < .35:
            pts = self._path_remainder(path, p)
            return sum(math.hypot(b[0]-a[0], b[1]-a[1]) for a,b in zip(pts,pts[1:]))
        return math.hypot(goal[0]-p.x, goal[1]-p.y)

    def _deadlock_update(self, now, dt, elapsed):
        if not self._deadlock_enabled():
            return
        p, st = self.stack.pose, self.robot.status()
        history=[h for h in self._hist if now-h[0]<=.4]
        if len(history)>=4:
            first,last=history[:2],history[-2:]
            span=(sum(h[0] for h in last)-sum(h[0] for h in first))/2.
            self._measured_speed=math.hypot(
                (sum(h[1] for h in last)-sum(h[1] for h in first))/2.,
                (sum(h[2] for h in last)-sum(h[2] for h in first))/2.)/max(span,.05)
        else:
            self._measured_speed=0.
        if st.hits_taken > self._seen_hit_count:
            self._hit_history.append((p.x, p.y, now))
            self._seen_hit_count = st.hits_taken
        visible = []
        occ = self.stack.grid.occupied()
        for tr in self._av_tracks:
            tx, ty, _ = tr.predict(now, 0., self.cfg.avoid)
            d = math.hypot(tx-p.x, ty-p.y)
            # Shorten endpoints past robot hulls: dynamic surface cells are not cover.
            ux, uy = (tx-p.x)/max(d,1e-9), (ty-p.y)/max(d,1e-9)
            los = d <= self.cfg.risk.range_m and line_free(self.stack.grid, occ,
                    (p.x+.20*ux,p.y+.20*uy),(tx-.20*ux,ty-.20*uy))
            if los and tr.age(now) <= 5.:
                visible.append(tr)
        # E is weighted continuous LOS exposure in seconds. Cover decays it.
        if visible:
            self._exposure += dt * min(2., sum(tr.weight for tr in visible))
        else:
            self._exposure = max(0., self._exposure-dt)
        goal = self._mission_destination()
        remaining = self._remaining_path(goal)
        normal_wait = self._normal_progress_wait(now)
        scoring_faces=self._scoring_faces()
        faces = sum(f.views > 0 for f in scoring_faces)
        front_blocked = self._scan is not None and self._scan.min_in_sector(0., .6) < .5
        key = self._goal_key(goal) if goal is not None else ("selection", self.mode.value)
        ps = self.progress.update(now, key, remaining,
                face_count=faces, shot_count=self.laser.shots, arrived=self._progress_arrived,
                paused=normal_wait, physically_blocked=front_blocked)
        self._progress_arrived = False
        self.progress_state = ps
        self._progress_actions = ps.actions
        self._progress_remaining = remaining
        gd = math.hypot(self._goal_xy()[0]-p.x,self._goal_xy()[1]-p.y) if self._goal_xy() else math.inf
        in_cm = self.geo.sector_of(p.x,p.y) in ('CM','COMMON')
        if self._ex_cand is not None:
            in_cm |= self.geo.sector_of(*self._ex_cand.vp) in ('CM','COMMON')
        state = self.posture_control.update(now, xy=(p.x,p.y), exposure=self._exposure,
                hits=st.hits_taken, visible_threat=bool(visible), in_cm=in_cm,
                in_goal=(self.mode.value=='GOAL' or (self.m5_type is not None and elapsed>=self.cfg.mission.t_base_cmd-1.)), goal_distance=gd,
                invulnerable=self.locks.weapon_locked(st,now), stall_stage=ps.stage,
                elapsed=elapsed, unfinished_scoring=any(f.views==0 for f in scoring_faces),
                lock_until=ps.lock_until)
        self.posture_state = state
        direction = 1 if self._v_cmd > .02 else -1 if self._v_cmd < -.02 else 0
        self.progress.note_decision(now, str(state.posture.value), state.level, direction)
        for event in state.transitions:
            self._decision_events.append(event)
            self._log(now, 'POSTURE '+str(event))
        if 'replan' in ps.actions:
            self._rc = {}
            self.stack.clear_goal()
            self.stats['progress_replans'] = self.stats.get('progress_replans',0)+1
        if 'backoff' in ps.actions and front_blocked:
            wait = self.progress.jittered(.7,'backoff')
            self._progress_backoff = (now, wait)
            self.stack.clear_goal()
        if 'abandon' in ps.actions:
            if self._ex_cand is not None:
                self.explorer.mark_bad(self._ex_cand.vp,now)
            self._ex_cand, self._ex_sel_t, self._cur_goal = None, -1e9, None
            self._ex_phase = 'travel'
            self.stack.clear_goal()
            self._progress_goal = None
            self._rc = {}
            if self.mode.value in ('STAGE','GOAL'):
                canonical=self._goal_xy() if self.mode.value=='GOAL' else self._stage_point()
                self._progress_backoff=None
                self._mandatory_navigation_goal(now,canonical)
                # Failure history remains; a new approach (or bounded map
                # recheck) owns a fresh episode instead of stage4 forever.
                self.progress.reset(now)
            if self.mode.value == 'STRIKE':
                self._strike_block_until = now + 30.
                self._set_mode(now,type(self.mode).EXPLORE,'progress abandon; retry after 30s')
            self.stats['progress_abandons'] = self.stats.get('progress_abandons',0)+1
        # Body collision response remains separate; legacy EVADE only owns L3 COVER.
        if state.allow_evade:
            from .avoid import Level
            self.avoid.level = Level.EVADE
        tracks = []
        for tr in self._av_tracks:
            tracks.append(dict(id=tr.tid,x=tr.x,y=tr.y,vx=tr.vx,vy=tr.vy,
                team='ally' if tr.friend else 'enemy' if tr.p_enemy>=.8 else 'unknown',
                confidence=1.-tr.p_enemy if tr.friend else tr.p_enemy,
                los=any(q.tid==tr.tid for q in visible), age=tr.age(now)))
        self._decision_snapshot = dict(t=elapsed, posture=state.posture.value, level=state.level,
                exposure=self._exposure, risk_weight=state.risk_weight, goal=goal,
                goal_key=self._goal_key(goal), remaining_path_m=remaining, tracks=tracks,
                hits=st.hits_taken, shots=self.laser.shots, observed_faces=faces,
                confirmed_faces=len(self.omap.confirmed_faces()),
                stage=ps.stage, no_progress_s=ps.stalled_s,
                normal_wait=normal_wait, transition_reason=state.reason,
                actions=list(ps.actions), transitions=list(state.transitions))
        self._decision_snapshot['goal_key'] = key
        self._decision_snapshot['exposed_dwell_expired'] = self._dwell_expired(now)

    def deadlock_telemetry(self):
        if not self._deadlock_enabled() and self.stack.initialized:
            # Read-only baseline instrumentation must not run the candidate
            # controller or invent its exposure/stall state.
            now=self.robot.now()
            goal=(self._strike.get('goal') if self.mode.value=='STRIKE'
                  else self._mission_destination())
            level=int(self.avoid.effective)
            return dict(t=now-(self.t_start if self.t_start is not None else now),
                posture='COVER' if level>=2 else 'AVOID',level=level,exposure=None,
                risk_weight=self.cfg.avoid.w_risk,goal=goal,goal_key=self._goal_key(goal),
                remaining_path_m=self._remaining_path(goal),stage=0,no_progress_s=None,
                normal_wait=self._normal_progress_wait(now),transition_reason='v2 diagnostic only',
                hits=self.robot.status().hits_taken,shots=self.laser.shots,
                observed_faces=sum(f.views>0 for f in self._scoring_faces()),
                confirmed_faces=len(self.omap.confirmed_faces()),
                tracks=[dict(id=tr.tid,x=tr.x,y=tr.y,vx=tr.vx,vy=tr.vy,
                    team='ally' if tr.friend else 'enemy' if tr.p_enemy>=.8 else 'unknown',
                    confidence=1.-tr.p_enemy if tr.friend else tr.p_enemy,
                    los=None,age=tr.age(now)) for tr in self._av_tracks])
        return dict(self._decision_snapshot)

    deadlock_diagnostics = deadlock_telemetry

    def _progress_gate(self, now, dt):
        if not self._deadlock_enabled():
            return False
        if (self._decision_snapshot.get('exposed_dwell_expired') and
                self.mode.value=='STRIKE' and self._strike.get('phase')=='aim' and not self.laser.firing()):
            self._strike_block_until=now+self.cfg.posture.commit_s
            self._set_mode(now,type(self.mode).EXPLORE,'exposed aim deferred')
            self._dwell_start=None
        if self._progress_backoff is not None:
            start, wait = self._progress_backoff
            elapsed = now-start
            if elapsed < .6:
                rear = self._scan.min_in_sector(math.pi,.6) if self._scan else 0.
                self._apply(-.12 if rear>.35 else 0.,0.,0.,0.,dt,limit=True)
            elif elapsed < .6+wait:
                self._apply(0.,0.,0.,0.,dt,limit=True)
            elif elapsed < 1.6+wait:
                # Right-hand passing; do not blindly reverse into another UGV.
                self._apply(.07,-.7,0.,0.,dt,limit=True)
            else:
                self._progress_backoff = None
                self._rc = {}
                return False
            return True
        return self._blocker_gate(now,dt)

    def _risk_weight(self):
        return self.posture_state.risk_weight if self._deadlock_enabled() and self.posture_state else self.cfg.avoid.w_risk*(.35+.65*self._kappa)

    def _right_hand_cost(self, goal, base_cost, now):
        """Sensor-observed encounters prefer our right of the mission direction.

        A shared passing convention does not require knowing the other team's
        policy. It is a finite preference, never a new forbidden region.
        """
        if not self._deadlock_enabled():
            return base_cost, False
        p, grid = self.stack.pose, self.stack.grid
        dx,dy=goal[0]-p.x,goal[1]-p.y
        length=math.hypot(dx,dy)
        if length<.05:
            return base_cost, False
        ux,uy=dx/length,dy/length
        candidates=[]
        for tr in self._av_tracks:
            if tr.age(now)>1.: continue
            tx,ty,_=tr.predict(now,0.,self.cfg.avoid)
            along=(tx-p.x)*ux+(ty-p.y)*uy
            lateral=ux*(ty-p.y)-uy*(tx-p.x)
            if .05<along<2.5 and abs(lateral)<.9:
                candidates.append((tx,ty))
        if not candidates:
            return base_cost, False
        X,Y=np.meshgrid(grid.ox+(np.arange(grid.nx)+.5)*grid.res,
                        grid.oy+(np.arange(grid.ny)+.5)*grid.res)
        left=ux*(Y-p.y)-uy*(X-p.x)
        cost=base_cost.copy()
        # .30m edge clearance is enforced independently by physical safety.
        for tx,ty in candidates:
            cost+=2.*np.exp(-((X-tx)**2+(Y-ty)**2)/1.3)*(.5+.5*np.tanh(left/.1))
        return cost, True

    def _observed_space_cost(self, base_cost):
        """Occluded cells are not evidence of a safe route behind a wall.

        Only scan-observed free cells avoid this finite uncertainty cost. This
        neither invents obstacle interiors nor prevents exploring unknown space.
        """
        if not self._deadlock_enabled():
            return base_cost
        return base_cost + (self.stack.grid.L >= -.2) * self.cfg.risk.unknown_space_cost

    def _dwell_expired(self,now):
        state = self.posture_state
        cap = bool(state and state.cap_exposed_dwell and self._decision_snapshot.get('tracks')
                   and any(tr['los'] for tr in self._decision_snapshot['tracks']))
        if not cap or self._measured_speed>.03:
            self._dwell_start = None
            return False
        if self._dwell_start is None:
            self._dwell_start = now
        return now-self._dwell_start >= self.cfg.risk.max_exposed_dwell_s

    def _dynamic_clearance(self,v,w,dt):
        if not self._deadlock_enabled() or not self.stack.initialized:
            return v,w
        p = self.stack.pose
        clearance = self.cfg.engage.min_clearance_m
        radius = math.hypot(self.cfg.robot.length,self.cfg.robot.width)/2
        for tr in self._av_tracks:
            if tr.age(self.robot.now())>1.:
                continue
            dx,dy=tr.x-p.x,tr.y-p.y
            d=math.hypot(dx,dy)
            closing = v*(dx*math.cos(p.th)+dy*math.sin(p.th))/max(d,1e-9)
            gap = d-radius-max(self.cfg.avoid.r_body,radius)
            if closing>0 and gap<clearance+.15:
                v*=clamp((gap-clearance)/.15,0.,1.)
                if gap<clearance+.05 and abs(w)<.1:
                    w=-.45
        return v,w

    def _blocker_gate(self,now,dt):
        if not self.cfg.engage.fire_on_blocker or self.posture_state is None:
            return False
        state = self.posture_state
        active_shot=self._blocker_active is not None and self.laser.firing()
        if state.posture.value!='ENGAGE_OK' or (self.progress_state.stage<2 and not active_shot) or self.weapon_locked(now):
            if self._blocker_active is not None and self.laser.firing():
                self.laser.abort()
            self._blocker_active=None
            return False
        p=self.stack.pose
        enemy=[]
        for tr in self._av_tracks:
            d=math.hypot(tr.x-p.x,tr.y-p.y)
            bearing=wrap(math.atan2(tr.y-p.y,tr.x-p.x)-p.th)
            ec=self.cfg.engage
            if (not tr.friend and tr.p_enemy>=ec.enemy_confidence and tr.evidence>=3
                    and ec.blocker_min_range_m<=d<=ec.blocker_max_range_m
                    and abs(bearing)<math.radians(self.cfg.robot.pan_limit) and tr.age(now)<.5
                    and self._is_path_blocker(tr)):
                enemy.append((d,tr,bearing))
        if not enemy:
            self._abort_blocker()
            return False
        distance,tr,bearing=min(enemy,key=lambda item:item[0])
        if self._blocker_active not in (None,tr.tid):
            self._abort_blocker()
        if self._dwell_expired(now) and not self.laser.firing():
            self._abort_blocker()
            return False
        pan,tilt=self.robot.get_pan_tilt()
        det=self._best_det(lambda d:d.cls=='UGV' and d.team==self.geo.opponent(self.team)
            and abs(wrap(self.cam.bearing(d.cx)+math.radians(pan)-bearing))<.15)
        if det is None or not self.vision_fresh(now):
            if self._blocker_active and self.laser.firing(): self.laser.abort()
            return False
        pan_c,tilt_c,res=self.aim.update(det,pan,tilt,now,distance)
        guard=self._friend_in_line(det,now,pan)
        for other in self._av_tracks:
            if other.tid==tr.tid or (not other.friend and other.p_enemy>=.8): continue
            od=math.hypot(other.x-p.x,other.y-p.y)
            ob=wrap(math.atan2(other.y-p.y,other.x-p.x)-p.th)
            guard |= od<distance+.2 and abs(wrap(ob-bearing))<math.radians(self.cfg.mission.friendly_fire_guard_deg)+math.atan2(self.cfg.avoid.r_body,max(od,.1))
        safe=(not guard and res.sensor_locked and self._may_fire(now) and self._fire_conf_ok(det)
              and self._shot_geometry_ok(det,now,pan,tilt))
        if not safe:
            if self._blocker_active and self.laser.firing(): self.laser.abort()
            return False
        active=self.laser.firing() and self._blocker_active==tr.tid
        permission=blocker_fire_allowed(self.cfg,now=now,posture=state.posture,
            stall_stage=max(2,self.progress_state.stage) if active else self.progress_state.stage,
            affiliation='enemy',confidence=tr.p_enemy,
            distance_m=distance,dome_visible=res.sensor_locked,friendly_in_line=guard,
            weapon_locked=self.weapon_locked(now),
            ammo=self.laser.ammo+(1 if active else 0),m2_complete=self.target_done,
            last_target_shot=None if active else self._blocker_last_fire.get(tr.tid),
            actually_blocking=self._is_path_blocker(tr))
        if not permission.allowed:
            self._abort_blocker()
            return False
        self._blocker_active=tr.tid
        self._apply(0.,res.body_w,pan_c,tilt_c,dt,limit=True)
        self.laser.track(res.aligned,now,dt)
        if not active and res.aligned and self.aim.held_for(now)>=self.cfg.aim.hold_before_fire and self.laser.ready():
            if self.laser.fire(now):
                self._blocker_last_fire[tr.tid]=now
                self.stats['blocker_shots']=self.stats.get('blocker_shots',0)+1
        return True

    def _abort_blocker(self):
        if self._blocker_active is not None and self.laser.firing():
            self.laser.abort()
        self._blocker_active=None

    def _is_path_blocker(self,track):
        """A nearby enemy to the side is not necessarily blocking our route."""
        p=self.stack.pose
        goal=self._mission_destination()
        if goal is None:
            return False
        path=self._rc.get('path')
        if path:
            from .avoid import carrot_point
            goal=carrot_point(path,p,min(1.5,self.cfg.engage.blocker_max_range_m))
        dx,dy=goal[0]-p.x,goal[1]-p.y
        length=math.hypot(dx,dy)
        if length<.05:
            return False
        projection=((track.x-p.x)*dx+(track.y-p.y)*dy)/length
        perpendicular=abs(dx*(track.y-p.y)-dy*(track.x-p.x))/length
        own_radius=math.hypot(self.cfg.robot.length,self.cfg.robot.width)/2.
        radius=own_radius+max(self.cfg.avoid.r_body,own_radius)
        return .05<projection<=length and perpendicular<=radius+self.cfg.engage.min_clearance_m
