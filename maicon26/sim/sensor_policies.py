"""Explicit simulator opponent policies using the ordinary Agent sensor boundary.

No world pose, layout truth, referee target location, peer command, or peer
Agent object is accepted. Public arena coordinates are used for patrol/CM
destinations. Enemy locations come exclusively from observed tracks.
"""
from __future__ import annotations

from copy import deepcopy
import math

from ..mission.manager import Agent, Mode
from ..util import clamp, wrap

SLOTS=('B-1','B-2','B-3','R-1','R-2','R-3')
POLICIES=('agent','idle','patrol_own','evader','hunter','blocker')
SCENARIOS=('selfplay','vs_evaders','vs_hunters','vs_blockers','mixed_allies','goal_race')


def policy_assignments(scenario):
    if scenario not in SCENARIOS:
        raise ValueError('Unknown sensor policy scenario: '+str(scenario))
    out={slot:'agent' for slot in SLOTS}
    kind={'vs_evaders':'evader','vs_hunters':'hunter','vs_blockers':'blocker'}.get(scenario)
    if kind:
        out.update({slot:kind for slot in SLOTS if slot.startswith('R')})
    elif scenario=='mixed_allies':
        out.update({'B-1':'idle','B-3':'patrol_own','R-1':'hunter','R-2':'blocker','R-3':'evader'})
    return out


def make_sensor_policy(policy, *args, **kwargs):
    if policy not in POLICIES:
        raise ValueError('Unknown policy: '+str(policy))
    if policy=='agent':
        return Agent(*args,**kwargs)
    return SensorPolicyAgent(*args,policy=policy,**kwargs)


class SensorPolicyAgent(Agent):
    """ROS or simulated HAL, same stop/watchdog/laser and sensor safety methods.

    Opponent strategy stays fixed across candidate/baseline variants. It does
    not run the tested progress policy, acquire legacy EVADE ownership, or
    learn a teammate's intended route. Physical safety and official phase
    limits still apply through Agent._decide/_apply/_nav_to.
    """
    def __init__(self,*args,policy,**kwargs):
        if policy not in POLICIES or policy=='agent':
            raise ValueError('SensorPolicyAgent requires an explicit dummy policy')
        # Never mutate the caller's team-agent configuration or hunting flag.
        if len(args)>2:
            args=list(args); cfg=deepcopy(args[2]);args[2]=cfg
        else:
            from ..config import Config
            cfg=deepcopy(kwargs.get('cfg') or Config());kwargs['cfg']=cfg
        cfg.progress.enabled=False
        cfg.engage.fire_on_blocker=False
        cfg.mission.engage_ugv=False
        self.policy=policy
        self._policy_goal=None
        self._policy_patrol_index=0
        self._policy_last_fire={}
        super().__init__(*args,**kwargs)
        self.stats['sensor_policy']=policy

    def _arbitrate(self,now,st):
        return Mode.EXPLORE

    def _avoid_gate(self,now,dt,t):
        # The explicitly selected policy owns movement; final physical safety
        # and rule keepout filters in Agent still run for every output.
        return False

    def _risk_active(self):
        # Fixed opponent behaviour must not inherit the candidate's cost policy.
        return False

    def deadlock_telemetry(self):
        p=self.stack.pose
        goal=self._policy_goal
        return dict(policy=self.policy,posture='AVOID',level=int(self.avoid.effective),
                    goal=goal,remaining_path_m=math.dist((p.x,p.y),goal) if goal else None,
                    risk_weight=0.,stage=0,no_progress_s=None,normal_wait=self.policy=='idle',
                    transition_reason='sensor_policy:'+self.policy,
                    hits=self.robot.status().hits_taken,shots=self.laser.shots,
                    observed_faces=sum(f.views>0 for b in self.coverage.boxes.values() for f in b.faces),
                    tracks=[dict(id=t.tid,x=t.x,y=t.y,vx=t.vx,vy=t.vy,
                                 team='ally' if t.friend else 'enemy' if t.p_enemy>=.8 else 'unknown',
                                 confidence=1.-t.p_enemy if t.friend else t.p_enemy,age=t.age(self.robot.now()))
                            for t in self._av_tracks],actions=[],transitions=[])

    def _mode_explore(self,now,dt,t):self._policy_step(now,dt)
    def _mode_stage(self,now,dt,t):self._policy_step(now,dt)
    def _mode_goal(self,now,dt,t):self._policy_step(now,dt)

    def _policy_tracks(self,now,enemy_only=False):
        rows=[]
        for tr in self._av_tracks:
            if tr.age(now)>1.5:
                continue
            if enemy_only and (tr.friend or tr.p_enemy<.8 or tr.evidence<3):
                continue
            rows.append(tr)
        p=self.stack.pose
        return sorted(rows,key=lambda tr:math.hypot(tr.x-p.x,tr.y-p.y))

    def _waypoint(self,now,dt,point):
        p=self.stack.pose
        x=clamp(point[0],.32,self.geo.W-.32)
        y=clamp(point[1],.32,self.geo.H-.32)
        self._policy_goal=(x,y)
        self._nav_to(now,dt,self._policy_goal,.18)
        return math.hypot(x-p.x,y-p.y)<.22

    def _patrol_points(self):
        own_left=self.team=='BLUE'
        x=self.geo.W*.26 if own_left else self.geo.W*.74
        return [(x,self.geo.H*.22),(x,self.geo.H*.50),(x,self.geo.H*.78)]

    def _blockade_point(self):
        lane=int(self.slot or 2)
        public=(self.geo.W/2,self.geo.base_cy[lane-1])
        grid=self.stack.grid
        blocked,_=grid.cost_maps(self.cfg.nav.inflate_hard,self.cfg.nav.inflate_soft,self.cfg.nav.soft_weight)
        if self._policy_goal is not None:
            iy,ix=grid.to_cell(*self._policy_goal)
            if grid.in_bounds(iy,ix) and not blocked[iy,ix]:
                return self._policy_goal
        choices=[]
        for dx in (-.6,-.4,-.2,0.,.2,.4,.6):
            for dy in (-.4,-.2,0.,.2,.4):
                point=(public[0]+dx,public[1]+dy)
                iy,ix=grid.to_cell(*point)
                if grid.in_bounds(iy,ix) and not blocked[iy,ix]:
                    choices.append((dx*dx+dy*dy,point))
        return min(choices)[1] if choices else public

    def _aim_enemy(self,now,dt,tr):
        """Return true after an actual camera/receiver-grounded aiming output."""
        if tr.friend or tr.p_enemy<.8 or tr.evidence<3 or tr.age(now)>1.5:
            if self.laser.firing():self.laser.abort()
            return False
        p=self.stack.pose
        distance=math.hypot(tr.x-p.x,tr.y-p.y)
        pan,tilt=self.robot.get_pan_tilt()
        bearing=wrap(math.atan2(tr.y-p.y,tr.x-p.x)-p.th)
        det=self._best_det(lambda d:d.cls=='UGV' and d.team==self.geo.opponent(self.team)
              and abs(wrap(self.cam.bearing(d.cx)+math.radians(pan)-bearing))<.18)
        if det is None or not self.vision_fresh(now):
            # Search with the body and camera; do not fire at an estimated point.
            if self.laser.firing():self.laser.abort()
            self._apply(0.,clamp(1.4*bearing,-.8,.8),clamp(math.degrees(bearing),-45.,45.),0.,dt,limit=True)
            return True
        pc,tc,res=self.aim.update(det,pan,tilt,now,distance)
        self._apply(0.,res.body_w,pc,tc,dt,limit=True)
        self.laser.track(res.aligned,now,dt)
        friendly=self._friend_in_line(det,now,pan)
        for other in self._av_tracks:
            if other.tid==tr.tid or (not other.friend and other.p_enemy>=.8):
                continue
            od=math.hypot(other.x-p.x,other.y-p.y)
            ob=wrap(math.atan2(other.y-p.y,other.x-p.x)-p.th)
            cone=math.radians(self.cfg.mission.friendly_fire_guard_deg)+math.atan2(self.cfg.avoid.r_body,max(od,.1))
            friendly |= od<distance+.2 and abs(wrap(ob-bearing))<cone
        safe=(not friendly and res.sensor_ok and self._may_fire(now)
              and self._fire_conf_ok(det) and self._shot_geometry_ok(det,now,pan,tilt))
        if not safe and self.laser.firing():
            self.laser.abort()
        if (safe and res.aligned and self.aim.held_for(now)>=self.cfg.aim.hold_before_fire
                and self.laser.ready() and now-self._policy_last_fire.get(tr.tid,-1e9)>=10.):
            if self.laser.fire(now):
                self._policy_last_fire[tr.tid]=now
                self.stats['policy_shots']=self.stats.get('policy_shots',0)+1
        return True

    def _policy_step(self,now,dt):
        p=self.stack.pose
        if self.policy=='idle':
            self._apply(0.,0.,0.,0.,dt,limit=True)
            return
        if self.policy=='patrol_own':
            points=self._patrol_points()
            if self._waypoint(now,dt,points[self._policy_patrol_index%len(points)]):
                self._policy_patrol_index+=1
            return
        if self.policy=='evader':
            tracks=self._policy_tracks(now)
            if not tracks or math.hypot(tracks[0].x-p.x,tracks[0].y-p.y)>=2.2:
                self._apply(0.,0.,0.,0.,dt,limit=True)
                return
            tr=tracks[0];d=max(.01,math.hypot(p.x-tr.x,p.y-tr.y))
            ux,uy=(p.x-tr.x)/d,(p.y-tr.y)/d
            self._waypoint(now,dt,(p.x+ux+.25*uy,p.y+uy-.25*ux))
            return
        enemies=self._policy_tracks(now,enemy_only=True)
        if not enemies and self.laser.firing():
            self.laser.abort()
        if self.policy=='blocker':
            point=self._blockade_point()
            if math.hypot(point[0]-p.x,point[1]-p.y)>.25:
                self._waypoint(now,dt,point)
            elif enemies and math.hypot(enemies[0].x-p.x,enemies[0].y-p.y)<=self.cfg.laser.max_range:
                self._aim_enemy(now,dt,enemies[0])
            else:
                self._apply(0.,0.,0.,0.,dt,limit=True)
            return
        # Hunter: chase only a confidently identified observed enemy.
        if enemies:
            tr=enemies[0];d=math.hypot(tr.x-p.x,tr.y-p.y)
            if d>1.2:
                ux,uy=(tr.x-p.x)/d,(tr.y-p.y)/d
                self._waypoint(now,dt,(tr.x-ux,tr.y-uy))
            else:
                self._aim_enemy(now,dt,tr)
        else:
            points=[(self.geo.W/2,self.geo.H*.22),(self.geo.W/2,self.geo.H*.78)]
            if self._waypoint(now,dt,points[self._policy_patrol_index%2]):
                self._policy_patrol_index+=1
