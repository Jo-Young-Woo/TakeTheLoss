"""Policy integration contracts; no world pose is passed to the Agent."""
import math
from types import SimpleNamespace
import numpy as np
from maicon26.config import Config
from maicon26.mission.manager import Mode
from maicon26.mission.posture import Posture, PostureState
from maicon26.perception.threat_board import ThreatTrack, EV_OBJECT
from maicon26.sim.run import build_match
from maicon26.nav.stack import NavOut
from maicon26.types import Pose2D, Scan


def agent():
    cfg=Config()
    cfg.progress.enabled=True
    cfg.risk.threaded=False
    cfg.depth.enable=cfg.map3d.enable=False
    match=build_match(3,cfg,teams=['B2'],m5_ends_match=False)
    a=match.agents[0]
    assert a._sense(0.,.05)
    a.mode=Mode.EXPLORE
    return a


def test_no_candidate_selection_stall_is_visible_and_replans():
    a=agent()
    a._ex_cand=None
    a._progress_goal=a._cur_goal=None
    for now in (0.,1.,2.,3.,4.,5.,6.):
        a._deadlock_update(now,1.,now)
    assert a.progress_state.stage==2
    assert a.deadlock_telemetry()['goal_key']==('selection','EXPLORE')
    assert a.stats['progress_replans']==1
    assert a.posture_state.posture==Posture.ENGAGE_OK


def test_raw_avoidance_carrot_does_not_replace_mission_progress_goal():
    a=agent()
    mission=(4.,2.)
    a._progress_goal=mission
    a.stack.step=lambda *args:NavOut(0.,0.,'moving',1.)
    a._nav_to(1.,.05,(2.,1.),.2,raw=True)
    assert a._mission_destination()==mission


def test_minimum_clearance_is_enforced_after_acceleration_ramp():
    a=agent()
    p=a.stack.pose
    p.th=0.
    # Put the observed body .45m ahead; edge clearance is below .30m.
    a._av_tracks=[ThreatTrack(7,p.x+.45,p.y,0.,0.,.02,EV_OBJECT,False,.5,0.)]
    a._v_cmd=.35
    sent=[]
    a.robot.set_velocity=lambda v,w:sent.append((v,w))
    a._apply(.4,0.,0.,0.,.05)
    assert sent[-1][0]==0.
    assert a._v_cmd==0.


def test_engage_and_low_hit_avoid_do_not_delegate_to_legacy_retreat():
    a=agent()
    a._av_ctx=SimpleNamespace()
    a.avoid.step=lambda *args:(_ for _ in ()).throw(AssertionError('retreat took ownership'))
    for posture in (Posture.ENGAGE_OK,Posture.AVOID):
        a.posture_state=PostureState(posture=posture,level=2,allow_evade=False)
        assert not a._avoid_gate(1.,.05,1.)


def test_exposure_dwell_resets_during_travel_and_defers_observation():
    a=agent()
    a.posture_state=PostureState(posture=Posture.ENGAGE_OK,cap_exposed_dwell=True)
    a._decision_snapshot={'tracks':[{'los':True}]}
    assert not a._dwell_expired(0.)
    assert a._dwell_expired(1.5)
    a._v_cmd=.1
    assert a._dwell_expired(2.), "Commanded motion alone cannot clear actual exposed dwell"
    a._measured_speed=.1
    assert not a._dwell_expired(2.)
    a._measured_speed=0.
    a._v_cmd=0.
    assert not a._dwell_expired(3.)
    assert not a._dwell_expired(4.)
    assert a._dwell_expired(4.5)


def test_hit_history_is_soft_cost_xy_timestamp_and_not_a_new_keepout():
    a=agent()
    old=a.stack.grid.keepout
    a.robot.status=lambda:SimpleNamespace(hits_taken=2,disabled_left=0.,drive_paused_left=0.,laser_on=False)
    a._deadlock_update(8.,.05,8.)
    hx,hy,stamp=a._hit_history[-1]
    assert (hx,hy)==(a.stack.pose.x,a.stack.pose.y) and stamp==8.
    assert a.stack.grid.keepout is old


def test_three_hit_goal_last_metre_exception_is_kept_by_integration():
    a=agent()
    p=a.stack.pose
    a.mode=Mode.GOAL
    a._goal_xy=lambda:(p.x+.5,p.y)
    a.robot.status=lambda:SimpleNamespace(hits_taken=3,disabled_left=0.,drive_paused_left=0.,laser_on=False)
    a._deadlock_update(241.,.05,241.)
    assert a.posture_state.posture==Posture.ENGAGE_OK


def test_hard_safety_still_handles_box_during_progress_backoff():
    a=agent()
    a._scan=Scan(-math.pi,2*math.pi/360,np.full(360,.13),0.,8.)
    a._progress_backoff=(0.,.6)
    commands=[]
    a.robot.set_velocity=lambda v,w:commands.append((v,w))
    assert a._progress_gate(.1,.05)
    assert commands[-1][0]==0.

def test_detector_shutdown_waits_for_native_inference_to_return():
    import threading
    import time
    from maicon26.perception.detector import AsyncDetector, Detector
    from maicon26.types import Frame
    entered, release, closed = threading.Event(), threading.Event(), threading.Event()
    class Controlled(Detector):
        def detect(self, frame):
            entered.set()
            assert release.wait(2.)
            return []
    detector=AsyncDetector(Controlled())
    detector.detect(Frame(None,1.))
    assert entered.wait(1.)
    waiter=threading.Thread(target=lambda:(detector.close(),closed.set()))
    waiter.start()
    assert not closed.wait(.03)
    release.set()
    waiter.join(1.)
    assert closed.is_set() and not detector._th.is_alive()

def test_encounter_prefers_right_of_mission_direction_with_finite_costs():
    a=agent()
    p,g=a.stack.pose,a.stack.grid
    a._av_tracks=[ThreatTrack(9,p.x+1.,p.y,0.,0.,.02,EV_OBJECT,False,.5,0.)]
    original=np.ones_like(g.L)
    cost,active=a._right_hand_cost((p.x+2.,p.y),original,0.)
    assert active and np.isfinite(cost).all()
    assert cost[g.to_cell(p.x+1.,p.y-.4)] < cost[g.to_cell(p.x+1.,p.y+.4)]
    assert np.array_equal(original,np.ones_like(g.L))
    # The same convention reverses geographic sides for westbound traffic.
    a._av_tracks=[ThreatTrack(10,p.x-.2,p.y,0.,0.,.02,EV_OBJECT,False,.5,0.)]
    cost,active=a._right_hand_cost((p.x-2.,p.y),original,0.)
    assert active
    assert cost[g.to_cell(p.x-.2,p.y+.4)] < cost[g.to_cell(p.x-.2,p.y-.4)]


def test_unknown_shortcut_is_costly_but_still_available_for_exploration():
    from maicon26.nav import risk_astar
    a=agent()
    g=a.stack.grid
    g.L[:]=0.
    # A longer observed corridor competes with a short occluded shortcut.
    for x in np.arange(1.,3.01,g.res):
        g.L[g.to_cell(x,2.)]=-2.
    for y in np.arange(1.,2.01,g.res):
        g.L[g.to_cell(1.,y)]=-2.
        g.L[g.to_cell(3.,y)]=-2.
    blocked=np.zeros_like(g.L,dtype=bool)
    cost=a._observed_space_cost(np.ones_like(g.L))
    result=risk_astar.plan(g,blocked,cost,(1.,1.),(3.,1.))
    assert result and max(y for x,y in result.path)>1.8
    # No observed passage is a reason to pay the cost, not fail A*.
    g.L[:]=0.
    cost=a._observed_space_cost(np.ones_like(g.L))
    assert np.isfinite(cost).all()
    assert risk_astar.plan(g,blocked,cost,(1.,1.),(3.,1.)) is not None


def test_route_hysteresis_cannot_retain_a_newly_observed_wall_path():
    a=agent()
    g=a.stack.grid
    blocked=np.zeros_like(g.L,dtype=bool)
    a._av_ctx=SimpleNamespace(blocked=blocked)
    a._field=None
    path=[(1.,1.),(1.2,1.),(1.4,1.)]
    cost=np.ones_like(g.L)
    assert math.isfinite(a._path_cost(path,.3,1.,cost))
    blocked[g.to_cell(1.2,1.)]=True
    assert math.isinf(a._path_cost(path,.3,1.,cost))


def test_late_camera_result_does_not_refresh_old_firing_evidence():
    a=agent()
    a._dets_t=10.
    a._det_ctx=(8.,a.stack.pose,0.,0.)
    assert not a.vision_fresh(10.)
    a._det_ctx=(9.95,a.stack.pose,0.,0.)
    assert a.vision_fresh(10.)
    a._det_ctx=(10.2,a.stack.pose,0.,0.)
    assert not a.vision_fresh(10.)
    a._det_ctx=(float('nan'),a.stack.pose,0.,0.)
    assert not a.vision_fresh(10.)


def test_lidar_wall_stop_has_authority_after_deceleration_ramp():
    a=agent()
    a._scan=Scan(-math.pi,2*math.pi/360,np.full(360,.13),0.,8.)
    a._v_cmd=.3
    commands=[]
    a.robot.set_velocity=lambda v,w:commands.append((v,w))
    a._apply(0.,0.,0.,0.,.05,limit=True)
    assert commands[-1][0]==0.
    assert a._v_cmd==0.


def blocker_agent(sensor_locked=True):
    a=agent()
    a.cfg.engage.fire_on_blocker=True
    a.posture_state=PostureState(posture=Posture.ENGAGE_OK)
    a.progress_state=SimpleNamespace(stage=2)
    p=a.stack.pose
    a._av_tracks=[ThreatTrack(8,p.x+1.,p.y,0.,0.,.02,3,False,.95,10.)]
    a._is_path_blocker=lambda tr:True
    a._dwell_expired=lambda now:False
    a.vision_fresh=lambda now:True
    a._best_det=lambda pred:SimpleNamespace(cls='UGV',team='RED')
    a._friend_in_line=lambda *args:False
    a._may_fire=lambda *args:True
    a._fire_conf_ok=lambda *args:True
    a._shot_geometry_ok=lambda *args:True
    a.aim=SimpleNamespace(update=lambda *args:(0.,0.,SimpleNamespace(
        sensor_ok=True,sensor_locked=sensor_locked,aligned=True,body_w=0.)),
        held_for=lambda now:10.)
    a._apply=lambda *args,**kwargs:None
    events=[]
    a.laser=SimpleNamespace(ammo=8,shots=0,firing=lambda:False,
        abort=lambda:events.append('abort'),ready=lambda:True,
        track=lambda *args:events.append('track'),
        fire=lambda now:events.append('fire') or True)
    return a,events


def test_blocker_fire_requires_observed_dome_not_bbox_fallback():
    a,events=blocker_agent(sensor_locked=False)
    assert not a._blocker_gate(10.,.05)
    assert 'fire' not in events
    a,events=blocker_agent(sensor_locked=True)
    assert a._blocker_gate(10.,.05)
    assert events==['track','fire']


def test_authorized_beam_survives_shot_progress_but_aborts_on_lost_blocker():
    a,events=blocker_agent()
    a._blocker_active=8
    a.progress_state=SimpleNamespace(stage=0)  # Shot itself is real progress.
    a.laser.firing=lambda:True
    assert a._blocker_gate(10.,.05)
    assert events==['track']  # Keep alignment for 2s, never issue another shot.
    a._av_tracks=[]
    assert not a._blocker_gate(10.1,.05)
    assert events==['track','abort']


def test_blocker_aim_does_not_extend_exposed_dwell_before_emission():
    a,events=blocker_agent()
    a._dwell_expired=lambda now:True
    assert not a._blocker_gate(10.,.05)
    assert not events


def test_distant_cm_waypoint_does_not_enable_engage_before_actual_entry():
    a=agent()
    a.stack.pose.x,a.stack.pose.y=5.56,4.16
    assert a.geo.sector_of(a.stack.pose.x,a.stack.pose.y) not in ('CM','COMMON')
    a._ex_cand=SimpleNamespace(vp=(a.geo.W/2,a.geo.H/2),path=[])
    a._deadlock_update(0.,.05,0.)
    assert a.posture_state.posture==Posture.AVOID
    a.stack.pose.x,a.stack.pose.y=a.geo.W/2,a.geo.H/2
    a._deadlock_update(.1,.05,.1)
    assert a.posture_state.posture==Posture.ENGAGE_OK
    assert 'cm' in a.posture_state.engage_reasons


def test_stale_risk_path_does_not_hide_valid_exploration_or_navigator_detour():
    a=agent()
    a.stack.pose.x,a.stack.pose.y,a.stack.pose.th=0.,0.,0.
    goal=(2.,0.);detour=[(0.,0.),(0.,2.),(2.,2.),goal]
    a._rc={'path':[(0.,0.),(1.,1.)]}
    a._ex_cand=SimpleNamespace(path=detour)
    assert a._remaining_path_route(goal)[0]=='exploration'
    assert a._remaining_path(goal)==6.
    a._rc={}
    assert a._remaining_path(goal)==6., 'Cache expiry alone is not progress'
    a._rc={'path':[(0.,0.),(1.,1.)]}
    a._ex_cand=SimpleNamespace(path=[(0.,0.),(1.,2.)])
    a.stack.navigator=SimpleNamespace(path=detour)
    assert a._remaining_path_route(goal)[0]=='navigator'
    assert a._remaining_path(goal)==6.
    a.stack.navigator.path=[]
    assert a._remaining_path_route(goal)[0]=='euclidean'
    assert a._remaining_path(goal)==2.
    a.close_deadlock()
