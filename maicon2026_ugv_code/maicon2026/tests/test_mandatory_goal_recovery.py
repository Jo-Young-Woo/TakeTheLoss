"""Failed physical approaches never invalidate the assigned M5 identity."""
import math
from types import SimpleNamespace
import pytest
from test_deadlock_integration import agent
from maicon26.mission.manager import Mode
from maicon26.perception.coverage import BoxModel,FaceRec


def clear_observed(a):
    a.stack.grid.L[:]=a.stack.grid.L_MIN
    a._av_tracks=[]
    a.drive_locked=lambda now:False
    a.laser.firing=lambda:False


@pytest.mark.parametrize("mode",[Mode.STAGE,Mode.GOAL])
def test_mandatory_abandon_changes_approach_and_restarts_episode(mode):
    a=agent();clear_observed(a);a.mode=mode;a.m5_base="R-2"
    canonical=a._mission_destination()
    a._deadlock_update(0.,.05,0.)
    a._deadlock_update(13.,.05,13.)
    chosen=a._mission_destination()
    assert chosen is not None and a._goal_key(chosen)!=a._goal_key(canonical)
    assert not a.progress.eligible(a._goal_key(canonical),13.)
    assert a.progress.eligible(a._goal_key(chosen),13.)
    assert a.m5_base=="R-2" and a.mode==mode
    if mode==Mode.GOAL:
        assert a.geo.in_base(*chosen,"R-2",margin=-.12)
    a._deadlock_update(14.,.05,14.)
    assert a.progress_state.stage==0 and a.progress_state.actions==()
    a.close_deadlock()


def test_goal_mode_uses_alternate_entry_in_same_assigned_base():
    a=agent();clear_observed(a);a.mode=Mode.GOAL;a.m5_base="R-2"
    goal=a.geo.base_center("R-2")
    a.progress.retry_at[a._goal_key(goal)]=40.
    sent=[];a._nav_to=lambda now,dt,destination,*args,**kwargs:sent.append(destination)
    a._mode_goal(13.,.05,250.)
    assert sent and sent[-1]!=goal
    assert a.geo.in_base(*sent[-1],"R-2",margin=-.12) and a.m5_base=="R-2"
    a.close_deadlock()


def test_unknown_or_disconnected_space_cannot_become_a_recovery_bypass():
    a=agent();a.mode=Mode.GOAL;a.m5_base="R-2"
    a.stack.grid.L[:]=0.
    original=a.geo.base_center("R-2")
    a.progress.retry_at[a._goal_key(original)]=43.
    assert a._mandatory_navigation_goal(13.,original) is None
    assert a._mission_destination() is None
    # A later actual free-space observation permits a bounded recheck.
    clear_observed(a)
    assert a._mandatory_navigation_goal(14.,original) is not None
    a.close_deadlock()


def test_cooldown_and_permanent_exclusion_are_respected_by_mandatory_resolver():
    a=agent();clear_observed(a);a.mode=Mode.GOAL;a.m5_base="R-2"
    original=a.geo.base_center("R-2");key=a._goal_key(original)
    a.progress.retry_at[key]=43.
    assert a._goal_key(a._mandatory_navigation_goal(42.,original))!=key
    a._mandatory_recovery=None
    assert a._mandatory_navigation_goal(43.,original)==original
    a.progress.excluded.add(key);a._mandatory_recovery=None
    assert a._goal_key(a._mandatory_navigation_goal(100.,original))!=key
    assert a.m5_base=="R-2"
    a.close_deadlock()


def test_observed_wall_disconnects_alternate_base_approaches():
    a=agent();clear_observed(a);a.mode=Mode.GOAL;a.m5_base="R-2"
    grid=a.stack.grid
    grid.add_static_box((2.,0.,2.3,a.geo.H))
    original=a.geo.base_center("R-2")
    a.progress.excluded.add(a._goal_key(original))
    assert a._mandatory_navigation_goal(13.,original) is None
    a.close_deadlock()


@pytest.mark.parametrize('ally',[True,False,None])
def test_occupied_stage_chooses_stable_observed_alternative_without_waiting_for_abandon(ally):
    from maicon26.perception.threat_board import ThreatTrack,EV_OBJECT
    a=agent();clear_observed(a);a.mode=Mode.STAGE;a.m5_base='R-2'
    canonical=a._stage_point()
    a._av_tracks=[ThreatTrack(7,*canonical,0.,0.,.01,EV_OBJECT,ally,.9,10.)]
    chosen=a._mandatory_navigation_goal(10.,canonical)
    assert chosen is not None and chosen!=canonical
    radius=math.hypot(a.cfg.robot.length,a.cfg.robot.width)/2
    assert math.dist(chosen,canonical)>=radius+max(radius,a.cfg.avoid.r_body)+.30
    assert a.progress.eligible(a._goal_key(canonical),10.)
    assert a.m5_base=='R-2' and a.m5_entered_local is None
    a._av_tracks=[]
    assert a._mandatory_navigation_goal(12.,canonical)==chosen
    a.stack.pose.x,a.stack.pose.y=chosen
    assert a._normal_progress_wait(12.)
    assert a.stats['occupied_stage_alternatives']==1
    a.close_deadlock()


def test_stage_avoids_observed_structure_but_never_invents_unknown_bypass():
    a=agent();clear_observed(a);a.mode=Mode.STAGE
    canonical=a._stage_point();x,y=canonical
    a.stack.grid.add_static_box((x-.1,y-.1,x+.1,y+.1))
    chosen=a._mandatory_navigation_goal(10.,canonical)
    assert chosen is not None and chosen!=canonical
    blocked,_=a.stack.grid.cost_maps(a.cfg.nav.inflate_hard,a.cfg.nav.inflate_soft,0.)
    assert not blocked[a.stack.grid.to_cell(*chosen)]
    a._mandatory_recovery=None
    a.stack.grid.L[:]=0.
    assert a._mandatory_navigation_goal(11.,canonical) is None
    assert not a._normal_progress_wait(11.)
    a.close_deadlock()


def test_free_stage_point_and_expired_track_do_not_create_alternative():
    from maicon26.perception.threat_board import ThreatTrack,EV_OBJECT
    a=agent();clear_observed(a);a.mode=Mode.STAGE
    canonical=a._stage_point()
    assert a._mandatory_navigation_goal(10.,canonical)==canonical
    a._av_tracks=[ThreatTrack(7,*canonical,0.,0.,.01,EV_OBJECT,True,.9,8.)]
    assert a._mandatory_navigation_goal(10.,canonical)==canonical
    assert a._mandatory_recovery is None
    a.close_deadlock()


def box(bid,x,y,views=0,size=.4):
    value=BoxModel(bid,x,y,"full",[],0.,0.,sx=size,sy=.4)
    value.faces=[FaceRec(bid,0,1.,0.,x+size/2,y,views=views)]
    return value


def test_progress_faces_only_include_available_m3_candidates_in_scoring_sectors():
    a=agent();clear_observed(a)
    own=box(1,1.4,2.,views=0)
    opponent=box(2,4.5,2.,views=0)
    common=box(3,a.geo.W/2,2.,views=1)
    obstacle=box(4,4.5,3.,views=1,size=1.2)
    a.coverage.boxes={b.bid:b for b in (own,opponent,common,obstacle)}
    assert {f.bid for f in a._scoring_faces()}=={2,3}
    a._progress_goal=(2.,2.)
    a._deadlock_update(0.,.05,0.)
    a._deadlock_update(3.,.05,3.)
    own.faces[0].views=1
    a._deadlock_update(4.,.05,4.)
    assert not a.progress_state.progressed and a.progress_state.stalled_s==4.
    opponent.faces[0].views=1
    a._deadlock_update(5.,.05,5.)
    assert a.progress_state.progressed and a.progress_state.reason=="new_face"
    assert a.deadlock_telemetry()["observed_faces"]==2
    a.close_deadlock()


def test_optional_time_pressure_ignores_unseen_own_or_long_structure_faces():
    a=agent();clear_observed(a);a.cfg.posture.engage_time_pressure_at=180.
    own=box(1,1.4,2.);obstacle=box(2,4.5,3.,size=1.2)
    a.coverage.boxes={1:own,2:obstacle}
    a._deadlock_update(0.,.05,190.)
    assert "time_pressure" not in a.posture_state.engage_reasons
    a.coverage.boxes[3]=box(3,4.5,2.)
    a._deadlock_update(1.,.05,191.)
    assert "time_pressure" in a.posture_state.engage_reasons
    a.close_deadlock()


def test_strike_approach_skips_failed_position_and_never_falls_back_to_excluded_goal():
    a=agent();clear_observed(a)
    face=SimpleNamespace(x=4.,y=2.5,vx=-1.,vy=0.,led_on=False)
    first,_=a._strike_goal(face,a.stack.pose)
    assert first is not None
    a.progress.excluded.add(a._goal_key(first))
    second,_=a._strike_goal(face,a.stack.pose)
    assert second is not None and a._goal_key(second)!=a._goal_key(first)
    a.progress.eligible=lambda key,now:False
    assert a._strike_goal(face,a.stack.pose)[0] is None
    a.mode=Mode.STRIKE;a.cmd=SimpleNamespace(target="tank")
    a._target_faces=lambda:[face]
    a._mode_strike(13.,.05,13.)
    assert a.mode==Mode.EXPLORE and not a.target_done and not a.target_failed
    a.close_deadlock()

@pytest.mark.parametrize('heading',[0.,math.pi/4,math.pi/2,3*math.pi/4])
def test_diagonal_other_body_corners_keep_actual_point_three_edge_clearance(heading):
    from maicon26.perception.threat_board import ThreatTrack,EV_OBJECT
    a=agent();clear_observed(a);p=a.stack.pose;p.th=heading
    radius=math.hypot(a.cfg.robot.length,a.cfg.robot.width)/2
    distance=2*radius+.29
    a._av_tracks=[ThreatTrack(7,p.x+distance*math.cos(heading),p.y+distance*math.sin(heading),
                             0.,0.,.01,EV_OBJECT,False,.5,a.robot.now())]
    assert a._dynamic_clearance(.3,0.,.05)[0]==0.
    a.close_deadlock()
