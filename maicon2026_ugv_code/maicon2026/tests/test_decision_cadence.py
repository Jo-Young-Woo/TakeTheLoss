"""Realtime policy and bounded route repair never throttle emergency stops."""
from pathlib import Path
from unittest.mock import patch
from types import SimpleNamespace
import pytest
from maicon26.config import Config
from maicon26.mission.manager import Mode
from maicon26.nav.navigator import Navigator
from maicon26.types import Pose2D
from test_deadlock_integration import agent
from test_risk_waypoint_validity import route_fixture


def test_profiles_use_realtime_policy_and_five_hz_route_repair():
    root=Path(__file__).resolve().parents[1]
    for path in (None,root/'config/default.json',root/'config/waveshare_real.json'):
        cfg=Config.load(str(path)) if path else Config()
        assert cfg.mission.decision_period_s == .05
        assert cfg.nav.replan_s == cfg.avoid.plan_period_s == .2
        assert cfg.loop_hz==20.
        assert cfg.risk.update_hz==5.


def test_mode_arbitration_is_periodic_but_control_and_unsafe_stop_are_not(monkeypatch):
    a=agent();a.t_start=0.;runs=[];decisions=[];stops=[]
    a.cfg.mission.decision_period_s=3.  # Explicit slow-policy override remains supported.
    monkeypatch.setattr(a,'_unsafe_reason',lambda *args:None)
    monkeypatch.setattr(a,'_progress_gate',lambda *args:False)
    monkeypatch.setattr(a,'_avoid_gate',lambda *args:False)
    monkeypatch.setattr(a.field_policy,'escape_goal',lambda *args:None)
    monkeypatch.setattr(a,'_continue_scoring_coverage',lambda *args:False)
    monkeypatch.setattr(a,'_arbitrate',lambda now,st:decisions.append(now) or Mode.EXPLORE)
    monkeypatch.setattr(a,'_run_mode',lambda now,*args:runs.append(now))
    times=[0.,.05,.1,2.99,3.,3.05,6.]
    try:
        for now in times:
            a._advance_decision_clock(now);a._decide(now,.05,now)
        assert decisions==[0.,3.,6.]
        assert runs==times
        monkeypatch.setattr(a,'_unsafe_reason',lambda *args:'fresh obstacle')
        monkeypatch.setattr(a,'_enter_safe',lambda *args:None)
        monkeypatch.setattr(a,'_stop_output',lambda:stops.append(True))
        a._advance_decision_clock(6.05);a._decide(6.05,.05,6.05)
        assert stops==[True] and decisions==[0.,3.,6.]
    finally:a.close_deadlock()


def test_invalid_interval_fails_explicitly():
    a=agent()
    try:
        for period in (0.,-1.,float('nan'),float('inf')):
            a.cfg.mission.decision_period_s=period
            with pytest.raises(ValueError):a._advance_decision_clock(1.)
    finally:a.close_deadlock()


def test_risk_planner_retries_failed_route_only_at_three_second_interval():
    a,goal,_,_=route_fixture(0.,obstacle=False);a._rc={}
    a.cfg.avoid.plan_period_s=3.
    try:
        with patch('maicon26.nav.risk_astar_realtime.plan',return_value=None) as planner:
            for now in (10.,10.05,11.,12.99):
                assert a._risk_carrot(now,goal,1.) is None
            assert planner.call_count==1
            a._risk_carrot(13.,goal,1.)
            assert planner.call_count==2
    finally:a.close_deadlock()


def test_no_path_builtin_navigator_waits_between_attempts_and_holds_zero():
    cfg=Config();cfg.progress.enabled=True
    cfg.nav.replan_s=3.
    nav=Navigator(cfg);nav.grid.static[:]=True
    nav.set_goal((5.,2.))
    with patch.object(nav,'_plan',wraps=nav._plan) as planner:
        for now in (0.,.05,1.,2.99):
            out=nav.step(Pose2D(1.,2.,0.),None,now,.05)
            assert out.v==out.w==0. and out.status=='blocked'
        assert planner.call_count==1
        nav.step(Pose2D(1.,2.,0.),None,3.,.05)
        assert planner.call_count==2


def test_hit_evidence_updates_between_posture_decisions(monkeypatch):
    a=agent();a._decision_due=False;a.posture_state=SimpleNamespace()
    status=a.robot.status();status.hits_taken=1
    monkeypatch.setattr(a.robot,'status',lambda:status)
    monkeypatch.setattr(a.posture_control,'update',lambda *args,**kw:pytest.fail('policy ran early'))
    try:
        a._deadlock_update(.05,.05,.05)
        assert a._seen_hit_count==1 and len(a._hit_history)==1
    finally:a.close_deadlock()
