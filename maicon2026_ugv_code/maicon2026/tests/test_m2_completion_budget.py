"""An already committed M2 may finish only inside a measured staging budget."""
from types import SimpleNamespace

import pytest

from maicon26.link.base import ServerMsg
from maicon26.mission.manager import Mode
from maicon26.nav.grid import OccupancyGrid
from maicon26.types import Detection, Pose2D
from test_m2_target_scope import agent_fixture, face_at
from test_state_machine import _solo


def active_target():
    agent = agent_fixture()
    agent.mode, agent.m5_type = Mode.STRIKE, None
    agent.latch, agent.halt_t = SimpleNamespace(stopped=False), None
    agent.stack = SimpleNamespace(pose=Pose2D(4., 2., 0.), grid=OccupancyGrid(agent.geo, .05))
    agent._strike.update(face=face_at(4.8, 2.), phase="aim", goal=(4., 2.), t0=220.)
    agent.robot.get_pan_tilt = lambda: (0., 0.)
    agent.laser.ready = lambda: True
    agent._dets = []
    return agent


@pytest.mark.spec("S6.3", "S9.8")
def test_completion_reserves_entire_emission_and_return_path_with_fixed_deadline():
    a = active_target()
    assert a._m2_can_finish_before_stage(222., 222.)
    budget = a._strike["stage_finish_budget"]
    assert budget["shot_s"] >= a.cfg.laser.shot_s >= a.cfg.laser.dwell_s
    assert budget["aim_s"] >= a.cfg.aim.sensor_fallback_s+a.cfg.aim.hold_before_fire
    assert budget["return_s"] > 0
    deadline = a._strike["stage_finish_by"]
    assert deadline+budget["return_s"] == pytest.approx(a.cfg.mission.t_base_cmd-a.cfg.explore.stage_margin_s)
    # A shorter newly measured return route cannot move the original cutoff.
    a.stack.pose = Pose2D(4.1, 2.45, 0.)
    a._m2_can_finish_before_stage(223., 223.)
    assert a._strike["stage_finish_by"] <= deadline
    # Previously budgeted aiming time is consumed, not charged again each tick.
    last_launch = a._strike["stage_finish_by"]-a.cfg.laser.shot_s
    assert a._m2_can_finish_before_stage(last_launch-.01, last_launch-.01)
    assert not a._m2_can_finish_before_stage(deadline-a.cfg.laser.shot_s+.01,
                                            deadline-a.cfg.laser.shot_s+.01)


@pytest.mark.spec("S6.3", "S9.8")
@pytest.mark.parametrize("reason", ["late_start", "wrong_camp", "no_ammo", "done", "failed", "m5", "stop", "halt", "not_strike"])
def test_completion_does_not_open_new_or_invalid_engagements(reason):
    a = active_target()
    if reason == "late_start":
        a._strike["t0"] = 222.
    elif reason == "wrong_camp":
        a._strike["face"] = face_at(1.5, 2.)
    elif reason == "no_ammo":
        a.laser.ready = lambda: False
    elif reason in ("done", "failed"):
        setattr(a, "target_"+reason, True)
    elif reason == "m5":
        a.m5_type = "COMMAND_POST"
    elif reason == "stop":
        a.latch.stopped = True
    elif reason == "halt":
        a.halt_t = 221.
    else:
        a.mode = Mode.EXPLORE
    assert not a._m2_can_finish_before_stage(222., 222.)


@pytest.mark.spec("S6.3", "S9.8")
def test_no_return_path_or_passed_match_deadline_disallows_completion():
    a = active_target()
    row, _ = a.stack.grid.to_cell(4., 2.25)
    a.stack.grid.L[row, :] = 4.  # Full wall separates target from staging.
    assert not a._m2_can_finish_before_stage(222., 222.)
    assert not a._m2_can_finish_before_stage(240., 240.)
    assert not a._m2_can_finish_before_stage(300., 300.)


@pytest.mark.spec("S6.3", "S9.8")
def test_last_fire_gate_cannot_start_a_shot_that_overruns_departure(monkeypatch):
    a = active_target()
    a._strike.update(phase="aim", t_aim0=222., lost_since=None, stage_finish_by=227.-.01)
    face = a._strike["face"]
    det = Detection("COMM_FACILITY", .99, (300, 200, 340, 300))
    shots = []
    result = SimpleNamespace(err_pan=0., err_tilt=0., aligned=True, sensor_ok=True, body_w=0.)
    a.aim = SimpleNamespace(update=lambda *args: (0., 0., result), held_for=lambda now: 1.)
    a.laser.firing, a.laser.track = lambda: False, lambda *args: None
    a.laser.fire = lambda now: shots.append(now) or True
    a.stats["strike_shots"] = 0
    monkeypatch.setattr(a, "_target_faces", lambda: [face])
    monkeypatch.setattr(a, "_best_det", lambda predicate: det)
    monkeypatch.setattr(a, "_range", lambda *args: (1., "lidar"))
    monkeypatch.setattr(a, "_strike_incidence_ok", lambda *args: True)
    monkeypatch.setattr(a, "_apply", lambda *args, **kwargs: None)
    monkeypatch.setattr(a, "_may_fire", lambda *args: True)
    monkeypatch.setattr(a, "_fire_conf_ok", lambda *args: True)
    monkeypatch.setattr(a, "_wrong_target_in_line", lambda *args: False)
    monkeypatch.setattr(a, "_shot_geometry_ok", lambda *args: True)
    a._mode_strike(222., .05, 222.)
    assert shots == []
    a._strike["stage_finish_by"] = 227.+.01
    a._mode_strike(222., .05, 222.)
    assert shots == [222.]


@pytest.mark.spec("S6.3", "S9.8")
def test_posted_m5_preempts_even_a_hypothetically_feasible_m2(monkeypatch):
    _, a, _ = _solo()
    a.team, a.slot, a.t_start, a.mode = "BLUE", 2, 0., Mode.STRIKE
    a.m5_type = "COMMAND_POST"
    calls = []
    monkeypatch.setattr(a, "_unsafe_reason", lambda *args: None)
    monkeypatch.setattr(a, "_avoid_gate", lambda *args: False)
    monkeypatch.setattr(a, "_mode_goal", lambda *args: calls.append("goal"))
    monkeypatch.setattr(a, "_m2_can_finish_before_stage", lambda *args: pytest.fail("M5 must win before M2 budget"))
    a._decide(240., .05, 240.)
    assert a.mode == Mode.GOAL and calls == ["goal"]


@pytest.mark.spec("S6.3", "S10.2", "S11.3")
@pytest.mark.parametrize("server_stop", [True, False])
def test_stop_and_local_deadline_preempt_completion_before_sensors(monkeypatch, server_stop):
    match, a, server = _solo()
    a.team, a.slot, a.t_start, a.mode = "BLUE", 2, 0., Mode.STRIKE
    a.latch.started = True
    a._strike["stage_finish_by"] = 1e6  # Cannot override organizer/local termination.
    match.world.t = 223. if server_stop else 302.
    messages = [ServerMsg("stop")] if server_stop else []
    server.poll = lambda: [messages.pop()] if messages else []
    monkeypatch.setattr(a, "_sense", lambda *args: pytest.fail("STOP precedes all sensing and M2"))
    a.tick()
    assert a.mode in (Mode.REPORT, Mode.DONE) and a.m6_submitted
    assert a.robot.u.cmd_v == a.robot.u.cmd_w == 0.
