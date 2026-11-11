"""Mission ownership must not turn recovery movement into fake progress."""
from types import SimpleNamespace

import pytest

from maicon26.config import Config
from maicon26.mission.manager import Mode
from maicon26.nav.stack import NavOut
from maicon26.sim.run import build_match


@pytest.fixture
def agent():
    cfg = Config()
    cfg.progress.enabled = True
    cfg.depth.enable = False
    cfg.map3d.enable = False
    match = build_match(cfg=cfg, teams=["B2"], seed=7)
    value = match.agents[0]
    yield value
    value.close_deadlock()


def test_reached_recovery_carrot_is_not_completed_mission_goal(agent, monkeypatch):
    agent._progress_goal = (1., 2.)
    monkeypatch.setattr(agent.stack, "set_goal", lambda *args: None)
    monkeypatch.setattr(agent.stack, "step", lambda *args: NavOut(status="reached"))
    monkeypatch.setattr(agent, "_apply", lambda *args, **kwargs: None)
    monkeypatch.setattr(agent, "_track_block", lambda *args: None)
    agent._nav_to(0., .05, (1.5, 2.5), .1, raw=True)
    assert agent._progress_goal == (1., 2.)
    assert not agent._progress_arrived
    agent._nav_to(.1, .05, (1., 2.), .1, raw=False)
    assert agent._progress_arrived


def test_empty_exploration_selection_accumulates_stall(agent, monkeypatch):
    agent.mode = Mode.EXPLORE
    agent._progress_goal = agent._cur_goal = agent._ex_cand = None
    monkeypatch.setattr(agent, "drive_locked", lambda now: False)
    monkeypatch.setattr(agent.laser, "firing", lambda: False)
    agent._deadlock_update(0., .05, 0.)
    agent._deadlock_update(6., .05, 6.)
    assert agent.progress_state.stage == 2
    assert agent.progress_state.stalled_s == 6.
    assert "engage" in agent.progress_state.actions
    assert agent.progress.goal == ("selection", "EXPLORE")


def test_exploration_candidate_owns_destination_over_old_carrot(agent):
    agent.mode = Mode.EXPLORE
    agent._progress_goal = (4., 1.)
    agent._cur_goal = (3., 1.)
    agent._ex_cand = SimpleNamespace(vp=(2., 2.))
    assert agent._mission_destination() == (2., 2.)


def test_observation_wait_is_bounded_and_prior_stall_not_erased(agent, monkeypatch):
    agent.mode = Mode.EXPLORE
    agent._ex_cand = SimpleNamespace(vp=(2.,2.), path=[])
    agent._ex_phase, agent._ex_obs_t0 = "observe", 0.
    monkeypatch.setattr(agent, "drive_locked", lambda now: False)
    monkeypatch.setattr(agent.laser, "firing", lambda: False)
    assert agent._normal_progress_wait(1.)
    assert not agent._normal_progress_wait(13.)
