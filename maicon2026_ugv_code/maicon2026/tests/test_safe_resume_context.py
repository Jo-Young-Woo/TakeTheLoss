"""Safety recovery keeps current mission ownership without restoring stale aim."""
import numpy as np
import pytest

from maicon26.config import Config
from maicon26.link.base import ServerMsg
from maicon26.mission.avoid import AvoidCtx, Level
from maicon26.mission.manager import Mode
from maicon26.perception.threat_board import EV_ENEMY, ThreatTrack
from maicon26.types import Pose2D
from test_state_machine import _solo


def _paused(previous, stopped_at, monkeypatch):
    cfg = Config()
    cfg.depth.enable = False
    match, agent, server = _solo(cfg=cfg)
    agent.team, agent.slot, agent.t_start = "BLUE", 2, 0.
    agent.latch.started = True
    agent.stack.pe.pose = Pose2D(3., 2.5, 0.)
    agent.mode = previous
    agent._enter_safe(stopped_at, "drive_locked")
    monkeypatch.setattr(agent, "_unsafe_reason", lambda now, status: None)
    calls = []
    monkeypatch.setattr(agent, "_mode_goal", lambda *args: calls.append(Mode.GOAL))
    monkeypatch.setattr(agent, "_mode_stage", lambda *args: calls.append(Mode.STAGE))
    monkeypatch.setattr(agent, "_run_mode", lambda *args: calls.append(agent.mode))
    monkeypatch.setattr(agent, "_arbitrate", lambda *args: Mode.EXPLORE)
    return match, agent, server, calls


def _fresh(agent, now):
    for index in range(agent.cfg.mission.safe_resume_scans):
        agent.sw.on_scan(now+index*.01, now)


@pytest.mark.spec("S8.3", "S9.8")
def test_goal_recovers_before_close_threat_takes_ownership_and_still_requires_fresh_scans(monkeypatch):
    _, agent, _, calls = _paused(Mode.GOAL, 245., monkeypatch)
    agent.m5_type, agent.m5_base = "COMMAND_POST", "R-2"
    # This context was prepared while mode was SAFE_STOP in this same tick.
    now, grid = 255., agent.stack.grid
    track = ThreatTrack(1, 3.7, 2.5, 0., 0., .08, EV_ENEMY, False, .95, now)
    agent._av_ctx = AvoidCtx(now, .05, now, agent.stack.pose, 0., 0., None, grid,
                            np.zeros(grid.L.shape, bool), np.ones(grid.L.shape),
                            None, [track], .3, slack_s=20., in_goal=False)
    agent.avoid.assess(agent._av_ctx)
    assert agent.avoid.effective >= Level.EVADE
    agent._decide(now, .05, now)
    assert agent.mode == Mode.SAFE_STOP and calls == []
    assert agent.robot.u.cmd_v == 0. and agent.robot.u.cmd_w == 0.
    _fresh(agent, now)
    agent._decide(now, .05, now)
    assert agent.mode == Mode.GOAL and calls == [Mode.GOAL]
    assert agent._av_ctx.in_goal and agent.avoid.stats["commit_skips"] == 1
    assert agent.stats["avoid_ticks"] == 0
    assert agent._safe["from"] == Mode.GOAL


@pytest.mark.spec("S9.8", "S10.3")
@pytest.mark.parametrize("previous,stopped_at,now,post_m5,expected", [
    (Mode.GOAL, 245., 255., True, Mode.GOAL),
    (Mode.STRIKE, 238., 241., True, Mode.GOAL),
    (Mode.EXPLORE, 221., 223., False, Mode.STAGE),
    (Mode.STRIKE, 50., 60., False, Mode.EXPLORE),
    (Mode.GOAL, 50., 60., False, Mode.EXPLORE),
])
def test_recovery_revalidates_current_posting_and_deadline_instead_of_stale_subtask(
        monkeypatch, previous, stopped_at, now, post_m5, expected):
    _, agent, server, calls = _paused(previous, stopped_at, monkeypatch)
    if post_m5:
        messages = [ServerMsg("base_command", data={"objective": "RED_COMMAND_POST"})]
        server.poll = lambda: [messages.pop()] if messages else []
        agent._serve(now)
    # A different safety reason while already stopped does not erase origin.
    agent._enter_safe(stopped_at+1., "tilt")
    assert agent._safe["from"] == previous
    _fresh(agent, now)
    agent._decide(now, .05, now)
    assert agent.mode == expected and calls == [expected]
    assert not agent.latch.stopped and not agent.m6_submitted


@pytest.mark.spec("S0.10", "S10.2", "S11.3")
@pytest.mark.parametrize("server_stop", [True, False])
def test_reporting_or_local_deadline_during_safety_stop_never_resumes_driving(monkeypatch, server_stop):
    match, agent, server, calls = _paused(Mode.GOAL, 245., monkeypatch)
    agent.m5_type, agent.m5_base = "COMMAND_POST", "R-2"
    match.world.t = 256. if server_stop else agent.cfg.mission.t_total+agent.cfg.mission.local_stop_grace_s+1.
    messages = [ServerMsg("stop")] if server_stop else []
    server.poll = lambda: [messages.pop()] if messages else []
    monkeypatch.setattr(agent, "_sense", lambda *args: pytest.fail("report must precede new sensors"))
    agent.tick()
    assert agent.mode in (Mode.REPORT, Mode.DONE) and agent.m6_submitted
    assert calls == [] and agent.robot.u.cmd_v == 0. and agent.robot.u.cmd_w == 0.
