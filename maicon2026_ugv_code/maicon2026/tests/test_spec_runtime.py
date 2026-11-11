"""Submission timing and restart-clock conformance without ROS or real HTTP."""
import json
import math
import dataclasses
from types import SimpleNamespace

import pytest

from maicon26.config import Config
from maicon26.link.base import ServerMsg
from maicon26.mission.manager import Mode
from maicon26.types import Command
from test_state_machine import _solo
from test_m3_submission_contract import agent_fixture, request_and_ack


@pytest.mark.spec("S5.2", "S5.3", "S5.6", "S5.7")
def test_m1_rule_report_then_timely_llm_replaces_all_fields_once():
    _, agent, _ = _solo()
    agent.cmd = Command(None, ["R-S1"], "SUPPLY_BASE")
    pending = {"done": False, "res": None, "t0": 0., "ver": agent._cmd_ver}
    agent._pending_llm = pending
    agent._poll_llm(.9)
    assert not agent.m1_submitted
    agent._poll_llm(1.)
    first = agent.submitter.jobs["m1"]
    pending.update(done=True, res={"target": "RADAR_FACILITY", "sectors": [], "goal_base": None})
    agent._poll_llm(2.)
    updated = agent.submitter.jobs["m1"]
    assert updated.token != first.token
    assert updated.payload == {"target": "RADAR_FACILITY", "sectors": ["R-S1"], "goal_base": "SUPPLY_BASE"}
    agent._poll_llm(2.1)
    assert agent.submitter.jobs["m1"] is updated


@pytest.mark.spec("S5.1", "S5.7", "S0.7", "S0.15")
def test_llm_result_after_timeout_does_not_replace_rule_evidence():
    _, agent, _ = _solo()
    agent.cmd = Command(None, ["R-S1"], "SUPPLY_BASE")
    pending = {"done": False, "res": None, "t0": 0., "ver": agent._cmd_ver}
    agent._pending_llm = pending
    agent._poll_llm(1.)
    first = agent.submitter.jobs["m1"]
    pending.update(done=True, res={"target": "RADAR_FACILITY", "sectors": [], "goal_base": None})
    agent._poll_llm(agent.cfg.mission.m1_llm_timeout+.01)
    assert agent.submitter.jobs["m1"] is first and agent.cmd.target is None


@pytest.mark.spec("S0.5", "S0.10", "S0.16", "S3.2", "S5.7", "S7.7", "S10.2", "S10.4", "S11.3", "S11.6")
def test_stop_tick_closes_queued_m1_m3_before_send_and_m6_does_not_wait_for_llm():
    match, agent, server = _solo()
    agent.submitter.request("m1", {"target": "RADAR_FACILITY"}, 0.)
    agent.submitter.request("m3", {"sectors": {}}, 0.)
    messages = [ServerMsg("stop")]
    server.poll = lambda: [messages.pop()] if messages else []
    # A non-returning LLM must not hold the first M6 snapshot for eight seconds.
    agent._m6 = {"rank": ["R-UGV-1", "R-UGV-2", "R-UGV-3"], "evs": [],
                 "llm": {"done": False, "res": None}, "t0": 0.}
    agent.tick()
    assert agent.submitter.state("m1") == agent.submitter.state("m3") == "cancelled"
    assert server.submits == [] and agent.m6_submitted
    agent._submit_m1(.01)
    agent._submit_m3(.01)
    match.world.step(.05)
    agent.tick()
    assert server.submits == [("m6", True)]
    assert agent.submitter.jobs["m6"].t_sent - agent.halt_t < 1.


@pytest.mark.spec("S7.2", "S7.6", "S7.7", "S10.4")
def test_changed_m3_full_snapshots_switch_to_one_second_cadence_at_225_seconds():
    agent, source = agent_fixture({"COMMON": {"DRUM": 1}})
    request_and_ack(agent, 320.)  # elapsed220
    assert len(agent.server.calls) == 1
    source["R-S1"] = {"TANK": 1}
    request_and_ack(agent, 324.)
    assert len(agent.server.calls) == 1
    request_and_ack(agent, 325.)
    assert len(agent.server.calls) == 2
    source["COMMON"]["DRUM"] = 0  # Correction decreases a count in the complete replacement.
    request_and_ack(agent, 325.8)
    assert len(agent.server.calls) == 2
    request_and_ack(agent, 326.)
    assert len(agent.server.calls) == 3
    payload = agent.server.calls[-1][1]["sectors"]
    assert payload["R-S1"]["TANK"] == 1 and payload["COMMON"]["DRUM"] == 0
    request_and_ack(agent, 327.)
    assert len(agent.server.calls) == 3, "Unchanged snapshots should not churn at the late rate"


@pytest.mark.spec("S0.9", "S1.6", "S2.4", "S2.8")
@pytest.mark.parametrize("elapsed", [120., 270.])
def test_synchronized_restart_restores_clock_without_replacing_navigation_pose(elapsed, tmp_path, monkeypatch):
    _, agent, server = _solo()
    import maicon26.mission.manager as manager
    monkeypatch.setattr(manager.time, "time", lambda: 1000.)
    pose = agent.stack.pose.copy()
    path = tmp_path / "clock.json"
    agent.cfg.hal.restart_state_path = str(path)
    messages = [ServerMsg("start", data={"src": "server_phase", "restarted": True,
                                         "elapsed_s": elapsed, "clock_sync": True, "clock_uncertainty_s": 1.})]
    server.poll = lambda: [messages.pop()] if messages else []
    agent._serve(5.)
    assert agent.t_start == 5.-elapsed and agent.stats["restored"]
    assert (agent.stack.pose.x, agent.stack.pose.y, agent.stack.pose.th) == (pose.x, pose.y, pose.th)
    assert json.loads(path.read_text())["wall_start"] == 1000.-elapsed


@pytest.mark.spec("S0.9", "S2.4")
@pytest.mark.parametrize("elapsed,sync", [(-1., True), (301., True), (math.nan, True), (True, True), (120., False)])
def test_invalid_restart_clock_does_not_invent_zero_or_latch_out_valid_recovery(elapsed, sync):
    _, agent, server = _solo()
    messages = [ServerMsg("start", data={"restarted": True, "elapsed_s": elapsed, "clock_sync": sync})]
    server.poll = lambda: [messages.pop()] if messages else []
    agent._serve(5.)
    assert agent.t_start is None and not agent.latch.started
    messages.append(ServerMsg("start", data={"restarted": True, "elapsed_s": 120., "clock_sync": True}))
    agent._serve(6.)
    assert agent.t_start == -114. and agent.latch.started


@pytest.mark.spec("S10.2", "S11.3", "S11.6")
def test_restart_in_reporting_goes_directly_to_stopped_reporting_without_start_time():
    _, agent, server = _solo()
    messages = [ServerMsg("stop", data={"src": "server_phase"})]
    server.poll = lambda: [messages.pop()] if messages else []
    agent.tick()
    assert agent.t_start is None and agent.mode == Mode.REPORT and agent.m6_submitted
    assert agent.robot.u.cmd_v == 0. and not agent.m1_submitted and not agent.m3_submitted


@pytest.mark.spec("S10.3")
def test_robot_pause_keeps_match_active_without_requesting_m6():
    _, agent, _ = _solo()
    agent.t_start, agent.latch.started, agent._saw_run = 0., True, True
    status = dataclasses.replace(agent.robot.status(), match_mode="mission", match_cmd="pause")
    assert status.paused  # Property derived from the real HAL match command.
    agent._match_api(12., .05, status)
    assert not agent.latch.stopped and not agent.m6_submitted
    assert agent.mode not in (Mode.HALT, Mode.REPORT, Mode.DONE)


@pytest.mark.spec("S5.6", "S9.8")
def test_m5_posted_objective_does_not_overwrite_m1_interpretation():
    _, agent, server = _solo()
    agent.cmd = Command("RADAR_FACILITY", ["R-S1"], "SUPPLY_BASE")
    original = agent.cmd.to_payload()
    messages = [ServerMsg("base_command", data={"objective": "RED_MAINTENANCE"})]
    server.poll = lambda: [messages.pop()] if messages else []
    agent._serve(240.)
    assert agent.m5_type == "MAINT_DEPOT" and agent.cmd.to_payload() == original


@pytest.mark.spec("S10.5", "S6.1", "S8.1", "S9.7")
def test_device_events_do_not_create_participant_m2_m4_or_m5_reports():
    _, agent, server = _solo()
    agent.team, agent.slot = "BLUE", 2
    agent.tracker = SimpleNamespace(tracks=[])
    agent._on_event(12., {"type": "target_hit", "by": "B-UGV-2"})
    agent._on_event(13., {"type": "ugv_hit", "by": "B-UGV-2", "victim": "R-UGV-1"})
    agent._on_env_event(241., {"type": "base_entered", "by": "B-UGV-2", "base": "R-2"})
    # An unscoped device event cannot identify the designated opponent target.
    assert not agent.target_done and agent.m5_confirmed_event == 241.
    assert not agent.submitter.jobs and not server.submits


@pytest.mark.spec("S0.7", "S0.15", "S5.1", "S9.7")
def test_unannounced_llm_protocol_and_m5_dwell_remain_explicit_unconfigured_slots():
    cfg = Config()
    assert cfg.llm.endpoint is None and cfg.llm.adapter_factory is None
    assert cfg.llm.api_key_env == "MAICON_LLM_API_KEY"
    assert cfg.mission.m5_dwell_s == 0.
    assert "api_key" not in cfg.to_dict()["llm"]
