"""G6: Agent -> real HTTP adapter -> in-process server, on one fake clock.

No World, Gazebo, network, real sleeps or running worker threads are needed.
Production worker bodies are stepped deterministically, including the async
submission wrapper used by the real launcher. Sensors deliberately fail if a
STOP/report tick tries to wait for navigation or acquire a new observation.
"""
import threading
from types import SimpleNamespace

import pytest

from maicon26.config import Config
from maicon26.hal.base import Robot
from maicon26.link.async_link import AsyncServerLink
from maicon26.link.http import HttpServerLink
from maicon26.mission.manager import Agent, Mode
from maicon26.sim.mockserver import FakeClock, MockServer
from maicon26.types import Command, Pose2D, RobotStatus


CODE = "G6Test42"  # Invented fixture credential, never a field credential.


class NoSensorRobot(Robot):
    """Only status and emergency outputs are available after server STOP."""
    can_abort_laser = True

    def __init__(self, clock):
        self.clock = clock
        self.velocities = []
        self.abort_count = 0

    def now(self):
        return self.clock()

    def status(self):
        return RobotStatus()

    def set_velocity(self, v, w):
        self.velocities.append((self.clock(), v, w))

    def set_pan_tilt(self, pan_deg, tilt_deg):
        raise AssertionError("reporting cannot create a pan/tilt mission")

    def get_pan_tilt(self):
        return 0., 0.

    def laser_fire(self):
        raise AssertionError("reporting cannot fire")

    def laser_abort(self):
        self.abort_count += 1

    def get_scan(self):
        raise AssertionError("STOP must precede sensor acquisition")

    def get_frame(self):
        raise AssertionError("STOP must precede sensor acquisition")

    def get_odom(self):
        raise AssertionError("reporting must not wait for localization")


class FrozenNavigation:
    initialized = False

    def __init__(self):
        self.pose = Pose2D(0., 0., 0.)
        self.cancellations = 0

    def clear_goal(self):
        self.cancellations += 1


class OnePoll:
    def __init__(self):
        self.done = False

    def is_set(self):
        return self.done

    def wait(self, _seconds):
        self.done = True
        return True


def poll_phase(http, transport):
    """Execute one real /me loop iteration and transfer its actual messages."""
    previous = http._stop_ev
    http._stop_ev = OnePoll()
    try:
        http._me_loop()
    finally:
        http._stop_ev = previous
    if isinstance(transport, AsyncServerLink):
        transport._poll_once()


def drain_submission_worker(transport):
    if not isinstance(transport, AsyncServerLink):
        return

    class QueueBoundary:
        def is_set(self):
            return transport._jobs.empty()

    previous = transport._stop
    transport._stop = QueueBoundary()
    try:
        transport._submit_loop()
    finally:
        transport._stop = previous


def reporting_fixture(monkeypatch, *, slot="B-2", asynchronous=False, initial="waiting"):
    # LLM and transport worker scheduling is controlled by this test. Their
    # production bodies are not replaced with fabricated success responses.
    monkeypatch.setattr(threading.Thread, "start", lambda _self: None)
    clock = FakeClock()
    mock = MockServer({CODE: slot}, clock=clock)
    mock.set_phase(initial)
    http = HttpServerLink("http://fixture.invalid", CODE, clock=clock, transport=mock,
                          decode_drone=False, me_period_s=.5)
    assert http.me()["phase"] == initial
    http._started_bg = True
    transport = AsyncServerLink(http) if asynchronous else http
    cfg = Config()
    cfg.team, cfg.slot = http.identity()
    cfg.hal.restart_state_path = None
    robot, navigation = NoSensorRobot(clock), FrozenNavigation()
    agent = Agent(robot, transport, cfg, stack=navigation,
                  llm=SimpleNamespace(complete=lambda *_: pytest.fail("LLM must not block first M6")))
    return clock, mock, http, transport, robot, navigation, agent


@pytest.mark.spec("S0.5", "S0.10", "S0.16", "S3.2", "S5.7", "S7.7", "S10.2", "S10.4", "S11.3", "S11.6")
@pytest.mark.parametrize("slot", ["B-2", "R-2"])
@pytest.mark.parametrize("asynchronous", [False, True], ids=["http", "async-http"])
def test_agent_early_reporting_cancels_m1_m3_and_mock_receives_m6_within_one_second(monkeypatch, slot, asynchronous):
    clock, mock, http, transport, robot, navigation, agent = reporting_fixture(
        monkeypatch, slot=slot, asynchronous=asynchronous)
    mock.start()
    poll_phase(http, transport)
    agent._serve(clock())
    assert agent.latch.started and agent.t_start == 0.
    opposite = "R" if slot.startswith("B") else "B"
    # Existing accepted evidence is preserved; pending corrections must not be
    # sent after an early server-side M5 capture ends the running phase.
    before_m1 = {"target": "RADAR_FACILITY", "search_sectors": [f"{opposite}-S1"]}
    before_m3 = {"sectors": {f"{opposite}-S1": {"TANK": 1}}}
    assert http.submit("m1", before_m1)
    assert http.submit("m3", before_m3)
    accepted_before = {kind: dict(value) for kind, value in mock.team(CODE).reports.items()}
    clock.advance(42.)
    agent.cmd = Command("RADAR_FACILITY", [f"{opposite}-S2"], "SUPPLY_BASE")
    agent.submitter.request("m1", agent.cmd.to_payload(), clock())
    agent.submitter.request("m3", {"sectors": {f"{opposite}-S1": {"TANK": 2}}}, clock())
    if asynchronous:
        transport.submit_async("m1", before_m1, "already-queued-m1")
        transport.submit_async("m3", before_m3, "already-queued-m3")
    mock.set_phase("reporting")
    reported_at = clock()
    clock.advance(.49)  # Near-worst-case delay of the configured /me poll.
    poll_phase(http, transport)
    agent.tick()
    assert agent.mode == Mode.REPORT and agent.stop_source == "server"
    assert agent.submitter.state("m1") == agent.submitter.state("m3") == "cancelled"
    assert agent._m6["llm"]["done"] is False, "A pending LLM cannot hold the observed-evidence ranking"
    # Stop is before either new jobs or old pending jobs reach HTTP.
    agent._submit_m1(clock())
    agent._submit_m3(clock())
    assert not agent.submitter.request("m1", before_m1, clock(), force=True)
    assert not agent.submitter.request("m3", before_m3, clock(), force=True)
    clock.advance(.05)
    agent.tick()
    drain_submission_worker(transport)
    clock.advance(.05)
    agent.tick()
    assert agent.mode == Mode.DONE and agent.m6_result == "acked"
    receipts = [r for r in mock.requests if r["method"] == "POST" and r["received_at"] >= reported_at]
    assert [r["path"].split("/")[-1] for r in receipts] == ["situation"]
    assert 0 <= receipts[0]["received_at"] - reported_at < 1.
    assert mock.attempts["interpretation"] == mock.attempts["detection"] == 1
    assert mock.team(CODE).reports["m1"] == accepted_before["m1"]
    assert mock.team(CODE).reports["m3"] == accepted_before["m3"]
    assert set(mock.team(CODE).reports["m6"]["ranking"]) == {f"{opposite}-{i}" for i in (1, 2, 3)}
    assert navigation.cancellations > 0 and robot.abort_count == 1
    assert robot.velocities and all(v == w == 0. for _, v, w in robot.velocities)
    assert clock() == pytest.approx(42.59), "No sleep/long-poll should advance the reporting path"


@pytest.mark.spec("S10.2", "S11.3", "S11.6")
@pytest.mark.parametrize("asynchronous", [False, True], ids=["http", "async-http"])
def test_agent_restart_in_reporting_posts_m6_without_start_sensors_or_motion(monkeypatch, asynchronous):
    clock, mock, http, transport, robot, _, agent = reporting_fixture(
        monkeypatch, asynchronous=asynchronous, initial="reporting")
    poll_phase(http, transport)
    agent.tick()
    clock.advance(.05)
    agent.tick()
    drain_submission_worker(transport)
    clock.advance(.05)
    agent.tick()
    assert agent.t_start is None and agent.latch.stopped
    assert agent.mode == Mode.DONE and agent.m6_result == "acked"
    assert mock.attempts["interpretation"] == mock.attempts["detection"] == 0
    receipt = next(r for r in mock.requests if r["path"].endswith("/situation"))
    assert receipt["received_at"] - mock.matches["report"].reporting_at < 1.
    assert all(v == w == 0. for _, v, w in robot.velocities)
