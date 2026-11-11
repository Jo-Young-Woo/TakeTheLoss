"""Authoritative M3 replacement, scored territory and receive-time contract."""
import copy
import threading
import time
from types import SimpleNamespace

import pytest

from maicon26.config import Config
from maicon26.link import schema
from maicon26.link.async_link import AsyncServerLink
from maicon26.link.base import ServerLink
from maicon26.link.http import HttpServerLink
from maicon26.mission.manager import Agent
from maicon26.mission.submit import Submitter


class RecordingLink(ServerLink):
    def __init__(self, results=()):
        self.calls, self.results = [], list(results)
        self.allowed = True

    def poll(self):
        return []

    def submit(self, kind, payload):
        self.calls.append((kind, copy.deepcopy(payload)))
        return self.results.pop(0) if self.results else True

    def submission_allowed(self, kind):
        return self.allowed if kind == "m3" else None


def agent_fixture(raw=None, team="BLUE", results=()):
    agent = Agent.__new__(Agent)
    agent.cfg, agent.team = Config(), team
    agent.cfg.explore.m3_counting = "omap"
    source = {} if raw is None else raw
    agent.omap = SimpleNamespace(counts_by_sector=lambda: source)
    agent.coverage = SimpleNamespace(boxes={})
    agent.t_start = 100.
    agent.latch = SimpleNamespace(stopped=False)
    agent.server = RecordingLink(results)
    agent.submitter = Submitter(agent.server)
    agent._m3_last = (-1e9, None)
    agent._m3_final_requested = False
    agent.log = []
    agent._log = lambda now, message: agent.log.append((now, message))
    return agent, source


def request_and_ack(agent, now):
    agent._submit_m3_if_due(now, now-agent.t_start)
    agent.submitter.update(now)
    agent.submitter.update(now+.01)


@pytest.mark.parametrize("team,prefix,other", [("BLUE", "R", "B"), ("RED", "B", "R")])
def test_every_snapshot_contains_all_scored_sectors_and_only_six_counted_classes(team, prefix, other):
    agent, _ = agent_fixture({f"{prefix}-S1": {"TANK": 2, "EMPTY": 8, "DESTROYED_BUILDING": 4},
                              "COMMON": {"DRUM": 1}, f"{other}-S2": {"MISSILE": 9}}, team)
    counts = agent._m3_counts()
    assert set(counts) == {f"{prefix}-S1", f"{prefix}-S2", f"{prefix}-S3", "COMMON"}
    assert all(set(bucket) == set(schema.OBJECT_CODES) for bucket in counts.values())
    assert counts[f"{prefix}-S2"] == dict.fromkeys(schema.OBJECT_CODES, 0)
    assert sum(sum(bucket.values()) for bucket in counts.values()) == 3
    wire = schema.m3_to_server(schema.m3_payload(counts), team)
    assert [s["sector"] for s in wire["sectors"]] == schema.scoring_sectors(team)
    assert wire["sectors"][1]["objects"] == []  # Explicit zero sector, not an omitted sector.


def test_resubmission_is_complete_current_snapshot_not_delta_or_monotonic_max():
    agent, source = agent_fixture({"R-S1": {"TANK": 1}, "COMMON": {"DRUM": 2}})
    request_and_ack(agent, 110.)
    assert agent.submitter.acked("m3")
    first = copy.deepcopy(agent.server.calls[-1][1])
    source["R-S3"] = {"AMMO_BOX": 3}
    request_and_ack(agent, 115.)
    replacement = agent.server.calls[-1][1]["sectors"]
    assert replacement["R-S1"]["TANK"] == 1 and replacement["COMMON"]["DRUM"] == 2
    assert replacement["R-S3"]["AMMO_BOX"] == 3
    assert first["sectors"]["R-S3"]["AMMO_BOX"] == 0  # Earlier request snapshot is immutable.
    source.pop("R-S1")  # Corrected/invalidated evidence must really clear the previous value.
    request_and_ack(agent, 120.)
    assert agent.server.calls[-1][1]["sectors"]["R-S1"]["TANK"] == 0
    assert agent.server.calls[-1][1]["sectors"]["COMMON"]["DRUM"] == 2


def test_early_snapshot_forced_final_flush_and_deadline_survive_missing_stop():
    agent, _ = agent_fixture()
    request_and_ack(agent, 109.)
    assert not agent.server.calls
    request_and_ack(agent, 110.)
    assert len(agent.server.calls) == 1
    assert agent.submitter.jobs["m3"].deadline == 390.
    request_and_ack(agent, 379.)
    assert len(agent.server.calls) == 1  # Unchanged periodic snapshots need not churn.
    request_and_ack(agent, 380.)
    assert len(agent.server.calls) == 2  # 280-second final flush leaves retry/receive margin.
    agent._guard_m3_submission(390.)
    request_and_ack(agent, 399.)
    assert len(agent.server.calls) == 2 and "m3" in agent.submitter.closed
    assert not agent.submitter.revive("m3", 399.)


def test_failed_ack_refreshes_complete_payload_and_does_not_ack_the_new_job_from_old_token():
    agent, source = agent_fixture({"R-S1": {"TANK": 1}}, results=(False, True))
    request_and_ack(agent, 110.)
    assert agent.submitter.state("m3") == "retry" and not agent.submitter.acked("m3")
    old_token = agent.submitter.jobs["m3"].token
    source["R-S2"] = {"DRUM": 2}
    agent._submit_m3_if_due(115., 15.)
    new_token = agent.submitter.jobs["m3"].token
    assert new_token != old_token
    agent.server._async_results.append((old_token, True))
    agent.submitter.update(115.)
    assert not agent.submitter.acked("m3")
    agent.submitter.update(115.01)
    assert agent.submitter.acked("m3")
    assert agent.server.calls[-1][1]["sectors"]["R-S1"]["TANK"] == 1
    assert agent.server.calls[-1][1]["sectors"]["R-S2"]["DRUM"] == 2


def test_stop_phase_gate_and_cutoff_prevent_pending_sends_and_retries():
    agent, _ = agent_fixture(results=(False,))
    agent.server.allowed = False
    request_and_ack(agent, 110.)
    assert not agent.server.calls
    agent.server.allowed = True
    agent._submit_m3_if_due(115., 15.)
    agent.latch.stopped = True
    agent._guard_m3_submission(115.01)  # Same pre-update ordering as Agent._tick.
    agent.submitter.update(115.01)
    assert not agent.server.calls and agent.submitter.state("m3") == "cancelled"
    assert not agent.submitter.request("m3", {}, 116., force=True)


def test_http_in_progress_normalization_and_409_never_mean_ack():
    clock = [0.]
    link = HttpServerLink("http://unused", "ABCD1234", clock=lambda: clock[0])
    state = {"phase": "IN_PROGRESS", "slot": "B-2", "ready": True}
    link._get_json = lambda *a, **k: (200, dict(state))
    posted = []

    def post(path, body):
        posted.append((path, body))
        state["phase"] = "FINISHED"  # Deadline/phase changed after the preflight check.
        return 409, {"detail": "only IN_PROGRESS accepts detection"}

    link._post_json = post
    assert link.me()["phase"] == "running"
    assert link.submission_allowed("m3") is True
    assert link.submit("m3", {"sectors": {"R-S1": {"TANK": 1}}}) is False
    assert link.phase == "finished" and link.submission_allowed("m3") is False
    assert "409" in link._submit_err
    assert link.submit("m3", {"sectors": {}}) is False
    assert len(posted) == 1


@pytest.mark.parametrize("bad_count", [True, False, 1.9, 2.0, "2", None, {}, []])
def test_wire_rejects_noninteger_counts_and_http_returns_failure_without_posting(bad_count):
    payload = {"sectors": {"R-S1": {"TANK": bad_count}}}
    with pytest.raises(ValueError, match="count must be an integer"):
        schema.m3_to_server(payload, "BLUE")
    link = HttpServerLink("http://unused", "ABCD1234", team="BLUE", slot=2, clock=lambda: 1.)
    link.phase, link._me_ok_t = "running", 1.
    posted = []
    link._post_json = lambda path, body: (posted.append(body) or (200, {"accepted": True, "received": body}))
    assert link.submit("m3", payload) is False
    assert posted == []
    assert "invalid payload" in link._submit_err
    # Validation failure must not kill the transport or poison a later valid report.
    assert link.submit("m3", {"sectors": {"R-S1": {"TANK": 2}, "COMMON": {"DRUM": 0}}}) is True
    assert len(posted) == 1 and link._submit_err is None


def test_async_close_drops_queued_m3_and_timeout_retries_cannot_build_duplicate_backlog():
    started, release = threading.Event(), threading.Event()

    class Slow(RecordingLink):
        def submit(self, kind, payload):
            self.calls.append((kind, copy.deepcopy(payload)))
            started.set()
            release.wait(1.)
            return True

    inner = Slow()
    wrapper = AsyncServerLink(inner)
    try:
        wrapper.submit_async("m1", {}, "blocking")
        assert started.wait(.5)
        wrapper.submit_async("m3", {"sectors": {}}, "snapshot")
        wrapper.submit_async("m3", {"sectors": {}}, "snapshot")
        wrapper.close_submission("m3")
        release.set()
        end = time.monotonic()+1.
        results = []
        while time.monotonic() < end and len(results) < 2:
            results.extend(wrapper.submit_results())
            time.sleep(.01)
        assert inner.calls == [("m1", {})]
        assert sorted(results) == [("blocking", True), ("snapshot", False)]
    finally:
        release.set()
        wrapper.close()
