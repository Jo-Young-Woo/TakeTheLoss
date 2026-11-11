"""Local M1 adapter boundary; no model download or external provider calls."""
import json
import sys
import threading
import time
from types import ModuleType, SimpleNamespace

import pytest

from maicon26.preflight import load_llm_factory
from test_state_machine import _solo


def wait_done(agent):
    until = time.monotonic() + 1.0
    while not agent._pending_llm["done"] and time.monotonic() < until:
        time.sleep(0.001)
    assert agent._pending_llm["done"], "M1 worker did not return"


def test_plugin_loader_returns_factory_client(monkeypatch):
    module = ModuleType("test_local_llm_adapter")
    client = SimpleNamespace(complete=lambda prompt, timeout: "{}")
    calls = []
    module.create = lambda: calls.append(True) or client
    monkeypatch.setitem(sys.modules, module.__name__, module)
    assert load_llm_factory("test_local_llm_adapter:create") is client
    assert calls == [True]


@pytest.mark.parametrize("spec", ["", "module", "module:", ":factory"])
def test_plugin_loader_rejects_bad_spec(spec):
    with pytest.raises(ValueError, match="module:function"):
        load_llm_factory(spec)


def test_plugin_loader_rejects_missing_complete(monkeypatch):
    module = ModuleType("test_invalid_llm_adapter")
    module.create = lambda: object()
    monkeypatch.setitem(sys.modules, module.__name__, module)
    with pytest.raises(TypeError, match="complete"):
        load_llm_factory("test_invalid_llm_adapter:create")


def test_plugin_factory_error_propagates_to_startup(monkeypatch):
    module = ModuleType("test_broken_llm_adapter")
    def fail():
        raise RuntimeError("model unavailable")
    module.create = fail
    monkeypatch.setitem(sys.modules, module.__name__, module)
    with pytest.raises(RuntimeError, match="model unavailable"):
        load_llm_factory("test_broken_llm_adapter:create")


def test_unambiguous_command_still_uses_configured_llm():
    _match, agent, _server = _solo()
    calls = []
    def complete(prompt, timeout):
        calls.append((prompt, timeout))
        return json.dumps({"target": "RADAR_FACILITY", "sectors": ["R-S1"], "goal_base": "SUPPLY_BASE"})
    agent.llm = SimpleNamespace(complete=complete)
    command = "R-S1을 탐색하고 레이더시설을 타격한 뒤 군수보급소로 이동하라"
    agent._on_command(1.0, command)
    wait_done(agent)
    assert len(calls) == 1 and command in calls[0][0]
    assert calls[0][1] == agent.cfg.mission.m1_llm_timeout
    assert not agent.m1_submitted
    agent._poll_llm(1.1)
    payload = agent.submitter.jobs["m1"].payload
    assert payload["target"] == "RADAR_FACILITY" and payload["goal_base"] == "SUPPLY_BASE"
    assert agent.m1_submitted


@pytest.mark.parametrize("failure", ["exception", "invalid_json"])
def test_llm_failure_falls_back_to_rule_without_killing_worker(failure, capsys):
    _match, agent, _server = _solo()
    def complete(prompt, timeout):
        if failure == "exception":
            raise RuntimeError("provider-secret-do-not-log")
        return "Not valid JSON"
    agent.llm = SimpleNamespace(complete=complete)
    agent._on_command(1.0, "R-S1을 탐색하고 레이더시설을 타격하라")
    wait_done(agent)
    agent._poll_llm(1.1)
    assert agent.submitter.jobs["m1"].payload["target"] == "RADAR_FACILITY"
    assert "provider-secret-do-not-log" not in capsys.readouterr().out


def test_unresponsive_adapter_does_not_block_rule_submission():
    _match, agent, _server = _solo()
    release = threading.Event()
    def complete(prompt, timeout):
        release.wait(2)
        return '{"target": "COMM_FACILITY"}'
    agent.llm = SimpleNamespace(complete=complete)
    try:
        agent._on_command(1.0, "R-S1을 탐색하고 레이더시설을 타격하라")
        assert not agent._pending_llm["done"]
        due = 1.0 + agent.cfg.mission.m1_submit_after + agent.cfg.mission.m1_llm_timeout + 0.01
        agent._poll_llm(due)
        assert agent.submitter.jobs["m1"].payload["target"] == "RADAR_FACILITY"
        release.set()
        wait_done(agent)
        agent._poll_llm(due + 1.0)
        assert agent.submitter.jobs["m1"].payload["target"] == "RADAR_FACILITY"
    finally:
        release.set()
