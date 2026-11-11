"""Ollama transport contract and M1/M6 boundary, without a model or ROS."""
import io
import json
import threading
import time
import urllib.error
from contextlib import contextmanager
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from types import SimpleNamespace

import pytest

from maicon26.llm.ollama import OllamaClient, create_client
from maicon26.mission.threat import Evidence, llm_rank
from maicon26.preflight import load_llm_factory
from test_state_machine import _solo


def envelope(content='{"target":"RADAR_FACILITY"}', **overrides):
    result = {"done": True, "message": {"role": "assistant", "content": content}}
    result.update(overrides)
    return result


class Response(io.BytesIO):
    def __init__(self, body, content_type="application/json; charset=utf-8", status=200):
        super().__init__(body)
        self.status = status
        self.headers = {"Content-Type": content_type}


def fake_response(client, body, **kwargs):
    calls = []
    def open_response(req, timeout):
        calls.append((req, timeout))
        return Response(body, **kwargs)
    client._opener = SimpleNamespace(open=open_response)
    return calls


@contextmanager
def chat_server(contents):
    requests = []
    class Handler(BaseHTTPRequestHandler):
        def do_POST(self):
            body = self.rfile.read(int(self.headers["Content-Length"]))
            requests.append((self.path, json.loads(body)))
            result = json.dumps(envelope(contents[len(requests) - 1])).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(result)))
            self.end_headers()
            self.wfile.write(result)
        def log_message(self, *_args):
            pass
    server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    thread = threading.Thread(target=lambda: server.serve_forever(poll_interval=.02), daemon=True)
    thread.start()
    try:
        yield f"http://127.0.0.1:{server.server_port}", requests
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=1)


def test_factory_requires_explicit_model_and_uses_existing_loader(monkeypatch):
    monkeypatch.delenv("MAICON_OLLAMA_MODEL", raising=False)
    with pytest.raises(ValueError, match="MAICON_OLLAMA_MODEL"):
        create_client()
    monkeypatch.setenv("MAICON_OLLAMA_MODEL", "team-model:tag")
    monkeypatch.setenv("MAICON_OLLAMA_URL", "http://localhost:11434/")
    client = load_llm_factory("maicon26.llm.ollama:create_client")
    assert client.model == "team-model:tag"
    assert client.url == "http://localhost:11434/api/chat"
    client.close()


@pytest.mark.parametrize("url", ["file:///tmp/model", "localhost:11434", "http://",
    "http://user:secret@localhost:11434", "http://localhost/api/chat",
    "http://localhost?secret=x", "http://localhost#fragment", "http://localhost:bad",
    "http://local host"])
def test_invalid_server_origin_rejected(url):
    with pytest.raises(ValueError):
        OllamaClient("model", url)


@pytest.mark.parametrize("model", ["", " ", "bad\nmodel", "two words", "x" * 257])
def test_invalid_model_rejected(model):
    with pytest.raises(ValueError):
        OllamaClient(model)


def test_request_is_text_only_nonstreaming_json_and_timeout_is_capped():
    client = OllamaClient("model")
    calls = fake_response(client, json.dumps(envelope()).encode())
    assert json.loads(client.complete("JSON 응답", 500))["target"] == "RADAR_FACILITY"
    req, timeout = calls[0]
    body = json.loads(req.data)
    assert timeout == 60.0
    assert req.get_method() == "POST" and req.full_url.endswith("/api/chat")
    assert body["stream"] is False and body["format"] == "json"
    assert body["messages"][-1] == {"role": "user", "content": "JSON 응답"}
    assert "tools" not in body and "images" not in body and "keep_alive" not in body


@pytest.mark.parametrize("value", [0, -1, float("nan"), float("inf")])
def test_invalid_deadline_rejected_without_io(value):
    client = OllamaClient("model")
    calls = fake_response(client, b"{}")
    with pytest.raises(ValueError, match="timeout"):
        client.complete("JSON", value)
    assert not calls


@pytest.mark.parametrize("body", [b"not JSON", b"\xff", b"[]", b"{}",
    json.dumps(envelope(done=False)).encode(),
    json.dumps(envelope(error="model missing")).encode(),
    json.dumps(envelope(message={"role": "tool", "content": "{}"})).encode(),
    json.dumps(envelope(message={"role": "assistant", "content": "{}", "tool_calls": [{}]})).encode(),
    json.dumps(envelope(content="")).encode(),
    json.dumps(envelope(content="```json\n{}\n```")).encode(),
    json.dumps(envelope(content="[]")).encode(),
    json.dumps(envelope(content='{"risk": NaN}')).encode(),
    json.dumps(envelope(content=3)).encode()])
def test_invalid_response_rejected(body):
    client = OllamaClient("model")
    fake_response(client, body)
    with pytest.raises(ValueError):
        client.complete("JSON", 1)


def test_response_limit_mime_and_http_failure_are_checked():
    client = OllamaClient("model", max_response_bytes=32)
    fake_response(client, b" " * 100)
    with pytest.raises(ValueError, match="byte limit"):
        client.complete("JSON", 1)
    fake_response(client, b"{}", content_type="text/html")
    with pytest.raises(ValueError, match="application/json"):
        client.complete("JSON", 1)
    def fail(_req, timeout):
        raise urllib.error.HTTPError("http://secret-host", 404, "provider secret", {}, None)
    client._opener = SimpleNamespace(open=fail)
    with pytest.raises(RuntimeError, match="^Ollama request failed$"):
        client.complete("JSON", 1)


def test_deadline_returns_and_does_not_accumulate_timed_out_workers():
    client = OllamaClient("model")
    entered, release, finished = threading.Event(), threading.Event(), threading.Event()
    def blocked(_prompt, _timeout):
        entered.set()
        release.wait(2)
        finished.set()
        return "{}"
    client._request = blocked
    start = time.monotonic()
    try:
        with pytest.raises(TimeoutError):
            client.complete("JSON", .04)
        assert entered.is_set() and time.monotonic() - start < .5
        with pytest.raises(RuntimeError, match="already in progress"):
            client.complete("JSON", .04)
    finally:
        release.set()
        assert finished.wait(1)
        client.close()


def test_close_wakes_pending_caller_and_blocks_future_requests():
    client = OllamaClient("model")
    entered, release, returned = threading.Event(), threading.Event(), threading.Event()
    errors = []
    def blocked(_prompt, _timeout):
        entered.set()
        release.wait(2)
        return "{}"
    client._request = blocked
    def caller():
        try:
            client.complete("JSON", 10)
        except RuntimeError as exc:
            errors.append(str(exc))
        finally:
            returned.set()
    thread = threading.Thread(target=caller, daemon=True)
    thread.start()
    try:
        assert entered.wait(1)
        client.close()
        assert returned.wait(.5) and errors == ["Ollama client is closed"]
        with pytest.raises(RuntimeError, match="closed"):
            client.complete("JSON", 1)
    finally:
        release.set()
        thread.join(timeout=1)


def test_local_http_works_with_m1_and_m6_existing_parsers():
    m1 = json.dumps({"target": "RADAR_FACILITY", "sectors": ["R-S1"], "goal_base": "SUPPLY_BASE"})
    ranking = ["R-UGV-2", "R-UGV-1", "R-UGV-3"]
    with chat_server([m1, json.dumps({"ranking": ranking})]) as (url, requests):
        client = OllamaClient("test-model", url)
        _match, agent, _server = _solo()
        agent.llm = client
        try:
            agent._on_command(1.0, "R-S1을 탐색하고 레이더시설을 타격한 뒤 군수보급소로 이동하라")
            until = time.monotonic() + 2
            while not agent._pending_llm["done"] and time.monotonic() < until:
                time.sleep(.001)
            assert agent._pending_llm["done"]
            agent._poll_llm(1.1)
            assert agent.submitter.jobs["m1"].payload["target"] == "RADAR_FACILITY"
            assert llm_rank([Evidence(i, False) for i in ranking], client, 1) == ranking
            assert len(requests) == 2 and all(path == "/api/chat" for path, _ in requests)
        finally:
            client.close()


def test_m6_still_rejects_invalid_identity_permutation():
    client = OllamaClient("model")
    fake_response(client, json.dumps(envelope('{"ranking":["R-UGV-1","R-UGV-1","R-UGV-3"]}')).encode())
    assert llm_rank([Evidence(f"R-UGV-{i}", False) for i in (1, 2, 3)], client, 1) is None
