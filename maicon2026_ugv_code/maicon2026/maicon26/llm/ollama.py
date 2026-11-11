"""Bounded Ollama chat adapter for the existing M1/M6 LLM interface.

Factory: ``maicon26.llm.ollama:create_client``. No ROS or Ollama SDK needed.
Only JSON text is returned; mission parsers retain responsibility for accepting
target/sector/base values and an M6 permutation. Nothing here executes a command.
"""
from __future__ import annotations

import json
import math
import os
import threading
import time
import urllib.error
import urllib.parse
import urllib.request


class _NoRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        return None


def _reject_constant(_value):
    raise ValueError("Ollama returned a non-finite JSON value")


class OllamaClient:
    """One request at a time, bounded caller wait, bounded response allocation.

    A daemon worker bounds caller latency even during slow DNS or trickling HTTP
    headers. A timed-out worker retains its slot until it exits, preventing
    accumulation of background requests. Timeout does not promise cancellation
    of model inference on the Ollama server.
    """

    def __init__(self, model: str, base_url: str = "http://127.0.0.1:11434", *,
                 max_response_bytes: int = 65536) -> None:
        if not isinstance(model, str) or not model.strip() or len(model) > 256:
            raise ValueError("MAICON_OLLAMA_MODEL must name an installed model")
        if any(c.isspace() or ord(c) < 32 for c in model):
            raise ValueError("MAICON_OLLAMA_MODEL must not contain whitespace")
        parts = urllib.parse.urlsplit(base_url)
        if (parts.scheme not in ("http", "https") or not parts.hostname
                or parts.username is not None or parts.password is not None
                or parts.path not in ("", "/") or parts.query or parts.fragment
                or any(c.isspace() or ord(c) < 32 for c in base_url)):
            raise ValueError("MAICON_OLLAMA_URL must be an http(s) server origin")
        try:
            parts.port
        except ValueError:
            raise ValueError("MAICON_OLLAMA_URL contains an invalid port") from None
        if type(max_response_bytes) is not int or not 1 <= max_response_bytes <= 1048576:
            raise ValueError("response byte limit must be between 1 and 1048576")
        self.model = model
        self.url = base_url.rstrip("/") + "/api/chat"
        self.max_response_bytes = max_response_bytes
        # The explicitly configured endpoint must not be redirected through an
        # ambient proxy or HTTP redirect to an unrelated destination.
        self._opener = urllib.request.build_opener(urllib.request.ProxyHandler({}), _NoRedirect())
        self._busy = threading.Lock()
        self._closed = threading.Event()

    def _request(self, prompt: str, timeout: float) -> str:
        payload = {
            "model": self.model, "stream": False, "format": "json",
            "messages": [{"role": "system", "content": (
                "Return one JSON object matching the requested schema. "
                "Do not generate tools, executable code, or robot motion commands.")},
                {"role": "user", "content": prompt}],
            "options": {"temperature": 0},
        }
        req = urllib.request.Request(self.url, data=json.dumps(payload).encode("utf-8"),
                                     headers={"Content-Type": "application/json",
                                              "Accept": "application/json"}, method="POST")
        try:
            with self._opener.open(req, timeout=timeout) as response:
                if response.status != 200:
                    raise ValueError("Ollama returned a non-success response")
                content_type = response.headers.get("Content-Type", "").split(";", 1)[0].strip().lower()
                if content_type != "application/json":
                    raise ValueError("Ollama response is not application/json")
                body = response.read(self.max_response_bytes + 1)
        except urllib.error.HTTPError as exc:
            exc.close()
            raise RuntimeError("Ollama request failed") from None
        except (urllib.error.URLError, OSError):
            raise RuntimeError("Ollama request failed") from None
        if len(body) > self.max_response_bytes:
            raise ValueError("Ollama response exceeds byte limit")
        try:
            envelope = json.loads(body.decode("utf-8"), parse_constant=_reject_constant)
            if not isinstance(envelope, dict) or envelope.get("done") is not True or "error" in envelope:
                raise ValueError("Ollama response is incomplete or reports an error")
            message = envelope.get("message")
            if (not isinstance(message, dict) or message.get("role") != "assistant"
                    or message.get("tool_calls")):
                raise ValueError("Ollama must return an assistant text response without tool calls")
            content = message.get("content")
            if not isinstance(content, str) or not content.strip():
                raise ValueError("Ollama response has no text content")
            parsed = json.loads(content, parse_constant=_reject_constant)
            if not isinstance(parsed, dict):
                raise ValueError("Ollama content must be a JSON object")
        except (UnicodeError, json.JSONDecodeError, RecursionError):
            raise ValueError("Ollama returned malformed JSON") from None
        return content

    def complete(self, prompt: str, timeout: float) -> str:
        timeout = float(timeout)
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("Ollama timeout must be finite and positive")
        timeout = min(timeout, 60.0)
        if not isinstance(prompt, str) or not prompt.strip() or len(prompt.encode("utf-8")) > 65536:
            raise ValueError("Ollama prompt must be nonempty and at most 65536 bytes")
        if self._closed.is_set():
            raise RuntimeError("Ollama client is closed")
        if not self._busy.acquire(blocking=False):
            raise RuntimeError("Ollama request already in progress")
        deadline = time.monotonic() + timeout
        done, result = threading.Event(), {}

        def work():
            try:
                if self._closed.is_set():
                    raise RuntimeError("Ollama client is closed")
                result["content"] = self._request(prompt, timeout)
            except Exception as exc:
                result["error"] = exc
            finally:
                self._busy.release()
                done.set()

        try:
            threading.Thread(target=work, name="ollama-request", daemon=True).start()
        except Exception:
            self._busy.release()
            raise
        while True:
            if self._closed.is_set():
                raise RuntimeError("Ollama client is closed")
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("Ollama completion deadline exceeded")
            if done.wait(min(remaining, 0.05)):
                if self._closed.is_set():
                    raise RuntimeError("Ollama client is closed")
                if time.monotonic() >= deadline:
                    raise TimeoutError("Ollama completion deadline exceeded")
                if "error" in result:
                    raise result["error"]
                return result["content"]

    def close(self) -> None:
        """Stop accepting requests and wake waiting callers; never block shutdown."""
        self._closed.set()


def create_client() -> OllamaClient:
    """Load an explicitly selected model; never download or start a service."""
    return OllamaClient(os.environ.get("MAICON_OLLAMA_MODEL", ""),
                        os.environ.get("MAICON_OLLAMA_URL", "http://127.0.0.1:11434"))
