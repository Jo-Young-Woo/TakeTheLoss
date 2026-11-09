"""Redact registered credentials and common secret fields at output boundaries."""
from __future__ import annotations

import logging
import re
import threading
import traceback
import urllib.parse
import contextlib
import functools
import os
import sys


class SecretRedactor:
    def __init__(self):
        self._values = set()
        self._lock = threading.Lock()

    def register(self, value):
        if value:
            with self._lock:
                self._values.add(str(value))
                self._values.add(urllib.parse.quote(str(value), safe=""))

    def __call__(self, value):
        text = str(value)
        with self._lock:
            values = sorted(self._values, key=len, reverse=True)
        for secret in values:
            text = re.sub(re.escape(secret), "[REDACTED]", text, flags=re.IGNORECASE)
        text = re.sub(r"(?i)(\b(?:mission_code|access_token|api_key|token)\s*[=:]\s*)[^&\s,;\}\]]+",
                      r"\1[REDACTED]", text)
        text = re.sub(r"(?i)([\"'](?:mission_code|access_token|api_key|token)[\"']\s*:\s*[\"'])[^\"']*",
                      r"\1[REDACTED]", text)
        text = re.sub(r"(?i)(\bBearer\s+)\S+", r"\1[REDACTED]", text)
        return text


REDACT = SecretRedactor()
_installed = False
_install_lock = threading.Lock()


def install_log_redaction():
    """Cover every logger/handler, including future urllib3/requests handlers.

    Sanitize exception text before handlers format it. The original exception
    object is not passed down the logging chain once rendered.
    """
    global _installed
    with _install_lock:
        if _installed:
            return
        previous = logging.getLogRecordFactory()

        def factory(*args, **kwargs):
            record = previous(*args, **kwargs)
            record.msg = REDACT(record.getMessage())
            record.args = ()
            if record.exc_info:
                record.exc_text = REDACT("".join(traceback.format_exception(*record.exc_info)))
                record.exc_info = None
            elif record.exc_text:
                record.exc_text = REDACT(record.exc_text)
            if record.stack_info:
                record.stack_info = REDACT(record.stack_info)
            return record

        logging.setLogRecordFactory(factory)
        _installed = True


def register_secret(value):
    REDACT.register(value)
    install_log_redaction()


def safe_detail(value, limit=300):
    """Bound text only after redaction, so truncation cannot expose a prefix."""
    import json
    if isinstance(value, (list, dict)):
        try:
            value = json.dumps(value, ensure_ascii=False, default=lambda _: "<unserializable>")
        except (ValueError, TypeError):
            value = "<invalid error detail>"
    return REDACT(value)[:limit]


class _RedactingStream:
    """Line-buffer Python writes so split print calls cannot expose a credential."""
    def __init__(self, stream):
        self.stream, self.pending = stream, ""
        self.lock = threading.RLock()

    def write(self, text):
        with self.lock:
            self.pending += str(text)
            while "\n" in self.pending:
                line, self.pending = self.pending.split("\n", 1)
                self.stream.write(REDACT(line) + "\n")
        return len(text)

    def flush(self):
        with self.lock:
            if self.pending:
                self.stream.write(REDACT(self.pending))
                self.pending = ""
            self.stream.flush()

    def __getattr__(self, name):
        return getattr(self.stream, name)


def _register_environment_secrets():
    for name, value in os.environ.items():
        if name in ("MAICON_CODE", "MAICON_MISSION_CODE") or re.search(
                r"(?:LLM|OPENAI|ANTHROPIC).*_(?:KEY|TOKEN)$", name, re.I):
            register_secret(value)


def redacted_output(function):
    """Protect startup/plugin Python output and the exception reaching callers.

    This is an output boundary, not isolation of native libraries or subprocesses.
    Adapters must not send credentials directly to file descriptors or children.
    """
    @functools.wraps(function)
    def wrapped(*args, **kwargs):
        _register_environment_secrets()
        stdout, stderr = _RedactingStream(sys.stdout), _RedactingStream(sys.stderr)
        with contextlib.redirect_stdout(stdout), contextlib.redirect_stderr(stderr):
            try:
                return function(*args, **kwargs)
            except Exception as error:
                # Preserve standard failure categories used by startup cleanup/tests,
                # but never pass an adapter's custom __str__ or traceback onward.
                allowed = (ValueError, TypeError, RuntimeError, TimeoutError, OSError,
                           ConnectionError, ImportError, AttributeError)
                cls = type(error) if type(error) in allowed else RuntimeError
                raise cls(REDACT(str(error))) from None
            finally:
                stdout.flush()
                stderr.flush()
    return wrapped
