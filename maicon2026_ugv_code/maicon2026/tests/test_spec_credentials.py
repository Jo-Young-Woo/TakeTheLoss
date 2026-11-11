"""Credential entry follows the local API guide without network access."""
import getpass
import warnings

import pytest

from maicon26.credentials import read_mission_code
from maicon26.link.secrets import REDACT


@pytest.mark.spec("S0.2", "S0.4")
def test_environment_priority_preserves_case_and_registers_case_insensitive_redaction(tmp_path):
    path = tmp_path / "credential.txt"
    path.write_text("Z9y8X7w6")
    value = "Ab12Cd34"
    result = read_mission_code(path, environ={"MAICON_CODE": value, "MAICON_MISSION_CODE": "Z9y8X7w6"})
    assert result == value
    assert value.casefold() not in REDACT(value.upper()).casefold()


@pytest.mark.spec("S0.2", "S0.4")
def test_legacy_environment_and_file_are_supported_without_interactive_prompt(tmp_path):
    value = "Ab12Cd34"
    assert read_mission_code(environ={"MAICON_MISSION_CODE": value}) == value
    path = tmp_path / "credential.txt"
    path.write_text(value+"\n", encoding="utf-8")
    assert read_mission_code(path, environ={}) == value


@pytest.mark.spec("S0.2", "S0.4")
@pytest.mark.parametrize("source", ["environment", "file"])
def test_invalid_noninteractive_format_warns_without_echo_and_continues(source, tmp_path, capsys):
    value = "unusual-field-format"
    path = tmp_path / "credential.txt"
    path.write_text(value)
    result = read_mission_code(path if source == "file" else None,
                               environ={} if source == "file" else {"MAICON_CODE": value})
    captured = capsys.readouterr()
    assert result == value
    assert "WARN" in captured.err and value.casefold() not in (captured.out+captured.err).casefold()


@pytest.mark.spec("S0.2", "S0.4")
def test_hidden_input_retries_at_most_three_times_without_echo(capsys):
    attempts = []
    values = iter(("bad", "too-long-code", "Ab12Cd34"))
    def prompt(message):
        attempts.append(message)
        return next(values)
    assert read_mission_code(environ={}, prompt=prompt) == "Ab12Cd34"
    assert len(attempts) == 3
    captured = capsys.readouterr()
    assert "bad" not in captured.err and "too-long-code" not in captured.err
    attempts.clear()
    with pytest.raises(ValueError, match="세 번"):
        read_mission_code(environ={}, prompt=lambda message: attempts.append(message) or "invalid")
    assert len(attempts) == 3


@pytest.mark.spec("S0.4")
def test_non_tty_getpass_fallback_is_stopped_before_echo():
    reached = []
    def would_echo(message):
        warnings.warn("hidden terminal unavailable", getpass.GetPassWarning)
        reached.append("unsafe-echo")
        return "Ab12Cd34"
    with pytest.raises(ValueError, match="숨김 입력"):
        read_mission_code(environ={}, prompt=would_echo)
    assert reached == []


@pytest.mark.spec("S0.4")
def test_unreadable_file_error_does_not_disclose_its_name_or_contents(tmp_path):
    name = "operator-private-name"
    with pytest.raises(ValueError) as error:
        read_mission_code(tmp_path / name, environ={})
    assert name not in str(error.value)


@pytest.mark.spec("S0.4", "SA.1")
def test_llm_startup_print_split_writes_and_uncaught_traceback_are_redacted(monkeypatch, capsys):
    import sys
    import traceback
    from types import ModuleType
    from maicon26.preflight import load_llm_factory
    from maicon26.link.secrets import register_secret

    secret = "FixturePrivateValue37"
    register_secret(secret)
    module = ModuleType("private_factory_fixture")

    def fail():
        print(secret.upper())
        sys.stderr.write(secret[:7])
        sys.stderr.write(secret[7:] + "\n")
        raise RuntimeError("adapter connection failed " + secret.lower())

    module.create = fail
    monkeypatch.setitem(sys.modules, module.__name__, module)
    with pytest.raises(RuntimeError) as caught:
        load_llm_factory(module.__name__ + ":create")
    output = capsys.readouterr()
    rendered = "".join(traceback.format_exception(caught.type, caught.value, caught.tb))
    assert secret.lower() not in (output.out + output.err + rendered).lower()
    assert "[REDACTED]" in output.out and "[REDACTED]" in output.err
    assert "adapter connection failed" in str(caught.value)
