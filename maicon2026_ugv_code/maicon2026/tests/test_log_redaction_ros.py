"""Actual ROS logging compatibility, isolated from pytest's capture topology."""
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import textwrap

import pytest


ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.spec("S0.4", "SA.1")
@pytest.mark.parametrize("installation_order", ["before_launch", "after_launch"])
def test_ros_nonpropagating_logger_redacts_before_its_own_handler_without_reconfiguration(installation_order):
    if importlib.util.find_spec("launch") is None:
        pytest.skip("ROS launch is unavailable; source the Humble environment for this integration check")
    # Import ROS only in the child: launch.logging intentionally changes the
    # default logger class. Its own real stream handler must remain supported.
    program = textwrap.dedent('''
        import json
        import logging
        import sys
        from maicon26.link.secrets import register_secret

        old, new = "RoSFixtureCode91", "RoSFixtureCode92"  # Invented test values.
        if sys.argv[1] == "before_launch":
            register_secret(old)
        import launch.logging as ros_logging

        logger = logging.getLogger("fixture.private.transport")
        assert isinstance(logger, ros_logging.LaunchLogger)
        assert logger.propagate is False
        stream = ros_logging.handlers.StreamHandler(sys.stderr)
        stream.setFormatter(logging.Formatter("%(levelname)s %(message)s"))
        logger.addHandler(stream)
        logger.setLevel(logging.WARNING)

        root_records = []
        class RootProbe(logging.Handler):
            def emit(self, record):
                root_records.append(record)
        root_probe = RootProbe()
        logging.getLogger().addHandler(root_probe)
        before = (logging.getLoggerClass(), tuple(logger.handlers), logger.propagate,
                  tuple(logging.getLogger().handlers), logger.level)
        register_secret(old)
        register_secret(new)
        factory = logging.getLogRecordFactory()
        register_secret(old)
        assert logging.getLogRecordFactory() is factory
        logger.warning("credential rotation old=%s new=%s", old.lower(), new.upper())
        try:
            raise RuntimeError("transport failed https://fixture.invalid/?mission_code=" + old.swapcase())
        except RuntimeError:
            logger.exception("transport exception")
        after = (logging.getLoggerClass(), tuple(logger.handlers), logger.propagate,
                 tuple(logging.getLogger().handlers), logger.level)
        assert before == after
        assert root_records == []
        stream.flush()
        print(json.dumps({"logger_class": type(logger).__name__, "propagate": logger.propagate,
                          "root_captured": len(root_records), "configuration_unchanged": before == after}))
    ''')
    completed = subprocess.run([sys.executable, "-c", program, installation_order], cwd=ROOT,
                               capture_output=True, text=True, encoding="utf-8", timeout=20)
    assert completed.returncode == 0, completed.stdout + completed.stderr
    report = json.loads(completed.stdout)
    assert report == {"logger_class": "LaunchLogger", "propagate": False,
                      "root_captured": 0, "configuration_unchanged": True}
    output = completed.stdout + completed.stderr
    assert "rosfixturecode91" not in output.lower() and "rosfixturecode92" not in output.lower()
    assert "credential rotation old=[REDACTED] new=[REDACTED]" in completed.stderr
    assert "transport exception" in completed.stderr and "RuntimeError:" in completed.stderr
    assert "mission_code=[REDACTED]" in completed.stderr
