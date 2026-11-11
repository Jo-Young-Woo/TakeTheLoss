import os, sys
import pytest
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))


def pytest_configure(config):
    config.addinivalue_line("markers", "spec(*ids): official API specification identifiers covered by a test")
    config.addinivalue_line("markers", "slow: long simulation scenario; deselect with -m 'not slow'")


@pytest.fixture(autouse=True)
def isolate_registered_test_credentials():
    # Production retains previous credentials to redact in-flight errors after
    # rotation. Independent tests must not redact unrelated words merely because
    # another test intentionally used one as an invalid credential.
    from maicon26.link.secrets import REDACT
    with REDACT._lock:
        original = set(REDACT._values)
    yield
    with REDACT._lock:
        REDACT._values = original
