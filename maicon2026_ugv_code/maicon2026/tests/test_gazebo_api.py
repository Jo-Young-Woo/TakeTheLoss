"""Actual JPEG adapter contracts; all requests stay in-process, no listener."""
import datetime as dt
import importlib.util
import io
import json
import threading
from pathlib import Path
from types import SimpleNamespace
from urllib.parse import urlencode

from PIL import Image
import pytest

from maicon26.link import schema
from maicon26.sim.gazebo_api import GazeboApiAdapter, jpeg_from_ros_image, make_http_server


EPOCH = dt.datetime(2026, 11, 9, tzinfo=dt.timezone.utc)


def jpeg(color=(255, 0, 0), format="JPEG"):
    buffer = io.BytesIO()
    Image.new("RGB", (8, 4), color).save(buffer, format=format, quality=95)
    return buffer.getvalue()


def adapter(config=None):
    result = GazeboApiAdapter({"Test0001": "B-2", "Test0002": "R-1"}, epoch=EPOCH, operator_config=config)
    result.update_time(10.)
    assert result.update_state({"phase": "running", "elapsed": 10., "destroyed": {"BLUE": 0, "RED": 0}})
    return result


def get(api, endpoint="drone", code="Test0001", **query):
    return api.request("GET", "/api/report/"+endpoint+"?"+urlencode({"mission_code": code, **query}))


def post(api, endpoint, body):
    return api.request("POST", "/api/report/"+endpoint, body={"mission_code": "Test0001", **body})


@pytest.mark.spec("S4.1", "S4.2", "S4.6", "S4.8")
def test_real_jpeg_bytes_capture_kst_and_camp_degradation_reuse_mock_contract():
    api = adapter()
    first = jpeg()
    assert api.capture_jpeg(first, 10.)
    status, raw, headers = get(api)
    assert status == 200 and raw == first
    assert headers["X-Captured-At"] == "2026-11-09T09:00:10.000+09:00"
    assert {k: headers[k] for k in ("X-Frame-Interval-S", "X-Camera", "X-Test-Mode", "X-Camp", "X-Destroyed")} == {
        "X-Frame-Interval-S": "1.0", "X-Camera": "main", "X-Test-Mode": "off", "X-Camp": "BLUE", "X-Destroyed": "0"}
    for stamp, destroyed, expected in ((11., 1, 10.), (12., 1, 12.), (14., 2, 12.), (17., 2, 17.)):
        api.update_time(stamp)
        api.update_state({"phase": "running", "elapsed": stamp, "destroyed": {"BLUE": destroyed, "RED": 0}})
        assert api.capture_jpeg(jpeg((0, 255, 0)), stamp)
        assert get(api)[2]["X-Captured-At"] == api.mock._kst(expected, iso=True)
        assert get(api, code="Test0002")[2]["X-Captured-At"] == api.mock._kst(stamp, iso=True)
    api.update_state({"phase": "running", "elapsed": 17., "destroyed": {"BLUE": 3, "RED": 0}})
    assert get(api)[0] == 403 and get(api, code="Test0002")[0] == 200


@pytest.mark.spec("S4.5", "S4.6")
def test_old_duplicate_bad_and_far_future_inputs_never_replace_last_good_timestamp():
    api = adapter()
    first = jpeg()
    assert api.capture_jpeg(first, 10.)
    api.update_time(12.)
    assert not api.capture_jpeg(jpeg(), 8.)
    assert not api.capture_jpeg(jpeg(), 10.)
    assert not api.capture_jpeg(b"not a jpeg", 12.)
    assert not api.capture_jpeg(jpeg(format="PNG"), 12.)
    assert not api.capture_jpeg(jpeg(), 100.)
    assert not api.capture_jpeg(jpeg(), float("nan"))
    assert get(api)[1] == first
    assert get(api)[2]["X-Captured-At"] == api.mock._kst(10., iso=True)
    assert api.stats["stale_rejected"] == 1 and api.stats["future_rejected"] == 1


@pytest.mark.spec("S4.5", "S4.6")
def test_small_dds_clock_lead_is_buffered_but_never_published_in_the_future():
    api = adapter()
    assert not api.capture_jpeg(jpeg(), 10.1)
    assert get(api)[0] == 503
    api.update_time(10.05)
    assert get(api)[0] == 503
    api.update_time(10.1)
    assert get(api)[0] == 200
    assert get(api)[2]["X-Captured-At"] == api.mock._kst(10.1, iso=True)


@pytest.mark.spec("S4.3", "S4.4", "S4.7")
def test_phase_and_practice_headers_are_mockserver_rules_not_adapter_reimplementations():
    api = adapter({"test_mode": True})
    api.capture_jpeg(jpeg(), 10.)
    api.update_state({"phase": "reporting", "elapsed": 10., "destroyed": {"BLUE": 3, "RED": 0}})
    assert get(api)[0] == 409
    status, _, headers = get(api, camera="main")
    assert status == 200 and headers["X-Test-Mode"] == "on"
    assert "X-Camp" not in headers and "X-Destroyed" not in headers
    assert get(api, camera="test1")[0] == 503, "A missing camera cannot reuse main pixels"


@pytest.mark.spec("S1.4", "S4.5")
def test_gazebo_clock_rewind_invalidates_images_and_requires_adapter_restart():
    api = adapter()
    api.capture_jpeg(jpeg(), 10.)
    assert not api.update_time(0.)
    assert get(api)[0] == 409
    assert not api.update_time(11.) and not api.capture_jpeg(jpeg(), 11.)
    assert not api.mock._frames and api.clock.invalid


@pytest.mark.spec("S2.1", "S2.5", "S9.3", "S9.4")
def test_only_explicit_operator_content_is_published_at_start_and_240_seconds():
    empty = adapter()
    assert json.loads(get(empty, "order")[1])["reason"] == schema.REASON_NO_POST
    config = {"orders": {"B-2": "운영자가 제공한 시험 명령"},
              "targets": {"BLUE": {"objective": "RED_MAINTENANCE", "objective_label": "정비소"}}}
    api = adapter(config)
    order = json.loads(get(api, "order")[1])["order"]
    assert order["tactical_command"] == config["orders"]["B-2"]
    assert order["issued_at"] == "2026-11-09 09:00:00"
    assert not json.loads(get(api, "target")[1])["accepted"]
    api.update_time(240.)
    api.update_state({"phase": "running", "elapsed": 240., "destroyed": {"BLUE": 0, "RED": 0}})
    target = json.loads(get(api, "target")[1])["order"]
    assert target == {**config["targets"]["BLUE"], "issued_at": "2026-11-09 09:04:00"}
    assert not json.loads(get(api, "target", code="Test0002")[1])["accepted"]


@pytest.mark.spec("S2.4", "S9.3")
def test_live_clock_longpoll_releases_state_lock_and_does_not_advance_simulation_time():
    api = adapter({"targets": {"BLUE": {"objective": "RED_MAINTENANCE", "objective_label": "정비소"}}})
    api.update_time(200.)
    api.update_state({"phase": "running", "elapsed": 200., "destroyed": {"BLUE": 0, "RED": 0}})
    replies = []
    worker = threading.Thread(target=lambda: replies.append(get(api, "target", wait=120)), daemon=True)
    try:
        worker.start()
        api.update_time(240.)
        api.update_state({"phase": "running", "elapsed": 240., "destroyed": {"BLUE": 0, "RED": 0}})
        worker.join(timeout=1.)
        assert not worker.is_alive()
        assert json.loads(replies[0][1])["accepted"]
        assert api.clock() == 240., "An HTTP wait must not fast-forward Gazebo"
    finally:
        api.close()
        worker.join(timeout=1.)


@pytest.mark.spec("S0.5", "S10.4", "S11.3", "S11.4")
def test_reporting_closes_m1_m3_and_accepts_m6_then_relays_only_local_receipt():
    api = adapter()
    assert post(api, "interpretation", {"target": "RED_RADAR"})[0] == 200
    api.update_state({"phase": "reporting", "elapsed": 10., "destroyed": {"BLUE": 0, "RED": 0}})
    assert post(api, "interpretation", {"target": "RED_RADAR"})[0] == 409
    assert post(api, "detection", {"sectors": [{"sector": "CM", "objects": []}]})[0] == 409
    for ranking in (["R-1"], ["R-2"]):
        assert post(api, "situation", {"ranking": ranking})[0] == 200
    assert api.drain_controls() == [{"cmd": "m6_received", "slot": "B-2"}]
    assert api.drain_controls() == []


@pytest.mark.parametrize("encoding,pixel", [("bgr8", bytes([0, 0, 255])), ("rgb8", bytes([255, 0, 0]))])
def test_ros_frame_encoder_preserves_channel_order_and_ignores_padded_bytes(encoding, pixel):
    row = pixel*8+bytes([0, 255, 0, 255])
    message = SimpleNamespace(width=8, height=4, step=len(row), encoding=encoding, data=row*4)
    image = Image.open(io.BytesIO(jpeg_from_ros_image(message, quality=95))).convert("RGB")
    red, green, blue = image.getpixel((7, 3))
    assert red > 245 and green < 10 and blue < 10
    message.data = message.data[:-1]
    with pytest.raises(ValueError, match="Truncated"):
        jpeg_from_ros_image(message)


@pytest.mark.spec("S0.4")
def test_diagnostics_are_bounded_and_credentials_are_redacted_case_insensitively():
    api = adapter()
    for _ in range(150):
        assert get(api, "me", code="tEsT0001")[0] == 200
    assert len(api.mock.requests) == 128
    assert "test0001" not in json.dumps(api.mock.requests).lower()


def test_nonloopback_bind_is_rejected_before_constructing_any_socket(monkeypatch):
    monkeypatch.setattr("socket.socket", lambda *a, **kw: pytest.fail("socket must not be created"))
    with pytest.raises(ValueError, match="loopback"):
        make_http_server(adapter(), "0.0.0.0")


@pytest.mark.spec("S0.2", "S0.4")
def test_launcher_rejects_secret_arguments_without_echo(capsys):
    path = Path(__file__).resolve().parents[1]/"scripts/run_arena_mock_api.py"
    spec = importlib.util.spec_from_file_location("arena_mock_cli", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    value = "Test0001"
    with pytest.raises(ValueError) as error:
        module.parse_args(["--mission-code", value])
    assert value.lower() not in (str(error.value)+capsys.readouterr().err).lower()
