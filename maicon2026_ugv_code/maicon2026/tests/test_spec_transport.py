"""API transport contracts exercised without sockets, threads or real-time sleeps.

All credentials below are invented fixture strings. Specification IDs refer to
the supplied 2026-11-04 participant API snapshot, not a live server contract.
"""
import datetime as dt
import json
import logging
import threading
from urllib.parse import parse_qs, urlsplit

import pytest

from maicon26.link import schema
from maicon26.link.async_link import AsyncServerLink
from maicon26.link.clock_sync import ClockSync
from maicon26.link.http import HttpServerLink
from maicon26.link.secrets import register_secret, safe_detail
from maicon26.sim.mockserver import FakeClock, MockServer


CODE = "TeSt1234"
DATE = "Wed, 04 Nov 2026 03:00:00 GMT"
PAYLOADS = {
    "m1": {"target": "RADAR_FACILITY", "search_sectors": ["R-S1", "COMMON"]},
    "m3": {"sectors": {"R-S1": {"TANK": 2, "DRUM": 1}, "COMMON": {"TRUCK": 1}}},
    "m6": {"ranking": ["R-UGV-2", "R-UGV-1"]},
}


class OneIteration:
    """Event replacement: one loop iteration, record its requested wait."""
    def __init__(self, clock=None):
        self.done = False
        self.waits = []
        self.clock = clock

    def is_set(self):
        return self.done

    def set(self):
        self.done = True

    def wait(self, seconds):
        self.waits.append(seconds)
        if self.clock is not None:
            self.clock.advance(seconds)
        self.done = True
        return True


def fixture_link(phase="running", **kwargs):
    clock = FakeClock()
    server = MockServer({CODE: "B-2"}, clock=clock)
    server.set_phase(phase)
    link = HttpServerLink("http://fixture.invalid", CODE, transport=server, clock=clock,
                          decode_drone=False, **kwargs)
    assert link.me() is not None
    return link, server, clock


def drain_messages(link):
    # Avoid start_background(); no thread ever contacts a fixture or external host.
    messages = []
    while not link._q.empty():
        messages.append(link._q.get_nowait())
    return messages


@pytest.mark.spec("S5.9", "S7.6", "S11.5")
@pytest.mark.parametrize("kind", ["m1", "m3", "m6"])
def test_received_mismatch_is_not_ack_and_never_posts_more_than_twice(kind, caplog):
    link, server, _ = fixture_link("reporting" if kind == "m6" else "running")
    endpoint = schema.ENDPOINTS[kind]
    server.inject(endpoint, "status", status=200,
                  body={"accepted": True, "received": {}}, times=5)
    with caplog.at_level(logging.WARNING):
        results = [link.submit(kind, PAYLOADS[kind]) for _ in range(5)]
    assert results == [False] * 5
    assert server.attempts[endpoint] == 2
    assert link.stats["submit_ok"] == 0
    assert len([r for r in caplog.records if "received payload differs" in r.message]) == 1


@pytest.mark.spec("S5.9", "S7.6", "S11.5")
@pytest.mark.parametrize("kind", ["m1", "m3", "m6"])
def test_one_mismatch_then_correct_echo_recovers(kind):
    link, server, _ = fixture_link("reporting" if kind == "m6" else "running")
    endpoint = schema.ENDPOINTS[kind]
    server.inject(endpoint, "status", status=200, body={"accepted": True, "received": {}})
    assert not link.submit(kind, PAYLOADS[kind])
    assert link.submit(kind, PAYLOADS[kind])
    assert server.attempts[endpoint] == 2


@pytest.mark.spec("S5.9", "S7.6")
@pytest.mark.parametrize("kind", ["m1", "m3"])
def test_received_reordering_is_not_a_mismatch(kind):
    link, server, _ = fixture_link()
    server.received_order = "reverse"
    assert link.submit(kind, PAYLOADS[kind])
    assert not link._echo_mismatch_attempts
    assert server.attempts[schema.ENDPOINTS[kind]] == 1


@pytest.mark.spec("S2.6", "S9.5", "SA.5")
@pytest.mark.parametrize("reason,delay", [
    (schema.REASON_ORDER_NOT_YET, .5), (schema.REASON_TARGET_NOT_YET, .5),
    (schema.REASON_NOT_ASSIGNED, 10.), (schema.REASON_NO_POST, 5.),
    (schema.REASON_NO_REHEARSAL, 5.), ("unrecognised fixture reason", 1.),
])
def test_reason_backoff_and_warning_only_on_change(reason, delay, caplog):
    link, _, _ = fixture_link()
    with caplog.at_level(logging.WARNING):
        assert link._reason_delay("order", reason) == delay
        assert link._reason_delay("order", reason) == delay
        assert len(caplog.records) == 1
        # A different endpoint has its own warning state.
        assert link._reason_delay("target", reason) == delay
        assert len(caplog.records) == 2
        other = schema.REASON_NO_POST if reason != schema.REASON_NO_POST else schema.REASON_NOT_ASSIGNED
        link._reason_delay("order", other)
        link._reason_delay("order", reason)
        assert len(caplog.records) == 4


@pytest.mark.spec("S2.6", "S9.5")
@pytest.mark.parametrize("kind", ["order", "target"])
def test_pending_publication_loop_has_no_busy_retry(kind):
    link, server, clock = fixture_link("waiting")
    link._stop_ev = OneIteration(clock)
    link._wait_loop(kind)
    assert server.attempts[kind] == 1
    assert link._stop_ev.waits == [.5]


@pytest.mark.spec("S5.8", "S7.8", "S11.7")
@pytest.mark.parametrize("reason", ["not_assigned", "excluded", "not_started"])
@pytest.mark.parametrize("reset_phase", ["idle", "waiting"])
def test_permanent_409_stops_one_kind_until_next_match(reason, reset_phase):
    link, server, _ = fixture_link()
    server.inject("detection", "status", status=409,
                  body={"detail": schema.submission_error("m3", reason, "running")})
    assert not link.submit("m3", PAYLOADS["m3"])
    assert link.submission_allowed("m3") is False
    assert not link.submit("m3", PAYLOADS["m3"])
    assert server.attempts["detection"] == 1
    # Rejecting M3 does not silently discard independent M1 evidence.
    assert link.submit("m1", PAYLOADS["m1"])
    server.set_phase(reset_phase)
    link.me()
    server.set_phase("running")
    link.me()
    assert link.submit("m3", PAYLOADS["m3"])
    assert server.attempts["detection"] == 2


@pytest.mark.spec("S5.8", "S7.8")
def test_phase_mismatch_409_is_retriable_when_running_resumes():
    link, server, _ = fixture_link()
    server.inject("detection", "status", status=409,
                  body={"detail": schema.submission_error("m3", "phase", "ready")})
    assert not link.submit("m3", PAYLOADS["m3"])
    assert "m3" not in link._permanent_rejections
    assert link.submit("m3", PAYLOADS["m3"])


def drone_link(headers=None, **kwargs):
    link, server, clock = fixture_link(**kwargs)
    metadata = {"Content-Type": "image/jpeg", "X-Captured-At": "2026-11-04T12:00:00+09:00",
                "X-Frame-Interval-S": "1", "X-Camera": "main", "X-Test-Mode": "off",
                "X-Camp": "BLUE", "X-Destroyed": "0", "Date": DATE}
    metadata.update(headers or {})
    requests = []

    def transport(request, timeout):
        requests.append((request, timeout))
        if urlsplit(request.full_url).path.endswith("/drone"):
            return 200, b"fixture JPEG bytes; decoding deliberately disabled", metadata
        return server(request, timeout)

    link._transport = transport
    return link, server, clock, metadata, requests


@pytest.mark.spec("S4.3", "S4.4", "S4.7")
@pytest.mark.parametrize("raw,expected", [("2", 2.), (".25", .25), ("0", 1.),
    ("-3", 1.), ("nan", 1.), ("inf", 1.), ("-inf", 1.), ("broken", 1.), (None, 1.)])
def test_drone_interval_positive_finite_or_one_second(raw, expected):
    link, _, _, _, _ = drone_link({"X-Frame-Interval-S": raw})
    frame = link.get_uav()
    assert frame is not None
    assert frame.interval_s == expected


@pytest.mark.spec("S4.3", "S4.4", "S4.7")
@pytest.mark.parametrize("raw,expected", [("-2", 0), ("0", 0), ("1", 1), ("2", 2),
                                         ("3", 2), ("99", 2), ("bad", None), (None, None)])
def test_drone_destroyed_metadata_is_clamped(raw, expected):
    link, _, _, _, _ = drone_link({"X-Destroyed": raw})
    assert link.get_uav().destroyed == expected


@pytest.mark.spec("S1.5", "S4.7")
def test_me_assignment_overrides_conflicting_drone_camp(caplog):
    link, _, _, _, _ = drone_link({"X-Camp": "RED"})
    with caplog.at_level(logging.WARNING):
        frame = link.get_uav()
    assert frame.camp == "RED"
    assert link.identity() == ("BLUE", 2)
    assert link.team == "BLUE"
    assert any("/me remains authoritative" in r.message for r in caplog.records)


@pytest.mark.spec("S1.5", "S4.7")
def test_camp_is_only_provisional_when_me_unavailable_and_never_invents_slot():
    link, _, _, _, _ = drone_link({"X-Camp": "RED"})
    link._ident = None
    link._me_err = "offline fixture"
    link._started_bg = True  # identity() must return the cache, not fetch /me.
    assert link.get_uav().camp == "RED"
    assert link.team == "RED"
    assert link._provisional_camp == "RED"
    assert link.identity() is None


@pytest.mark.spec("S4.3", "S4.4", "S4.7")
def test_drone_metadata_and_request_mode_do_not_invent_detections():
    link, _, _, _, requests = drone_link({"X-Destroyed": "1", "X-Frame-Interval-S": "2"})
    frame = link.get_uav()
    assert frame.camera == "main" and frame.test_mode is False
    assert frame.destroyed == 1 and frame.interval_s == 2
    assert frame.detections == [] and frame.boxes == []
    assert "camera" not in parse_qs(urlsplit(requests[-1][0].full_url).query)
    assert frame.stamp == dt.datetime(2026, 11, 4, 3, tzinfo=dt.timezone.utc).timestamp()


@pytest.mark.spec("S4.3", "S4.4", "S4.7")
def test_duplicate_and_older_captured_frames_are_ignored():
    link, _, clock, headers, requests = drone_link()
    assert link.get_uav() is not None
    assert link.get_uav() is None
    assert len(requests) == 1  # period is enforced even if the caller polls rapidly.
    clock.advance(1)
    assert link.get_uav() is None
    headers["X-Captured-At"] = "2026-11-04T11:59:59+09:00"
    clock.advance(1)
    assert link.get_uav() is None
    headers["X-Captured-At"] = "2026-11-04T12:00:02+09:00"
    clock.advance(1)
    assert link.get_uav() is not None


@pytest.mark.spec("S0.2", "S0.3")
@pytest.mark.parametrize("value,valid", [(CODE, True), ("abcdefgh", True), ("12345678", True),
    ("short", False), ("123456789", False), ("테스트12345", False), ("abcd_123", False),
    ("abcd1234\n", False), (None, False), (True, False)])
def test_mission_code_is_eight_ascii_alphanumeric_characters(value, valid):
    assert schema.mission_code_valid(value) is valid
    if valid:
        link = HttpServerLink("http://fixture.invalid", value)
        assert parse_qs(urlsplit(link._url("me")).query)["mission_code"] == [value]
    else:
        with pytest.raises(ValueError):
            HttpServerLink("http://fixture.invalid", value)


@pytest.mark.spec("S0.2")
def test_nonstandard_fixture_code_requires_explicit_opt_out():
    with pytest.raises(ValueError):
        HttpServerLink("http://fixture.invalid", "fixture")
    assert HttpServerLink("http://fixture.invalid", "fixture", strict_code=False)._code == "fixture"


@pytest.mark.spec("S0.4")
@pytest.mark.parametrize("case", [str.lower, str.upper, str.swapcase], ids=["lower", "upper", "swap"])
def test_global_record_factory_redacts_other_loggers_and_interpolated_args(case, caplog):
    register_secret(CODE)
    with caplog.at_level(logging.WARNING):
        logging.getLogger("thirdparty.transport.fixture").warning("request URL=%s", "http://fixture.invalid/?q=" + case(CODE))
    assert CODE.lower() not in caplog.text.lower()
    assert "[REDACTED]" in caplog.text


@pytest.mark.spec("S0.4")
def test_global_record_factory_redacts_exception_url_before_handler_sees_it(caplog):
    register_secret(CODE)
    try:
        raise ConnectionError("GET http://fixture.invalid/?mission_code=" + CODE.lower())
    except ConnectionError:
        with caplog.at_level(logging.ERROR):
            logging.getLogger("urllib3.fixture").exception("fixture transport exception")
    assert CODE.lower() not in caplog.text.lower()
    assert "ConnectionError" in caplog.text and "[REDACTED]" in caplog.text
    assert caplog.records[-1].exc_info is None


@pytest.mark.spec("S0.4")
@pytest.mark.parametrize("detail", [
    {"detail": [{"loc": ["query", "mission_code"], "msg": CODE.lower()}]},
    [{"raw": CODE.upper(), "token": "unregistered-fixture-token"}],
    "non-JSON response mission_code=" + CODE.swapcase() + "&token=unregistered-fixture-token",
])
def test_raw_error_details_mask_before_truncation(detail):
    register_secret(CODE)
    rendered = safe_detail(detail)
    assert CODE.lower() not in rendered.lower()
    assert "unregistered-fixture-token" not in rendered
    assert "[REDACTED]" in rendered
    assert safe_detail(CODE, limit=5) == "[REDA"


@pytest.mark.spec("S0.4")
def test_set_mission_code_keeps_old_and_new_values_redacted(caplog):
    link = HttpServerLink("http://fixture.invalid", CODE)
    replacement = "NeXt5678"
    link.set_mission_code(replacement)
    with caplog.at_level(logging.WARNING):
        logging.getLogger("http.fixture").warning("old=%s new=%s", CODE.lower(), replacement.upper())
    assert CODE.lower() not in caplog.text.lower()
    assert replacement.lower() not in caplog.text.lower()
    assert caplog.text.count("[REDACTED]") == 2
    assert parse_qs(urlsplit(link._url("me")).query)["mission_code"] == [replacement]


@pytest.mark.spec("S0.4")
def test_non_json_and_transport_exception_paths_do_not_leak_credentials():
    link = HttpServerLink("http://fixture.invalid", CODE)
    link._transport = lambda request, timeout: (503, ("raw=" + CODE.lower()).encode(), {"Date": DATE})
    assert link.me() is None
    assert CODE.lower() not in link._me_err.lower()

    def fail(request, timeout):
        raise TimeoutError(request.full_url)

    link._transport = fail
    assert link.me() is None
    assert CODE.lower() not in link._me_err.lower()
    assert "[REDACTED]" in link._me_err


@pytest.mark.spec("S0.7", "S2.4", "S2.5", "S9.4")
@pytest.mark.parametrize("elapsed", [0., 1., 137., 299., 300.])
def test_clock_sync_http_date_and_kst_issued_at_recover_elapsed(elapsed):
    sync = ClockSync()
    assert sync.observe(DATE, 100., 100.)
    assert sync.elapsed("2026-11-04 12:00:00", 100. + elapsed) == elapsed
    assert sync.uncertainty_s == 1.


@pytest.mark.spec("S0.7", "S2.4")
@pytest.mark.parametrize("issued,now", [("2026-11-04 12:00:01", 100.),
    ("2026-11-04 12:00:00", 401.), ("invalid", 100.), (None, 100.)])
def test_clock_sync_rejects_future_old_or_malformed_match_times(issued, now):
    sync = ClockSync()
    assert sync.elapsed(issued, now) is None
    assert sync.observe(DATE, 100., 100.)
    assert sync.elapsed(issued, now) is None


@pytest.mark.spec("S0.7", "S2.4")
@pytest.mark.parametrize("date,sent,received", [(None, 0., 0.), ("bad", 0., 0.),
    (DATE, 5., 4.), (DATE, 0., 5.01), (DATE, 0., float("nan")),
    ("Wed, 04 Nov 2026 03:00:00", 0., 0.)])
def test_clock_sync_rejects_unusable_date_or_network_sample(date, sent, received):
    sync = ClockSync()
    assert not sync.observe(date, sent, received)
    assert sync.offset is None


@pytest.mark.spec("S0.7", "S2.4", "S3.1", "S10.2")
def test_restart_recovers_elapsed_once_then_obeys_early_report_stop():
    clock = FakeClock()
    server = MockServer({CODE: "B-2"}, clock=clock)
    server.start(orders={"B-2": "fixture command"})
    clock.advance(137)
    link = HttpServerLink("http://fixture.invalid", CODE, clock=clock, transport=server)
    assert link.me()["phase"] == "running"
    assert link._restart_pending
    link._wait_loop("order")
    messages = drain_messages(link)
    start = next(m for m in messages if m.kind == "start")
    assert start.data["restarted"] is True and start.data["clock_sync"] is True
    assert start.data["elapsed_s"] == 137
    assert [m.kind for m in messages] == ["start", "command"]
    server.set_phase("reporting")  # An M5 win may end the match before 300 s.
    link._stop_ev = OneIteration()
    link._me_loop()
    messages = drain_messages(link)
    assert [m.kind for m in messages] == ["stop"]
    assert messages[0].data["phase"] == "reporting"
    assert not link.submission_allowed("m1")
    assert not link.submission_allowed("m3")
    assert link.submission_allowed("m6")


@pytest.mark.spec("S1.6", "S2.5", "S9.3", "S9.4", "S9.8")
@pytest.mark.parametrize("first", ["order", "target"])
@pytest.mark.parametrize("target_s,warning", [(240., False), (244., False), (246., True), (270., True)])
def test_target_publication_time_crosscheck_is_order_independent_and_diagnostic_only(first, target_s, warning, caplog):
    clock = FakeClock()
    server = MockServer({CODE: "B-2"}, clock=clock)
    server.start(orders={"B-2": "fixture command"})
    clock.advance(target_s)
    server.publish_target("BLUE", "RED_MAINTENANCE", "정비소")
    clock.advance(280. - target_s)
    link = HttpServerLink("http://fixture.invalid", CODE, transport=server, clock=clock)
    assert link.me()["phase"] == "running"
    offset = link.clock_sync.offset
    kinds = ("order", "target") if first == "order" else ("target", "order")
    with caplog.at_level(logging.WARNING):
        link._wait_loop(kinds[0])
        assert not caplog.records  # Only one publication timestamp is insufficient.
        link._wait_loop(kinds[1])
        for _ in range(3):
            link._crosscheck_target_time()
    messages = drain_messages(link)
    start = next(m for m in messages if m.kind == "start")
    target = next(m for m in messages if m.kind == "base_command")
    assert start.data["elapsed_s"] == 280.
    assert target.text == "정비소" and target.data["objective"] == "RED_MAINTENANCE"
    assert link.clock_sync.offset == offset and clock() == 280.
    assert link.clock_sync.publication_delta(link._order_issued_at, link._target_issued_at) == target_s
    records = [r for r in caplog.records if "publication time differs" in r.message]
    assert len(records) == int(warning)


@pytest.mark.spec("S2.5", "S9.4")
def test_invalid_target_timestamp_is_a_warning_without_changing_clock_or_objective(caplog):
    link, _, clock = fixture_link()
    link._order_issued_at = "2026-11-04 12:00:00"
    link._target_issued_at = "invalid fixture timestamp"
    offset = link.clock_sync.offset
    with caplog.at_level(logging.WARNING):
        link._crosscheck_target_time()
        link._crosscheck_target_time()
    assert len(caplog.records) == 1
    assert link.clock_sync.offset == offset and clock() == 0.
    assert link.clock_sync.publication_delta(link._order_issued_at, link._target_issued_at) is None


@pytest.mark.spec("S2.5", "S9.4")
@pytest.mark.parametrize("requested,expected", [(None, 20), (-10, 1), (0, 1), (1, 1), (120, 120), (999, 120)])
def test_longpoll_wait_is_bounded_and_http_timeout_has_five_second_margin(requested, expected):
    kwargs = {} if requested is None else {"wait_s": requested}
    link, _, _ = fixture_link(**kwargs)
    calls = []

    def respond(request, timeout):
        calls.append((parse_qs(urlsplit(request.full_url).query), timeout))
        return 200, json.dumps({"accepted": False, "reason": schema.REASON_ORDER_NOT_YET}).encode(), {"Date": DATE}

    link._transport = respond
    link._stop_ev = OneIteration()
    link._wait_loop("order")
    assert calls[0][0]["wait"] == [str(expected)]
    assert calls[0][1] == expected + 5


def inactive_async(inner, monkeypatch):
    monkeypatch.setattr(threading.Thread, "start", lambda self: None)
    return AsyncServerLink(inner)


def process_one_job(wrapper, monkeypatch):
    original_put = wrapper._res.put

    def finish(result):
        original_put(result)
        wrapper._stop.set()

    monkeypatch.setattr(wrapper._res, "put", finish)
    wrapper._submit_loop()


@pytest.mark.spec("S0.5", "S5.7", "S7.7", "S10.2")
@pytest.mark.parametrize("kind", ["m1", "m3"])
def test_queued_submission_is_cancelled_before_post_after_stop(kind, monkeypatch):
    link, server, _ = fixture_link()
    wrapper = inactive_async(link, monkeypatch)
    wrapper.submit_async(kind, PAYLOADS[kind], "queued-fixture")
    # Match STOP processing closes queued reports before the worker drains them.
    wrapper.close_submission(kind)
    server.set_phase("reporting")
    process_one_job(wrapper, monkeypatch)
    assert wrapper.submit_results() == [("queued-fixture", False)]
    assert server.attempts[schema.ENDPOINTS[kind]] == 0


@pytest.mark.spec("S0.5", "S5.7", "S7.7", "S10.2")
@pytest.mark.parametrize("kind", ["m1", "m3"])
def test_submission_worker_checks_changed_server_phase_before_post(kind, monkeypatch):
    link, server, clock = fixture_link()
    wrapper = inactive_async(link, monkeypatch)
    wrapper.submit_async(kind, PAYLOADS[kind], "queued-fixture")
    server.set_phase("reporting")
    clock.advance(1.01)  # Cached phase is stale; receipt phase must be rechecked.
    process_one_job(wrapper, monkeypatch)
    assert wrapper.submit_results() == [("queued-fixture", False)]
    assert server.attempts[schema.ENDPOINTS[kind]] == 0


@pytest.mark.spec("S10.2", "S11.1", "S11.6")
def test_m6_can_be_sent_immediately_after_report_phase_without_waiting_for_longpoll(monkeypatch):
    link, server, clock = fixture_link()
    server.set_phase("reporting")
    wrapper = inactive_async(link, monkeypatch)
    wrapper.submit_async("m6", PAYLOADS["m6"], "m6-fixture")
    process_one_job(wrapper, monkeypatch)
    assert wrapper.submit_results() == [("m6-fixture", True)]
    receipt = next(r for r in server.requests if r["path"].endswith("/situation"))
    assert receipt["received_at"] - server.matches["report"].reporting_at < 1.
    assert clock() == 0.
