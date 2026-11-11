"""Official API fixture tests. All time/network effects are in-process."""
import json
from pathlib import Path
from urllib.request import Request

import pytest

from maicon26.link import schema
from maicon26.link.http import HttpServerLink
from maicon26.sim.mockserver import FakeClock, MockServer


@pytest.fixture
def server():
    return MockServer({"Test0001": "B-1"})


def get(server, endpoint, *, api="report", code="Test0001", **params):
    status, raw, headers = server.request("GET", f"/api/{api}/{endpoint}", params={"mission_code": code, **params})
    return status, json.loads(raw) if headers.get("Content-Type") == "application/json" else raw, headers


def post(server, endpoint, body=None, *, api="report", code="Test0001"):
    status, raw, headers = server.request("POST", f"/api/{api}/{endpoint}", body={"mission_code": code, **(body or {})})
    return status, json.loads(raw), headers


@pytest.mark.spec("S0.1", "S0.2", "S0.3", "S0.4", "SA.1")
def test_transport_adapter_accepts_urllib_request_without_opening_socket(server, monkeypatch):
    def forbidden(*args, **kwargs):
        raise AssertionError("network must not be used")
    monkeypatch.setattr("urllib.request.urlopen", forbidden)
    link = HttpServerLink("http://mock.invalid", "test0001")
    link._req = server
    assert link.me() == {"slot": "B-1", "camp": "BLUE", "phase": "idle", "ready": False}
    server.set_phase("ready")
    status, result = link._post_json("ready", {})
    assert status == 200 and result == {"accepted": True}
    diagnostic = json.dumps([server.requests, server.fault_log], ensure_ascii=False).lower()
    assert "test0001" not in diagnostic


@pytest.mark.spec("S0.2", "S1.3", "S1.5", "S2.7", "S9.6", "SA.1")
def test_authentication_and_missing_mission_code_have_distinct_errors(server):
    assert get(server, "me", code="tEsT0001")[0] == 200
    assert get(server, "me", code="unknown1")[:2] == (401, {"detail": "임무코드가 올바르지 않습니다"})
    status, raw, _ = server.request("GET", "/api/report/me")
    assert status == 422
    assert json.loads(raw)["detail"] == [{"loc": ["query", "mission_code"], "msg": "Field required"}]
    server.team("Test0001").assigned = False
    assert get(server, "me")[1] == {"slot": None, "camp": None, "phase": "idle", "ready": False}


@pytest.mark.spec("S1.2", "S1.4", "S1.5", "S1.6")
def test_ready_guards_and_persisted_acceptance(server):
    assert post(server, "ready")[0] == 409
    server.set_phase("waiting")
    assert post(server, "ready")[0] == 409
    server.set_phase("ready")
    team = server.team("Test0001")
    team.assigned = False
    assert post(server, "ready")[1]["detail"] == schema.ready_error("not_assigned", "ready")
    team.assigned = True
    server.matches["report"].before_start = True
    assert post(server, "ready")[1]["detail"] == schema.ready_error("before_start", "ready")
    server.matches["report"].before_start = False
    team.config_confirmed = False
    assert post(server, "ready")[1]["detail"] == "UGV 경기 설정이 아직 확인되지 않았습니다 — 잠시 뒤 다시 보내세요"
    team.config_confirmed = True
    assert post(server, "ready")[:2] == (200, {"accepted": True})
    assert get(server, "me")[1]["ready"] is True
    assert post(server, "ready")[0] == 200


@pytest.mark.spec("S2.2", "S2.3", "S2.5", "S2.6", "S2.8", "S2.9", "S9.5")
def test_order_reasons_and_stable_team_specific_posting(server):
    assert get(server, "order")[1] == {"accepted": False, "reason": "임무는 경기 진행이 시작되면 게시됩니다"}
    assert get(server, "target")[1]["reason"] == "M5 목표거점은 종료 60초 전에 게시됩니다"
    server.team("Test0001").assigned = False
    before = server.clock()
    assert get(server, "order", wait=120)[1]["reason"] == "이 경기에 편성되지 않은 팀입니다"
    assert server.clock() == before
    server.team("Test0001").assigned = True
    server.set_phase("running")
    assert get(server, "order")[1]["reason"] == "게시된 임무가 없습니다"
    server.publish_order("B-1", "시험 명령")
    first = get(server, "order", wait=120)[1]
    assert first == {"accepted": True, "order": {"tactical_command": "시험 명령", "issued_at": "2026-11-04 12:00:00"}}
    server.set_phase("finished")
    assert get(server, "order")[1] == first
    assert "objective" not in first["order"]


@pytest.mark.spec("S0.5", "S0.6", "S0.8", "S0.9", "S0.11", "S2.1", "S2.2", "S2.3", "S2.4", "S9.3", "S9.4", "S9.8")
def test_long_poll_uses_fake_clock_publication_instant_and_clamps_wait(server):
    server.clock.schedule(10, lambda: server.start(orders={"B-1": "첫 임무"}, targets={"BLUE": ("RED_MAINTENANCE", "정비소")}))
    assert get(server, "order", wait=0)[1]["accepted"] is False
    assert server.clock() == 0
    assert get(server, "order", wait=120)[1]["accepted"] is True
    assert server.clock() == 10
    assert get(server, "target", wait=900)[1]["accepted"] is False
    assert server.clock() == 130  # input > 120 is capped
    target = get(server, "target", wait=120)[1]
    assert server.clock() == 250
    assert target["order"] == {"objective": "RED_MAINTENANCE", "objective_label": "정비소", "issued_at": "2026-11-04 12:04:10"}
    assert "tactical_command" not in target["order"]
    assert get(server, "target")[1] == target


@pytest.mark.spec("S2.3", "S2.4", "S9.3")
def test_long_poll_all_teams_observe_same_publication_and_short_timeout(server):
    server.add_team("Test0002", "B-2")
    server.clock.schedule(3, lambda: server.start(orders={"B-1": "명령1", "B-2": "명령2"}))
    first = get(server, "order", wait=120)[1]
    second = get(server, "order", wait=120, code="Test0002")[1]
    assert first["order"]["issued_at"] == second["order"]["issued_at"]
    assert first["order"]["tactical_command"] != second["order"]["tactical_command"]
    with pytest.raises(TimeoutError, match="long-poll"):
        server.request("GET", "/api/report/target", params={"mission_code": "Test0001", "wait": 120}, timeout=2)
    assert server.clock() == 5


@pytest.mark.spec("S5.2", "S5.3", "S5.4", "S5.5", "S5.9")
def test_m1_last_report_replaces_omitted_fields_with_null(server):
    server.start()
    body = {"target": "red_radar", "search_sectors": ["r-s1", "cm"], "base": "red_supply_depot"}
    status, result, _ = post(server, "interpretation", body)
    assert status == 200
    assert result["received"]["target"] == "RED_RADAR"
    assert set(result["received"]["search_sectors"]) == {"R-S1", "CM"}
    replaced = post(server, "interpretation", {"base": "RED_MAINTENANCE"})[1]["received"]
    assert replaced == {"target": None, "search_sectors": None, "base": "RED_MAINTENANCE"}
    assert server.team("Test0001").reports["m1"] == replaced


@pytest.mark.spec("S7.2", "S7.3", "S7.4", "S7.5", "S7.6", "SA.4", "SA.5")
def test_m3_accepts_all_sectors_but_replaces_entire_report(server):
    server.start()
    body = {"sectors": [{"sector": "b-s1", "objects": [{"type": "tank", "count": 0}]},
                        {"sector": "r-s1", "objects": [{"type": "military_truck", "count": 99}]}]}
    received = post(server, "detection", body)[1]["received"]
    sectors = {sec["sector"]: sec["objects"] for sec in received["sectors"]}
    assert sectors["B-S1"] == [{"type": "TANK", "count": 0}]
    assert sectors["R-S1"] == [{"type": "MILITARY_TRUCK", "count": 99}]
    assert post(server, "detection", {"sectors": [{"sector": "cm", "objects": []}]})[0] == 200
    assert server.team("Test0001").reports["m3"] == {"sectors": [{"sector": "CM", "objects": []}]}


@pytest.mark.spec("S5.8", "S7.8", "S11.7", "SA.1", "SA.4")
@pytest.mark.parametrize("endpoint,payload", [
    ("interpretation", {}),
    ("interpretation", {"target": "레이더시설"}),
    ("interpretation", {"search_sectors": [1]}),
    ("interpretation", {"search_sectors": ["CM", "cm"]}),
    ("interpretation", {"sector": "CM", "base": "RED_MAINTENANCE"}),
    ("detection", {"sectors": []}),
    ("detection", {"sectors": [{"sector": 1, "objects": []}]}),
    ("detection", {"sectors": [{"sector": "CM", "objects": []}, {"sector": "cm", "objects": []}]}),
    ("detection", {"sectors": [{"sector": "CM", "objects": [{"type": "wall", "count": 1}]}]}),
    ("detection", {"sectors": [{"sector": "CM", "objects": [{"type": "tank", "count": 100}]}]}),
    ("detection", {"sectors": [{"sector": "CM", "objects": [{"type": "tank", "count": -1}]}]}),
    ("detection", {"sectors": [{"sector": "CM", "objects": [{"type": "tank", "count": True}]}]}),
    ("detection", {"sectors": [{"sector": "CM", "objects": [{"type": "tank", "count": 1}, {"type": "TANK", "count": 2}]}]}),
    ("situation", {"ranking": ["B-2"]}),
    ("situation", {"ranking": ["R-2", "r-2"]}),
    ("situation", {"ranking": ["R-S1"]}),
    ("situation", {}),
])
def test_wire_validation_rejects_invalid_values(server, endpoint, payload):
    server.set_phase("reporting" if endpoint == "situation" else "running")
    status, result, _ = post(server, endpoint, payload)
    assert status == 422
    assert isinstance(result["detail"], (list, str))
    if isinstance(result["detail"], list):
        assert all("loc" in entry and "msg" in entry for entry in result["detail"])


@pytest.mark.spec("S5.5", "S7.5", "S11.4")
@pytest.mark.parametrize("order", ["spec", "reverse", "shuffle"])
def test_received_order_options_do_not_change_values_or_m6_ranking(order):
    server = MockServer({"Test0001": "B-1"}, received_order=order, seed=7)
    server.start()
    m1 = {"target": "RED_RADAR", "search_sectors": ["R-S1", "CM"], "base": "RED_MAINTENANCE"}
    response = post(server, "interpretation", m1)[1]
    assert schema.equal_received("m1", m1, response["received"], "BLUE")
    m3 = {"sectors": [{"sector": "R-S1", "objects": [{"type": "tank", "count": 2}, {"type": "drum", "count": 1}]},
                      {"sector": "CM", "objects": []}]}
    response = post(server, "detection", m3)[1]
    assert schema.equal_received("m3", m3, response["received"], "BLUE")
    server.set_phase("reporting")
    assert post(server, "situation", {"ranking": ["r-2", "r-1", "r-3"]})[1]["received"]["ranking"] == ["R-2", "R-1", "R-3"]


@pytest.mark.spec("S0.5", "S5.7", "S5.8", "S7.7", "S7.8", "S10.4", "S10.6", "S11.6")
def test_receipt_time_after_early_end_rejects_inflight_m3(server):
    server.start()
    server.clock.schedule(1, lambda: server.set_phase("reporting"))
    server.inject("detection", "delay", delay_s=2)
    status, result, _ = post(server, "detection", {"sectors": [{"sector": "CM", "objects": []}]})
    assert status == 409
    assert result["detail"] == "객체 인식 보고(M3)는 '진행' 단계에서만 받습니다 (지금은 '보고')"
    assert server.requests[-1]["sent_at"] == 0 and server.requests[-1]["received_at"] == 2
    assert "m3" not in server.team("Test0001").reports
    assert post(server, "situation", {"ranking": ["R-1"]})[0] == 200


@pytest.mark.spec("S5.8", "S7.8", "S11.7")
@pytest.mark.parametrize("endpoint,kind,phase", [("interpretation", "m1", "running"), ("detection", "m3", "running"), ("situation", "m6", "reporting")])
def test_submission_permanent_rejections(server, endpoint, kind, phase):
    server.set_phase(phase)
    team = server.team("Test0001")
    for attribute, value, reason in (("assigned", False, "not_assigned"), ("excluded", True, "excluded"), ("started", False, "not_started")):
        old = getattr(team, attribute)
        setattr(team, attribute, value)
        assert post(server, endpoint)[0:2] == (409, {"detail": schema.submission_error(kind, reason, phase)})
        setattr(team, attribute, old)


@pytest.mark.spec("S0.8", "S0.14", "S10.1", "S10.2", "S10.4", "S11.3", "S11.4", "S11.6", "SA.3")
def test_reporting_window_and_all_six_team_early_finish():
    server = MockServer({f"Test000{n}": f"{'B' if n <= 3 else 'R'}-{(n - 1) % 3 + 1}" for n in range(1, 7)})
    server.start()
    server.clock.advance(300)
    assert get(server, "me")[1]["phase"] == "reporting"
    for n in range(1, 7):
        opponent = "R" if n <= 3 else "B"
        assert post(server, "situation", {"ranking": [f"{opponent}-2", f"{opponent}-1"]}, code=f"Test000{n}")[0] == 200
        assert server.matches["report"].phase == ("finished" if n == 6 else "reporting")
    assert server.clock() == 300
    assert set(get(server, "me")[1]) == {"slot", "camp", "phase", "ready"}
    server.reset()
    server.start()
    server.clock.advance(330)
    assert server.matches["report"].phase == "finished"
    assert post(server, "situation", {"ranking": ["R-1"]})[0] == 409


@pytest.mark.spec("S4.1", "S4.2", "S4.5", "S4.6", "S4.8")
def test_drone_capture_retains_good_frame_and_enforces_camp_intervals(server):
    server.start()
    server.capture(b"jpeg-initial")
    status, image, headers = get(server, "drone")
    assert status == 200 and image == b"jpeg-initial"
    assert headers == {"Content-Type": "image/jpeg", "Date": "Wed, 04 Nov 2026 03:00:00 GMT", "X-Captured-At": "2026-11-04T12:00:00.000+09:00",
                       "X-Frame-Interval-S": "1.0", "X-Camera": "main", "X-Test-Mode": "off", "X-Camp": "BLUE", "X-Destroyed": "0"}
    server.clock.advance(.5)
    assert not server.capture(b"too-soon")
    assert get(server, "drone")[1] == image
    server.destroyed["BLUE"] = 1
    server.clock.advance(.5)
    server.capture(b"second")
    assert get(server, "drone")[1] == image
    server.clock.advance(1)
    server.capture(b"third")
    assert get(server, "drone")[1] == b"third"
    assert get(server, "drone")[2]["X-Frame-Interval-S"] == "2.0"
    server.clock.advance(1)
    assert not server.capture(b"broken", valid=False)
    assert get(server, "drone")[1] == b"third"
    server.destroyed["BLUE"] = 2
    server.clock.advance(4)
    server.capture(b"five-seconds")
    assert get(server, "drone")[1] == b"five-seconds"
    assert get(server, "drone")[2]["X-Frame-Interval-S"] == "5.0"
    server.destroyed["BLUE"] = 3
    assert get(server, "drone")[0] == 403


@pytest.mark.spec("S4.3", "S4.4", "S4.6", "S4.7", "S4.8", "SR.2")
def test_drone_practice_mode_headers_and_all_error_conditions(server):
    server.test_mode = True
    server.capture(b"practice", "test1")
    server.team("Test0001").assigned = False
    status, raw, headers = get(server, "drone", camera="test1")
    assert status == 200 and raw == b"practice"
    assert headers["X-Camera"] == "test1" and headers["X-Test-Mode"] == "on"
    assert "X-Camp" not in headers and "X-Destroyed" not in headers
    assert get(server, "drone")[0] == 409  # no camera remains a match request
    server.test_mode = False
    assert get(server, "drone", camera="test1")[1]["detail"].endswith("— 지금은 테스트 모드가 아닙니다")
    server.team("Test0001").assigned = True
    server.set_phase("reporting")
    assert get(server, "drone")[1]["detail"] == "드론 화면은 경기 진행 중에만 받을 수 있습니다 (지금: 보고)"
    server.set_phase("running")
    assert get(server, "drone")[0] == 503
    assert get(server, "drone", camera="invalid")[0] == 422
    server.capture(b"main")
    assert get(server, "drone", camera="test1")[2]["X-Camera"] == "main"  # ignored outside practice
    server.camera_enabled["main"] = False
    assert get(server, "drone")[0] == 503
    assert get(server, "drone", api="rehearsal")[0] == 404


@pytest.mark.spec("SR.1", "SR.2", "SR.3", "SR.4", "S2.5", "S9.4")
def test_rehearsal_isolated_state_five_second_countdown_and_complete_timeline(server):
    assert get(server, "me", api="rehearsal")[1]["phase"] == "idle"
    for endpoint in ("order", "target"):
        assert get(server, endpoint, api="rehearsal", wait=120)[1] == {"accepted": False, "reason": "진행 중인 리허설이 없습니다"}
    assert server.clock() == 0
    server.set_phase("ready", api="rehearsal")
    assert post(server, "ready", api="rehearsal")[0] == 200
    server.start(api="rehearsal", orders={"B-1": "연습 명령"}, targets={"BLUE": ("RED_SUPPLY_DEPOT", "군수보급소")})
    server.clock.advance(4.99)
    assert get(server, "me", api="rehearsal")[1]["phase"] == "ready"
    server.clock.advance(.01)
    assert get(server, "order", api="rehearsal")[1]["order"]["issued_at"] == "2026-11-04 12:00:05"
    assert get(server, "me")[1]["phase"] == "idle"
    assert not server.team("Test0001").ready
    server.clock.advance(240)
    assert get(server, "target", api="rehearsal")[1]["order"]["objective"] == "RED_SUPPLY_DEPOT"
    server.clock.advance(60)
    assert get(server, "me", api="rehearsal")[1]["phase"] == "reporting"
    assert post(server, "interpretation", {"base": "RED_MAINTENANCE"}, api="rehearsal")[0] == 409
    assert post(server, "situation", {"ranking": ["R-3"]}, api="rehearsal")[0] == 200
    server.clock.advance(30)
    assert get(server, "me", api="rehearsal")[1]["phase"] == "finished"
    assert not server.team("Test0001").reports


@pytest.mark.spec("S0.4", "S0.5", "S5.3", "S7.6", "S11.3")
def test_faults_are_deterministic_redacted_and_response_loss_does_not_undo_receipt(server):
    server.start()
    server.inject("interpretation", "drop")
    with pytest.raises(TimeoutError, match="lost after receipt"):
        post(server, "interpretation", {"target": "RED_RADAR"})
    assert server.team("Test0001").reports["m1"]["target"] == "RED_RADAR"
    server.inject("interpretation", "duplicate")
    assert post(server, "interpretation", {"target": "RED_COMMS"})[1]["received"]["target"] == "RED_RADAR"
    assert server.team("Test0001").reports["m1"]["target"] == "RED_RADAR"
    server.inject("me", "status", status=500, body={"detail": "fault"})
    assert get(server, "me")[0] == 500
    server.inject("me", "non_json")
    assert server.request("GET", "/api/report/me", params={"mission_code": "Test0001"})[1].startswith(b"injected non-JSON")
    server.inject("me", "delay", delay_s=10)
    with pytest.raises(TimeoutError):
        server.request("GET", "/api/report/me", params={"mission_code": "Test0001"}, timeout=1)
    assert "test0001" not in repr((server.requests, server.fault_log)).lower()
    a, b = MockServer(), MockServer()
    a.chaos(seed=42, count=20)
    b.chaos(seed=42, count=20)
    assert list(a._faults["*"]) == list(b._faults["*"])


@pytest.mark.spec("S1.4", "S1.6", "S2.8", "SR.4")
def test_reset_cancels_old_match_schedules(server):
    server.start(orders={"B-1": "old"})
    server.clock.advance(100)
    server.reset()
    server.clock.advance(250)
    assert get(server, "me")[1]["phase"] == "idle"
    assert get(server, "order")[1]["accepted"] is False
    assert not server.team("Test0001").ready


@pytest.mark.spec("S1.5", "S2.6", "S4.7", "S5.8", "S7.8", "S9.5", "S11.7", "SA.1", "SR.4")
def test_complete_published_error_strings_match_source_text():
    source = Path(__file__).resolve().parents[1] / "docs" / "API_SOURCE_EXTRACT.txt"
    text = source.read_text(encoding="utf-8")
    for literal in (schema.ERR_INVALID_CODE, schema.REASON_ORDER_NOT_YET, schema.REASON_TARGET_NOT_YET,
                    schema.REASON_NOT_ASSIGNED, schema.REASON_NO_POST, schema.REASON_NO_REHEARSAL,
                    schema.ready_error("unconfirmed", "ready"),
                    schema.submission_error("m1", "phase", "ready"),
                    schema.submission_error("m3", "phase", "reporting"),
                    schema.submission_error("m6", "phase", "running"),
                    schema.drone_error("phase", "reporting")):
        assert literal in text


@pytest.mark.spec("S0.3", "S1.5", "SA.1")
def test_malformed_json_and_wrong_post_content_type_are_structured_422(server):
    response = server(Request("http://mock.invalid/api/report/ready", data=b"{", method="POST"), timeout=2)
    assert response[0] == 422 and isinstance(json.loads(response[1])["detail"], list)
    response = server.request("POST", "/api/report/ready", body={"mission_code": "Test0001"}, headers={"Content-Type": "text/plain"})
    assert response[0] == 422


@pytest.mark.spec("SA.1", "S5.8", "S7.8", "S11.7")
def test_injected_business_validation_can_return_string_detail(server):
    # The guide guarantees location-bearing lists for format errors. A fixture
    # can also exercise a business-rule/string detail without inventing an
    # undocumented exact production validation sentence.
    server.inject("interpretation", "status", status=422, body={"detail": "시험용 업무 규칙 위반"})
    assert post(server, "interpretation")[:2] == (422, {"detail": "시험용 업무 규칙 위반"})


@pytest.mark.spec("S0.5", "S2.3")
def test_fake_clock_runs_scheduled_callbacks_in_stable_order():
    clock = FakeClock()
    events = []
    clock.schedule(2, lambda: events.append((clock(), 1)))
    clock.schedule(2, lambda: events.append((clock(), 2)))
    clock.schedule(3, lambda: events.append((clock(), 3)))
    clock.wait_until(lambda: len(events) >= 2, 10)
    assert events == [(2, 1), (2, 2)] and clock() == 2
