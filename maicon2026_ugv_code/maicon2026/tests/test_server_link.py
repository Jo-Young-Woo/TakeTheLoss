"""경기 서버 HTTP 연동 시험 — 「참가팀 단계별 API 안내」(2026-11-04 스냅숏)의 형식을 흉내 낸 모의 서버로 link/http.py·link/schema.py 를 본다.

모의 서버는 문서의 규칙을 그대로 따른다: /me phase, ready 는 ready 단계에서만(아니면 409), /order·/target 은 wait 동안 붙잡고 있다가 게시되면 응답,
M1/M3/M6 는 단계가 맞을 때만(아니면 409), 형식이 틀리면 422. 실제 서버가 아니므로 필드 이름까지 맞는지는 현장 리허설(/api/rehearsal)에서 확인한다.
"""
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import pytest

from maicon26.link import schema
from maicon26.link.async_link import AsyncServerLink
from maicon26.link.http import HttpServerLink
from maicon26.mission.command import CommandParser, parse_base_command

CODE = "AbC12345"


# ====================================================================== schema 변환
def test_m1_to_server_prefixes_opponent_camp():
    p = {"target": "RADAR_FACILITY", "sectors": ["R-S1", "COMMON", "ALPHA"], "goal_base": "SUPPLY_BASE"}
    assert schema.m1_to_server(p, "BLUE") == {"target": "RED_RADAR", "search_sectors": ["R-S1", "CM"], "base": "RED_SUPPLY_DEPOT"}
    assert schema.m1_to_server(p, "RED")["target"] == "BLUE_RADAR"
    q = {"target": "GUARD_POST", "sectors": [], "goal_base": "MAINT_DEPOT"}
    assert schema.m1_to_server(q, "BLUE") == {"target": "RED_OUTPOST", "base": "RED_MAINTENANCE"}   # 모르는 항목은 빼고(미보고) 틀린 값으로 채우지 않는다
    assert schema.m1_to_server({"target": "COMM_FACILITY", "sectors": None, "goal_base": "COMMAND_POST"}, "RED") == \
        {"target": "BLUE_COMMS", "base": "BLUE_COMMAND_POST"}
    assert schema.m1_to_server({"target": None, "sectors": [], "goal_base": None}, "BLUE") == {}


def test_m3_to_server_scoring_sectors_and_types():
    p = {"sectors": {"R-S1": {"TANK": 2, "DRUM": 1}, "R-S2": {"TRUCK": 1}, "COMMON": {"AMMO_BOX": 3}, "B-S1": {"MISSILE": 5}, "ALPHA": {"TANK": 1}}}
    out = schema.m3_to_server(p, "BLUE")
    secs = {s["sector"]: s["objects"] for s in out["sectors"]}
    assert list(secs) == ["R-S1", "R-S2", "R-S3", "CM"]              # 채점 구역 4개를 항상, 이 순서로
    assert secs["R-S1"] == [{"type": "drum", "count": 1}, {"type": "tank", "count": 2}]
    assert secs["R-S2"] == [{"type": "military_truck", "count": 1}]
    assert secs["R-S3"] == []
    assert secs["CM"] == [{"type": "ammo_box", "count": 3}]
    assert "B-S1" not in secs                                         # 우리 진영 섹터는 채점하지 않는다
    assert schema.m3_to_server({"sectors": {}}, "RED")["sectors"][0]["sector"] == "B-S1"


def test_m3_count_capped():
    out = schema.m3_to_server({"sectors": {"R-S1": {"TANK": 120}}}, "BLUE")
    assert out["sectors"][0]["objects"] == [{"type": "tank", "count": 99}]


def test_m6_to_server_callsigns():
    assert schema.m6_to_server({"ranking": ["R-UGV-2", "R-UGV-1", "R-UGV-3"]}, "BLUE") == {"ranking": ["R-2", "R-1", "R-3"]}
    assert schema.m6_to_server({"ranking": ["B-UGV-1", "R-UGV-2", "R-UGV-2"]}, "BLUE") == {"ranking": ["R-2"]}   # 자기 진영·중복은 버린다
    assert schema.m6_to_server({"ranking": ["B-UGV-2"]}, "RED") == {"ranking": ["B-2"]}


def test_parse_slot_objective_sector():
    assert schema.parse_slot("B-1") == ("BLUE", 1) and schema.parse_slot("R-3") == ("RED", 3) and schema.parse_slot(None) is None
    assert schema.parse_objective("RED_MAINTENANCE") == "MAINT_DEPOT"
    assert schema.parse_objective("BLUE_SUPPLY_DEPOT") == "SUPPLY_BASE"
    assert schema.parse_objective("RED_COMMAND_POST") == "COMMAND_POST"
    assert schema.parse_objective("RED_BOGUS") is None
    assert schema.objective_camp("RED_MAINTENANCE") == "RED"
    assert schema.server_sector("common") == "CM" and schema.server_sector("r-s2") == "R-S2" and schema.server_sector("ALPHA") is None


def test_command_parser_server_style_sentences():
    p = CommandParser(team="BLUE")
    c = p.parse("상대 중앙진지를 제외하고 나머지 상대진영 탐색 구역을 정찰하라. 레이더시설을 타격하고 군수보급소를 목표거점으로 이동하라")
    assert c.target == "RADAR_FACILITY" and c.goal_base == "SUPPLY_BASE"
    assert set(c.sectors) == {"R-S1", "R-S2", "R-S3"}                  # 중앙(CM) 제외, 나머지
    assert CommandParser().parse("R-S1 과 공통 구역을 탐색").sectors == ["R-S1", "COMMON"]
    assert CommandParser().parse("B-S2 를 제외하고 R-S1 을 탐색").sectors == ["R-S1"]
    assert parse_base_command("정비소") == "MAINT_DEPOT"


# ====================================================================== 모의 서버
class Srv:
    def __init__(self):
        self.s = {"phase": "waiting", "ready": False, "slot": "B-1", "order": None, "target": None, "posts": [], "ready_posts": 0,
                  "ready_fail": 1, "drone_n": 0, "gets": []}
        outer = self

        class H(BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def _send(self, code, obj=None, raw=None, hdr=None):
                body = raw if raw is not None else json.dumps(obj if obj is not None else {}).encode()
                self.send_response(code)
                self.send_header("Content-Type", "image/jpeg" if raw is not None else "application/json")
                for k, v in (hdr or {}).items():
                    self.send_header(k, v)
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def _code_ok(self, code):
                if code != CODE:
                    self._send(401, {"detail": "임무코드가 올바르지 않습니다"})
                    return False
                return True

            def do_GET(self):
                u = urlparse(self.path)
                q = {k: v[0] for k, v in parse_qs(u.query).items()}
                outer.s["gets"].append(self.path)
                if not self._code_ok(q.get("mission_code")):
                    return
                name = u.path.rsplit("/", 1)[-1]
                if name == "me":
                    return self._send(200, {"slot": outer.s["slot"], "camp": "BLUE", "phase": outer.s["phase"], "ready": outer.s["ready"]})
                if name in ("order", "target"):
                    wait = float(q.get("wait", 0))
                    t0 = time.monotonic()
                    while outer.s[name] is None and time.monotonic() - t0 < wait:
                        time.sleep(0.02)
                    o = outer.s[name]
                    if o is None:
                        return self._send(200, {"accepted": False, "reason": "임무는 경기 진행이 시작되면 게시됩니다"})
                    return self._send(200, {"accepted": True, "order": o})
                if name == "drone":
                    if outer.s["phase"] != "running":
                        return self._send(409, {"detail": "드론 화면은 경기 진행 중에만 받을 수 있습니다"})
                    outer.s["drone_n"] += 1
                    return self._send(200, raw=b"\xff\xd8jpeg", hdr={"X-Captured-At": f"2026-10-20T14:17:{10 + outer.s['drone_n']:02d}.073+09:00",
                                                                      "X-Frame-Interval-S": "2.0", "X-Destroyed": "1", "X-Camp": "BLUE"})
                self._send(404, {"detail": "no"})

            def do_POST(self):
                n = int(self.headers.get("Content-Length", 0))
                b = json.loads(self.rfile.read(n) or b"{}")
                if not self._code_ok(b.get("mission_code")):
                    return
                name = urlparse(self.path).path.rsplit("/", 1)[-1]
                if name == "ready":
                    outer.s["ready_posts"] += 1
                    if outer.s["phase"] != "ready" or outer.s["ready_fail"] > 0:
                        outer.s["ready_fail"] -= 1
                        return self._send(409, {"detail": "UGV 경기 설정이 아직 확인되지 않았습니다 — 잠시 뒤 다시 보내세요"})
                    outer.s["ready"] = True
                    return self._send(200, {"accepted": True})
                need = {"interpretation": "running", "detection": "running", "situation": "reporting"}.get(name)
                if need is None:
                    return self._send(404, {"detail": "no"})
                if outer.s["phase"] != need:
                    return self._send(409, {"detail": f"지금은 '{outer.s['phase']}'"})
                outer.s["posts"].append((name, b))
                if name == "interpretation" and not any(k in b for k in ("target", "search_sectors", "base")):
                    return self._send(422, {"detail": [{"loc": ["body"], "msg": "항목이 하나도 없음"}]})
                self._send(200, {"accepted": True, "received": {k: v for k, v in b.items() if k != "mission_code"}})

        self.httpd = ThreadingHTTPServer(("127.0.0.1", 0), H)
        self.url = f"http://127.0.0.1:{self.httpd.server_address[1]}"
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()

    def close(self):
        self.httpd.shutdown()


@pytest.fixture()
def srv():
    s = Srv()
    yield s
    s.close()


def _link(srv, **kw):
    return HttpServerLink(srv.url, CODE, wait_s=kw.pop("wait_s", 1), me_period_s=0.05, **kw)


def _wait(cond, t=4.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < t:
        if cond():
            return True
        time.sleep(0.02)
    return False


def test_mission_code_validation():
    with pytest.raises(ValueError):
        HttpServerLink("http://x", "short")
    with pytest.raises(ValueError):
        HttpServerLink("http://x", "")


def test_ready_handshake_waits_for_init_and_retries_409(srv):
    link = _link(srv)
    srv.s["phase"] = "waiting"
    inited = {"v": False}
    th = threading.Thread(target=lambda: srv.s.update(result=link.ready_handshake(lambda: inited["v"], poll_s=0.05)))
    th.start()
    time.sleep(0.3)
    srv.s["phase"] = "ready"
    time.sleep(0.3)
    assert srv.s["ready_posts"] == 0                                    # 초기화 전에는 보내지 않는다
    inited["v"] = True
    th.join(5)
    assert srv.s["result"] is True and srv.s["ready"] is True
    assert srv.s["ready_posts"] == 2                                    # 첫 409 뒤 다시
    assert link.identity() == ("BLUE", 1)


def test_ready_handshake_when_already_ready_or_running(srv):
    link = _link(srv)
    srv.s.update(phase="ready", ready=True)
    assert link.ready_handshake(lambda: True, poll_s=0.05) is True and srv.s["ready_posts"] == 0     # 재실행: 초기화 확인 후 준비완료 재전송 없음
    srv.s.update(phase="running", ready=False)
    assert link.ready_handshake(lambda: True, poll_s=0.05) is True


def test_poll_synthesizes_start_command_target_stop(srv):
    link = _link(srv)
    srv.s["phase"] = "ready"
    assert link.me()["phase"] == "ready"  # Observe ready before advancing the mock; avoid a thread race with restart detection.
    assert link.poll() == []                                            # 배경 시작, 게시 전
    srv.s["phase"] = "running"
    srv.s["order"] = {"tactical_command": "레이더시설을 타격하라", "issued_at": "2026-10-20 14:17:11"}
    got = []
    assert _wait(lambda: got.extend(link.poll()) or {m.kind for m in got} >= {"start", "command"})
    cmd = next(m for m in got if m.kind == "command")
    assert cmd.text == "레이더시설을 타격하라"
    assert sum(m.kind == "start" for m in got) == 1
    srv.s["target"] = {"objective": "RED_MAINTENANCE", "objective_label": "정비소", "issued_at": "x"}
    got2 = []
    assert _wait(lambda: got2.extend(link.poll()) or any(m.kind == "base_command" for m in got2))
    bc = next(m for m in got2 if m.kind == "base_command")
    assert bc.data["objective"] == "RED_MAINTENANCE" and bc.text == "정비소"
    srv.s["phase"] = "reporting"
    got3 = []
    assert _wait(lambda: got3.extend(link.poll()) or any(m.kind == "stop" for m in got3))
    time.sleep(0.3)
    assert [m.kind for m in link.poll()] == []                          # start/stop 은 한 번씩만
    link.close()


def test_poll_never_blocks_during_long_wait(srv):
    link = _link(srv, wait_s=3)
    t0 = time.monotonic()
    link.poll()
    link.poll()
    assert time.monotonic() - t0 < 0.5                                   # 긴 폴링은 별도 스레드 — 제어 루프를 막지 않는다
    link.close()


def test_submit_formats_and_phase_errors(srv):
    link = _link(srv)
    link.me()
    srv.s["phase"] = "ready"
    m1 = {"target": "RADAR_FACILITY", "sectors": ["R-S1", "COMMON"], "goal_base": "SUPPLY_BASE"}
    assert link.submit("m1", m1) is False                               # 409: 진행 단계 아님
    srv.s["phase"] = "running"
    assert link.submit("m1", m1) is True
    assert link.submit("m3", {"sectors": {"R-S1": {"TANK": 2}}}) is True
    assert link.submit("m1", {"target": None, "sectors": [], "goal_base": None}) is False       # 보낼 항목이 없다(서버 422 를 안 부른다)
    assert link.submit("m6", {"ranking": ["R-UGV-2"]}) is False        # 진행 중에는 409
    srv.s["phase"] = "reporting"
    assert link.submit("m6", {"ranking": ["R-UGV-2", "R-UGV-1", "R-UGV-3"]}) is True
    posts = dict(srv.s["posts"])
    assert posts["interpretation"]["target"] == "RED_RADAR" and posts["interpretation"]["search_sectors"] == ["R-S1", "CM"]
    assert posts["interpretation"]["mission_code"] == CODE
    assert posts["detection"]["sectors"][0] == {"sector": "R-S1", "objects": [{"type": "tank", "count": 2}]}
    assert posts["situation"]["ranking"] == ["R-2", "R-1", "R-3"]


def test_wrong_code_is_masked(srv):
    link = HttpServerLink(srv.url, "ZZZZ9999", wait_s=0, me_period_s=0.05)
    assert link.me() is None
    link.identity()
    ok, why = link.health()
    assert "ZZZZ9999" not in why and "ZZZZ9999" not in (link.last_error or "") and "ZZZZ9999" not in (link._me_err or "")


def test_drone_headers_feed_uav_monitor(srv):
    link = _link(srv)
    link.me()
    assert link.get_uav() is None                                       # 진행 중이 아님
    srv.s["phase"] = "running"
    link.me()
    f = link.get_uav()
    assert f is not None and f.interval_s == 2.0 and f.destroyed == 1 and f.detections == []
    assert f.jpeg == b"\xff\xd8jpeg"                                  # JPEG를 버리지 않고 소비자에게 제공
    link._drone_next = 0
    f2 = link.get_uav()
    assert f2 is not None and f2.stamp > f.stamp                        # 새 장이면 stamp 증가
    link._drone_next = 0
    srv.s["drone_n"] -= 1                                               # 서버가 같은 장을 다시 준다
    assert link.get_uav() is None                                       # 같은 장이면 새 프레임으로 치지 않는다


def test_drone_frame_carries_age_for_robot_clock_conversion():
    link = HttpServerLink("http://localhost", CODE, decode_drone=False)
    link.phase = "running"
    link._req = lambda *a: (200, b"jpeg", {"Content-Type": "image/jpeg", "X-Captured-At": "2026-11-06T12:00:00+09:00"})
    f = link.get_uav()
    assert f.age_s is not None and 0.0 <= f.age_s <= 30.0           # 과거 시각이어도 30 s 로 제한
    assert f.recv_mono is not None


def test_order_poll_backs_off_on_server_errors():
    link = HttpServerLink("http://localhost", CODE, wait_s=1)
    calls = []
    link._get_json = lambda *a, **k: (calls.append(time.monotonic()) or (409, {"detail": "단계 아님"}))
    th = threading.Thread(target=link._wait_loop, args=("order",), daemon=True)
    th.start()
    time.sleep(2.2)
    link.close()
    th.join(6)
    assert 2 <= len(calls) <= 4                                        # 0.5 → 1 → 2 s 간격 (예전엔 0.2 s 마다 10회 이상)
    assert "409" in (link.last_error or "")


def test_identity_does_not_block_once_background_runs():
    link = HttpServerLink("http://localhost", CODE)
    link._started_bg = True
    link.me = lambda: (_ for _ in ()).throw(AssertionError("제어 루프에서 /me 를 직접 부르면 안 된다"))
    assert link.identity() is None


def test_async_wrapper_forwards_handshake_and_identity(srv):
    inner = _link(srv)
    srv.s.update(phase="ready", ready=False, ready_fail=0)
    link = AsyncServerLink(inner)
    assert link.ready_handshake(lambda: True, poll_s=0.05) is True
    assert _wait(lambda: link.identity() == ("BLUE", 1))
    link.close()


def test_rehearsal_uses_rehearsal_paths(srv):
    link = HttpServerLink(srv.url, CODE, rehearsal=True, wait_s=0)
    link.me()
    assert any("/me" in g for g in srv.s["gets"]) and "rehearsal" in link.api


def test_rehearsal_does_not_request_nonexistent_drone_endpoint(srv):
    link = _link(srv, rehearsal=True)
    srv.s["phase"] = "running"
    link.me()
    assert link.get_uav() is None
    assert not any("/drone" in g for g in srv.s["gets"])


def test_practice_camera_uses_report_endpoint_without_phase_gate():
    link = HttpServerLink("http://localhost", CODE, rehearsal=True,
                          drone_camera="test1", decode_drone=False)
    captured = []

    def req(request, timeout):
        captured.append(request.full_url)
        return 200, b"jpeg", {"Content-Type": "image/jpeg", "X-Captured-At": "2026-11-06T12:00:00+09:00",
                               "X-Camera": "test1", "X-Test-Mode": "on"}

    link._req = req
    frame = link.get_uav()
    assert frame and frame.jpeg == b"jpeg" and frame.camera == "test1" and frame.test_mode
    assert frame.detections == [] and frame.image is None
    assert "/api/report/drone?" in captured[0] and "camera=test1" in captured[0]
    with pytest.raises(ValueError):
        HttpServerLink("http://localhost", CODE, drone_camera="unlisted")


def test_drone_drops_out_of_order_and_invalid_timestamps():
    link = HttpServerLink("http://localhost", CODE, decode_drone=False)
    link.phase = "running"
    header = {"Content-Type": "image/jpeg", "X-Captured-At": "2026-11-06T12:00:02+09:00"}
    link._req = lambda *args: (200, b"jpeg", header)
    assert link.get_uav() is not None
    for stamp in ("2026-11-06T12:00:01+09:00", "invalid", "2026-11-06T12:00:03"):
        link._drone_next = 0
        header["X-Captured-At"] = stamp
        assert link.get_uav() is None
    link._drone_next = 0
    header.update({"X-Captured-At": "2026-11-06T12:00:03+09:00", "X-Frame-Interval-S": "NaN"})
    assert link.get_uav().interval_s == 1.0


def test_me_unassigned_clears_constructor_and_async_identity(srv):
    inner = _link(srv, team="RED", slot=3)
    assert inner.me() and inner.identity() == ("BLUE", 1)
    wrapper = AsyncServerLink(inner)
    assert wrapper.identity() == ("BLUE", 1)
    srv.s["slot"] = None
    inner.me()
    assert wrapper.identity() is None
    assert not inner.submit("m3", {"sectors": {}})
    wrapper.close()


@pytest.mark.parametrize("phase,ready", [("ready", True), ("running", False)])
def test_restarted_process_must_initialize_before_handshake(srv, phase, ready):
    link = _link(srv)
    srv.s.update(phase=phase, ready=ready)
    calls = []

    def init_ok():
        calls.append(1)
        if len(calls) > 1:
            raise TimeoutError("required sensors unavailable")
        return False

    with pytest.raises(TimeoutError):
        link.ready_handshake(init_ok, poll_s=0, sleep=lambda _: None)
    assert srv.s["ready_posts"] == 0


def test_invalid_submission_is_rejected_locally(srv):
    link = _link(srv)
    link.me()
    assert not link.submit("m3", {"sectors": {"R-S1": {"TANK": "not-a-count"}}})
    assert not link.submit("m6", {"ranking": ["B-1"]})
    assert srv.s["posts"] == []


def test_async_submission_freezes_payload_before_worker_uses_it():
    from maicon26.link.base import ServerLink

    class SlowLink(ServerLink):
        def __init__(self):
            self.entered, self.release = threading.Event(), threading.Event()
            self.got = None
        def poll(self):
            return []
        def submit(self, kind, payload):
            self.entered.set()
            self.release.wait(2)
            self.got = payload
            return True

    inner = SlowLink()
    wrapper = AsyncServerLink(inner)
    payload = {"sectors": {"R-S1": {"TANK": 1}}}
    try:
        wrapper.submit_async("m3", payload, "snapshot")
        assert inner.entered.wait(1)
        payload["sectors"]["R-S1"]["TANK"] = 9
        inner.release.set()
        assert _wait(lambda: inner.got is not None)
        assert inner.got["sectors"]["R-S1"]["TANK"] == 1
    finally:
        inner.release.set()
        wrapper.close()
