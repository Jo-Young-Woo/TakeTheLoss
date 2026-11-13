"""경기 서버 HTTP 연결 — 「참가팀 단계별 API 안내」p0~p12(2026-11-06 확인)로 구현. **실제 서버에서는 아직 실행해 보지 못했다**(tests/test_server_link.py 의 모의 서버로 시험).

서버는 먼저 보내지 않는다. Agent 의 ServerLink 모델(START/명령/목표거점/STOP 메시지를 poll)에 맞추기 위해 이 클래스가 서버 상태를 메시지로 바꾼다.

  GET  /me                  → phase(idle waiting ready running reporting finished)·slot·camp·ready.   0.5~1 s 주기 폴링(별도 스레드)
                               phase 가 running 이 되면 "start", reporting/finished 가 되면 "stop" 을 한 번 낸다.
                               (경기 0:00 의 정확한 신호는 로봇의 /mil/match cmd=start 이고, 이쪽은 보조다 — Agent 가 둘 중 먼저 온 것을 래치한다)
  GET  /order?wait=120      → 전술명령(0:00 게시). 긴 폴링이라 별도 스레드에서 기다렸다가 "command" 메시지로 낸다.
  GET  /target?wait=120     → M5 목표거점(4:00 게시). "base_command" 메시지(text=거점 이름, data.objective=보고값).
  POST /interpretation | /detection | /situation   M1 · M3 · M6 (변환은 link/schema.py)
  GET  /drone               → JPEG·선택적 BGR 및 주기·파괴 수 메타데이터. 영상의 경기장 좌표 검출은 별도 모듈이 필요하다.

임무코드(8자리)는 요청에만 쓰고 로그·오류 문구에는 가려서 남긴다. 리허설 경기는 rehearsal=True (/api/rehearsal/ — 형식 동일, drone 제외).
한 인스턴스는 한 경기용이다. 공식 API에는 match_id가 없고 /order는 종료 후에도 남으므로 경기 사이 재생성한다.
모든 대기 호출은 Agent 제어 루프 밖의 스레드에서 한다. poll() 은 큐만 비운다(블로킹 없음).
"""
from __future__ import annotations

import datetime
import json
import logging
import math
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
from queue import Empty, Queue
from typing import Any, Callable, Dict, List, Optional, Tuple

from . import schema
from .base import ServerLink, ServerMsg, UavBox, UavDet, UavFrame
from .clock_sync import ClockSync
from .secrets import REDACT, register_secret, safe_detail
from ..validate import ValidationLog, clean_issued_at, raw_label, sanitize_command_text

RUN_PHASES = ("running",)
STOP_PHASES = ("reporting", "finished")


class HttpServerLink(ServerLink):
    def __init__(self, base_url: str, mission_code: str, team: Optional[str] = None, slot: Optional[int] = None,
                 rehearsal: bool = False, timeout: float = 3.0, wait_s: int = 20, me_period_s: float = 0.5,
                 clock: Callable[[], float] = time.monotonic, drone_camera: Optional[str] = None,
                 decode_drone: bool = True, transport=None, strict_code: bool = True,
                 target_time_tolerance_s: float = 5.0,
                 uav_layout_detector: Optional[Callable[[Any], List[UavBox]]] = None,
                 uav_layout_max_age_s: float = 12.0,
                 uav_layout_source: str = "gazebo_uav_image",
                 uav_robot_detector: Optional[Callable[[Any], List[UavDet]]] = None,
                 max_command_chars: int = 500, vlog: Optional[ValidationLog] = None) -> None:
        if not mission_code or (strict_code and not schema.mission_code_valid(mission_code)):
            raise ValueError("임무코드는 영문+숫자 8자리다 (현장에서 안내)")
        self.root = base_url.rstrip("/")
        if drone_camera not in (None, "main", "test1", "test2"):
            raise ValueError("drone_camera must be main, test1, test2 or None")
        self.rehearsal, self.drone_camera, self.decode_drone = rehearsal, drone_camera, decode_drone
        if uav_layout_detector is not None and (not callable(uav_layout_detector) or not decode_drone):
            raise ValueError("UAV layout callback requires decoded images and a callable detector")
        if uav_robot_detector is not None and (not callable(uav_robot_detector) or not decode_drone):
            raise ValueError("UAV robot callback requires decoded images and a callable detector")
        if not math.isfinite(uav_layout_max_age_s) or uav_layout_max_age_s <= 0:
            raise ValueError("UAV layout maximum age must be positive and finite")
        self.uav_layout_detector = uav_layout_detector  # Explicit simulation opt-in; default remains image-only.
        self.uav_robot_detector = uav_robot_detector
        self.uav_layout_max_age_s = uav_layout_max_age_s
        if not isinstance(uav_layout_source, str) or not uav_layout_source:
            raise ValueError("UAV layout source label must be a non-empty string")
        self.uav_layout_source = uav_layout_source    # frame.layout_source 에 붙는 이름: 시뮬 gazebo_uav_image | 실기 real_uav_image
        self.api = f"{self.root}/api/{'rehearsal' if rehearsal else 'report'}"
        self._code = mission_code
        register_secret(mission_code)
        self._transport = transport
        self.clock_sync = ClockSync()
        if not math.isfinite(target_time_tolerance_s) or target_time_tolerance_s < 0:
            raise ValueError("target timestamp tolerance must be finite and nonnegative")
        self.target_time_tolerance_s = target_time_tolerance_s  # Diagnostic assumption, not a server rule.
        self.team, self.slot = team, slot
        self.timeout, self.wait_s, self.me_period_s = timeout, max(1, min(int(wait_s), 120)), me_period_s
        self._clock = clock
        self._q: "Queue[ServerMsg]" = Queue()
        self._lock = threading.Lock()
        self._stop_ev = threading.Event()
        self._threads: List[threading.Thread] = []
        self._started_bg = False
        # 서버 상태 캐시
        self.phase: Optional[str] = None
        self.ready: bool = False
        self._ident: Optional[Tuple[str, int]] = (team, slot) if team and slot else None
        self._sent: Dict[str, bool] = {}
        self._me_ok_t: Optional[float] = None
        self._me_err: Optional[str] = None
        self._submit_err: Optional[str] = None
        self._closed_submissions: set = set()
        self._permanent_rejections: set = set()
        self._warning_state = {}
        self._echo_mismatch_attempts = {}
        self._credential_generation = 0
        self._restart_pending = False
        self._order_issued_at = None
        self._target_issued_at = None
        # 9장 V6: issued_at 형식·유한 검사, tactical_command 제어문자 제거와 길이 제한. 버린 이유는 vlog 로 센다(run_real 이 robot.vlog 를 넘겨 한 파일에 모은다)
        self.max_command_chars = max(int(max_command_chars), 1)
        self.vlog = vlog if vlog is not None else ValidationLog()
        self.last_error: Optional[str] = None
        # 드론
        self._drone_next = 0.0
        self._drone_captured: Optional[str] = None
        self._drone_stamp: Optional[float] = None
        self._drone_interval = 1.0
        self._provisional_camp = None
        self.stats: Dict[str, int] = {"me": 0, "order": 0, "target": 0, "submit_ok": 0, "submit_rej": 0, "drone": 0}

    # ------------------------------------------------------------------ HTTP
    def _mask(self, s: str) -> str:
        return REDACT(s)

    def _warn_change(self, key, message):
        message = self._mask(message)
        if self._warning_state.get(key) != message:
            self._warning_state[key] = message
            logging.getLogger(__name__).warning(message)

    def set_mission_code(self, value: str) -> None:
        if not value:
            raise ValueError("mission code must not be empty")
        register_secret(value)
        with self._lock:
            self._code = value
            self._credential_generation += 1
            self._me_ok_t = None
            self._me_err = self._submit_err = None
            self._permanent_rejections.clear()

    def _url(self, path: str, params: Optional[Dict[str, Any]] = None) -> str:
        q = {"mission_code": self._code}
        q.update(params or {})
        return f"{self.api}/{path}?{urllib.parse.urlencode(q)}"

    def _req(self, req: urllib.request.Request, timeout: float):
        """(상태코드, 본문 bytes, 헤더). HTTP 오류도 값으로 돌려준다(409/422 는 정상 흐름이다). 연결 실패만 예외."""
        sent = self._clock()
        try:
            if self._transport is not None:
                result = self._transport(req, timeout)
            else:
                try:
                    with urllib.request.urlopen(req, timeout=timeout) as r:
                        result = r.status, r.read(), r.headers
                except urllib.error.HTTPError as e:
                    result = e.code, e.read(), e.headers
        except Exception as error:
            raise ConnectionError(self._mask(f"{type(error).__name__}: {error}")) from None
        self.clock_sync.observe(result[2].get("Date"), sent, self._clock())
        return result

    def _get_json(self, path: str, params: Optional[Dict[str, Any]] = None, timeout: Optional[float] = None) -> Tuple[int, Any]:
        st, body, _ = self._req(urllib.request.Request(self._url(path, params), method="GET"), timeout or self.timeout)
        try:
            return st, json.loads(body.decode("utf-8") or "{}")
        except (ValueError, UnicodeError):
            return st, {"detail": safe_detail(body.decode("utf-8", errors="replace"))}

    def _post_json(self, path: str, body: Dict[str, Any]) -> Tuple[int, Any]:
        b = dict(body)
        b["mission_code"] = self._code
        req = urllib.request.Request(f"{self.api}/{path}", data=json.dumps(b).encode("utf-8"),
                                     headers={"Content-Type": "application/json"}, method="POST")
        st, raw, _ = self._req(req, self.timeout)
        try:
            return st, json.loads(raw.decode("utf-8") or "{}")
        except (ValueError, UnicodeError):
            return st, {"detail": safe_detail(raw.decode("utf-8", errors="replace"))}

    @staticmethod
    def _detail(js: Any) -> str:
        d = js.get("detail") if isinstance(js, dict) else None
        return safe_detail(d) if d is not None else ""

    # ------------------------------------------------------------------ 상태(/me)
    def me(self) -> Optional[Dict[str, Any]]:
        """GET /me. 성공하면 캐시(phase·ready·정체성)를 갱신한다. 실패하면 None."""
        generation = self._credential_generation
        try:
            st, js = self._get_json("me")
        except Exception as e:
            if generation != self._credential_generation:
                return None
            self._me_err = self._mask(f"me {type(e).__name__}: {e}")
            return None
        if generation != self._credential_generation:
            return None
        self.stats["me"] += 1
        if st != 200 or not isinstance(js, dict):
            self._me_err = f"me HTTP {st} {self._mask(self._detail(js))}"
            return None
        self._me_ok_t, self._me_err = self._clock(), None
        js = dict(js)
        phase = str(js.get("phase") or "").strip().lower()
        js["phase"] = "running" if phase == "in_progress" else phase
        if js["phase"] not in schema.PHASE_LABELS:
            self._warn_change("unknown_phase", f"Unknown server phase: {safe_detail(phase)}")
            js["phase"] = None
        with self._lock:
            old_phase = self.phase
            if old_phase is None and js["phase"] == "running":
                self._restart_pending = True
            if js["phase"] in ("idle", "waiting") and old_phase != js["phase"]:
                self._permanent_rejections.clear()
                self._echo_mismatch_attempts.clear()
            self.phase, self.ready = js.get("phase"), bool(js.get("ready"))
            ident = schema.parse_slot(js.get("slot"))
            # slot=null means no assignment. Never retain a constructor override or an old match's slot.
            self._ident = ident
            if ident is not None:
                self.team, self.slot = ident
        return js

    def ready_handshake(self, init_ok: Callable[[], bool], should_quit: Callable[[], bool] = lambda: False,
                        poll_s: float = 2.0, sleep: Callable[[float], None] = time.sleep) -> bool:
        """1단계: 모델·필수 센서 초기화가 끝난 뒤(init_ok) phase=ready & ready=false 면 POST /ready. 409·통신 실패는 2 s 뒤 다시.
        안 보내면 그 팀을 빼고 시작할 수 있고 빠진 경기는 0점이다. 이미 ready이거나 진행 중이어도 로컬 초기화와 슬롯 확인 뒤에만 True."""
        while not should_quit():
            initialized = init_ok()   # Even a restarted/already-ready process must initialize its own sensors/model.
            js = self.me()
            if js is not None:
                ph = js.get("phase")
                if initialized and self._ident is not None and (ph in ("running", "reporting", "finished") or js.get("ready")):
                    return True
                if ph == "ready" and initialized and self._ident is not None:
                    try:
                        st, r = self._post_json("ready", {})
                        if st == 200 and isinstance(r, dict) and r.get("accepted"):
                            self.ready = True
                            return True
                        self.last_error = self._mask(f"ready HTTP {st} {self._detail(r)}")
                        self._warn_change("ready", self.last_error)
                    except Exception as e:
                        self.last_error = self._mask(f"ready {type(e).__name__}: {e}")
            sleep(poll_s)
        return False

    # ------------------------------------------------------------------ 배경 스레드
    def start_background(self) -> None:
        if self._started_bg:
            return
        self._started_bg = True
        for fn, name in ((self._me_loop, "me"), (lambda: self._wait_loop("order"), "order"), (lambda: self._wait_loop("target"), "target")):
            t = threading.Thread(target=fn, name=f"http-{name}", daemon=True)
            t.start()
            self._threads.append(t)

    def _emit(self, key: str, msg: ServerMsg) -> None:
        with self._lock:
            if self._sent.get(key):
                return
            self._sent[key] = True
        self._q.put(msg)

    def _me_loop(self) -> None:
        while not self._stop_ev.is_set():
            self.me()
            ph = self.phase
            if ph in RUN_PHASES and not self._restart_pending:
                self._emit("start", ServerMsg("start", stamp=self._clock(), data={"src": "server_phase"}))
            elif ph in RUN_PHASES:
                self._maybe_emit_recovered_start()
            if ph in STOP_PHASES:
                self._emit("stop", ServerMsg("stop", stamp=self._clock(), data={"src": "server_phase", "phase": ph}))
            self._stop_ev.wait(self.me_period_s if ph in ("running", "reporting") else 1.0)

    def _maybe_emit_recovered_start(self):
        elapsed = self.clock_sync.elapsed(self._order_issued_at, self._clock())
        if self._restart_pending and elapsed is not None and self.phase == "running":
            self._emit("start", ServerMsg("start", stamp=self._clock(), data={
                "src": "server_phase", "restarted": True, "elapsed_s": elapsed,
                "clock_sync": True, "clock_uncertainty_s": self.clock_sync.uncertainty_s}))
            self._restart_pending = False
            return True
        return False

    def _reason_delay(self, kind, reason):
        category = schema.classify_reason(reason)
        if category == "NOT_ASSIGNED":
            identity_note = "unassigned /me" if self._ident is None else "assignment differs from /me; checking again"
            message = f"{kind}: NOT_ASSIGNED ({identity_note}); contact operator"
            delay = 10.
        elif category == "NO_POST":
            message, delay = f"{kind}: NO_POST; contact operator, retrying late publication", 5.
        elif category == "NO_REHEARSAL":
            message, delay = f"{kind}: waiting for operator to open rehearsal", 5.
        elif category == "NOT_YET":
            message, delay = f"{kind}: publication pending", .5
        else:
            message, delay = f"{kind}: unknown reason {safe_detail(reason)}", 1.
        self._warn_change(("reason", kind), message)
        return delay

    def _clean_issued_at(self, kind: str, order: Dict[str, Any]):
        """V6: issued_at 이 문서 형식('YYYY-MM-DD HH:MM:SS') 문자열이거나 유한한 양수일 때만 쓴다. 없으면 None, 쓰레기도 None(+ 이유 기록)."""
        raw = order.get("issued_at")
        clean = clean_issued_at(raw)
        if raw is not None and clean is None:
            self.vlog.bad("server_" + kind, "issued_at_invalid", raw_label(raw))
        return clean

    def _clean_command(self, raw: Any) -> str:
        """V6: tactical_command 의 제어문자를 빼고(줄바꿈·탭은 공백 하나) link.max_command_chars 로 자른다. 잘랐거나 뺐으면 이유를 센다(경고는 5 s 에 한 줄)."""
        text, cut, removed = sanitize_command_text(raw, self.max_command_chars)
        if cut:
            self.vlog.bad("server_order", "command_truncated", f"{self.max_command_chars}자로 자름")
        if removed:
            self.vlog.bad("server_order", "command_control_chars", f"제어문자 {removed}개 제거")
        return text

    def _crosscheck_target_time(self):
        if self._order_issued_at is None or self._target_issued_at is None:
            return
        delta = self.clock_sync.publication_delta(self._order_issued_at, self._target_issued_at)
        if delta is None or abs(delta-240.) > self.target_time_tolerance_s:
            self._warn_change("target_timestamp", "target: publication time differs from the expected 4:00; keeping server objective and existing clock")

    def _wait_loop(self, kind: str) -> None:
        """/order 또는 /target 을 긴 폴링으로 기다려 한 번 받으면 끝낸다. accepted:false(게시 전)는 오류가 아니라 다시 묻는다."""
        fails = 0                                            # 연속 실패 수: 서버가 오류를 즉시 돌려줄 때 초당 수 회로 두드리지 않게 늘려 쉰다
        while not self._stop_ev.is_set():
            try:
                st, js = self._get_json(kind, {"wait": self.wait_s}, timeout=self.wait_s + max(5., self.timeout))
            except Exception as e:
                self.last_error = self._mask(f"{kind} {type(e).__name__}: {e}")
                fails += 1
                self._stop_ev.wait(self._backoff(fails))
                continue
            self.stats[kind] += 1
            if st == 200 and isinstance(js, dict) and js.get("accepted") and isinstance(js.get("order"), dict):
                o = js["order"]
                issued = self._clean_issued_at(kind, o)                          # V6: 형식이 아니면 '없는 것'
                if kind == "order":
                    self._order_issued_at = issued
                    self._maybe_emit_recovered_start()
                    self._crosscheck_target_time()
                    self._emit("order", ServerMsg("command", text=self._clean_command(o.get("tactical_command", "")), stamp=self._clock(),
                                                  data={"issued_at": issued}))
                else:
                    self._target_issued_at = issued
                    self._crosscheck_target_time()
                    self._emit("target", ServerMsg("base_command", text=str(o.get("objective_label", "")), stamp=self._clock(),
                                                   data={"objective": o.get("objective"), "issued_at": issued}))
                return
            reason = str(js.get("reason", "")) if isinstance(js, dict) else ""
            if st in (401, 422):
                self.last_error = self._mask(f"{kind} HTTP {st} {reason or self._detail(js)}")
                self._stop_ev.wait(5.0)                     # 기다려도 오지 않는다/코드 오류: 느리게 재시도(로그만)
            elif st == 200:
                fails = 0
                self._stop_ev.wait(self._reason_delay(kind, reason))
            else:
                fails += 1                                  # 409·5xx 등: 점점 늘려 쉰다(최대 5 s)
                self.last_error = self._mask(f"{kind} HTTP {st} {reason or self._detail(js)}")
                self._stop_ev.wait(self._backoff(fails))

    @staticmethod
    def _backoff(fails: int) -> float:
        return min(0.5 * (2 ** max(fails - 1, 0)), 5.0)

    # ------------------------------------------------------------------ ServerLink
    def poll(self) -> List[ServerMsg]:
        self.start_background()
        out: List[ServerMsg] = []
        while True:
            try:
                out.append(self._q.get_nowait())
            except Empty:
                return out

    def submit(self, kind: str, payload: Dict[str, Any]) -> bool:
        """M1·M3·M6 제출. 200 & accepted → True. 409(단계 아님)·422(형식)·401 은 False 로 돌려 Submitter 가 재시도/기록한다."""
        if kind in self._closed_submissions or kind in self._permanent_rejections:
            self._submit_err = f"submit {kind}: local receive-time deadline or STOP"
            return False
        if kind in ("m1", "m3", "m6"):
            # Check actual server state in the submission worker. A queued
            # request must not use a stale local IN_PROGRESS assumption.
            fresh = self._me_ok_t is not None and self._clock() - self._me_ok_t <= max(1., 2*self.me_period_s)
            expected = STOP_PHASES[:1] if kind == "m6" else RUN_PHASES
            if not fresh or self.phase not in expected:
                if self.me() is None:
                    self._submit_err = f"submit {kind}: server phase could not be confirmed"
                    return False
            if self.submission_allowed(kind) is False:
                self._submit_err = f"submit {kind}: phase does not allow submission, current phase={self.phase}"
                return False
        ident = self._ident
        if kind not in schema.ENDPOINTS or ident is None:
            self._submit_err = f"submit {kind}: 정체성 미확인" if ident is None else f"알 수 없는 종류 {kind}"
            return False
        try:
            body = schema.to_server(kind, payload, ident[0])
        except (ValueError, TypeError, AttributeError) as e:
            self._submit_err = self._mask(f"submit {kind}: invalid payload ({e})")
            return False
        if kind == "m1" and not body:
            self._submit_err = "M1 보낼 항목이 하나도 없다(서버는 422)"
            return False
        if kind == "m6" and not body.get("ranking"):
            self._submit_err = "M6 requires 1–3 opponent callsigns"
            return False
        fingerprint = (kind, json.dumps(body, sort_keys=True, ensure_ascii=False))
        if self._echo_mismatch_attempts.get(fingerprint, 0) >= 2:
            self._submit_err = f"submit {kind}: received mismatch retry budget exhausted"
            return False
        try:
            st, js = self._post_json(schema.ENDPOINTS[kind], body)
        except Exception as e:
            self._submit_err = self._mask(f"submit {kind} {type(e).__name__}: {e}")
            return False
        ok = st == 200 and isinstance(js, dict) and bool(js.get("accepted"))
        if ok and not schema.equal_received(kind, body, js.get("received"), ident[0]):
            self._echo_mismatch_attempts[fingerprint] = self._echo_mismatch_attempts.get(fingerprint, 0)+1
            self._warn_change(("received", fingerprint), f"submit {kind}: received payload differs; at most one retransmission")
            self._submit_err = f"submit {kind}: received payload mismatch"
            self.stats["submit_rej"] += 1
            return False
        self.stats["submit_ok" if ok else "submit_rej"] += 1
        self._submit_err = None if ok else self._mask(f"submit {kind} HTTP {st} {self._detail(js)}")
        if st == 409:
            reason = schema.classify_submission_409(js.get("detail") if isinstance(js, dict) else None)
            if reason in ("NOT_ASSIGNED", "EXCLUDED", "UGV_NOT_STARTED"):
                self._permanent_rejections.add(kind)
                self._warn_change(("permanent", kind), f"submit {kind}: {reason}; retries stopped for this match")
            self.me()  # Rejection is never ACK; refresh phase before any retry.
        return ok

    def submission_allowed(self, kind: str) -> Optional[bool]:
        if kind in self._closed_submissions or kind in self._permanent_rejections:
            return False
        if self.phase is None:
            return None
        return self.phase == ("reporting" if kind == "m6" else "running")

    def close_submission(self, kind: str) -> None:
        self._closed_submissions.add(kind)

    def get_uav(self) -> Optional[UavFrame]:
        """공식 p4 JPEG 수신. 경기 영상 또는 명시한 연습 카메라; 리허설 경로에는 drone API가 없다.
        JPEG와 선택적 BGR를 제공한다. 명시적 시뮬 보정 콜백만 박스 형상 후보를 추가하며 UGV/M3 분류는 만들지 않는다.
        """
        now = self._clock()
        if self.rehearsal and self.drone_camera is None:
            return None
        if now < self._drone_next or (self.drone_camera is None and self.phase != "running"):
            return None
        self._drone_next = now + max(self._drone_interval, 1.0)
        try:
            params = {"mission_code": self._code}
            if self.drone_camera is not None:
                params["camera"] = self.drone_camera
            url = f"{self.root}/api/report/drone?{urllib.parse.urlencode(params)}"
            st, body, hdr = self._req(urllib.request.Request(url, method="GET"), self.timeout)
        except Exception as e:
            self.last_error = self._mask(f"drone {type(e).__name__}: {e}")
            return None
        if st == 403:                                       # 우리 진영 시설물 3개 파괴: 더는 받을 수 없다
            self._drone_next = now + 5.0
            return None
        if st != 200 or not body or not str(hdr.get("Content-Type", "")).lower().startswith("image/jpeg"):
            return None
        try:
            interval = float(hdr.get("X-Frame-Interval-S", 1.0))
            self._drone_interval = interval if math.isfinite(interval) and interval > 0 else 1.
        except (TypeError, ValueError):
            self._drone_interval = 1.
        self._drone_next = now + max(self._drone_interval, 1.)
        camera = str(hdr.get("X-Camera", ""))
        test_mode = str(hdr.get("X-Test-Mode", "off")).lower() == "on"
        camp = str(hdr.get("X-Camp", "")).upper()
        if self.drone_camera is None:
            if test_mode or camera != "main":
                self._warn_change("drone_mode", "drone: match request received unexpected test-mode/camera headers")
            if camp in ("BLUE", "RED"):
                if self._ident is not None and self._me_err is None:
                    if camp != self._ident[0]:
                        self._warn_change("drone_camp", "drone: X-Camp differs from /me; /me remains authoritative")
                elif self._me_err is not None:
                    self._provisional_camp = camp
                    self.team = camp  # Never invent a slot or an assigned identity from this header.
        elif not test_mode:
            self._warn_change("drone_mode", "drone: practice request received X-Test-Mode other than on")
        try:
            destroyed = max(0, min(2, int(hdr.get("X-Destroyed"))))
        except (ValueError, TypeError, OverflowError):
            destroyed = None
        if self.drone_camera is None and destroyed is not None and self._drone_interval != (1., 2., 5.)[destroyed]:
            self._warn_change("drone_interval", "drone: destroyed count and frame interval disagree")
        cap = hdr.get("X-Captured-At")
        if cap is None or cap == self._drone_captured:
            return None
        try:
            captured = datetime.datetime.fromisoformat(cap)
            if captured.tzinfo is None:
                return None   # Contract specifies +09:00; never mix wall-clock and monotonic timestamps.
            stamp = captured.timestamp()
        except (TypeError, ValueError, OverflowError):
            return None
        if self._drone_stamp is not None and stamp <= self._drone_stamp:
            return None
        self._drone_captured, self._drone_stamp = cap, stamp
        self.stats["drone"] += 1
        # Preserve receipt time before optional inference, which runs inside
        # AsyncServerLink's UAV worker. Its processing delay must remain visible
        # when Agent converts capture age to the robot clock.
        received_mono = time.monotonic()
        source_age = (self._clock() + self.clock_sync.offset - stamp
                      if self.clock_sync.offset is not None else None)
        image = None
        if self.decode_drone:
            try:
                import cv2
                import numpy as np
                image = cv2.imdecode(np.frombuffer(body, dtype=np.uint8), cv2.IMREAD_COLOR)
            except ImportError:
                pass  # JPEG remains available without the optional image decoder.
            except Exception as e:
                self.last_error = f"drone image decode: {type(e).__name__}"
        # 서버·로봇 시계 차이로 나이가 음수이거나 터무니없어도 트랙이 망가지지 않게 0~30 s 로 제한한다.
        age = min(max(datetime.datetime.now(datetime.timezone.utc).timestamp() - stamp, 0.0), 30.0)
        frame = UavFrame(stamp=stamp, image=image, jpeg=body, interval_s=self._drone_interval,
                         age_s=age, recv_mono=time.monotonic(),
                         destroyed=destroyed, camera=camera or None,
                         test_mode=test_mode, camp=camp if camp in ("BLUE", "RED") else None)
        if self.uav_layout_detector is not None:
            frame.recv_mono = received_mono
            if source_age is not None and math.isfinite(source_age):
                frame.age_s = min(max(source_age, 0.), 30.)
            try:
                # HTTP Date is second-resolution. Allow that bounded rounding
                # uncertainty, never a substantially future or stale layout.
                uncertainty = self.clock_sync.uncertainty_s or 0.
                if (source_age is None or not math.isfinite(source_age) or
                        not -uncertainty <= source_age <= self.uav_layout_max_age_s):
                    raise ValueError("unusable capture age")
                if image is None:
                    raise ValueError("decoded image unavailable")
                expected_camera = getattr(self.uav_layout_detector, "camera", None)
                if expected_camera and camera and camera != expected_camera:
                    raise ValueError("camera differs from the calibrated one")   # 보정은 카메라마다 다르다 — 다른 카메라 프레임에 적용하지 않는다
                boxes = list(self.uav_layout_detector(image))
                if len(boxes) > 512 or any(not isinstance(box, UavBox) for box in boxes):
                    raise ValueError("invalid footprint candidates")
                frame.boxes = boxes
                frame.layout_source = self.uav_layout_source
                self.stats["drone_layout_ok"] = self.stats.get("drone_layout_ok", 0) + 1
                # Keep None: Agent must use its converted frame.stamp, not the
                # raw server epoch, when ingesting these same-image footprints.
                frame.layout_stamp = None
            except Exception as error:
                self.stats["drone_layout_rejected"] = self.stats.get("drone_layout_rejected", 0)+1
                self.last_error = "drone layout inference: " + type(error).__name__
        if self.uav_robot_detector is not None:
            try:
                uncertainty = self.clock_sync.uncertainty_s or 0.
                if (source_age is None or not math.isfinite(source_age) or
                        not -uncertainty <= source_age <= self.uav_layout_max_age_s or image is None):
                    raise ValueError("unusable capture image or age")
                detections = list(self.uav_robot_detector(image))
                if len(detections) > 16 or any(not isinstance(d,UavDet) or
                        not (math.isfinite(d.x) and math.isfinite(d.y)) or
                        d.team not in (None,"BLUE","RED") for d in detections):
                    raise ValueError("invalid image robot observations")
                frame.detections = detections
                frame.recv_mono = received_mono
                frame.age_s = min(max(source_age,0.),30.)
            except Exception as error:
                self.stats["drone_robots_rejected"] = self.stats.get("drone_robots_rejected",0)+1
                self.last_error = "drone robot inference: " + type(error).__name__
        return frame

    def identity(self) -> Optional[Tuple[str, int]]:
        """배경 /me 루프가 돌고 있으면 캐시만 돌려준다(Agent 제어 루프에서 불려도 막히지 않는다). 시작 전(핸드셰이크 단계)에만 직접 묻는다."""
        if self._ident is None and not self._started_bg:
            self.me()
        return self._ident

    def health(self) -> Tuple[bool, str]:
        now = self._clock()
        if self._started_bg and (self._me_ok_t is None or now - self._me_ok_t > 5.0) and self._me_err:
            return False, self._me_err
        if self._submit_err:
            return False, self._submit_err
        return True, ""

    def close(self) -> None:
        self._stop_ev.set()
