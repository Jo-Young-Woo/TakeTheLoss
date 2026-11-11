"""In-process API fixture for the 2026-11-04 participant guide; no sockets.

The wire validators and published error text live in ``link.schema``. Missing
mission content is never invented: callers supply orders/objectives explicitly.
``FakeClock`` makes long polls, receipt deadlines and faults deterministic without
sleeping. It advances a single simulated timeline, not concurrent wall time.
Unspecified server internals (notably omitted error suffixes) are not asserted.
"""
from __future__ import annotations

import copy
import datetime as dt
import heapq
import json
import math
import random
import re
from email.utils import format_datetime
from collections import Counter, defaultdict, deque
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, Optional
from urllib.parse import parse_qs, urlsplit

from ..link import schema


class FakeClock:
    """Monotonic seconds and scheduled callbacks; never waits in real time."""

    def __init__(self, now: float = 0.0):
        self.now = float(now)
        self._events = []
        self._seq = 0

    def __call__(self):
        return self.now

    def schedule(self, at: float, callback: Callable[[], None]):
        self._seq += 1
        heapq.heappush(self._events, (max(self.now, float(at)), self._seq, callback))

    def advance(self, seconds: float):
        end = self.now + max(0.0, float(seconds))
        while self._events and self._events[0][0] <= end:
            at, _, callback = heapq.heappop(self._events)
            self.now = max(self.now, at)
            callback()
        self.now = max(self.now, end)

    def wait_until(self, predicate: Callable[[], bool], deadline: float):
        while not predicate() and self.now < deadline:
            at = self._events[0][0] if self._events else deadline
            self.advance(max(0.0, min(at, deadline) - self.now))


@dataclass
class Team:
    slot: Optional[str]
    assigned: bool = True
    excluded: bool = False
    started: bool = True
    config_confirmed: bool = True
    ready: bool = False
    reports: Dict[str, dict] = field(default_factory=dict)

    @property
    def camp(self):
        identity = schema.parse_slot(self.slot) if self.assigned else None
        return identity[0] if identity else None


@dataclass
class Match:
    phase: str = "idle"
    active: bool = False
    before_start: bool = False
    start_at: Optional[float] = None
    reporting_at: Optional[float] = None
    teams: Dict[str, Team] = field(default_factory=dict, repr=False)
    orders: Dict[str, dict] = field(default_factory=dict)
    targets: Dict[str, dict] = field(default_factory=dict)
    generation: int = 0


@dataclass
class Fault:
    """Test-only transport fault. Payload is excluded from repr/logs."""
    kind: str
    delay_s: float = 0.0
    status: int = 503
    body: Any = field(default=None, repr=False)
    headers: dict = field(default_factory=dict, repr=False)


class MockServer:
    """Callable replacement for HttpServerLink._req(Request, timeout).

    Direct ``request(method, path, params=..., body=...)`` returns the same
    ``(status, bytes, headers)`` tuple. Secret values are never retained in public
    request/fault diagnostics; authentication storage is private.
    """

    def __init__(self, teams: Optional[Dict[str, Optional[str]]] = None, *,
                 clock: Optional[FakeClock] = None, received_order: str = "spec", seed: int = 0):
        if received_order not in ("spec", "reverse", "shuffle"):
            raise ValueError("received_order must be spec, reverse or shuffle")
        self.clock = clock or FakeClock()
        self.received_order = received_order
        self._rng = random.Random(seed)
        self._codes = {}
        self.matches = {"report": Match(), "rehearsal": Match()}
        self.requests = []
        self.attempts = Counter()
        self.fault_log = []
        self._faults = defaultdict(deque)
        self._previous = {}
        self.test_mode = False
        self.camera_enabled = {name: True for name in ("main", "test1", "test2")}
        self.destroyed = {"BLUE": 0, "RED": 0}
        self._frames = {}
        self._camp_frames = {}
        self.epoch = dt.datetime(2026, 11, 4, 12, tzinfo=dt.timezone(dt.timedelta(hours=9)))
        for code, slot in (teams or {}).items():
            self.add_team(code, slot)

    def _redact(self, value):
        if isinstance(value, dict):
            return {k: "[redacted]" if str(k).lower() in ("mission_code", "token") else self._redact(v)
                    for k, v in value.items()}
        if isinstance(value, (list, tuple)):
            return [self._redact(v) for v in value]
        if isinstance(value, str):
            for code in self._codes:
                value = re.sub(re.escape(code), "[redacted]", value, flags=re.IGNORECASE)
        return value

    def add_team(self, code: str, slot: Optional[str]):
        if not isinstance(code, str) or not code:
            raise ValueError("a test mission code is required")
        key = code.upper()
        if key in self._codes:
            raise ValueError("duplicate test mission code")
        ident = f"team-{len(self._codes) + 1}"
        self._codes[key] = ident
        for match in self.matches.values():
            match.teams[ident] = Team(slot, assigned=slot is not None)

    def team(self, code: str, api: str = "report") -> Team:
        ident = self._codes.get(str(code).upper())
        if ident is None:
            raise ValueError("unknown test team")
        return self.matches[api].teams[ident]

    def _kst(self, stamp: float, iso: bool = False):
        value = self.epoch + dt.timedelta(seconds=stamp)
        return value.isoformat(timespec="milliseconds") if iso else value.strftime("%Y-%m-%d %H:%M:%S")

    def set_phase(self, phase: str, api: str = "report"):
        if phase not in schema.PHASE_LABELS:
            raise ValueError("unknown test phase")
        match = self.matches[api]
        match.phase, match.active = phase, phase != "idle"
        if phase == "reporting":
            match.reporting_at = self.clock()
            generation = match.generation
            self.clock.schedule(self.clock() + 30, lambda: self._finish(api, generation))

    def _finish(self, api, generation):
        match = self.matches[api]
        if match.generation == generation and match.phase == "reporting":
            match.phase = "finished"

    def reset(self, api: str = "report"):
        """Fixture for a new match/server reset; old scheduled events are inert."""
        old = self.matches[api]
        self.matches[api] = Match(generation=old.generation + 1,
                                 teams={key: Team(t.slot, assigned=t.assigned) for key, t in old.teams.items()})

    def publish_order(self, slot: str, text: str, api: str = "report"):
        self.matches[api].orders[slot] = {"tactical_command": text, "issued_at": self._kst(self.clock())}

    def publish_target(self, camp: str, objective: str, label: str, api: str = "report"):
        self.matches[api].targets[camp] = {"objective": objective, "objective_label": label,
                                          "issued_at": self._kst(self.clock())}

    def start(self, *, api: str = "report", orders: Optional[Dict[str, str]] = None,
              targets: Optional[Dict[str, tuple]] = None):
        """Operator fixture: rehearsal starts five seconds after the click."""
        match = self.matches[api]
        generation = match.generation
        match.active = True
        match.before_start = api == "rehearsal"
        at = self.clock() + (5.0 if api == "rehearsal" else 0.0)

        def begin():
            if self.matches[api].generation != generation:
                return
            match.phase, match.before_start, match.start_at = "running", False, self.clock()
            for slot, command in (orders or {}).items():
                self.publish_order(slot, command, api)
            self.clock.schedule(self.clock() + 240, target)
            self.clock.schedule(self.clock() + 300, end)

        def target():
            if self.matches[api].generation == generation and match.phase == "running":
                for camp, (objective, label) in (targets or {}).items():
                    self.publish_target(camp, objective, label, api)

        def end():
            if self.matches[api].generation == generation and match.phase == "running":
                self.set_phase("reporting", api)

        self.clock.schedule(at, begin)
        self.clock.advance(0)

    def capture(self, jpeg: bytes, camera: str = "main", *, valid: bool = True, captured_at=None):
        """Supply a decoded/validated camera fixture; invalid captures keep the last good image."""
        if camera not in self.camera_enabled:
            raise ValueError("unknown test camera")
        stamp = self.clock() if captured_at is None else float(captured_at)
        previous = self._frames.get(camera)
        if (not valid or not math.isfinite(stamp) or stamp < 0 or stamp > self.clock()
                or not self.camera_enabled[camera] or (previous and stamp - previous[0] < 1)):
            return False
        frame = (stamp, bytes(jpeg))
        self._frames[camera] = frame
        if camera == "main":
            for camp in self.destroyed:
                count = self.destroyed[camp]
                if count >= 3:
                    continue
                period = (1.0, 2.0, 5.0)[max(0, count)]
                old = self._camp_frames.get(camp)
                if old is None or frame[0] - old[0] >= period:
                    self._camp_frames[camp] = frame
        return True

    def inject(self, endpoint: str, kind: str, *, delay_s: float = 0.0, status: int = 503,
               body: Any = None, headers: Optional[dict] = None, times: int = 1):
        """Queue delay/status/non_json/drop/duplicate faults (not API behavior).

        A drop occurs after handling, modeling loss of a committed response.
        Duplicate replays the preceding response without committing it twice.
        """
        if kind not in ("delay", "status", "non_json", "drop", "duplicate"):
            raise ValueError("unknown test fault")
        for _ in range(max(0, int(times))):
            self._faults[endpoint.strip("/")].append(Fault(kind, max(0, delay_s), status, body, headers or {}))

    def chaos(self, *, seed: int, count: int, endpoint: str = "*", kinds=("delay", "status", "non_json", "drop", "duplicate")):
        """Reproducible fault script; no probabilities or real-time sleeps."""
        rng = random.Random(seed)
        for _ in range(count):
            kind = rng.choice(kinds)
            self.inject(endpoint, kind, delay_s=rng.choice((0.1, 0.5, 2.0)) if kind == "delay" else 0.0)

    def _date(self):
        value = (self.epoch + dt.timedelta(seconds=self.clock())).astimezone(dt.timezone.utc)
        return format_datetime(value, usegmt=True)

    def _json(self, status, body, headers=None):
        return status, json.dumps(body, ensure_ascii=False).encode("utf-8"), {"Content-Type": "application/json", "Date": self._date(), **(headers or {})}

    def _invalid(self, loc, msg):
        return self._json(422, {"detail": [{"loc": list(loc), "msg": msg}]})

    def __call__(self, request, timeout):
        """urllib Request transport adapter; urllib is used only to parse, never send."""
        raw = request.data
        body = None
        if raw is not None:
            try:
                body = json.loads(raw.decode("utf-8"))
            except (ValueError, UnicodeDecodeError):
                return self._invalid(("body",), "Invalid JSON")
        return self.request(request.get_method(), request.full_url, body=body, timeout=timeout,
                            headers=dict(request.header_items()))

    def request(self, method: str, path: str, *, params=None, body=None, timeout: float = 135.0, headers=None):
        parts = urlsplit(path)
        route = parts.path.strip("/").split("/")
        query = {k: values[-1] for k, values in parse_qs(parts.query, keep_blank_values=True).items()}
        query.update(params or {})
        endpoint = route[-1]
        method = method.upper()
        self.attempts[endpoint] += 1
        record = {"method": method, "path": parts.path, "params": self._redact(query), "body": self._redact(body),
                  "sent_at": self.clock(), "received_at": None}
        self.requests.append(record)
        fault = self._faults[endpoint].popleft() if self._faults[endpoint] else (
            self._faults["*"].popleft() if self._faults["*"] else None)
        if fault:
            self.fault_log.append({"endpoint": endpoint, "kind": fault.kind, "at": self.clock()})
            if fault.delay_s > timeout:
                self.clock.advance(timeout)
                raise TimeoutError("mock transport timeout")
            self.clock.advance(fault.delay_s)
        record["received_at"] = self.clock()
        if fault and fault.kind == "status":
            return self._json(fault.status, fault.body if fault.body is not None else {"detail": "injected transport fault"}, fault.headers)
        if fault and fault.kind == "non_json":
            return fault.status, b"injected non-JSON transport response", {"Date": self._date(), **fault.headers}
        if fault and fault.kind == "duplicate" and endpoint in self._previous:
            return copy.deepcopy(self._previous[endpoint])
        response = self._handle(method, route, query, body, timeout, headers or {})
        self._previous[endpoint] = copy.deepcopy(response)
        if fault and fault.kind == "drop":
            raise TimeoutError("mock response lost after receipt")
        return response

    def _handle(self, method, route, query, body, timeout, headers):
        if len(route) != 3 or route[0] != "api" or route[1] not in self.matches:
            return self._json(404, {"detail": "unsupported mock route"})
        api, endpoint = route[1:]
        allowed = {"me": "GET", "ready": "POST", "order": "GET", "target": "GET",
                   **{ep: "POST" for ep in schema.ENDPOINTS.values()}}
        if api == "report":
            allowed["drone"] = "GET"
        if endpoint not in allowed:
            return self._json(404, {"detail": "unsupported mock route"})
        if method != allowed[endpoint]:
            return self._json(405, {"detail": "unsupported mock method"})
        if method == "POST" and not isinstance(body, dict):
            return self._invalid(("body",), "Input should be an object")
        if method == "POST":
            content_type = next((v for k, v in headers.items() if k.lower() == "content-type"), "application/json")
            if not str(content_type).lower().startswith("application/json"):
                return self._invalid(("body",), "Content-Type must be application/json")
        data = query if method == "GET" else body
        if "mission_code" not in data:
            return self._invalid(("query" if method == "GET" else "body", "mission_code"), "Field required")
        code = data["mission_code"]
        ident = self._codes.get(code.upper()) if isinstance(code, str) else None
        if ident is None:
            return self._json(401, {"detail": schema.ERR_INVALID_CODE})
        match = self.matches[api]
        team = match.teams[ident]
        if endpoint == "me":
            return self._json(200, {"slot": team.slot if team.assigned else None, "camp": team.camp,
                                    "phase": match.phase, "ready": team.ready})
        if endpoint == "ready":
            reason = ("not_assigned" if not team.assigned else "phase" if match.phase != "ready" else
                      "before_start" if match.before_start else "unconfirmed" if not team.config_confirmed else None)
            if reason:
                return self._json(409, {"detail": schema.ready_error(reason, match.phase)})
            team.ready = True
            return self._json(200, {"accepted": True})
        if endpoint in ("order", "target"):
            return self._order(match, team, api, endpoint, query, timeout)
        if endpoint == "drone":
            return self._drone(match, team, query)
        kind = next(k for k, ep in schema.ENDPOINTS.items() if ep == endpoint)
        reason = ("not_assigned" if not team.assigned else "excluded" if team.excluded else
                  "not_started" if not team.started else "phase" if match.phase != ("reporting" if kind == "m6" else "running") else None)
        if reason:
            return self._json(409, {"detail": schema.submission_error(kind, reason, match.phase)})
        try:
            payload = {k: v for k, v in body.items() if k != "mission_code"}
            received = (schema.validate_m6(payload, team.camp) if kind == "m6" else
                        {"m1": schema.validate_m1, "m3": schema.validate_m3}[kind](payload))
        except schema.ValidationError as exc:
            return self._json(422, {"detail": self._redact(exc.detail)})
        received = self._received_order(kind, received)
        team.reports[kind] = copy.deepcopy(received)
        if kind == "m6":
            eligible = [t for t in match.teams.values() if t.assigned and not t.excluded and t.started]
            if len(eligible) == 6 and all("m6" in t.reports for t in eligible):
                match.phase = "finished"
        return self._json(200, {"accepted": True, "received": received})

    def _received_order(self, kind, received):
        received = copy.deepcopy(received)
        lists = []
        if kind == "m1" and received.get("search_sectors"):
            lists.append(received["search_sectors"])
        elif kind == "m3":
            lists.append(received["sectors"])
            lists.extend(sec["objects"] for sec in received["sectors"])
        for values in lists:
            if self.received_order == "reverse":
                values.reverse()
            elif self.received_order == "shuffle":
                self._rng.shuffle(values)
        return received

    def _order(self, match, team, api, endpoint, query, timeout):
        try:
            wait = max(0, min(120, int(query.get("wait", 0))))
        except (ValueError, TypeError):
            return self._invalid(("query", "wait"), "Input should be an integer")

        def response():
            if api == "rehearsal" and not match.active:
                return {"accepted": False, "reason": schema.REASON_NO_REHEARSAL}
            if not team.assigned:
                return {"accepted": False, "reason": schema.REASON_NOT_ASSIGNED}
            order = match.orders.get(team.slot) if endpoint == "order" else match.targets.get(team.camp)
            if order is not None:
                return {"accepted": True, "order": copy.deepcopy(order)}
            not_yet = (match.phase in ("idle", "waiting", "ready") if endpoint == "order" else
                       match.phase in ("idle", "waiting", "ready") or
                       (match.phase == "running" and (match.start_at is None or self.clock() < match.start_at + 240)))
            reason = (schema.REASON_ORDER_NOT_YET if endpoint == "order" else schema.REASON_TARGET_NOT_YET) if not_yet else schema.REASON_NO_POST
            return {"accepted": False, "reason": reason}

        initial = response()
        immediate = initial.get("accepted") or not team.assigned or (api == "rehearsal" and not match.active)
        if not immediate and wait:
            deadline = self.clock() + min(wait, timeout)
            self.clock.wait_until(lambda: bool(response().get("accepted")), deadline)
            if not response().get("accepted") and timeout < wait:
                raise TimeoutError("mock long-poll client timeout")
        return self._json(200, response())

    def _drone(self, match, team, query):
        camera = query.get("camera")
        if camera is not None and camera not in self.camera_enabled:
            return self._invalid(("query", "camera"), "Input should be main, test1 or test2")
        practice = self.test_mode and camera is not None
        camera = camera if practice else "main"
        count = self.destroyed.get(team.camp, 0)
        reason = None
        if not practice:
            reason = "not_assigned" if not team.assigned else "phase" if match.phase != "running" else "destroyed" if count >= 3 else None
        if reason:
            return self._json(403 if reason == "destroyed" else 409,
                              {"detail": schema.drone_error(reason, match.phase, test_mode_requested="camera" in query and not self.test_mode)})
        frame = self._frames.get(camera) if practice else self._camp_frames.get(team.camp)
        if not self.camera_enabled[camera] or frame is None:
            reason = "disabled" if not self.camera_enabled[camera] else "no_frame"
            return self._json(503, {"detail": schema.drone_error(reason, match.phase)})
        interval = 1.0 if practice else (1.0, 2.0, 5.0)[max(0, count)]
        headers = {"Content-Type": "image/jpeg", "Date": self._date(), "X-Captured-At": self._kst(frame[0], iso=True),
                   "X-Frame-Interval-S": str(interval), "X-Camera": camera, "X-Test-Mode": "on" if practice else "off"}
        if not practice:
            headers.update({"X-Camp": team.camp, "X-Destroyed": str(count)})
        return 200, frame[1], headers
