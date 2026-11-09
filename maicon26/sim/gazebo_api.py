"""Local Gazebo frames/state plugged into the existing exact API fixture.

No sockets are opened on import. HTTP routes, authentication, phase gates,
schema validation and drone headers remain implemented by MockServer.
"""
from __future__ import annotations

from collections import Counter, deque
import datetime as dt
import heapq
import io
import ipaddress
import json
import math
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlsplit

from ..link import schema
from ..link.secrets import register_secret
from .mockserver import MockServer


class SimulationClock:
    """Only /clock advances time; long-poll waits release the shared condition."""
    def __init__(self, condition):
        self.condition = condition
        self.now, self.initialized, self.closed, self.invalid = 0., False, False, False
        self._events, self._seq = [], 0

    def __call__(self):
        return self.now

    def schedule(self, at, callback):
        self._seq += 1
        heapq.heappush(self._events, (max(self.now, float(at)), self._seq, callback))

    def update(self, stamp):
        stamp = float(stamp)
        if not math.isfinite(stamp) or stamp < 0:
            return False
        if self.initialized and stamp < self.now:
            self.invalid = True
            self.condition.notify_all()
            return False
        if self.invalid:
            return False
        self.now, self.initialized = stamp, True
        while self._events and self._events[0][0] <= stamp:
            _, _, callback = heapq.heappop(self._events)
            callback()
        self.condition.notify_all()
        return True

    def advance(self, seconds):
        if seconds:
            raise RuntimeError("Only Gazebo clock messages may advance the local API clock")

    def wait_until(self, predicate, deadline):
        wall_deadline = time.monotonic()+max(0., deadline-self.now)
        while not predicate() and self.now < deadline and not self.closed and not self.invalid:
            remaining = wall_deadline-time.monotonic()
            if remaining <= 0:
                break  # A paused simulator must not leave an HTTP worker blocked forever.
            self.condition.wait(remaining)


def jpeg_from_ros_image(message, quality=85, max_pixels=16000000):
    """Encode the actual pixels, preserving raw row stride and channel order."""
    from PIL import Image
    width, height, step = int(message.width), int(message.height), int(message.step)
    formats = {"rgb8": ("RGB", "RGB", 3), "bgr8": ("RGB", "BGR", 3),
               "rgba8": ("RGBA", "RGBA", 4), "bgra8": ("RGBA", "BGRA", 4),
               "mono8": ("L", "L", 1)}
    if message.encoding.lower() not in formats:
        raise ValueError("Unsupported UAV image encoding")
    mode, rawmode, channels = formats[message.encoding.lower()]
    if width <= 0 or height <= 0 or width*height > max_pixels or step < width*channels or not 1 <= quality <= 95:
        raise ValueError("Invalid UAV image dimensions, stride or JPEG quality")
    data = bytes(message.data)
    if len(data) < height*step:
        raise ValueError("Truncated UAV image")
    image = Image.frombytes(mode, (width, height), data, "raw", rawmode, step, 1)
    output = io.BytesIO()
    image.convert("RGB").save(output, format="JPEG", quality=quality)
    return output.getvalue()


def validate_operator_config(config):
    """Operator supplies real command text/objectives; no scenario is invented."""
    if not isinstance(config, dict) or set(config)-{"orders", "targets", "test_mode"}:
        raise ValueError("Operator config supports orders, targets and test_mode only")
    orders, targets = config.get("orders", {}), config.get("targets", {})
    if not isinstance(orders, dict) or not isinstance(targets, dict) or not isinstance(config.get("test_mode", False), bool):
        raise ValueError("Invalid operator configuration shape")
    for slot, command in orders.items():
        if schema.parse_slot(slot) is None or slot not in ("B-1", "B-2", "B-3", "R-1", "R-2", "R-3") or not isinstance(command, str) or not command.strip():
            raise ValueError("Each order needs a canonical slot and explicit nonempty command")
    for camp, objective in targets.items():
        if (camp not in ("BLUE", "RED") or not isinstance(objective, dict)
                or set(objective) != {"objective", "objective_label"}
                or schema.parse_objective(objective.get("objective")) is None
                or schema.objective_camp(objective.get("objective")) != schema.opponent_camp(camp)
                or not isinstance(objective.get("objective_label"), str) or not objective["objective_label"].strip()):
            raise ValueError("Each target needs an explicit opposing-camp objective and label")
    return {"orders": dict(orders), "targets": {k: dict(v) for k, v in targets.items()},
            "test_mode": config.get("test_mode", False)}


class GazeboApiAdapter:
    """Thread-safe adapter. Last-good images keep their original capture times."""
    def __init__(self, teams, *, operator_config=None, epoch=None, max_input_age_s=2.,
                 future_buffer_s=.25, max_frame_bytes=8*1024*1024, diagnostic_limit=128):
        if not teams or any(schema.parse_slot(slot) is None for slot in teams.values()):
            raise ValueError("An explicit local team/slot is required")
        for code in teams:
            register_secret(code)
        self.config = validate_operator_config({} if operator_config is None else operator_config)
        if not math.isfinite(max_input_age_s) or max_input_age_s <= 0 or not math.isfinite(future_buffer_s) or future_buffer_s < 0:
            raise ValueError("Frame age limits must be finite and nonnegative")
        if epoch is not None and (not isinstance(epoch, dt.datetime) or epoch.tzinfo is None):
            raise ValueError("Epoch must include a timezone")
        self._condition = threading.Condition(threading.RLock())
        self.clock = SimulationClock(self._condition)
        self.mock = MockServer(teams, clock=self.clock)
        self._epoch = epoch
        self.max_input_age_s, self.future_buffer_s, self.max_frame_bytes = max_input_age_s, future_buffer_s, int(max_frame_bytes)
        self.diagnostic_limit = max(1, int(diagnostic_limit))
        self._phase, self._start, self._pending = None, None, {}
        self._controls = deque()
        self.stats = Counter()
        self.mock.test_mode = self.config["test_mode"]

    def update_time(self, stamp):
        with self._condition:
            first = not self.clock.initialized
            accepted = self.clock.update(stamp)
            if not accepted:
                self.stats["clock_rejected"] += 1
                if self.clock.invalid:
                    self.mock.reset()
                    self.mock._frames.clear()
                    self.mock._camp_frames.clear()
                    self._pending.clear()
                return False
            if first:
                epoch = self._epoch or dt.datetime.now(dt.timezone.utc)-dt.timedelta(seconds=self.clock())
                self.mock.epoch = epoch.astimezone(dt.timezone(dt.timedelta(hours=9)))
            for camera, (jpeg, captured_at) in list(self._pending.items()):
                if captured_at <= self.clock():
                    del self._pending[camera]
                    self._capture_validated(jpeg, captured_at, camera)
            return True

    def update_state(self, state):
        with self._condition:
            if not self.clock.initialized or self.clock.invalid:
                return False
            if not isinstance(state, dict) or state.get("phase") not in schema.PHASE_LABELS:
                return False
            phase, destroyed = state["phase"], state.get("destroyed")
            if not isinstance(destroyed, dict) or any(type(destroyed.get(c)) is not int or not 0 <= destroyed[c] <= 3 for c in ("BLUE", "RED")):
                return False
            elapsed = state.get("elapsed")
            if phase == "running" and (type(elapsed) not in (int, float) or not math.isfinite(elapsed) or elapsed < 0 or elapsed > self.clock()):
                return False
            if phase == "ready" and self._phase not in (None, "idle", "waiting", "ready"):
                self.mock.reset()
                self.mock._frames.clear()
                self.mock._camp_frames.clear()
                self._pending.clear()
                self._start = None
            if phase != self._phase:
                self.mock.set_phase(phase)
            self._phase = phase
            self.mock.destroyed.update({camp: destroyed[camp] for camp in ("BLUE", "RED")})
            if phase == "running":
                if self._start is None:
                    self._start = self.clock()-elapsed
                    self.mock.matches["report"].start_at = self._start
                    for slot, command in self.config["orders"].items():
                        self.mock.publish_order(slot, command)
                        self.mock.matches["report"].orders[slot]["issued_at"] = self.mock._kst(self._start)
                if elapsed >= 240:
                    for camp, objective in self.config["targets"].items():
                        if camp not in self.mock.matches["report"].targets:
                            self.mock.publish_target(camp, objective["objective"], objective["objective_label"])
                            self.mock.matches["report"].targets[camp]["issued_at"] = self.mock._kst(self._start+240)
            self._condition.notify_all()
            return True

    def _valid_jpeg(self, jpeg):
        from PIL import Image
        if not isinstance(jpeg, bytes) or not jpeg or len(jpeg) > self.max_frame_bytes:
            return False
        try:
            with Image.open(io.BytesIO(jpeg)) as image:
                if image.format != "JPEG" or image.width*image.height > 16000000:
                    return False
                image.load()
            return True
        except (ValueError, OSError):
            return False

    def capture_jpeg(self, jpeg, captured_at, camera="main"):
        """Validate pixels and briefly buffer at most one clock-leading frame/camera."""
        if not self._valid_jpeg(jpeg):
            with self._condition:
                self.stats["invalid_jpeg"] += 1
            return False
        with self._condition:
            if (camera not in self.mock.camera_enabled or type(captured_at) not in (int, float)
                    or not math.isfinite(captured_at) or captured_at < 0
                    or not self.clock.initialized or self.clock.invalid):
                self.stats["invalid_stamp"] += 1
                return False
            if captured_at > self.clock():
                if captured_at-self.clock() > self.future_buffer_s:
                    self.stats["future_rejected"] += 1
                    return False
                old = self._pending.get(camera)
                if old is None or captured_at > old[1]:
                    self._pending[camera] = (jpeg, captured_at)
                self.stats["future_deferred"] += 1
                return False  # It is not an accepted/publishable capture yet.
            return self._capture_validated(jpeg, captured_at, camera)

    def _capture_validated(self, jpeg, captured_at, camera):
        if self.clock()-captured_at > self.max_input_age_s:
            self.stats["stale_rejected"] += 1
            return False
        accepted = self.mock.capture(jpeg, camera, captured_at=captured_at)
        self.stats["captured" if accepted else "cadence_or_duplicate"] += 1
        return accepted

    def request(self, method, path, *, body=None, headers=None, timeout=125.):
        with self._condition:
            result = self.mock.request(method, path, body=body, headers=headers, timeout=timeout)
            if (method.upper() == "POST" and urlsplit(path).path == "/api/report/situation"
                    and result[0] == 200 and isinstance(body, dict)):
                team = self.mock.team(body.get("mission_code", ""))
                control = {"cmd": "m6_received", "slot": team.slot}
                if control not in self._controls:
                    self._controls.append(control)
            del self.mock.requests[:-self.diagnostic_limit]
            return result

    def drain_controls(self):
        with self._condition:
            controls = list(self._controls)
            self._controls.clear()
            return controls

    def invalid_json(self):
        with self._condition:
            return self.mock._invalid(("body",), "Invalid JSON")

    def close(self):
        with self._condition:
            self.clock.closed = True
            self._condition.notify_all()


def make_http_server(adapter, bind="127.0.0.1", port=8000, *, body_limit=65536):
    """Create only a loopback listener, without starting a serving thread."""
    try:
        address = ipaddress.ip_address(bind)
    except ValueError:
        raise ValueError("Bind must be a literal loopback IP address") from None
    if not address.is_loopback:
        raise ValueError("The arena mock API only binds to loopback")

    class Handler(BaseHTTPRequestHandler):
        server_version, sys_version, protocol_version = "MaiconLocalMock", "", "HTTP/1.1"

        def log_message(self, format, *args):
            pass  # Base handler request lines contain GET credentials.

        def send_error(self, code, message=None, explain=None):
            payload = b'{"detail":"Malformed local HTTP request"}'
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)
            self.close_connection = True

        def _handle(self):
            body = None
            try:
                if self.command == "POST":
                    length = int(self.headers.get("Content-Length", "0"))
                    if length < 0 or length > body_limit:
                        self.send_error(413)
                        return
                    self.connection.settimeout(5.)
                    raw = self.rfile.read(length)
                    try:
                        body = json.loads(raw.decode("utf-8"))
                    except (ValueError, UnicodeError):
                        self._write_result(adapter.invalid_json())
                        return
                query = parse_qs(urlsplit(self.path).query)
                try:
                    wait = max(0, min(120, int(query.get("wait", ["0"])[-1])))
                except ValueError:
                    wait = 0  # MockServer owns the official structured422 for bad wait.
            except (ValueError, UnicodeError, OSError):
                self.send_error(400)
                return
            try:
                self._write_result(adapter.request(self.command, self.path, body=body,
                    headers=dict(self.headers.items()), timeout=wait+5.))
            except (BrokenPipeError, ConnectionResetError, TimeoutError):
                self.close_connection = True

        def _write_result(self, result):
            status, payload, headers = result
            self.send_response_only(status)
            for key, content in headers.items():
                self.send_header(key, content)
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)

        do_GET = _handle
        do_POST = _handle

    class Server(ThreadingHTTPServer):
        daemon_threads = True
        address_family = socket.AF_INET6 if address.version == 6 else socket.AF_INET

        def handle_error(self, request, client_address):
            pass  # Never print a traceback with credential-bearing HTTP locals.

    return Server((bind, int(port)), Handler)
