# -*- coding: utf-8 -*-
"""키보드 텔레옵의 순수 로직. rclpy 와 터미널 없이 시험된다(tests/test_all.py).

로봇 API 값은 maicon2026/maicon26/hal/mil_api.py 와 같은 근거(2차 사전교육 자료)를 쓴다.
  /cmd_vel   linear.x ±0.6 m/s, angular.z ±3 rad/s(+ 좌회전), 약 10 Hz 반복, 0.5 s 끊기면 로봇이 자동 정지
  /mil/gimbal pan·tilt 위치값 −100~100 (pan + 오른쪽 1단위 0.5°, tilt + 위 1단위 0.15°)
  제어 허용  mode == 'practice' or cmd in ('start', 'resume')
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Callable, List, Optional, Tuple

V_CAP, W_CAP = 0.6, 3.0                      # API 한도. 이 값보다 크게는 못 보낸다
LEVELS = (0.2, 0.4, 0.6, 0.8, 1.0)           # 속도 단계 1~5 = 최대의 20~100 %
GIMBAL_MAX = 100
ARROWS = ("UP", "DOWN", "LEFT", "RIGHT")
OTHER_PUBLISHER, MISSION_MODE = "other_publisher", "mission_mode"

HELP = [
    "방향키  누르고 있는 동안만 이동 (↑ 전진  ↓ 후진  ← 좌회전  → 우회전)",
    "Space   즉시 정지        1~5  속도 단계 (1 = 0.12 m/s)",
    "W/S     틸트 위/아래     A/D  팬 왼쪽/오른쪽 (대문자 = 큰 걸음)     C  팬틸트 가운데",
    "M       구간 마커        F F  발사 (--allow-fire 일 때, 1초 안에 두 번)",
    "H       도움말           Q    종료 (정지 명령을 보내고 끝낸다)",
]


def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


# ============================================================================ 키 해석
class KeyParser:
    """터미널 바이트 → 키 이름. 방향키는 ESC [ A~D 와 응용 커서 모드 ESC O A~D 를 모두 받는다."""
    _ARROW = {ord("A"): "UP", ord("B"): "DOWN", ord("C"): "RIGHT", ord("D"): "LEFT"}

    def __init__(self) -> None:
        self.buf = b""

    def feed(self, data: bytes) -> List[str]:
        self.buf += data
        out: List[str] = []
        while self.buf:
            b = self.buf
            if b[0] == 0x1B:
                if len(b) == 1:
                    break                                    # 뒤가 더 올 수 있다. 안 오면 flush() 가 ESC 키로 본다
                if b[1] in (ord("["), ord("O")):
                    if len(b) < 3:
                        break
                    if b[2] in self._ARROW:
                        out.append(self._ARROW[b[2]])
                        self.buf = b[3:]
                        continue
                    i = 2                                    # 다른 제어열(F키, Home 등)은 끝 바이트까지 버린다
                    while i < len(b) and not 0x40 <= b[i] <= 0x7E:
                        i += 1
                    if i >= len(b):
                        break
                    self.buf = b[i + 1:]
                    continue
                out.append("ESC")
                self.buf = b[1:]
                continue
            c = b[0]
            self.buf = b[1:]
            if c == 0x03:
                out.append("CTRL_C")
            elif c == 0x20:
                out.append("SPACE")
            elif 0x21 <= c <= 0x7E:
                out.append(chr(c))
        return out

    def flush(self) -> List[str]:
        """입력이 잠깐 끊겼을 때 부른다. 홀로 남은 ESC 는 ESC 키이고, 끊긴 제어열은 버린다."""
        b, self.buf = self.buf, b""
        return ["ESC"] if b == b"\x1b" else []


# ============================================================================ 누르고 있는지 판정
class Hold:
    """터미널은 키를 뗀 순간을 알려 주지 않는다. 키 반복(autorepeat)이 이어지는 동안만 누른 것으로 본다.

    첫 입력 뒤 first_hold_s 안에 반복이 오면 계속 누른 것이고, 그 뒤로는 release_s 동안 입력이 없으면 뗀 것이다.
    """

    def __init__(self, first_hold_s: float = 0.6, release_s: float = 0.2) -> None:
        self.first_hold_s, self.release_s = first_hold_s, release_s
        self.key: Optional[str] = None
        self.repeats = 0
        self.t_last = -math.inf

    def press(self, key: str, now: float) -> None:
        if key != self.key:
            self.key, self.repeats = key, 0
        else:
            self.repeats += 1
        self.t_last = now

    def active(self, now: float) -> Optional[str]:
        if self.key is None:
            return None
        if now - self.t_last > (self.first_hold_s if self.repeats == 0 else self.release_s):
            self.key = None
        return self.key

    def clear(self) -> None:
        self.key = None


class DoublePress:
    """window_s 안에 두 번 눌러야 참. 발사를 한 번 누름으로 내보내지 않기 위한 것."""

    def __init__(self, window_s: float = 1.0) -> None:
        self.window_s, self.t = window_s, -math.inf

    def press(self, now: float) -> bool:
        ok = now - self.t <= self.window_s
        self.t = -math.inf if ok else now
        return ok


# ============================================================================ 속도
def ramp(cur: float, tgt: float, acc: float, dt: float) -> float:
    """가속 제한. 0 쪽으로 줄이는 것은 두 배 빠르게."""
    slowing = abs(tgt) < abs(cur) or cur * tgt < 0
    step = acc * (2.0 if slowing else 1.0) * dt
    return tgt if abs(tgt - cur) <= step else cur + math.copysign(step, tgt - cur)


def target_velocity(key: Optional[str], level: int, max_lin: float, max_ang: float) -> Tuple[float, float]:
    k = LEVELS[int(clamp(level, 1, len(LEVELS))) - 1]
    v, w = min(max_lin, V_CAP) * k, min(max_ang, W_CAP) * k
    return {"UP": (v, 0.0), "DOWN": (-v, 0.0), "LEFT": (0.0, w), "RIGHT": (0.0, -w)}.get(key or "", (0.0, 0.0))


def drive_allowed(mode: str, cmd: str, others: int, state_guard: bool = True) -> Tuple[bool, str]:
    """주행·팬틸트·발사를 보내도 되는가. (허용, 이유)."""
    mode, cmd = (mode or "").strip().lower(), (cmd or "").strip().lower()
    if others > 0:
        return False, f"다른 /cmd_vel 송신자 {others}개 (팀 노드가 켜져 있다)"
    if mode == "mission":
        return False, "경기 모드 — 텔레옵 금지"
    if mode == "practice":
        return True, ""
    if not state_guard:
        return True, "모드 확인 없이 진행(--no-state-guard)"
    return False, "모드를 아직 못 받았다(/mil/match 또는 /mil/state)"


# ============================================================================ 텔레옵 본체
@dataclass
class Options:
    max_lin: float = V_CAP
    max_ang: float = W_CAP
    level: int = 1
    first_hold_s: float = 0.6
    release_s: float = 0.2
    acc_lin: float = 1.0
    acc_ang: float = 6.0
    cmd_hz: float = 10.0
    state_guard: bool = True
    allow_fire: bool = False
    idle_exit_s: float = 120.0
    other_exit_s: float = 3.0
    gimbal_step: int = 5
    gimbal_big_step: int = 20
    gimbal_speed: float = 150.0
    gimbal_acc: float = 20.0
    gimbal_timeout_s: float = 3.0


class Teleop:
    """키 입력과 시계만 받아 명령을 정한다. ros 는 teleop_keys.RosIO 이거나 시험용 가짜다.

    ros 가 줘야 하는 것: mode, cmd(문자열), other_cmd_publishers(), publish_cmd(v, w), event(dict),
    call_gimbal(pan, tilt, speed, acc, cb) -> bool, call_fire(cb) -> bool, discover()
    """

    def __init__(self, ros, opt: Options, now: float, say: Callable[[str], None] = print) -> None:
        self.ros, self.o, self.say = ros, opt, say
        self.parser = KeyParser()
        self.hold = Hold(opt.first_hold_s, opt.release_s)
        self.fire_arm = DoublePress(1.0)
        self.level = int(clamp(opt.level, 1, len(LEVELS)))
        self.v = self.w = 0.0
        self.pan = self.tilt = 0
        self.pt_dirty = False
        self.gimbal_busy: Optional[float] = None
        self.gimbal_resp = self.fire_resp = "-"
        self.markers = 0
        self.t_key = self.t_bytes = now
        self.t_cmd = self.t_slow = -math.inf
        self.other_since: Optional[float] = None
        self.others = 0
        self.reason = ""
        self.quit: Optional[str] = None
        self.quit_kind = ""
        self._warned: set = set()

    # ------------------------------------------------------------------ 입력
    def on_bytes(self, data: bytes, now: float) -> None:
        self.t_bytes = now
        for k in self.parser.feed(data):
            self.on_key(k, now)

    def on_key(self, k: str, now: float) -> None:
        self.t_key = now
        if k in ARROWS:
            self.hold.press(k, now)
            return
        low = k.lower() if len(k) == 1 else k
        if k == "SPACE":
            self.stop_now("space")
        elif k in ("1", "2", "3", "4", "5"):
            self.level = int(k)
            self.ros.event({"type": "level", "level": self.level})
        elif low in ("w", "s", "a", "d"):
            step = self.o.gimbal_big_step if k.isupper() else self.o.gimbal_step
            if low == "w":
                self.tilt += step
            elif low == "s":
                self.tilt -= step
            elif low == "d":
                self.pan += step                              # API: pan + = 오른쪽
            else:
                self.pan -= step
            self.pan = int(clamp(self.pan, -GIMBAL_MAX, GIMBAL_MAX))
            self.tilt = int(clamp(self.tilt, -GIMBAL_MAX, GIMBAL_MAX))
            self.pt_dirty = True
        elif low == "c":
            self.pan = self.tilt = 0
            self.pt_dirty = True
        elif low == "m":
            self.markers += 1
            self.ros.event({"type": "marker", "n": self.markers})
            self.say(f"마커 {self.markers}")
        elif low == "f":
            self._fire(now)
        elif low == "h":
            for line in HELP:
                self.say(line)
        elif low == "q" or k in ("ESC", "CTRL_C"):
            self.request_quit("사용자 종료")

    def stop_now(self, why: str) -> None:
        self.hold.clear()
        self.v = self.w = 0.0
        if self.others == 0:
            self.ros.publish_cmd(0.0, 0.0)
        self.ros.event({"type": "stop", "why": why})

    def request_quit(self, why: str, kind: str = "") -> None:
        if self.quit is None:
            self.quit, self.quit_kind = why, kind

    def send_zero_on_exit(self) -> bool:
        """끝낼 때 0 을 보내도 되는가. 다른 송신자가 있거나 경기 모드면 아무것도 보내지 않는다."""
        return self.quit_kind not in (OTHER_PUBLISHER, MISSION_MODE)

    # ------------------------------------------------------------------ 주기 처리
    def tick(self, now: float) -> None:
        if self.parser.buf and now - self.t_bytes > 0.05:
            for k in self.parser.flush():
                self.on_key(k, now)
        if now - self.t_slow >= 1.0:
            self.t_slow = now
            self._slow(now)
        self.others = self.ros.other_cmd_publishers()
        ok, self.reason = drive_allowed(self.ros.mode, self.ros.cmd, self.others, self.o.state_guard)
        self._watch_others(now)
        if (self.ros.mode or "").strip().lower() == "mission":
            self.request_quit("경기 모드에서는 텔레옵을 쓰지 않는다", MISSION_MODE)
            return
        period = 1.0 / self.o.cmd_hz
        if now - self.t_cmd >= period - 1e-6:
            dt = period if not math.isfinite(self.t_cmd) else min(now - self.t_cmd, 0.5)
            self.t_cmd = now
            if self.others > 0:
                self.v = self.w = 0.0                         # 다른 송신자가 있으면 0 도 보내지 않는다
            elif not ok:
                self.v = self.w = 0.0
                self.ros.publish_cmd(0.0, 0.0)
            else:
                tv, tw = target_velocity(self.hold.active(now), self.level, self.o.max_lin, self.o.max_ang)
                self.v = clamp(ramp(self.v, tv, self.o.acc_lin, dt), -V_CAP, V_CAP)
                self.w = clamp(ramp(self.w, tw, self.o.acc_ang, dt), -W_CAP, W_CAP)
                self.ros.publish_cmd(self.v, self.w)
        self._pump_gimbal(now, ok)

    def _watch_others(self, now: float) -> None:
        if self.others > 0:
            if self.other_since is None:
                self.other_since = now
                self.hold.clear()
                self.ros.event({"type": "guard", "why": "other_cmd_vel_publisher", "count": self.others})
                self.say(f"경고: /cmd_vel 에 다른 송신자 {self.others}개. 송신을 멈춘다")
            elif now - self.other_since >= self.o.other_exit_s:
                self.request_quit("/cmd_vel 송신자가 따로 있다. 팀 노드를 끄고 다시 실행한다", OTHER_PUBLISHER)
        else:
            self.other_since = None

    def _slow(self, now: float) -> None:
        self.ros.discover()
        if now - self.t_key >= self.o.idle_exit_s:
            self.request_quit(f"{self.o.idle_exit_s:.0f}초 동안 키 입력이 없다")

    def _warn_once(self, key: str, text: str) -> None:
        if key not in self._warned:
            self._warned.add(key)
            self.say(text)

    # ------------------------------------------------------------------ 팬틸트·발사
    def _pump_gimbal(self, now: float, ok: bool) -> None:
        if self.gimbal_busy is not None and now - self.gimbal_busy > self.o.gimbal_timeout_s:
            self.gimbal_busy, self.gimbal_resp = None, "시간 초과"
        if not self.pt_dirty or self.gimbal_busy is not None or not ok:
            return
        pan, tilt = self.pan, self.tilt

        def done(resp: Optional[str]) -> None:
            self.gimbal_busy = None
            self.gimbal_resp = resp or "응답 없음"
            self.ros.event({"type": "gimbal", "pan": pan, "tilt": tilt, "resp": self.gimbal_resp})

        self.pt_dirty = False
        self.gimbal_busy = now
        if not self.ros.call_gimbal(pan, tilt, self.o.gimbal_speed, self.o.gimbal_acc, done):
            self.gimbal_busy, self.gimbal_resp = None, "서비스 없음"
            self._warn_once("gimbal", "경고: /mil/gimbal 서비스를 찾지 못했다. 팬틸트 키는 동작하지 않는다")

    def _fire(self, now: float) -> None:
        if not self.o.allow_fire:
            self._warn_once("fire_off", "발사는 꺼져 있다. 켜려면 --allow-fire (사람 눈 방향 발사 금지)")
            return
        if not self.fire_arm.press(now):
            self.say("발사 준비: 1초 안에 F 를 한 번 더")
            return
        ok, why = drive_allowed(self.ros.mode, self.ros.cmd, self.others, self.o.state_guard)
        if not ok:
            self.say("발사 안 함: " + why)
            return

        def done(resp: Optional[str], left_ms: Optional[float]) -> None:
            self.fire_resp = (resp or "응답 없음") + (f" {left_ms:.0f}ms" if isinstance(left_ms, (int, float)) else "")
            self.ros.event({"type": "fire", "resp": resp, "left_ms": left_ms})
            self.say("발사 응답: " + self.fire_resp)

        if not self.ros.call_fire(done):
            self.fire_resp = "서비스 없음"
            self._warn_once("fire", "경고: /mil/fire 서비스를 찾지 못했다")

    # ------------------------------------------------------------------ 화면
    def status(self, rec: str = "") -> str:
        mode = (self.ros.mode or "?") + "/" + (self.ros.cmd or "?")
        odom = getattr(self.ros, "odom_v", None)
        batt = getattr(self.ros, "battery", None)
        parts = [rec, f"L{self.level}", f"v{self.v:+.2f} w{self.w:+.2f}",
                 f"odom{odom:+.2f}" if isinstance(odom, float) else "", f"pan{self.pan:+d} tilt{self.tilt:+d}({self.gimbal_resp})",
                 f"fire:{self.fire_resp}" if self.o.allow_fire else "", f"M{self.markers}", mode,
                 f"batt{batt:.0f}%" if isinstance(batt, (int, float)) else "", ("! " + self.reason) if self.reason else ""]
        return "  ".join(p for p in parts if p)


def now_wall() -> float:
    return time.time()
