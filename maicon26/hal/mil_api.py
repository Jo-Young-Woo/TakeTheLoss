"""WINGBOT-UGV ROS2 API(2차 사전교육 자료 p9~18)의 순수 로직 — rclpy 없이 시험된다.

PDF 에서 확인된 규칙만 여기에 있다(근거 쪽수 표시). PDF 에 없는 값은 config.HalCfg 에 [가정]으로 둔다.

  /mil/gimbal   pan·tilt 는 각도가 아니라 '위치값'(−100~100). 1단위 = 팬 0.5° / 틸트 0.15°, pan + = 오른쪽, tilt + = 위쪽 [p14].
                ok 는 '접수'이지 '도착'이 아니다. 위치 피드백 토픽이 없다 → GimbalModel 로 현재 위치를 추정한다.
  /cmd_vel      linear.x ±0.6 m/s, angular.z ±3 rad/s(+ 좌회전), 약 10 Hz 반복, 0.5 s 끊기면 자동 정지 [p15]
  /mil/fire     1회 5 s 발광, 발사 시작부터 10 s 뒤 재발사. 응답 fired / empty / locked / reloading + left_ms [p16]
  /mil/state    hp, hp_max, battery_pct (2 Hz)  /  /mil/match  mode(practice|mission), cmd(idle…stop) [p10~12]
  /mil/oled     4줄(0~3) × 21자(영문·숫자·기호), 전송 간격 ≥ 0.5 s, 마지막 출력 10 s 뒤 기본 화면 복귀 [p13]

프로젝트 내부 부호 약속(좌회전 +, 팬 왼쪽 +)과 API(팬 오른쪽 +)가 다르다 — 변환은 이 모듈의 함수 한 곳에서만 한다.
"""
from __future__ import annotations

import math
from collections import deque
from typing import Deque, List, Optional, Tuple

from ..util import clamp

# ---- 경기 단계 [p11]
CMD_STATES = ("idle", "standby", "start", "pause", "resume", "report", "stop")
CONTROL_CMDS = ("start", "resume")

FIRE_FIRED, FIRE_EMPTY, FIRE_LOCKED, FIRE_RELOADING = "fired", "empty", "locked", "reloading"


def control_allowed(mode: str, cmd: str) -> Optional[bool]:
    """PDF p11: control = mode == 'practice' or cmd in ('start', 'resume'). 아직 못 받았으면 None(모름)."""
    mode, cmd = (mode or "").strip().lower(), (cmd or "").strip().lower()
    if not mode and not cmd:
        return None
    return mode == "practice" or cmd in CONTROL_CMDS


# ============================================================================ 단위 변환
def pan_deg_to_unit(pan_deg: float, deg_per_unit: float = 0.5, sign: float = -1.0, unit_max: float = 100.0) -> int:
    """내부 팬각(deg, 왼쪽 +) → API 위치값(오른쪽 +, 정수). sign=−1: 부호 반전. 한계는 ±unit_max."""
    return int(round(clamp(sign * pan_deg / deg_per_unit, -unit_max, unit_max)))


def pan_unit_to_deg(unit: float, deg_per_unit: float = 0.5, sign: float = -1.0) -> float:
    return sign * unit * deg_per_unit


def tilt_deg_to_unit(tilt_deg: float, deg_per_unit: float = 0.15, sign: float = 1.0, unit_max: float = 100.0) -> int:
    return int(round(clamp(sign * tilt_deg / deg_per_unit, -unit_max, unit_max)))


def tilt_unit_to_deg(unit: float, deg_per_unit: float = 0.15, sign: float = 1.0) -> float:
    return sign * unit * deg_per_unit


def quantize_pan_tilt(pan_deg: float, tilt_deg: float, h) -> Tuple[int, int]:
    """내부 (pan°, tilt°) → API (pan 단위, tilt 단위). h = config.HalCfg."""
    return (pan_deg_to_unit(pan_deg, h.pan_deg_per_unit, h.pan_sign, h.gimbal_unit_max),
            tilt_deg_to_unit(tilt_deg, h.tilt_deg_per_unit, h.tilt_sign, h.gimbal_unit_max))


def limit_cmd_vel(v: float, w: float, v_cap: float = 0.6, w_cap: float = 3.0) -> Tuple[float, float]:
    """PDF p15 의 속도 한계. NaN/inf 는 0 으로(잘못된 값이 로봇에 나가지 않게)."""
    if not (math.isfinite(v) and math.isfinite(w)):
        return 0.0, 0.0
    return clamp(v, -v_cap, v_cap), clamp(w, -w_cap, w_cap)


# ============================================================================ 팬틸트 위치 추정
class GimbalModel:
    """위치 피드백이 없는 서보의 현재 각도를 추정한다.

    가정: 명령이 *접수된*(ok) 시각 + latency 부터 축마다 speed(°/s)로 목표를 향해 움직인다. ST3215 의 실제 가감속은 모른다 →
    latency·speed 는 실측(field F-GIM)으로 맞춘다. 사진 촬영 시각의 팬/틸트가 필요하므로 과거 위치(at)도 보간해 준다.
    """

    def __init__(self, speed_dps: float, latency_s: float, hist_s: float = 3.0) -> None:
        self.speed, self.latency, self.hist_s = max(speed_dps, 1e-3), max(latency_s, 0.0), hist_s
        self._t = 0.0
        self._pos = (0.0, 0.0)
        self._tgt = (0.0, 0.0)
        self._t_go = 0.0
        self._hist: Deque[Tuple[float, float, float]] = deque()

    @staticmethod
    def _toward(p: float, g: float, step: float) -> float:
        return g if abs(g - p) <= step else p + math.copysign(step, g - p)

    def _advance(self, t: float) -> None:
        if t <= self._t:
            return
        t0 = max(self._t, self._t_go)
        if t > t0:
            step = self.speed * (t - t0)
            self._pos = (self._toward(self._pos[0], self._tgt[0], step), self._toward(self._pos[1], self._tgt[1], step))
        self._t = t
        self._hist.append((t, self._pos[0], self._pos[1]))
        while self._hist and self._hist[0][0] < t - self.hist_s:
            self._hist.popleft()

    def command(self, t: float, pan_deg: float, tilt_deg: float) -> None:
        """접수된 명령(내부 부호, deg)을 t 에 반영."""
        self._advance(t)
        self._tgt = (pan_deg, tilt_deg)
        self._t_go = t + self.latency

    def hold(self, t: float) -> None:
        """Pause stops the hardware servo; freeze its estimate at that instant."""
        self._advance(t)
        self._tgt = self._pos
        self._t_go = t

    def position(self, t: float) -> Tuple[float, float]:
        self._advance(t)
        return self._pos

    def at(self, t: float, now: float) -> Tuple[float, float]:
        """과거 시각 t(≤ now)의 위치. 이력 밖이면 가장 가까운 끝 값."""
        self._advance(now)
        h = self._hist
        if not h or t >= h[-1][0]:
            return self._pos
        if t <= h[0][0]:
            return (h[0][1], h[0][2])
        for i in range(len(h) - 1, 0, -1):
            if h[i - 1][0] <= t <= h[i][0]:
                a, b = h[i - 1], h[i]
                k = (t - a[0]) / max(b[0] - a[0], 1e-9)
                return (a[1] + (b[1] - a[1]) * k, a[2] + (b[2] - a[2]) * k)
        return self._pos

    @property
    def target(self) -> Tuple[float, float]:
        return self._tgt


# ============================================================================ 레이저 상태 추정
class FireModel:
    """/mil/fire 응답과 시간 규칙(5 s 발광, 시작 후 10 s 재발사)으로 탄약·발광·재장전을 추정한다.
    API 에 탄약 토픽이 없다 — 시작은 가득(config.laser.ammo) 가정, `empty` 응답이 오면 0 으로 고친다."""

    def __init__(self, ammo: int, shot_s: float, period_s: float) -> None:
        self.ammo, self.shot_s, self.period_s = int(ammo), shot_s, period_s
        self.t_fire = -1e9
        self.laser_end = -1e9
        self.ready_at = -1e9
        self.fired = 0
        self.rejected = {FIRE_EMPTY: 0, FIRE_LOCKED: 0, FIRE_RELOADING: 0, "none": 0}
        self.last_msg = ""

    def on_response(self, t: float, message: Optional[str], left_ms: Optional[float] = None) -> str:
        msg = (message or "").strip().lower()
        self.last_msg = msg or "none"
        if msg == FIRE_FIRED:
            self.ammo = max(self.ammo - 1, 0)
            self.fired += 1
            self.t_fire, self.laser_end = t, t + self.shot_s
            self.ready_at = t + (left_ms / 1000.0 if left_ms else self.period_s)
            return FIRE_FIRED
        if msg == FIRE_EMPTY:
            self.ammo = 0
        elif msg == FIRE_RELOADING and left_ms is not None:
            self.ready_at = max(self.ready_at, t + left_ms / 1000.0)
        self.rejected[msg if msg in self.rejected else "none"] += 1
        return msg or "none"

    def cancel_emission(self, t: float) -> None:
        """일시정지 등으로 레이저가 꺼졌다(PDF p12: 일시정지 시 레이저 꺼짐)."""
        self.laser_end = min(self.laser_end, t)

    def laser_on(self, t: float) -> bool:
        return t < self.laser_end

    def cooldown_left(self, t: float) -> float:
        return max(self.ready_at - t, 0.0)


# ============================================================================ 피격 추정
class HitModel:
    """/mil/state 의 hp 감소로 피격을 센다. 피격 후 lock_s 동안 조준·발사 제한(PDF p16: 10 s).
    누적 피격이 hp_max 에 이르면 주행 정지도 건다 — 이 규칙은 2차 자료(p11~16)에 없고 기존 코드(4회 누적 → 주행 정지)를 hp_max 로 옮긴 [가정]이다."""

    def __init__(self, lock_s: float, drive_pause_s: float) -> None:
        self.lock_s, self.drive_pause_s = lock_s, drive_pause_s
        self.hp: Optional[int] = None
        self.hp_max: Optional[int] = None
        self.hits = 0
        self.disabled_until = -1e9
        self.drive_until = -1e9

    def on_state(self, t: float, hp: Optional[int], hp_max: Optional[int]) -> int:
        """새 피격 수를 돌려준다."""
        if hp_max is not None and hp_max > 0:
            self.hp_max = int(hp_max)
        if hp is None:
            return 0
        hp = int(hp)
        new = 0
        if self.hp is None:                              # 팀 노드가 경기 도중 (재)시작한 경우: 이미 입은 피격은 개수만 반영, 잠금 시각은 모른다
            if self.hp_max is not None:
                self.hits = max(self.hp_max - hp, 0)
        elif hp < self.hp:
            new = self.hp - hp
            self.hits += new
            self.disabled_until = t + self.lock_s
            if self.hp_max is not None and self.hits >= self.hp_max:
                self.drive_until = t + self.drive_pause_s
        self.hp = hp
        return new

    def note_locked_response(self, t: float, hold_s: float) -> None:
        """발사/팬틸트 서비스가 locked 를 돌려줬다(제어가 허용된 상태에서) — 하드웨어가 직접 알려 준 잠금."""
        self.disabled_until = max(self.disabled_until, t + hold_s)

    def disabled_left(self, t: float) -> float:
        return max(self.disabled_until - t, 0.0)

    def drive_left(self, t: float) -> float:
        return max(self.drive_until - t, 0.0)


# ============================================================================ OLED
class OledScreen:
    """4줄 × 21자. 바뀐 줄만, 0.5 s 이상 간격으로 한 번에 한 줄씩 보낸다. 10 s 뒤 기본 화면으로 돌아가므로
    바뀌지 않아도 refresh_s 마다 다시 보낸다. 영문·숫자·기호만 표시된다 → 그 밖의 문자는 '?'."""

    def __init__(self, cols: int = 21, lines: int = 4, min_interval_s: float = 0.5, refresh_s: float = 8.0) -> None:
        self.cols, self.n, self.min_interval, self.refresh = cols, lines, min_interval_s, refresh_s
        self.want: List[Optional[str]] = [None] * lines
        self.sent: List[Optional[str]] = [None] * lines
        self.sent_t: List[float] = [-1e9] * lines
        self._last = -1e9

    def clean(self, s: str) -> str:
        s = "".join(ch if 32 <= ord(ch) < 127 else "?" for ch in str(s).replace("\t", " "))
        return s[: self.cols]

    def set_text(self, text: str) -> None:
        rows = str(text).split("\n")[: self.n]
        for i in range(self.n):
            self.want[i] = self.clean(rows[i]) if i < len(rows) else None

    def next(self, now: float) -> Optional[Tuple[int, str]]:
        if now - self._last < self.min_interval:
            return None
        pick = None
        for i in range(self.n):                                    # 1순위: 내용이 바뀐 줄
            if self.want[i] is not None and self.want[i] != self.sent[i]:
                pick = i
                break
        if pick is None:                                           # 2순위: 가장 오래 안 보낸 줄(기본 화면 복귀 방지)
            old = [(self.sent_t[i], i) for i in range(self.n) if self.want[i] is not None and now - self.sent_t[i] >= self.refresh]
            if old:
                pick = min(old)[1]
        if pick is None:
            return None
        self.sent[pick], self.sent_t[pick], self._last = self.want[pick], now, now
        return pick, self.want[pick] or ""


# ============================================================================ IMU
def quat_to_rpy_deg(x: float, y: float, z: float, w: float) -> Tuple[float, float, float]:
    """쿼터니언 → (roll, pitch, yaw) deg. yaw 식은 PDF p17 과 같다."""
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    s = clamp(2 * (w * y - z * x), -1.0, 1.0)
    pitch = math.asin(s)
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def accel_g(ax: float, ay: float, az: float) -> float:
    return math.sqrt(ax * ax + ay * ay + az * az) / 9.80665
