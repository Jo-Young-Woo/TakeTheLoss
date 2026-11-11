"""WINGBOT-UGV 실기 어댑터 — 2차 사전교육 자료(PDF p9~18)의 ROS2 API 로 구현. **로봇에서는 아직 실행해 보지 못했다.**

구조
  WingbotRobot  Robot 인터페이스 구현(전송 방식과 무관한 로직). 속도 한계·10 Hz 반복·팬틸트 단위 변환과 위치 추정·레이저/피격 추정·
                OLED 줄 갱신·경기 단계(/mil/match) 잠금 처리가 여기에 있고 rclpy 없이 시험된다(tests/test_mil_hal.py).
  Transport     ROS2 와의 접점(발행·서비스 호출). 실기는 hal/ros_mil.py 의 RosMilTransport, 시험은 FakeTransport.
  RosWingbot    = WingbotRobot + RosMilTransport.

API 에서 코드가 해야 하는 일 (PDF 쪽수)
  - /cmd_vel 은 약 10 Hz 로 반복해서 보낸다, 0.5 s 끊기면 로봇이 멈춘다, 여러 노드가 동시에 보내면 안 된다 [p15]
  - 경기 모드(mission)에서 cmd 가 start/resume 이 아니면 주행은 무시되고 조준·발사는 locked 다. 재개 때 로봇은 스스로 움직이지 않는다 → 명령 재전송 [p11·12]
  - 팬틸트는 서비스(접수만 확인, 도착 아님), 위치 피드백이 없다 → 추정 [p14]
  - 레이저는 서비스, 응답 fired/empty/locked/reloading, 탄약 토픽이 없다 → 추정 [p16]
  - 카메라 /image_raw 는 팬틸트 카메라(깊이 카메라 아님), 10 FPS 이하 권장 [p17]

좌표/부호: 내부 v 전진 +, w 반시계 +, pan 왼쪽 +, tilt 위 +. API 는 pan 오른쪽 + → mil_api.py 에서만 변환한다. 센서 stamp 는 robot.now() 시계.
"""
from __future__ import annotations

import functools
import threading
import time
import math
from typing import Any, Callable, Dict, Optional, Tuple

import numpy as np

from ..config import Config
from ..types import DepthFrame, Frame, Pose2D, RobotStatus, Scan
from .base import Robot
from .clock import StampSync
from .depth_io import camera_info_K, make_depth_frame
from .diffdrive import DiffDrive
from ..validate import (FrameWatch, OdomJumpDetector, ValidationLog, check_aligned_size, check_camera_matrix, finite, is_practice,
                        unknown_match_fields, unknown_response)
from .mil_api import (CMD_STATES, FIRE_FIRED, FIRE_LOCKED, FireModel, GimbalModel, HitModel, OledScreen, control_allowed, limit_cmd_vel,
                      pan_unit_to_deg, quantize_pan_tilt, tilt_unit_to_deg)

LIDAR_ANGLE_OFFSET = 0.0   # ROS 전송층이 센서 TF를 적용한다. 장착 방향은 TF에서 보정하고 여기서 이중 적용하지 않는다.


def _never_raises(source: str):
    """수신 콜백(전송 계층이 부른다)이 예외를 밖으로 던지지 않게 한다(9장). 던졌다면 stats["invalid"][source]["exception"] 으로 센다 —
    경고는 같은 이유 5 s 에 한 줄. 정상 입력의 동작과 반환값은 그대로다."""
    def deco(fn):
        @functools.wraps(fn)
        def wrapped(self, *a, **k):
            try:
                return fn(self, *a, **k)
            except Exception as e:                       # noqa: BLE001 — 콜백 경계
                try:
                    self.vlog.bad(source, "exception", f"{type(e).__name__}: {e}"[:80])
                except Exception:                        # pragma: no cover
                    pass
                return None
        return wrapped
    return deco


class Transport:
    """ROS2 접점. 콜백은 다른 스레드에서 불릴 수 있다."""

    def publish_cmd(self, v: float, w: float) -> None:
        raise NotImplementedError

    def publish_oled(self, line: int, text: str) -> None:
        raise NotImplementedError

    def call_gimbal(self, pan_unit: float, tilt_unit: float, speed: float, acc: float,
                    cb: Callable[[Optional[str]], None]) -> None:
        """비동기 호출. cb(message) — 응답 실패/서비스 없음이면 cb(None)."""
        raise NotImplementedError

    def call_fire(self, cb: Callable[[Optional[str], Optional[float]], None]) -> None:
        """비동기 호출. cb(message, left_ms)."""
        raise NotImplementedError

    def call_oak_power(self, on: bool) -> str:
        """OAK start/stop 서비스(비동기). 돌려주는 값은 Robot.oak_power 와 같다(ok·pending·fail·none). 기본: 지원 안 함."""
        return "none"

    def close(self) -> None:
        pass

    def readiness(self) -> Tuple[bool, list]:
        return True, []


class WingbotRobot(Robot):
    can_abort_laser = False        # 즉시 소등 API 가 없다(일시정지 시에만 꺼진다, p12·16) → 아군 보호각을 넓게 쓴다

    def __init__(self, cfg: Optional[Config] = None, transport: Optional[Transport] = None,
                 clock: Callable[[], float] = time.monotonic) -> None:
        self.cfg = cfg or Config()
        h = self.cfg.hal
        self._clock, self._t0 = clock, clock()
        self._lock = threading.RLock()
        self._dd = DiffDrive(self.cfg.robot)       # 차동 구동 변환(바퀴 속도를 직접 받는 드라이버용)
        self._sync_scan = StampSync()
        self._sync_frame = StampSync(floor_s=self.cfg.camera.min_latency_s)
        self._sync_depth = StampSync(floor_s=self.cfg.depth.min_latency_s)
        self._sync_oak = StampSync(floor_s=self.cfg.camera.min_latency_s)
        # 명령
        self._cmd = (0.0, 0.0, self.now())          # (v, w, 설정 시각)
        self._last_pub = (0.0, 0.0)
        self._closed = False
        # 팬틸트
        self.gimbal = GimbalModel(h.gimbal_speed, h.gimbal_latency_s)
        self._measured_gimbal = None  # (internal pan, tilt, receipt time), simulation opt-in only
        self._want: Tuple[int, int] = (0, 0)
        self._g_has_goal = False
        self._sent: Optional[Tuple[int, int]] = None
        self._g_pending_t: Optional[float] = None
        self._g_pending_big = False                      # 지금 응답을 기다리는 명령이 settle_move_deg 보다 크게 움직이는 명령인가(4장)
        self._g_move_t = -1e9                            # settle_move_deg 보다 크게 움직인 마지막 짐벌 명령이 접수(ok)된 시각(4장)
        self._g_next_try = -1e9
        self._g_last_send = -1e9
        self._g_request = 0
        self._control_epoch = 0
        self._fire_request = 0
        self._fire_pending: Optional[int] = None
        self._fire_retry_at = -1e9
        # 레이저·피격
        self.fire = FireModel(self.cfg.laser.ammo, self.cfg.laser.shot_s, self.cfg.laser.shot_s + self.cfg.laser.cooldown_s,
                              self.cfg.laser.practice_empty_backoff_s, left_ms_max=h.fire_left_ms_max)
        self.hit = HitModel(self.cfg.laser.disabled_s, h.drive_pause_s)
        # 경기 단계·상태
        self._mode, self._cmd_state = "", ""
        self._degraded: frozenset = frozenset()          # 저하 플래그(preflight.ALL_FLAGS) — set_degraded 로만 바뀐다
        self._control: Optional[bool] = None
        self._battery: Optional[float] = None
        self._hp: Optional[int] = None
        self._hp_max: Optional[int] = None
        self._imu: Tuple[Optional[float], Optional[float], Optional[float]] = (None, None, None)
        # 충돌 감지(IMU 주기). 기본 꺼짐 — 켜면 on_imu 가 가속도·자이로로 이벤트를 만들고 Agent 가 pop_impacts() 로 가져간다.
        self._impact_det = None
        self._impact_n = 0
        if getattr(self.cfg, "impact", None) is not None and self.cfg.impact.enable:
            from ..nav.pose_guard import ImpactDetector                        # 지연 import: HAL 이 nav 를 쓰지 않는 기본 경로에는 영향 없음
            ic = self.cfg.impact
            self._impact_det = ImpactDetector(trigger_dg=ic.trigger_dg, major_dg=ic.major_dg, gyro_dps=ic.gyro_dps,
                                              major_gyro_dps=ic.major_gyro_dps, refractory_s=ic.refractory_s, noise_k=ic.noise_k)
        # 센서
        self._odom = Pose2D()
        self._scan: Optional[Scan] = None
        self._frame: Optional[Frame] = None
        self._depth: Optional[DepthFrame] = None
        self._oak_frame: Optional[Frame] = None            # OAK 컬러(6장) — 최신 한 장, 메모리에만 둔다(디스크에 쓰지 않는다)
        self._oak_info: Optional[tuple] = None             # (K 9개, 폭, 높이)
        self._depth_K: Optional[tuple] = None
        self._depth_info: Optional[tuple] = None  # (width, height, optical frame), matching _depth_K
        self._depth_last_header: Optional[float] = None
        self._depth_blind_at: Optional[float] = None   # 검증을 통과한 영상이 왔지만 쓸 수 있는 픽셀이 없던 시각(카메라는 살아 있다)
        self._last_img_t = -1e9
        self._received: Dict[str, float] = {}
        self.screen = OledScreen(h.oled_cols, h.oled_lines, h.oled_min_interval_s, h.oled_refresh_s)
        self.stats: Dict[str, Any] = {"gimbal_sent": 0, "gimbal_ok": 0, "gimbal_locked": 0, "gimbal_fail": 0, "fire_calls": 0,
                                      "cmd_pub": 0, "cmd_zero_stale": 0, "cmd_zero_locked": 0, "frames": 0, "frames_dropped": 0,
                                      "oled_sent": 0, "match_changes": 0, "imu_impacts": 0}
        # 9장: 버린 데이터의 이유는 stats["invalid"][출처][이유] 로 센다. validation_status.json 쓰기는 scripts/run_real.py 가 시작한다
        # (시험·시뮬에서는 파일을 만들지 않는다).
        self.vlog = ValidationLog(self.stats)
        self.fire.vlog = self.vlog
        self._frame_watch = FrameWatch(h.frozen_frames, h.dark_frames)        # V4 — 팬틸트 카메라
        self._oak_watch = FrameWatch(h.frozen_frames, h.dark_frames)          # V8 — OAK 컬러(V4 와 같은 규칙)
        self._odom_watch = OdomJumpDetector(h.v_cap, h.w_cap)                 # V7 — 기록만 한다
        self._dark_n: Dict[str, int] = {}
        self.tr: Optional[Transport] = transport
        if transport is None:
            self._connect()

    # ------------------------------------------------------------------ 연결
    def _connect(self) -> None:
        raise NotImplementedError("전송 계층이 없다 — RosWingbot 를 쓰거나 transport= 를 넘긴다")

    # ------------------------------------------------------------------ 시간
    def now(self) -> float:
        return self._clock() - self._t0

    # ------------------------------------------------------------------ 저하 모드 (preflight.ReadyState)
    def set_degraded(self, flags) -> None:
        """no_drive / no_vision / no_fire. 가드는 플래그마다 한 곳이다:
        no_drive → _publish_cmd(HAL 출력이 항상 0), no_fire → _gimbal_pump 와 laser_fire(서비스 호출), no_vision → Agent._ingest_vision."""
        flags = frozenset(str(f) for f in (flags or ()))
        with self._lock:
            prev, self._degraded = self._degraded, flags
            self.stats["degraded"] = sorted(flags)
        if "no_drive" in flags and "no_drive" not in prev and self.tr is not None:
            self._publish_cmd(0.0, 0.0)                  # 이미 나가던 명령이 있으면 바로 끊는다

    def degraded_flags(self) -> frozenset:
        with self._lock:
            return self._degraded

    def is_degraded(self, flag: str) -> bool:
        return flag in self._degraded

    # ------------------------------------------------------------------ 구동 [p15]
    def wheel_command(self, v: float, w: float):
        """(v, w) → (왼쪽 m/s, 오른쪽 m/s, 왼쪽 rad/s, 오른쪽 rad/s). 드라이버가 /cmd_vel 대신 좌우 바퀴 속도를 받을 때 쓴다(이 로봇은 /cmd_vel)."""
        vl, vr = self._dd.wheels_from_twist(v, w)
        wl, wr = self._dd.wheel_rates(v, w)
        return vl, vr, wl, wr

    def set_velocity(self, v: float, w: float) -> None:
        """다음 호출 전까지 유지. 실제 발행은 pump() 가 cmd_hz(10 Hz)로 반복한다. 0 은 즉시 발행(정지 지연 방지)."""
        h = self.cfg.hal
        v, w = limit_cmd_vel(float(v), float(w), h.v_cap, h.w_cap)
        with self._lock:
            self._cmd = (v, w, self.now())
        if v == 0.0 and w == 0.0 and self._last_pub != (0.0, 0.0):
            self._publish_cmd(0.0, 0.0)

    def _publish_cmd(self, v: float, w: float) -> None:
        # Recheck under the same lock as on_match: a stale pump cannot publish
        # motion after a concurrent pause/stop callback.
        with self._lock:
            if self._closed or self._control is not True:
                v = w = 0.0
            if "no_drive" in self._degraded:             # 저하 모드: LiDAR·Nav2·TF 없이 움직이지 않는다 — HAL 출력의 유일한 가드
                if v != 0.0 or w != 0.0:
                    self.stats["cmd_zero_no_drive"] = self.stats.get("cmd_zero_no_drive", 0) + 1
                v = w = 0.0
            if self._depth_gates_drive() and not self._depth_fresh(self.now()):
                if self._depth_blind_alive(self.now()):
                    # 카메라는 살아 있는데 앞이 안 보인다: 전진만 막는다. 후진·회전까지 막으면 벽에 붙은 채 갇힌다.
                    v = 0.0 if v > 0 else max(v, -self.cfg.depth.blind_reverse_mps)
                    self.stats["cmd_limit_depth_blind"] = self.stats.get("cmd_limit_depth_blind", 0) + 1
                else:
                    v = w = 0.0
                    self.stats["cmd_zero_depth"] = self.stats.get("cmd_zero_depth", 0) + 1
            self._last_pub = (v, w)
            self.stats["cmd_pub"] += 1
            if self.tr is not None:
                self.tr.publish_cmd(v, w)

    # ------------------------------------------------------------------ 팬틸트 [p14]
    def set_pan_tilt(self, pan_deg: float, tilt_deg: float) -> None:
        with self._lock:
            self._want = quantize_pan_tilt(pan_deg, tilt_deg, self.cfg.hal)
            self._g_has_goal = True
        self._gimbal_pump(self.now())

    def get_pan_tilt(self) -> Tuple[float, float]:
        with self._lock:
            if self.cfg.camera.calibration_profile:
                return self._measured_gimbal[:2] if self._measured_gimbal is not None else (0., 0.)
            return self.gimbal.position(self.now())

    def on_gimbal_feedback(self, pan: float, tilt: float) -> None:
        """Measured feedback only. Service acknowledgement is never a measurement."""
        if not (math.isfinite(pan) and math.isfinite(tilt)):
            raise ValueError("non-finite gimbal feedback")
        with self._lock:
            self._measured_gimbal = (float(pan), float(tilt), self.now())
            self.stats["gimbal_feedback"] = {"pan": float(pan), "tilt": float(tilt)}

    def _gimbal_pump(self, t: float) -> None:
        h = self.cfg.hal
        with self._lock:
            if self._closed or not self._g_has_goal or self._control is not True or self.tr is None:
                return                                   # 잠금 중에는 locked 만 돌아온다 — 재개 때 다시 보낸다
            if "no_fire" in self._degraded:
                return                                   # 저하 모드: 짐벌 서비스를 부르지 않는다
            if self._g_pending_t is not None and t - self._g_pending_t < h.gimbal_timeout_s:
                return
            if self._sent == self._want or t < self._g_next_try or t - self._g_last_send < h.gimbal_min_interval_s:
                return
            want = self._want
            big = self._is_big_move(self._sent, want)
            self._g_request += 1
            request, epoch = self._g_request, self._control_epoch
            self._g_pending_t, self._g_last_send = t, t
            self._g_pending_big = big
            self.stats["gimbal_sent"] += 1

        def done(msg: Optional[str], want=want, big=big) -> None:
            tr = self.now()
            m = (msg or "").strip().lower()
            with self._lock:
                if self._closed or request != self._g_request or epoch != self._control_epoch:
                    return
                self._g_pending_t = None
                self._g_pending_big = False
                if m == "ok":
                    self.stats["gimbal_ok"] += 1
                    self._sent = want
                    if big:
                        self._g_move_t = tr              # ok 는 접수일 뿐 도착이 아니다(p72) — 접수 시각부터 정착 시간을 센다
                    self.gimbal.command(tr, pan_unit_to_deg(want[0], h.pan_deg_per_unit, h.pan_sign),
                                        tilt_unit_to_deg(want[1], h.tilt_deg_per_unit, h.tilt_sign))
                elif m == "locked":
                    self.stats["gimbal_locked"] += 1
                    self._g_next_try = tr + h.locked_retry_s
                    if self._control:                    # 제어가 허용인데 locked = 피격 후 제한(하드웨어가 직접 알려 준 값)
                        self.hit.note_locked_response(tr, h.locked_hold_s)
                else:
                    self.stats["gimbal_fail"] += 1
                    self._g_next_try = tr + h.locked_retry_s
                    why = unknown_response("gimbal", msg)                # V2: 모르는 응답 문자열은 원문과 함께 센다. 동작은 그대로(실패로 본다)
                    if why is not None:
                        self.vlog.bad("gimbal", why)

        # Serialize the actual enqueue with pause/stop, not only its response.
        # Transport calls are non-blocking; synchronous test callbacks are safe
        # because this is an RLock. Never hold it while waiting for a response.
        with self._lock:
            if (self._closed or self._control is not True or epoch != self._control_epoch or
                    request != self._g_request):
                return
            try:
                self.tr.call_gimbal(want[0], want[1], float(np.clip(h.gimbal_speed, 1, 270)), float(np.clip(h.gimbal_acc, 1, 254)), done)
            except Exception as e:
                # 요청 필드 이름·서비스 타입은 자료에 없는 추정값이다. 틀려도 Agent.tick() 까지 올려 경기를 끝내지 않는다.
                self._transport_error("gimbal", e)
                if request == self._g_request:
                    self._g_pending_t = None
                    self._g_next_try = t + h.locked_retry_s
                    self.stats["gimbal_fail"] += 1

    def _is_big_move(self, prev: Optional[Tuple[int, int]], want: Tuple[int, int]) -> bool:
        """직전에 접수된 명령(prev)에서 want 로 가는 각도 차가 aim.settle_move_deg 보다 큰가. 이전 명령을 모르면(첫 명령) 큰 이동으로 본다."""
        if prev is None:
            return True
        h = self.cfg.hal
        dpan = abs(pan_unit_to_deg(want[0], h.pan_deg_per_unit, h.pan_sign) - pan_unit_to_deg(prev[0], h.pan_deg_per_unit, h.pan_sign))
        dtilt = abs(tilt_unit_to_deg(want[1], h.tilt_deg_per_unit, h.tilt_sign) - tilt_unit_to_deg(prev[1], h.tilt_deg_per_unit, h.tilt_sign))
        return max(dpan, dtilt) > self.cfg.aim.settle_move_deg

    def gimbal_settle_left(self, t: Optional[float] = None) -> float:
        """4장: 크게 움직인 마지막 짐벌 명령 뒤로 aim.gimbal_settle_s 가 지나기까지 남은 시간(s). 설정이 0 이면 항상 0.
        접수 응답을 기다리는 큰 이동 명령이 있으면 아직 시작도 안 한 것이므로 전체 시간을 돌려준다."""
        settle = self.cfg.aim.gimbal_settle_s
        if settle <= 0.0:
            return 0.0
        t = self.now() if t is None else t
        with self._lock:
            if self._g_pending_t is not None and self._g_pending_big:
                return settle
            return max(self._g_move_t + settle - t, 0.0)

    def _transport_error(self, what: str, e: BaseException) -> None:
        self.stats["transport_errors"] = self.stats.get("transport_errors", 0) + 1
        self.stats["transport_last_error"] = f"{what}: {type(e).__name__}: {e}"

    # ------------------------------------------------------------------ 레이저 [p16]
    def laser_fire(self) -> bool:
        t = self.now()
        h = self.cfg.hal
        with self._lock:
            if self._closed or self._control is not True or self.tr is None:
                return False
            if "no_fire" in self._degraded:
                return False                             # 저하 모드: 발사 서비스를 부르지 않는다
            if t < self._fire_retry_at:
                return False
            if self.fire.laser_on(t) or self.fire.cooldown_left(t) > 0 or self.hit.disabled_left(t) > 0:
                return False                             # 우리 추정으로 이미 불가 — 서비스를 부르지 않는다
            if self.cfg.aim.gimbal_settle_s > 0.0 and self.gimbal_settle_left(t) > 0.0:
                self.stats["fire_wait_settle"] = self.stats.get("fire_wait_settle", 0) + 1
                return False                             # 4장: 짐벌이 아직 정착 중 — hold_before_fire 와 AND. 발사 단일 지점이라 모든 사격 경로가 같다
            self._fire_request += 1
            request, epoch = self._fire_request, self._control_epoch
            self._fire_pending = request
            # An uncertain service result must not generate duplicate requests.
            self._fire_retry_at = t + self.fire.period_s
            self.stats["fire_calls"] += 1
        ev, res = threading.Event(), {"msg": None}

        def done(msg: Optional[str], left_ms: Optional[float] = None) -> None:
            tr = self.now()
            with self._lock:
                if self._closed or request != self._fire_request or self._fire_pending != request:
                    ev.set()
                    return
                self._fire_pending = None
                r = self.fire.on_response(tr, msg, left_ms)
                # 응답 본문이 있으면 상태 모델이 판단한다. 호출 자체가 실패(서비스 미준비·예외)면 발사되지 않았으니 짧게만 쉰다.
                self._fire_retry_at = tr if msg is not None else tr + h.locked_retry_s
                if epoch != self._control_epoch or self._control is not True:
                    self.fire.cancel_emission(tr)
                    r = FIRE_LOCKED
                if r == FIRE_LOCKED and self._control:
                    self.hit.note_locked_response(tr, h.locked_hold_s)
            res["msg"] = r
            ev.set()

        with self._lock:
            if (self._closed or self._control is not True or epoch != self._control_epoch or
                    request != self._fire_request):
                if self._fire_pending == request:
                    self._fire_pending = None
                    self._fire_retry_at = t
                return False
            try:
                self.tr.call_fire(done)
            except Exception as e:                       # 위 gimbal 과 같은 이유: 전송 오류가 제어 루프를 죽이면 안 된다
                self._transport_error("fire", e)
                if self._fire_pending == request:
                    self._fire_pending = None
                    self._fire_retry_at = t + h.locked_retry_s     # 매 step 마다 같은 예외를 되풀이하지 않는다
                return False
        if ev.wait(h.fire_wait_s):
            return res["msg"] == FIRE_FIRED
        return False                                     # 응답 유실은 성공이 아니다. 늦은 응답은 상태 모델에만 반영한다.

    def laser_abort(self) -> None:
        pass                                             # 소등 API 없음. 일시정지·피격 때 펌웨어가 끈다(p12)

    # ------------------------------------------------------------------ 센서 getter
    def get_scan(self) -> Optional[Scan]:
        return self._scan

    def get_frame(self) -> Optional[Frame]:
        return self._frame

    def get_depth(self) -> Optional[DepthFrame]:
        with self._lock:
            return self._depth

    def _depth_gates_drive(self) -> bool:
        """엄격 깊이(require_calibration)가 구동·준비 판정을 막는가. depth.gate_drive=False 면 깊이는 선택 센서 —
        피아식별·거리 측정에만 쓰고, 영상이 끊겨도 로봇을 세우지 않는다(항법 장애물은 2D LiDAR)."""
        d = self.cfg.depth
        return bool(d.enable and d.require_calibration and d.gate_drive)

    def _depth_fresh(self, now: float, max_age_s: float = 1.0) -> bool:
        """Caller holds _lock. Only successfully validated depth refreshes this clock."""
        received = self._received.get("depth")
        return (self._depth is not None and received is not None and
                0 <= now - received <= max_age_s and now - self._depth.stamp <= max_age_s)

    def _depth_blind_alive(self, now: float, max_age_s: float = 1.0) -> bool:
        """Caller holds _lock. 최근에 검증된 영상이 왔지만 쓸 수 있는 픽셀이 없었다."""
        return self._depth_blind_at is not None and 0 <= now - self._depth_blind_at <= max_age_s

    def get_odom(self) -> Pose2D:
        with self._lock:
            return self._odom.copy()

    def readiness(self, sensor_max_age_s: float = 1.0, state_max_age_s: float = 2.0) -> Tuple[bool, list]:
        """Received data, not default zero values. Standby is ready but locked.

        Match is latched (not necessarily periodic); scan/odom/image/IMU/state
        must be recent. Enabled calibrated depth must also be recent; permissive
        simulation depth stays optional. Service availability is also checked.
        """
        now = self.now()
        with self._lock:
            missing = [k for k in ("scan", "odom", "image", "imu", "state", "match")
                       if k not in self._received or (k != "match" and
                           now - self._received[k] > (state_max_age_s if k == "state" else sensor_max_age_s))]
            if not self._mode or self._control is None:
                missing.append("match_mode")
            if self._depth_gates_drive() and not self._depth_fresh(now, sensor_max_age_s):
                missing.append("depth")
        if self.tr is not None:
            _, transport_missing = self.tr.readiness()
            missing.extend(transport_missing)
        else:
            missing.append("transport")
        return not missing, missing

    # ------------------------------------------------------------------ 상태
    def status(self) -> RobotStatus:
        t = self.now()
        with self._lock:
            r, p, g = self._imu
            return RobotStatus(ammo=self.fire.ammo, ammo_free=self.fire.practice, laser_on=self.fire.laser_on(t), cooldown_left=max(self.fire.cooldown_left(t), self._fire_retry_at - t, 0.0),
                               disabled_left=self.hit.disabled_left(t), drive_paused_left=self.hit.drive_left(t),
                               hits_taken=self.hit.hits, battery_v=12.0, control=self._control, match_mode=self._mode,
                               match_cmd=self._cmd_state, hp=self._hp, hp_max=self._hp_max, battery_pct=self._battery,
                               roll_deg=r, pitch_deg=p, accel_g=g)

    # ------------------------------------------------------------------ OLED [p13]
    def show(self, text: str) -> None:
        """줄바꿈으로 최대 4줄. 바뀐 줄만 0.5 s 간격으로 보낸다(pump)."""
        with self._lock:
            self.screen.set_text(text)

    # ------------------------------------------------------------------ 주기 작업 (전송 계층의 타이머가 cmd_hz 로 부른다)
    def pump(self) -> None:
        if self.tr is None:
            return
        t = self.now()
        h = self.cfg.hal
        with self._lock:
            v, w, ts = self._cmd
            closed, ctrl = self._closed, self._control
        if closed:
            return
        if t - ts > h.cmd_stale_s:                       # Agent 가 멈췄다(예외·지연): 마지막 속도를 반복하지 않고 정지시킨다
            v = w = 0.0
            self.stats["cmd_zero_stale"] += 1
        if ctrl is not True:                             # 상태 수신 전에도 제어를 허용하지 않는다.
            v = w = 0.0
            self.stats["cmd_zero_locked"] += 1
        self._publish_cmd(v, w)
        self._gimbal_pump(t)
        with self._lock:
            nxt = self.screen.next(t)
        if nxt is not None:
            self.stats["oled_sent"] += 1
            self.tr.publish_oled(nxt[0], nxt[1])

    # ------------------------------------------------------------------ 수신 콜백 (전송 계층이 부른다)
    @_never_raises("match")
    def on_match(self, mode: str, cmd: str) -> None:
        """/mil/match: 경기 모드·단계. 제어 허용이 바뀌는 순간의 처리가 핵심이다(p12: 재개 시 스스로 움직이지 않음 → 명령·팬틸트 재전송)."""
        t = self.now()
        raw_mode, raw_cmd = mode, cmd
        mode, cmd = str(mode or "").strip().lower(), str(cmd or "").strip().lower()
        for why in unknown_match_fields(raw_mode, raw_cmd, CMD_STATES):       # V3: 모르는 값은 경고만. 제어 허용 판단·연습 모드 판단은 그대로다
            self.vlog.bad("match", why)
        with self._lock:
            self._received["match"] = t
            changed = (mode, cmd) != (self._mode, self._cmd_state)
            prev, new = self._control, control_allowed(mode, cmd)
            prev_mode = self._mode
            self._mode, self._cmd_state, self._control = mode, cmd, new
            # 2장: 연습 모드는 탄약·HP 가 자동 회복된다(p70). 모르는 mode 는 연습 모드로 보지 않는다.
            practice = is_practice(mode)
            self.fire.set_practice(practice)
            self.hit.set_practice(practice)
            if prev_mode == "practice" and mode == "mission":       # 전환을 실제로 본 순간에만 초기화(처음부터 mission 이면 기존 복원 경로)
                self.fire.reset_for_mission()
                self.hit.reset_for_mission()
            if changed:
                self.stats["match_changes"] += 1
            if prev != new:                              # 제어 허용이 바뀔 때만 진행 중 요청을 무효화한다(연습 모드의 cmd 변화는 제어와 무관)
                self._control_epoch += 1
                self._g_request += 1
                self._g_pending_t = None
            if new is False and prev is not False:
                self.fire.cancel_emission(t)             # 일시정지 시 레이저 꺼짐
                self.gimbal.hold(t)
                self._cmd = (0.0, 0.0, t)
            if new and prev is not True:
                self._sent = None                        # 팬틸트 재전송 강제
                self._g_pending_t, self._g_next_try = None, -1e9
                self._cmd = (0.0, 0.0, t)                # Agent 가 새 명령을 줄 때까지 0
        if new is False and prev is not False and self.tr is not None:
            self._publish_cmd(0.0, 0.0)

    @_never_raises("state")
    def on_state(self, hp: Optional[int], hp_max: Optional[int], battery_pct: Optional[float]) -> None:
        t = self.now()
        with self._lock:
            # A discovered message type with missing/unusable fields is not a
            # successful state sample. Preserve partial data for diagnostics.
            if (hp is not None and hp_max is not None and battery_pct is not None and
                    all(math.isfinite(v) for v in (hp, hp_max, battery_pct)) and
                    hp_max > 0 and 0 <= hp <= hp_max and 0 <= battery_pct <= 100):
                self._received["state"] = t
            self.hit.on_state(t, hp, hp_max)
            if hp is not None:
                self._hp = int(hp)
            if hp_max is not None:
                self._hp_max = int(hp_max)
            if battery_pct is not None:
                self._battery = float(battery_pct)

    @_never_raises("odom")
    def on_odom(self, x: float, y: float, th: float) -> None:
        if not all(finite(v) for v in (x, y, th)):
            self.vlog.bad("odom", "nonfinite")
            return
        now = self.now()
        with self._lock:
            # V7: 연속한 odom 사이 이동이 상한(v_cap·dt·3 + 0.05 m 등)을 넘으면 odom_jump 로 센다. **값은 바꾸지 않는다** — 기록만.
            why = self._odom_watch.update(now, float(x), float(y), float(th))
            self._odom = Pose2D(float(x), float(y), float(th))
            self._received["odom"] = now
        if why is not None:
            self.vlog.bad("odom", why, f"({float(x):.2f},{float(y):.2f},{float(th):.2f})")
        else:
            self.vlog.good("odom")

    @_never_raises("imu")
    def on_imu(self, roll: float, pitch: float, yaw: float, g: float,
               ax: Optional[float] = None, ay: Optional[float] = None, gz_dps: Optional[float] = None) -> None:
        """ax, ay [g] (차체 x 앞·y 왼쪽)와 gz_dps [deg/s, 왼쪽 +] 는 충돌 감지용이다(선택 — 안 주면 가속도 크기 g 만 본다)."""
        if not all(math.isfinite(v) for v in (roll, pitch, yaw, g)):
            return
        now = self.now()
        with self._lock:
            self._imu = (roll, pitch, g)
            self._received["imu"] = now
            if g > self.cfg.hal.impact_g:
                self.stats["imu_impacts"] += 1
        det = self._impact_det
        if det is not None:
            if max(abs(float(roll)), abs(float(pitch))) > self.cfg.hal.tilt_stop_deg:     # 기울기 정지 영역: 충돌 판단은 하지 않는다(manager 의 tilt 정지가 맡는다)
                return
            try:
                if det.add(now, float(g), ax, ay, gz_dps) is not None:
                    self._impact_n += 1
                    self.stats["impact_events"] = self._impact_n
            except Exception:                                                  # noqa: BLE001 — 충돌 감지기의 오류가 IMU 콜백을 죽이지 않는다
                self.stats["impact_detector_errors"] = self.stats.get("impact_detector_errors", 0) + 1

    def pop_impacts(self):
        """지금까지 쌓인 충돌 이벤트(pose_guard.ImpactEvent, 시간순)를 꺼낸다. 감지기가 꺼져 있으면 빈 목록."""
        det = self._impact_det
        return det.pop() if det is not None else []

    @_never_raises("scan")
    def on_scan(self, angle_min: float, angle_inc: float, ranges, hdr_t: float, range_max: float,
                *, receipt_wall_s=None, receipt_seq=None) -> None:
        now = self.now()
        ranges = np.asarray(ranges, dtype=float)
        if not (math.isfinite(angle_min) and math.isfinite(angle_inc) and angle_inc != 0 and
                math.isfinite(range_max) and range_max > 0 and np.any(np.isfinite(ranges) & (ranges > 0))):
            self.stats["scan_invalid"] = self.stats.get("scan_invalid", 0) + 1
            return
        stamp = self._sync_scan.to_robot(hdr_t, now) if hdr_t > 0 else now
        self._scan = Scan(angle_min + LIDAR_ANGLE_OFFSET, angle_inc, ranges, stamp=stamp, range_max=float(range_max),
                          receipt_wall_s=receipt_wall_s, receipt_seq=receipt_seq)
        self._received["scan"] = now

    @_never_raises("oak_rgb")
    def on_oak_frame(self, image: np.ndarray, hdr_t: float) -> None:
        """OAK 컬러 BGR (H,W,3). 팬틸트 카메라와 달리 차체에 고정이라 pan/tilt 는 0이다. 준비 확인(/ready)에는 쓰이지 않는다."""
        if image is None or getattr(image, "ndim", 0) != 3 or image.shape[2] != 3 or min(image.shape[:2]) <= 0:
            self.stats["oak_frame_invalid"] = self.stats.get("oak_frame_invalid", 0) + 1
            self.vlog.bad("oak_rgb", "bad_dims")
            return
        now = self.now()
        h, w = image.shape[:2]
        info = self._oak_info
        if info is not None and (int(w), int(h)) != (info[1], info[2]):          # V8: 크기는 CameraInfo 와 같아야 한다
            self.vlog.bad("oak_rgb", "size_mismatch_info", f"{w}x{h} != CameraInfo {info[1]}x{info[2]}")
            return
        frozen, dark = self._oak_watch.update(image, hdr_t if finite(hdr_t) else None)
        if dark:
            self._note_dark("oak_rgb", self._oak_watch)
        else:
            self._dark_n.pop("oak_rgb", None)
        if frozen:                                                               # 같은 영상이 이어지면 새 영상으로 치지 않는다(수신 시각 갱신 없음)
            self.vlog.bad("oak_rgb", "camera_frozen")
            return
        stamp = self._sync_oak.to_robot(hdr_t, now) if finite(hdr_t) and hdr_t > 0 else now
        with self._lock:
            self._oak_frame = Frame(image, stamp, 0.0, 0.0, int(w), int(h), meta={"source": "oak"})
            self._received["oak_rgb"] = now
        self.vlog.good("oak_rgb")
        self.stats["oak_frames"] = self.stats.get("oak_frames", 0) + 1

    @_never_raises("oak_info")
    def on_oak_info(self, K: tuple, width: int, height: int) -> None:
        """OAK CameraInfo. K 가 이상하면(유한하지 않음·fx,fy ≤ 0·주점이 영상 밖) 버리고 이전 값도 지운다 → OAK 관측이 꺼진다(V8)."""
        try:
            k = tuple(float(v) for v in K)
            w, h = int(width), int(height)
            why = check_camera_matrix(k, w, h)
            if why is None and getattr(self.cfg.oak, "profile", "") == "rgbd":
                why = check_aligned_size(w, h)                                   # 깊이를 컬러에 정렬한 프로파일은 16 의 배수
        except (TypeError, ValueError, OverflowError):
            why, k, w, h = "K_malformed", (), 0, 0
        if why is not None:
            self.drop_oak_info(why)
            return
        with self._lock:
            self._oak_info = (k, w, h)
        self.vlog.good("oak_info")

    def drop_oak_info(self, reason: str, detail: str = "") -> None:
        """쓸 수 없는 OAK CameraInfo: 이전 값도 지워 OAK 관측을 끄고(manager._oak_camera 가 None) 이유를 센다."""
        with self._lock:
            self._oak_info = None
        self.vlog.bad("oak_info", reason, detail)

    def get_oak_frame(self) -> Optional[Frame]:
        with self._lock:
            return self._oak_frame

    def get_oak_info(self) -> Optional[tuple]:
        with self._lock:
            return self._oak_info

    def oak_power(self, on: bool) -> str:
        """OAK 를 켜고 끈다(oak.power_gating). 전송 계층이 서비스를 호출한다 — 막지 않고, 예외는 'fail' 로 센다."""
        tr = self.tr
        if tr is None or not self.cfg.oak.power_gating:
            return "none"
        try:
            return str(tr.call_oak_power(bool(on)))
        except Exception as e:                                                   # noqa: BLE001
            self._transport_error("oak_power", e)
            return "fail"

    @_never_raises("image")
    def on_image(self, image: np.ndarray, hdr_t: float, *, pan_tilt=None, meta=None) -> None:
        """BGR (H,W,3). 10 FPS 이하만 받는다(p17). 촬영 시각의 팬/틸트는 추정 위치에서 읽는다."""
        now = self.now()
        h = self.cfg.hal
        if self.cfg.camera.calibration_profile and pan_tilt is None:
            self.stats["camera_calibration_error"] = "image lacks measured TF/joint pose"
            return
        if now - self._last_img_t < 1.0 / max(h.cam_max_fps, 1e-3) - 1e-3:
            self.stats["frames_dropped"] += 1
            return
        self._last_img_t = now
        # V4: 크기·버퍼 검사는 ROS 메시지 입구(ros_mil._on_image)에서 한다. 여기서는 배열의 모양과 멈춤·암흑만 본다.
        if image is None or getattr(image, "ndim", 0) != 3 or image.shape[2] != 3 or min(image.shape[:2]) <= 0:
            self.vlog.bad("image", "bad_dims")
            return
        frozen, dark = self._frame_watch.update(image, hdr_t if finite(hdr_t) else None)
        if dark:
            self._note_dark("image", self._frame_watch)
        else:
            self._dark_n.pop("image", None)
        if frozen:                                       # 같은 영상이 hal.frozen_frames 장 이어졌다 → 새 영상으로 치지 않는다: 수신 시각을 갱신하지 않는다
            self.vlog.bad("image", "camera_frozen")
            return
        stamp = self._sync_frame.to_robot(hdr_t, now) if finite(hdr_t) and hdr_t > 0 else now
        with self._lock:
            pan, tilt = pan_tilt if pan_tilt is not None else self.gimbal.at(stamp, now)
        self.stats["frames"] += 1
        self._frame = Frame(image=image, stamp=stamp, pan=pan, tilt=tilt, width=int(image.shape[1]), height=int(image.shape[0]), meta=dict(meta or {}))
        self._received["image"] = now
        self.vlog.good("image")

    def _note_dark(self, source: str, watch: FrameWatch) -> None:
        """전부 검거나 하얀 영상이 이어지면 경고만 한다(버리지 않는다). 계속될 때는 dark_frames 장마다 한 번씩만 센다."""
        n = self._dark_n.get(source, 0) + 1
        self._dark_n[source] = n
        if n % max(watch.dark_frames, 1) == 1:
            self.vlog.bad(source, "camera_dark", "평균 밝기 <5 또는 >250 이 이어진다 — 렌즈 가림·노출 확인(영상은 그대로 쓴다)")

    def on_depth_info(self, m) -> None:
        with self._lock:
            try:
                K, width, height = camera_info_K(m)
                frame = str(getattr(getattr(m, "header", None), "frame_id", ""))
                if self.cfg.depth.require_calibration and (not frame or frame != self.cfg.depth.optical_frame):
                    raise ValueError("CameraInfo frame does not match depth.optical_frame")
                info = (width, height, frame)
                if self._depth_info != info or self._depth_K != K:
                    self.reject_depth("CameraInfo changed; waiting for matching depth Image")
                self._depth_K, self._depth_info = K, info
                self.stats.pop("depth_info_error", None)
            except (ValueError, TypeError, AttributeError, OverflowError) as e:
                # Never leave a previously valid K installed after invalid info.
                self._depth_K = self._depth_info = None
                self.stats["depth_info_error"] = str(e)
                self.reject_depth("CameraInfo: " + str(e))

    def reject_depth(self, reason: str) -> None:
        """Discard invalid depth immediately, including its readiness timestamp."""
        with self._lock:
            self._depth = None
            self._depth_blind_at = None                  # 검증 자체가 실패했다 = 카메라를 믿을 수 없다 → 전면 정지
            self._received.pop("depth", None)
            self.stats["depth_error"] = str(reason)
            self.stats["depth_rejected"] = self.stats.get("depth_rejected", 0) + 1
            if self._depth_gates_drive() and self._last_pub != (0.0, 0.0):
                self._publish_cmd(0.0, 0.0)

    def on_depth(self, m, *, tf_validated: bool = False) -> None:
        """깊이 영상 → DepthFrame. 깊이 카메라는 팬/틸트 아래 고정이라 pan/tilt 는 3D 등록에 쓰이지 않는다(참고용). stamp 는 robot.now() 시계."""
        # CameraInfo validation and publication share one lock: invalidation
        # cannot race decoding and allow an old calibration to reappear.
        with self._lock:
            try:
                dc = self.cfg.depth
                if dc.require_calibration:
                    if not dc.calibration_verified or not dc.optical_frame:
                        raise ValueError("depth calibration is not verified")
                    if not tf_validated:
                        raise ValueError("depth TF has not been validated")
                    if self._depth_K is None or self._depth_info is None:
                        raise ValueError("valid matching depth CameraInfo has not been received")
                    if (int(m.width), int(m.height), str(m.header.frame_id)) != self._depth_info:
                        raise ValueError("depth Image dimensions/frame do not match CameraInfo")
                    if m.header.frame_id != dc.optical_frame:
                        raise ValueError("depth Image frame does not match depth.optical_frame")
                hdr = m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec
                if not math.isfinite(hdr) or (dc.require_calibration and hdr <= 0):
                    raise ValueError("depth Image has an invalid/zero timestamp")
                if dc.require_calibration and self._depth_last_header is not None:
                    if hdr < self._depth_last_header:
                        raise ValueError("depth Image timestamp moved backwards")
                    if hdr == self._depth_last_header:
                        # Replayed buffers cannot keep the runtime deadman alive.
                        self.stats["depth_repeated"] = self.stats.get("depth_repeated", 0) + 1
                        return
                now = self.now()
                stamp = self._sync_depth.to_robot(hdr, now) if hdr > 0 else now
                pan, tilt = self.gimbal.position(now)
                frame = make_depth_frame(m, stamp, pan, tilt, dc.unit_scale, self._depth_K)
                if dc.require_calibration:
                    d = frame.depth
                    self._depth_last_header = hdr
                    # max_range 밖(먼 곳)도 '앞이 비어 있다'는 정보다 — 경기장에서 먼 벽만 보일 때 로봇이 서면 안 된다.
                    # 쓸 수 없는 것은 무반환(0/NaN)과 min_range 안쪽(너무 가까움)뿐이다.
                    if not np.any(np.isfinite(d) & (d > 0) & (d >= dc.min_range)):
                        self._depth = None
                        self._received.pop("depth", None)
                        self._depth_blind_at = now
                        self.stats["depth_error"] = "depth Image has no valid pixels (no return or closer than min_range)"
                        self.stats["depth_rejected"] = self.stats.get("depth_rejected", 0) + 1
                        cur = self._cmd
                        if self._last_pub != (0.0, 0.0):
                            self._publish_cmd(cur[0], cur[1])     # 전진 중이었다면 지금 바로 전진을 끊는다
                        return
                    self._depth_blind_at = None
                self._depth = frame
                self._received["depth"] = now
                self.stats.pop("depth_error", None)
            except (ValueError, TypeError, AttributeError, OverflowError) as e:
                self.reject_depth(str(e))

    # ------------------------------------------------------------------ 종료
    def close(self) -> None:
        """종료·예외 시 Twist() 정지(p18). 몇 번 반복해서 보낸다(유실 대비)."""
        with self._lock:
            if self._closed:
                return
            self._closed = True
        for _ in range(3):
            self._publish_cmd(0.0, 0.0)
        if self.tr is not None:
            self.tr.close()


class RosWingbot(WingbotRobot):  # pragma: no cover - rclpy 필요
    """실기: WingbotRobot + rclpy 전송."""

    def _connect(self) -> None:
        from .ros_mil import RosMilTransport
        self.tr = RosMilTransport(self)
