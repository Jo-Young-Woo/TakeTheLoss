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
from .mil_api import (FIRE_FIRED, FIRE_LOCKED, FireModel, GimbalModel, HitModel, OledScreen, control_allowed, limit_cmd_vel,
                      pan_unit_to_deg, quantize_pan_tilt, tilt_unit_to_deg)

LIDAR_ANGLE_OFFSET = 0.0   # ROS 전송층이 센서 TF를 적용한다. 장착 방향은 TF에서 보정하고 여기서 이중 적용하지 않는다.


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
        self._g_next_try = -1e9
        self._g_last_send = -1e9
        self._g_request = 0
        self._control_epoch = 0
        self._fire_request = 0
        self._fire_pending: Optional[int] = None
        self._fire_retry_at = -1e9
        # 레이저·피격
        self.fire = FireModel(self.cfg.laser.ammo, self.cfg.laser.shot_s, self.cfg.laser.shot_s + self.cfg.laser.cooldown_s)
        self.hit = HitModel(self.cfg.laser.disabled_s, h.drive_pause_s)
        # 경기 단계·상태
        self._mode, self._cmd_state = "", ""
        self._control: Optional[bool] = None
        self._battery: Optional[float] = None
        self._hp: Optional[int] = None
        self._hp_max: Optional[int] = None
        self._imu: Tuple[Optional[float], Optional[float], Optional[float]] = (None, None, None)
        # 센서
        self._odom = Pose2D()
        self._scan: Optional[Scan] = None
        self._frame: Optional[Frame] = None
        self._depth: Optional[DepthFrame] = None
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
        self.tr: Optional[Transport] = transport
        if transport is None:
            self._connect()

    # ------------------------------------------------------------------ 연결
    def _connect(self) -> None:
        raise NotImplementedError("전송 계층이 없다 — RosWingbot 를 쓰거나 transport= 를 넘긴다")

    # ------------------------------------------------------------------ 시간
    def now(self) -> float:
        return self._clock() - self._t0

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
            if (self.cfg.depth.enable and self.cfg.depth.require_calibration and
                    not self._depth_fresh(self.now())):
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
            if self._g_pending_t is not None and t - self._g_pending_t < h.gimbal_timeout_s:
                return
            if self._sent == self._want or t < self._g_next_try or t - self._g_last_send < h.gimbal_min_interval_s:
                return
            want = self._want
            self._g_request += 1
            request, epoch = self._g_request, self._control_epoch
            self._g_pending_t, self._g_last_send = t, t
            self.stats["gimbal_sent"] += 1

        def done(msg: Optional[str], want=want) -> None:
            tr = self.now()
            m = (msg or "").strip().lower()
            with self._lock:
                if self._closed or request != self._g_request or epoch != self._control_epoch:
                    return
                self._g_pending_t = None
                if m == "ok":
                    self.stats["gimbal_ok"] += 1
                    self._sent = want
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
            if t < self._fire_retry_at:
                return False
            if self.fire.laser_on(t) or self.fire.cooldown_left(t) > 0 or self.hit.disabled_left(t) > 0:
                return False                             # 우리 추정으로 이미 불가 — 서비스를 부르지 않는다
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
            if self.cfg.depth.enable and self.cfg.depth.require_calibration and not self._depth_fresh(now, sensor_max_age_s):
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
            return RobotStatus(ammo=self.fire.ammo, laser_on=self.fire.laser_on(t), cooldown_left=max(self.fire.cooldown_left(t), self._fire_retry_at - t, 0.0),
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
    def on_match(self, mode: str, cmd: str) -> None:
        """/mil/match: 경기 모드·단계. 제어 허용이 바뀌는 순간의 처리가 핵심이다(p12: 재개 시 스스로 움직이지 않음 → 명령·팬틸트 재전송)."""
        t = self.now()
        mode, cmd = (mode or "").strip().lower(), (cmd or "").strip().lower()
        with self._lock:
            self._received["match"] = t
            changed = (mode, cmd) != (self._mode, self._cmd_state)
            prev, new = self._control, control_allowed(mode, cmd)
            self._mode, self._cmd_state, self._control = mode, cmd, new
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

    def on_odom(self, x: float, y: float, th: float) -> None:
        if not all(math.isfinite(v) for v in (x, y, th)):
            return
        with self._lock:
            self._odom = Pose2D(float(x), float(y), float(th))
            self._received["odom"] = self.now()

    def on_imu(self, roll: float, pitch: float, yaw: float, g: float) -> None:
        if not all(math.isfinite(v) for v in (roll, pitch, yaw, g)):
            return
        with self._lock:
            self._imu = (roll, pitch, g)
            self._received["imu"] = self.now()
            if g > self.cfg.hal.impact_g:
                self.stats["imu_impacts"] += 1

    def on_scan(self, angle_min: float, angle_inc: float, ranges, hdr_t: float, range_max: float) -> None:
        now = self.now()
        ranges = np.asarray(ranges, dtype=float)
        if not (math.isfinite(angle_min) and math.isfinite(angle_inc) and angle_inc != 0 and
                math.isfinite(range_max) and range_max > 0 and np.any(np.isfinite(ranges) & (ranges > 0))):
            self.stats["scan_invalid"] = self.stats.get("scan_invalid", 0) + 1
            return
        stamp = self._sync_scan.to_robot(hdr_t, now) if hdr_t > 0 else now
        self._scan = Scan(angle_min + LIDAR_ANGLE_OFFSET, angle_inc, ranges, stamp=stamp, range_max=float(range_max))
        self._received["scan"] = now

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
        stamp = self._sync_frame.to_robot(hdr_t, now) if hdr_t > 0 else now
        with self._lock:
            pan, tilt = pan_tilt if pan_tilt is not None else self.gimbal.at(stamp, now)
        self.stats["frames"] += 1
        self._frame = Frame(image=image, stamp=stamp, pan=pan, tilt=tilt, width=int(image.shape[1]), height=int(image.shape[0]), meta=dict(meta or {}))
        self._received["image"] = now

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
            if self.cfg.depth.enable and self.cfg.depth.require_calibration and self._last_pub != (0.0, 0.0):
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
