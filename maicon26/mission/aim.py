"""조준 제어 + 레이저 관리.

AimController: 영상 오차 → 팬/틸트 증분(비주얼 서보), 팬이 한쪽으로 쏠리면 차체가 따라 돈다.
   - 카메라 축 = 레이저 축(근사). 시차/장착 오차는 AimCfg의 laser_offset_* 로 교정.
   - bbox 중앙이 아니라 **수광센서(원) 자체**를 겨냥한다. 센서 위치는 검출기가 Detection.extra['sensors'] 로 준다
     ((u, v, 반경 px, 신뢰도); 실기는 perception/receiver.py 가 흰 영역 안의 원을 찾는다). 표적 시설은 센서 1개,
     상대 UGV 는 좌/우 두 곳이라 직전 겨냥점에 가까운 쪽을 계속 쫓는다(번갈아 바뀌지 않게).
   - 센서를 못 찾으면 bbox 안의 대체 추측(aim_point_*)으로 조준은 계속하되, require_sensor 이면
     sensor_fallback_s 가 지나기 전에는 발사를 허용하지 않는다(8발을 추측에 쓰지 않게).
   - 허용 오차는 센서 반경에 맞춘다(빔이 원 안에 머물도록): min(거리별 허용치, 반경·sensor_radius_frac).
LaserManager: 8발·5초 발광·5초 재장전·피격 정지 규칙을 지키고, 발광 중 '정렬 유지 시간'을 센다
   (한 발에 2초 연속 수광이 필요 — 정렬이 끊기면 그 발은 낭비).
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

from ..config import Config
from ..hal.base import Robot
from ..perception.camera import CameraModel
from ..types import Detection
from ..util import LowPass, clamp


@dataclass
class AimResult:
    err_pan: float = 0.0       # deg, 목표가 왼쪽이면 +
    err_tilt: float = 0.0      # deg, 목표가 위쪽이면 +
    aligned: bool = False      # 발사 허용 오차 안?
    body_w: float = 0.0        # rad/s, 팬을 0으로 되돌리도록 차체가 도는 속도
    range_m: Optional[float] = None
    sensor_locked: bool = False   # 실제 수광센서 위치를 겨냥 중인가 (False 면 bbox 안의 대체 추측)
    sensor_ok: bool = True        # 발사 허용: 센서를 찾았거나, 못 찾은 채로 sensor_fallback_s 가 지났거나, require_sensor=False


class AimController:
    def __init__(self, cfg: Config) -> None:
        self.cfg = cfg
        self.cam = CameraModel(cfg.camera)
        self._side = 0                 # (센서를 못 찾았을 때 대체 추측용) UGV 수광센서 선택: -1 왼쪽, +1 오른쪽
        self._vel_u = LowPass(0.4, 0.0)
        self._fu = LowPass(0.55)
        self._fv = LowPass(0.55)
        self._last_u: Optional[float] = None
        self._last_t: Optional[float] = None
        self._last_pt: Optional[Tuple[float, float]] = None   # 직전 (필터 후) 겨냥점 — 센서 후보 중 연속인 쪽을 고르는 기준
        self._no_sensor_since: Optional[float] = None
        self.aligned_since: Optional[float] = None
        self.auto_resets = 0

    def reset(self) -> None:
        self._last_u = self._last_t = None
        self._vel_u = LowPass(0.4, 0.0)
        self._fu = LowPass(0.55)
        self._fv = LowPass(0.55)
        self.aligned_since = None
        self._side = 0
        self._last_pt = None
        self._no_sensor_since = None

    def _auto_reset(self) -> None:
        """추적 연속성이 끊겼다고 판단해 스스로 초기화한 횟수를 센다(진단용)."""
        self.reset()
        self.auto_resets += 1

    def aim_point(self, det: Detection) -> Tuple[float, float, Optional[float], bool]:
        """→ (u, v, 센서 반경 px 또는 None, 센서를 찾았는가)."""
        a = self.cfg.aim
        sensors = det.extra.get("sensors") if det.extra else None
        if sensors:
            ref = self._last_pt if self._last_pt is not None else (self.cam.cx, self.cam.cy)
            s = min(sensors, key=lambda q: math.hypot(q[0] - ref[0], q[1] - ref[1]))   # 직전 겨냥점에 가까운 센서를 계속 쫓는다
            return float(s[0]), float(s[1]), (float(s[2]) if len(s) > 2 and s[2] else None), True
        if det.cls == "UGV":                          # 대체 추측: 화면 중앙에서 가까운 쪽 센서 위치(bbox 28%/72%)
            if self._side == 0:
                self._side = -1 if det.cx >= self.cam.cx else 1
            fx = 0.28 if self._side == 1 else 0.72   # +1이면 화면 왼쪽 센서(bbox 28% 지점)
            return (*det.point(fx, a.aim_point_ugv[1]), None, False)
        return (*det.point(*a.aim_point_target), None, False)

    def update(self, det: Detection, pan: float, tilt: float, now: float, range_m: Optional[float]
               ) -> Tuple[float, float, AimResult]:
        """→ (새 pan 명령 deg, 새 tilt 명령 deg, 결과)."""
        a, r = self.cfg.aim, self.cfg.robot
        # 추적 연속성: 한동안 갱신이 없었거나(lost_timeout) 겨냥점이 갑자기 멀리 뛰면 다른 대상으로 보고 필터·직전 겨냥점·
        # 센서 선택을 모두 새로 시작한다. 안 그러면 옛 대상의 저역통과 값과 섞이고 '직전 겨냥점에 가까운 센서'가 엉뚱한 쪽을 고른다.
        if self._last_t is not None and now - self._last_t > a.lost_timeout:
            self._auto_reset()
        u, v, r_px, locked = self.aim_point(det)
        if self._last_pt is not None and math.hypot(u - self._last_pt[0], v - self._last_pt[1]) > a.target_jump_px:
            self._auto_reset()
            u, v, r_px, locked = self.aim_point(det)
        u, v = self._fu.update(u), self._fv.update(v)      # 검출 지터 완화
        self._last_pt = (u, v)
        if locked:
            self._no_sensor_since = None
        elif self._no_sensor_since is None:
            self._no_sensor_since = now
        # 주의: 시각이 0.0 일 수 있어 `or` 로 None 을 거르면 안 된다(0.0 은 거짓)
        waited = 0.0 if self._no_sensor_since is None else now - self._no_sensor_since
        sensor_ok = locked or not a.require_sensor or waited >= a.sensor_fallback_s
        # 이동 표적 선행(리드): 화면상 속도로 약간 앞을 겨냥
        if self._last_u is not None and self._last_t is not None and now > self._last_t:
            self._vel_u.update((u - self._last_u) / (now - self._last_t))
        self._last_u, self._last_t = u, now
        lead = clamp(self._vel_u.y or 0.0, -250.0, 250.0) * 0.10 if det.cls == "UGV" else 0.0
        # 레이저 장착 보정. (1) 축 오차: 고정 각도 laser_offset_*  (2) 시차: 레이저가 카메라보다 옆(lateral)·위(vertical)로 떨어져
        # 있으면 빔은 카메라 축과 평행하게 나가므로, 거리 r 의 표적을 맞히려면 표적이 화면 중심에서 atan(오프셋/r) 만큼
        # 치우쳐 보여야 한다(가까울수록 크다: 3 cm 면 1.1 m 에서 1.6°, 2.5 m 에서 0.7°). r 은 LiDAR/크기 기반 추정을 쓴다.
        rng = range_m if range_m is not None else 1.2
        rr = max(rng, 0.3)
        b_des = math.radians(a.laser_offset_pan_deg) + math.atan2(a.laser_lateral_m, rr)     # 왼쪽 + : 표적이 왼쪽에 보여야 한다
        e_des = math.radians(a.laser_offset_tilt_deg) + math.atan2(a.laser_vertical_m, rr)   # 위 + : 표적이 위쪽에 보여야 한다
        u_des = self.cam.cx - self.cam.f * math.tan(b_des)
        v_des = self.cam.cy - self.cam.fy * math.tan(e_des)
        err_pan = math.degrees(math.atan2(u_des - (u + lead), self.cam.f))
        err_tilt = math.degrees(math.atan2(v_des - v, self.cam.fy))
        pan_cmd = clamp(pan + a.kp_pan * err_pan, -r.pan_limit, r.pan_limit)
        tilt_cmd = clamp(tilt + a.kp_tilt * err_tilt, -r.tilt_limit, r.tilt_limit)
        body_w = clamp(a.body_kp * math.radians(pan), -r.w_max * 0.7, r.w_max * 0.7)
        # 거리에 따라 허용 오차를 줄인다(가까우면 관대, 멀면 엄격)
        tol =a.tol_deg + (a.tol_deg_far - a.tol_deg) * clamp((rng - 0.6) / 1.4, 0.0, 1.0)
        if locked and r_px:                                # 센서 반경 안에 빔이 머물도록 허용치를 반경에 맞춘다
            tol = min(tol, max(math.degrees(math.atan2(a.sensor_radius_frac * r_px, self.cam.f)), a.sensor_min_tol_deg))
        ok = abs(err_pan) <= tol and abs(err_tilt) <= 1.6 * tol
        if ok:
            if self.aligned_since is None:
                self.aligned_since = now
        else:
            self.aligned_since = None
        return pan_cmd, tilt_cmd, AimResult(err_pan, err_tilt, ok, body_w, range_m, locked, sensor_ok)

    def held_for(self, now: float) -> float:
        return 0.0 if self.aligned_since is None else now - self.aligned_since


class LaserManager:
    def __init__(self, cfg: Config, robot: Robot) -> None:
        self.cfg, self.robot = cfg, robot
        self.shots = 0
        self.wasted = 0
        self._shot_start: Optional[float] = None
        self._run = 0.0           # 이번 발의 현재 연속 정렬 시간
        self._best_run = 0.0      # 이번 발의 최대 연속 정렬 시간
        self._prev_on = False
        self._last_aligned_t: Optional[float] = None
        self.last_shot_best_run = 0.0

    def ready(self) -> bool:
        s = self.robot.status()
        return s.ammo > 0 and not s.laser_on and s.cooldown_left <= 0 and s.disabled_left <= 0 and s.control is not False

    @property
    def ammo(self) -> int:
        return self.robot.status().ammo

    def firing(self) -> bool:
        return self.robot.status().laser_on

    def fire(self, now: float) -> bool:
        if not self.ready():
            return False
        if self.robot.laser_fire():
            self.shots += 1
            self._shot_start = now
            self._run = self._best_run = 0.0
            self._last_aligned_t = None
            return True
        return False

    def abort(self) -> None:
        self.robot.laser_abort()

    def track(self, aligned: bool, now: float, dt: float) -> None:
        """발광 중 매 틱 호출: 연속 정렬 시간을 갱신하고, 발이 끝나면 낭비 여부를 기록."""
        on = self.robot.status().laser_on
        if on:
            if aligned:
                self._run += dt
                self._best_run = max(self._best_run, self._run)
                self._last_aligned_t = now
            elif self._last_aligned_t is None or now - self._last_aligned_t > 0.25:   # 0.25 s 유예(수광 쪽 유예와 동일)
                self._run = 0.0
        if self._prev_on and not on:
            self.last_shot_best_run = self._best_run
            if self._best_run < self.cfg.laser.dwell_s:
                self.wasted += 1
        self._prev_on = on

    def dwell_ok(self) -> bool:
        return self._best_run >= self.cfg.laser.dwell_s
