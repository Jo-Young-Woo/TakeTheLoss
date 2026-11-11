"""사전 서브게임(본선 전 평가): 180° 뒤돌아 시작 → 약 3 m 앞 레이더 시설을 찾아 타격. 60 s, 2회 시도 중 빠른 기록.

경기용 Agent와 달리 지도·자기위치추정 없이 '카메라 + LiDAR 안전'만으로 돈다 — 가장 단순해야 가장 빠르다.
  SEARCH  제자리 회전(+팬 스윕)으로 표적 클래스가 보일 때까지 돈다. 회전 방향은 한 번 정하면 유지.
  APPROACH 표적 중심을 화면 중앙으로 유지하며 strike_range까지 접근 (LiDAR 전방 안전 정지).
  AIM/FIRE 비주얼 서보로 수광창 정렬 → 정렬 유지 후 발사. 서버 'target_hit' 이벤트(또는 LED 검출)로 성공 판정.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional

from ..config import Config
from ..hal.base import Robot
from ..link.base import ServerLink
from ..perception.camera import CameraModel, estimate_range
from ..perception.detector import Detector, SimDetector
from ..util import clamp, wrap
from .aim import AimController, LaserManager


class Phase(str, Enum):
    WAIT = "WAIT"
    SEARCH = "SEARCH"
    APPROACH = "APPROACH"
    AIM = "AIM"
    DONE = "DONE"
    TIMEOUT = "TIMEOUT"


@dataclass
class SubgameResult:
    success: bool = False
    t_hit: Optional[float] = None
    shots: int = 0
    phase_log: List[tuple] = field(default_factory=list)


class SubgameRunner:
    def __init__(self, robot: Robot, server: Optional[ServerLink] = None, cfg: Optional[Config] = None,
                 detector: Optional[Detector] = None, target_cls: str = "RADAR_FACILITY",
                 limit_s: float = 60.0) -> None:
        self.robot, self.server = robot, server
        self.cfg = cfg or Config()
        self.cam = CameraModel(self.cfg.camera)
        self.detector = detector or SimDetector()
        self.aim = AimController(self.cfg)
        self.laser = LaserManager(self.cfg, robot)
        self.target_cls, self.limit_s = target_cls, limit_s
        self.phase = Phase.WAIT
        self.res = SubgameResult()
        self.t0: Optional[float] = None
        self._last: Optional[float] = None
        self._spin_dir = 0.0
        self._lost_since: Optional[float] = None
        self._frame_stamp = -1.0
        self._dets: list = []
        self._yaw_prev: Optional[float] = None
        self._yaw_acc = 0.0
        self._creep_until = 0.0
        self._creep_stage = 0
        self._creep_goal = 0.0

    def _set(self, now: float, p: Phase) -> None:
        if p != self.phase:
            self.phase = p
            self.res.phase_log.append((round(now - (self.t0 if self.t0 is not None else now), 2), p.value))

    def start(self) -> None:
        """서브게임 시작 신호(또는 수동 호출) — 타이머 시작."""
        self.t0 = self.robot.now()
        self._set(self.t0, Phase.SEARCH)

    def tick(self) -> Phase:
        now = self.robot.now()
        dt = 0.05 if self._last is None else clamp(now - self._last, 1e-3, 0.2)
        self._last = now
        if self.server is not None:
            for m in self.server.poll():
                if m.kind == "start" and self.t0 is None:
                    self.start()
                elif m.kind == "event" and m.data.get("type") == "target_hit":
                    self._hit(now)
                elif m.kind == "stop":
                    self._finish(now, False)
        if self.phase in (Phase.WAIT, Phase.DONE, Phase.TIMEOUT):
            if self.phase != Phase.WAIT:
                self.robot.set_velocity(0.0, 0.0)
            return self.phase
        if now - self.t0 > self.limit_s:
            self._finish(now, False)
            return self.phase
        fr = self.robot.get_frame()
        if fr is not None and fr.stamp != self._frame_stamp:
            self._frame_stamp = fr.stamp
            self._dets = self.detector.detect(fr)
        det = None
        c = [d for d in self._dets if d.cls == self.target_cls and d.conf >= self.cfg.perception.conf_min]
        if c:
            det = min(c, key=lambda d: abs(d.cx - self.cam.w / 2))
        scan = self.robot.get_scan()
        pan, tilt = self.robot.get_pan_tilt()
        # ---- 분기
        if det is None:
            self.laser.track(False, now, dt)
            self._lost_since = self._lost_since if self._lost_since is not None else now
            if self.phase in (Phase.APPROACH, Phase.AIM) and now - self._lost_since > 1.2 and not self.laser.firing():
                self._set(now, Phase.SEARCH)
            if self.phase == Phase.SEARCH:
                if self._spin_dir == 0.0:
                    self._spin_dir = 1.0
                th = self.robot.get_odom().th
                if self._yaw_prev is not None:
                    self._yaw_acc += abs(wrap(th - self._yaw_prev))
                self._yaw_prev = th
                if self._yaw_acc >= 2 * math.pi * 1.02 and self._creep_stage == 0 and scan is not None:
                    # 한 바퀴 돌아도 안 보임 → 표적이 인식 거리 밖일 수 있다: 가장 트인 방향으로 돌아 조금 전진한 뒤 다시 한 바퀴
                    self._yaw_acc = 0.0
                    self._creep_stage = 1
                    self._creep_goal = th + self._open_bearing(scan)
                if self._creep_stage == 1:
                    e = wrap(self._creep_goal - th)
                    self.robot.set_velocity(0.0, clamp(2.5 * e, -1.6, 1.6))
                    if abs(e) < 0.12:
                        self._creep_stage, self._creep_until = 2, now + 1.8
                elif self._creep_stage == 2:
                    front = scan.min_in_sector(0.0, 0.35) if scan is not None else 9.0
                    self.robot.set_velocity(0.0 if front < 0.45 else 0.3, 0.0)
                    if now >= self._creep_until:
                        self._creep_stage = 0
                        self._yaw_prev = None
                else:
                    self.robot.set_velocity(0.0, self._spin_dir * self.cfg.mission.spin_w)
                self.robot.set_pan_tilt(35.0 * math.sin(2.2 * now), 0.0)
            else:
                self.robot.set_velocity(0.0, 0.0)
            return self.phase
        self._lost_since = None
        # 표적이 보임 → 마지막 본 방향으로 회전 방향 고정(다시 놓쳤을 때)
        r, _ = estimate_range(self.cam, det, 0.40, scan, pan, 0.0)
        pan_c, tilt_c, ar = self.aim.update(det, pan, tilt, now, r)
        bearing = self.cam.bearing(det.cx, det.cy)
        self._spin_dir = 1.0 if bearing + math.radians(pan) >= 0 else -1.0
        sr = self.cfg.mission.strike_range
        near = r is not None and r <= sr + 0.15
        self._set(now, Phase.AIM if near else Phase.APPROACH)
        v = 0.0
        if r is not None and not self.laser.firing():
            v = clamp(0.9 * (r - sr), -0.10, self.cfg.nav.v_cruise)
            if abs(ar.err_pan) > 20.0:
                v = 0.0
        if scan is not None and v > 0:
            front = scan.min_in_sector(-0.35, 0.35)
            if front is not None and front < 0.30:
                v = 0.0
        self.robot.set_velocity(v, ar.body_w + clamp(1.2 * math.radians(pan), -1.0, 1.0) * 0.0)
        self.robot.set_pan_tilt(pan_c, tilt_c)
        self.laser.track(ar.aligned, now, dt)
        ready_r = r is not None and self.cfg.laser.min_range <= r <= self.cfg.laser.max_range
        if (ar.aligned and ar.sensor_ok and self.aim.held_for(now) >= self.cfg.aim.hold_before_fire and ready_r
                and self.laser.ready() and not self.laser.firing()):
            self.laser.fire(now)
        self.res.shots = self.laser.shots
        return self.phase

    @staticmethod
    def _open_bearing(scan) -> float:
        """스캔에서 가장 멀리까지 트인 방향(차체 기준 rad). ±20° 평균으로 외톨이 값을 거른다."""
        import numpy as np
        r = np.where(scan.valid(), scan.ranges, scan.range_max)
        k = max(1, int(round(math.radians(20) / abs(scan.angle_inc))))
        sm = np.convolve(np.r_[r[-k:], r, r[:k]], np.ones(2 * k + 1) / (2 * k + 1), mode="valid")
        return float(scan.angles[int(np.argmax(sm))])

    def _hit(self, now: float) -> None:
        self._finish(now, True)

    def _finish(self, now: float, ok: bool) -> None:
        if self.phase in (Phase.DONE, Phase.TIMEOUT):
            return
        self.res.success = ok
        self.res.t_hit = (now - self.t0) if (ok and self.t0 is not None) else None
        self.res.shots = self.laser.shots
        self._set(now, Phase.DONE if ok else Phase.TIMEOUT)
        self.robot.set_velocity(0.0, 0.0)
        self.robot.laser_abort()
