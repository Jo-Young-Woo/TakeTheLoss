"""IMU 충돌 대응(ImpactResponder) — 충돌을 '무엇에, 어느 쪽에서, 움직이다 부딪혔나'로 나누고 행동을 정한다.

설계 요약 (설정은 config.ImpactCfg, 기본 꺼짐)
  1. 충돌이 오면 일단 멈춘다(SAFE_STOP "impact"): 작은 충격 hold_s, 큰 충격 hold_major_s, 연속 충돌(storm_n 번/storm_window_s)이면 storm_hold_s.
     그동안 PoseGuard 가 LiDAR 가 차체에서 돌아갔는지 검증한다(정지 상태에서만 잰다) — 검증이 끝나거나 verify_timeout_s 가 지날 때까지 재개하지 않는다.
  2. 무엇에 부딪혔나:
       - UGV   : 접촉 방향 ugv_contact_m 안에 UGV 트랙이 있다 → 장애물로 기록하지 않는다(움직인다). 후진·회전은 하되 적 쪽으로 돌지 않는다(_enter_recover 의 회피 힌트).
       - 보이는 장애물: 접촉 방향 LiDAR 가 contact_lidar_m 안에서 본다 → 이미 지도에 있다. 후진·재계획만.
       - 보이지 않는 장애물: 움직이다 부딪혔는데 LiDAR 가 접촉 방향을 비었다고 본다(낮은 물체·유리) → 그 자리를 '접촉 장애물'로 기록(manager._bumps)하고 재계획.
       - 모름   : 서 있다가 맞았거나 방향을 모른다 → 기록하지 않고 위치 검증과 재계획만.
  3. 멈춘 뒤 무엇을 하나(action):
       - backoff: 앞에서 부딪혔고 전진 중이었다(UGV 포함) → RECOVER(후진 후 트인 쪽으로 회전). 단, 목표 도착(M5) 중 작은 충격이면 시간이 아까워 replan.
       - replan : 그 밖 → 목표·경로·탐색 후보를 지우고 새로 정한다(Nav2 가 새 목표를 받는다).
     연속 충돌(storm)이면 목표를 다시 정하고 storm_window_s 동안 속도를 줄인다(speed_scale).
  4. 충격 직후(scan_skip_s)와 기울어진 동안(tilt_scan_skip_deg 초과)의 스캔은 Agent 격자·커버리지에 쌓지 않는다(차체가 흔들려 바닥·천장을 훑는다).

순수 계산(ROS·로봇 없이 시험 가능). manager 는 handle() 로 사건을 넘기고 hold_active()/followup()/speed_scale() 만 부른다.
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Deque, Dict, List, Optional, Sequence, Tuple

from ..util import wrap


@dataclass
class ImpactCtx:
    """충돌이 처리되는 순간의 상태(manager 가 채운다). 각도 rad, 거리 m, 속도 m/s."""
    pose: Any                                  # Pose2D (경기장 좌표)
    v_cmd: float = 0.0                         # 마지막 선속도 명령(앞 +)
    w_cmd: float = 0.0
    scan: Any = None                           # 차체 기준으로 보정된 스캔(없으면 None)
    ugv_xy: Sequence[Tuple[float, float]] = ()  # 최근 UGV 트랙 위치(경기장 좌표)
    mode: str = ""                             # 현재 임무 모드 이름(GOAL 이면 시간이 급하다)


@dataclass
class ImpactPlan:
    t: float
    sector: str                                # front | rear | left | right | unknown
    severity: str                              # minor | major
    src: str                                   # accel | gyro | yaw_step
    contact: str                               # ugv | visible | invisible | unknown
    moving: str                                # forward | backward | turning | still
    action: str                                # backoff | replan
    hold_s: float
    storm: bool
    bearing: Optional[float] = None            # 차체 기준 접촉 방위(rad)
    bump_xy: Optional[Tuple[float, float]] = None      # 보이지 않는 장애물의 경기장 좌표(기록할 것)
    bump_th: float = 0.0
    peak_dg: float = 0.0

    def text(self) -> str:
        b = "방향 미상" if self.bearing is None else f"{math.degrees(self.bearing):+.0f}°"
        return (f"충돌 {self.severity}({self.src}, {self.peak_dg:.2f} g) {self.sector}[{b}] {self.moving} 중 — "
                f"{ {'ugv': 'UGV 와 접촉', 'visible': 'LiDAR 에 보이는 장애물', 'invisible': 'LiDAR 에 안 보이는 장애물', 'unknown': '대상 불명'}[self.contact] } "
                f"→ {self.action}(정지 {self.hold_s:.2f}s{', 연속 충돌' if self.storm else ''})")


class ImpactResponder:
    def __init__(self, cfg: Any) -> None:
        self.c = cfg
        self.hold_until = -1e9
        self.verify_until = -1e9
        self._times: Deque[float] = deque(maxlen=16)
        self._pending: Optional[ImpactPlan] = None
        self._slow_until = -1e9
        self._skip_until = -1e9
        self.plans: List[ImpactPlan] = []                    # 기록(보고용, 최근 20개)
        self.stats: Dict[str, int] = {"events": 0, "major": 0, "sector_front": 0, "sector_rear": 0, "sector_side": 0, "sector_unknown": 0,
                                      "contact_ugv": 0, "contact_visible": 0, "contact_invisible": 0, "contact_unknown": 0,
                                      "backoff": 0, "replan": 0, "storm": 0, "scan_skipped": 0, "ignored_no_start": 0}

    # ------------------------------------------------------------------ 분류
    @staticmethod
    def _moving(ctx: ImpactCtx) -> str:
        if abs(ctx.v_cmd) > 0.05:
            return "forward" if ctx.v_cmd > 0 else "backward"
        if abs(ctx.w_cmd) > 0.25:
            return "turning"
        return "still"

    def _contact(self, plan_bearing: Optional[float], moving: str, ctx: ImpactCtx) -> Tuple[str, Optional[Tuple[float, float]], float]:
        """(대상 종류, 보이지 않는 장애물의 경기장 좌표, 그 방향). 방위를 모르면 'unknown'."""
        if plan_bearing is None:
            return "unknown", None, 0.0
        p = ctx.pose
        th = p.th + plan_bearing                                              # 접촉 방향(경기장 좌표)
        for (ux, uy) in ctx.ugv_xy:
            d = math.hypot(ux - p.x, uy - p.y)
            if 0.0 < d <= self.c.ugv_contact_m and abs(wrap(math.atan2(uy - p.y, ux - p.x) - th)) <= math.radians(60):
                return "ugv", None, th
        seen = ctx.scan.min_in_sector(plan_bearing, math.radians(25)) if ctx.scan is not None else None
        if seen is not None and seen < self.c.contact_lidar_m:
            return "visible", None, th
        if moving != "still":                                                 # 움직이다 부딪혔는데 LiDAR 는 비었다 = 평면 밖의 물체
            ahead = 0.22
            return "invisible", (p.x + ahead * math.cos(th), p.y + ahead * math.sin(th)), th
        return "unknown", None, th

    def handle(self, now: float, events: Sequence[Any], ctx: ImpactCtx, *, match_running: bool = True) -> Optional[ImpactPlan]:
        """충돌 이벤트(pose_guard.ImpactEvent) 묶음을 처리해 가장 센 것 하나의 계획을 낸다. 경기 시작 전이면 기록만 하고 멈추지 않는다."""
        if not events:
            return None
        ev = max(events, key=lambda e: (e.severity == "major", e.peak_dg))
        for e in events:
            self.stats["events"] += 1
            self._times.append(e.t)
        sector = ev.sector
        self.stats["major"] += int(ev.severity == "major")
        self.stats["sector_" + ("side" if sector in ("left", "right") else sector)] += 1
        self._skip_until = max(self._skip_until, now + self.c.scan_skip_s)
        if not match_running:
            self.stats["ignored_no_start"] += 1
            return None
        moving = self._moving(ctx)
        contact, bump_xy, bump_th = self._contact(ev.bearing, moving, ctx)
        self.stats["contact_" + contact] += 1
        storm = sum(1 for t in self._times if now - t <= self.c.storm_window_s) >= self.c.storm_n
        hold = self.c.hold_major_s if ev.severity == "major" else self.c.hold_s
        if storm:
            hold = max(hold, self.c.storm_hold_s)
            self._slow_until = now + self.c.storm_window_s
            self.stats["storm"] += 1
        # 앞에서 부딪혔고 전진 중이었거나(보이는/안 보이는/UGV 모두), 앞쪽 접촉인데 방위만 아는 경우 → 후진·회전. M5 중 작은 충격은 시간이 아깝다.
        action = "replan"
        if self.c.backoff and sector == "front" and moving in ("forward", "turning") and contact != "unknown":
            if not (ctx.mode == "GOAL" and ev.severity == "minor" and not storm):
                action = "backoff"
        if storm:
            action = "replan" if contact == "unknown" else action
        self.stats[action] += 1
        plan = ImpactPlan(t=ev.t, sector=sector, severity=ev.severity, src=ev.src, contact=contact, moving=moving, action=action,
                          hold_s=hold, storm=storm, bearing=ev.bearing, bump_xy=bump_xy, bump_th=bump_th, peak_dg=ev.peak_dg)
        self.hold_until = max(self.hold_until, now + hold)
        self.verify_until = max(self.verify_until, now + self.c.verify_timeout_s)
        self._pending = plan if (self._pending is None or ev.severity == "major" or action == "backoff") else self._pending
        self.plans.append(plan)
        del self.plans[:-20]
        return plan

    # ------------------------------------------------------------------ 상태
    def hold_active(self, now: float, verifying: bool = False) -> bool:
        """지금 멈춰 있어야 하나: 정지 시간 안이거나, PoseGuard 가 검증 중이고 검증 시간 한도 안이다."""
        return now < self.hold_until or (verifying and now < self.verify_until)

    def followup(self) -> Optional[ImpactPlan]:
        """멈춤이 끝난 뒤 한 번만 꺼내는 후속 계획(backoff|replan). 없으면 None."""
        p, self._pending = self._pending, None
        return p

    @property
    def pending(self) -> Optional[ImpactPlan]:
        return self._pending

    def speed_scale(self, now: float) -> float:
        return 0.6 if now < self._slow_until else 1.0

    def skip_scan(self, now: float, roll_deg: Optional[float], pitch_deg: Optional[float]) -> bool:
        """이 스캔을 Agent 격자·커버리지에 쌓지 말아야 하나(충격 직후 또는 기울어진 동안)."""
        tilt = max(abs(roll_deg or 0.0), abs(pitch_deg or 0.0))
        skip = now < self._skip_until or tilt > self.c.tilt_scan_skip_deg
        if skip:
            self.stats["scan_skipped"] += 1
        return skip

    def report(self) -> Dict[str, Any]:
        last = self.plans[-1] if self.plans else None
        return {**self.stats, "last": None if last is None else {"t": round(last.t, 2), "sector": last.sector, "severity": last.severity,
                                                                 "contact": last.contact, "action": last.action, "moving": last.moving}}
