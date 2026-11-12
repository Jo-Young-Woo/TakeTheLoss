"""LiDAR 안전 제한 — 항법 백엔드(자체 A* / Nav2)와 무관하게 항상 적용된다.

"LiDAR 안전 정지는 임무 관리자·Nav2 보다 우선한다." Nav2 컨트롤러가 어떤 속도를 내든,
최종 명령은 Agent가 이 제한기를 거친 뒤 로봇에 보낸다(Nav2 출력은 /cmd_vel 이 아니라
/cmd_vel_nav2 로 받는다 — ros2/README.md 참고).
"""
from __future__ import annotations

import math
import numpy as np
from typing import Optional, Tuple

from ..config import Config
from ..types import Scan


class SafetyLimiter:
    def __init__(self, cfg: Config) -> None:
        self.n = cfg.nav
        self.rob = cfg.robot.radius
        self.budget = cfg.budget()
        self.decel = max(cfg.robot.a_max, .1)
        self.swept_stops = 0
        self._front_blocked_since: Optional[float] = None
        self.blocked_long = False          # 직전 호출에서 '오래 막힘'이 판정되었는가(호출자가 복구 동작을 시작하는 신호)
        self.blocked_turn = 1.0            # 복구 때 도는 방향(트인 쪽)
        self.turn_pref: Optional[float] = None   # 회피 계층의 힌트(+1 왼쪽/-1 오른쪽). 있으면 '트인 쪽' 대신 이 방향으로 돈다(위협 쪽으로 돌지 않게). None=기존 동작

    def limit(self, scan: Optional[Scan], v: float, w: float, now: float) -> Tuple[float, float]:
        self.blocked_long = False
        if scan is None:
            return v, w
        n = self.n
        front = scan.min_in_sector(0.0, math.radians(35))      # 진행 방향 쪽 전방 부채꼴(±35°)
        if v > 0:
            if front < n.stop_dist:
                v = 0.0
                left = scan.min_in_sector(math.radians(70), math.radians(35))
                right = scan.min_in_sector(-math.radians(70), math.radians(35))
                w = 0.9 if left >= right else -0.9              # 가장 트인 쪽으로 제자리 회전
                if self.turn_pref is not None:
                    w = 0.9 if self.turn_pref > 0 else -0.9      # 회피 힌트가 있으면 그쪽(적에게서 멀어지는 방향)
                if self._front_blocked_since is None:
                    self._front_blocked_since = now
                elif now - self._front_blocked_since > 2.0:     # 오래 막히면 호출자가 복구 모드로
                    self.blocked_long = True
                    self.blocked_turn = 1.0 if left >= right else -1.0
                    self._front_blocked_since = None
            else:
                self._front_blocked_since = None
                if front < n.slow_dist:
                    v *= (front - n.stop_dist) / (n.slow_dist - n.stop_dist)
                lft = scan.min_in_sector(math.radians(80), math.radians(25))      # 측면이 가까우면 살짝 비켜난다
                rgt = scan.min_in_sector(-math.radians(80), math.radians(25))
                lim = n.side_lim                                  # 반폭 + 정지 여유(footprint.ClearanceBudget.stop_side) — 40 cm 통로 중앙(0.20 m)에서는 작동하지 않는다
                if lft < lim:
                    w -= 1.2 * (lim - lft) / lim
                if rgt < lim:
                    w += 1.2 * (lim - rgt) / lim
        elif v < 0:
            # 후진도 같은 기준으로 지킨다(예전엔 전진만 보호했다). 후방 ±35° 의 반사가 정지 거리 안이면 멈추고, 감속 거리 안이면 줄인다.
            self._front_blocked_since = None
            rear = scan.min_in_sector(math.pi, math.radians(35))
            if rear < n.stop_dist:
                v = 0.0
            elif rear < n.slow_dist:
                v *= (rear - n.stop_dist) / (n.slow_dist - n.stop_dist)
        return self.limit_swept(scan, v, w)

    def limit_swept(self, scan: Scan, v: float, w: float) -> Tuple[float, float]:
        """Check the moving rectangle in the *base* scan frame, independent of
        UAV/SLAM registration. Include reaction time and braking distance.
        A nearby return may be left behind, but must never be driven deeper
        into the body margin. This allows backing out of an existing contact.
        """
        valid = scan.valid()
        if not valid.any():
            # Explicit no-return (+inf) is clear; all-NaN is not a clear scan.
            return (v, w) if np.isposinf(scan.ranges).any() else (0., 0.)
        fp = self.budget.fp
        horizon = .20 + max(abs(v) / (2 * self.decel), abs(w) / 8.)
        pad = self.budget.sensor_margin
        reach = fp.circumradius + pad + abs(v) * horizon + .03
        valid &= scan.ranges < reach
        if not valid.any():
            return v, w
        a, r = scan.angles[valid], scan.ranges[valid]
        px, py = r * np.cos(a), r * np.sin(a)
        initial = np.maximum(np.abs(px) - fp.half_l, np.abs(py) - fp.half_w)
        threshold = np.minimum(pad, initial) - .001

        def safe(cv, cw):
            # <= 1cm translation and <= 2deg corner motion per step.
            count = max(4, int(math.ceil(abs(cv) * horizon / .01)),
                        int(math.ceil(abs(cw) * horizon / math.radians(2))))
            ts = np.linspace(horizon / count, horizon, count)[:, None]
            th = cw * ts
            c, s = np.cos(th), np.sin(th)
            x = cv * ts if abs(cw) < 1e-6 else cv / cw * s
            y = np.zeros_like(ts) if abs(cw) < 1e-6 else cv / cw * (1 - c)
            dx, dy = px[None, :] - x, py[None, :] - y
            clearance = np.maximum(np.abs(c * dx + s * dy) - fp.half_l,
                                   np.abs(-s * dx + c * dy) - fp.half_w)
            return bool(np.all(clearance >= threshold[None, :]))

        if safe(v, w):
            return v, w
        self.swept_stops += 1
        # Dropping an unsafe turn preserves a straight escape/corridor route.
        if abs(v) > 1e-6 and safe(v, 0.):
            return v, 0.
        if abs(w) > 1e-6 and safe(0., w):
            return 0., w
        return 0., 0.
