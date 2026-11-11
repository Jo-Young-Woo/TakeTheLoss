"""저하 모드 no_drive 의 항법 대체물(1장). rclpy 를 import 하지 않는다.

LiDAR·Nav2·map→base TF 중 하나라도 없이 시작하면 로봇은 움직이지 않는다(hal/wingbot._publish_cmd 가 항상 0 을 낸다).
그때 FSM 이 '주행 목표'에서 멈춰 버리지 않도록, 목표를 Nav2 로 보내지 않고 즉시 '도달 불가(blocked)'를 돌려주는 래퍼다.
M1·M6 제출과 M3 의 기존 제출 경로는 Agent 가 스택과 무관하게 계속 돌린다.

경기 시작 전에 LiDAR 가 살아나면 DegradeController 가 set_no_drive(False) 를 불러 정상 스택으로 돌아간다.
"""
from __future__ import annotations

from typing import Any, Dict, Optional, Tuple

from ..types import Pose2D, Scan
from .stack import NavOut, NavStack, Pt


class NoDriveStack(NavStack):
    def __init__(self, inner: NavStack, enabled: bool = True) -> None:
        self._inner = inner
        self.enabled = bool(enabled)
        self.goals_refused = 0

    def set_no_drive(self, on: bool) -> None:
        self.enabled = bool(on)
        if self.enabled:
            try:
                self._inner.clear_goal()          # 이미 보낸 목표가 있으면 거둔다
            except Exception:
                pass

    # ---- 안쪽 스택의 속성(grid, pose, client …)은 그대로 보여 준다
    def __getattr__(self, name: str) -> Any:
        if name in ("_inner", "enabled", "goals_refused"):
            raise AttributeError(name)
        return getattr(self._inner, name)

    @property
    def initialized(self) -> bool:                                    # type: ignore[override]
        return self._inner.initialized

    @property
    def grid(self):                                                   # type: ignore[override]
        return self._inner.grid

    @property
    def pose(self) -> Pose2D:                                         # type: ignore[override]
        return self._inner.pose

    @property
    def replans(self) -> int:                                         # type: ignore[override]
        return self._inner.replans

    @property
    def recoveries(self) -> int:                                      # type: ignore[override]
        return self._inner.recoveries

    # ---- 위치·지도 갱신은 그대로 위임
    def start(self, scan: Scan, odom: Pose2D, xy_guess: Pt) -> Optional[Pose2D]:
        return self._inner.start(scan, odom, xy_guess)

    def update(self, odom: Pose2D, scan: Optional[Scan], now: float) -> None:
        self._inner.update(odom, scan, now)

    # ---- 주행: 켜져 있으면 목표를 받지 않는다
    def set_goal(self, goal: Pt, tol: Optional[float] = None, speed_scale: float = 1.0) -> None:
        if self.enabled:
            self.goals_refused += 1
            return
        self._inner.set_goal(goal, tol, speed_scale)

    def clear_goal(self) -> None:
        if not self.enabled:
            self._inner.clear_goal()

    def step(self, scan: Optional[Scan], now: float, dt: float) -> NavOut:
        if self.enabled:
            return NavOut(0.0, 0.0, "blocked")                        # 목표 도달 불가를 즉시 돌려준다
        return self._inner.step(scan, now, dt)

    def limit(self, scan: Optional[Scan], v: float, w: float, now: float) -> Tuple[float, float]:
        if self.enabled:
            return 0.0, 0.0
        return self._inner.limit(scan, v, w, now)

    def set_hint(self, turn_pref: Optional[float], allow_back: bool = True) -> None:
        self._inner.set_hint(turn_pref, allow_back)

    def set_obstacle_layer(self, mask: Optional[Any]) -> None:
        self._inner.set_obstacle_layer(mask)

    def set_keepout_layer(self, mask: Optional[Any]) -> bool:
        return self._inner.set_keepout_layer(mask)

    def spread(self) -> float:
        return self._inner.spread()

    def diagnostics(self) -> Dict[str, Any]:
        d = dict(self._inner.diagnostics())
        d["no_drive_stack"] = {"enabled": self.enabled, "goals_refused": self.goals_refused}
        return d

    def close(self) -> None:
        self._inner.close()
