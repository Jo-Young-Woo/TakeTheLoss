"""내장 스택(backend="builtin"): 내장 SLAM(파티클필터 + 점유격자) + 내장 A* 항법기.

ROS 없이 돌고, 시뮬 회귀시험과 'Nav2/SLAM 노드가 죽었을 때의 폴백'에 쓴다.
SLAM 동작: 매 스캔마다 (1) 오도메트리로 예측 → (2) 외벽 + 지금까지 만든 박스 지도에 매칭해 자세 보정
→ (3) 보정된 자세로 지도 갱신. 지도는 로그오즈라 박스가 옮겨지면 새 위치가 즉시 점유되고
옛 위치는 LiDAR가 통과해 보는 몇 회 안에 비워진다.
"""
from __future__ import annotations

from typing import Any, Dict, Optional, Tuple

from ..arena import ArenaGeometry
from ..config import Config
from ..types import Pose2D, Scan
from .localize import MapLikelihood, PoseEstimator, init_pose_from_scan
from .navigator import Navigator
from .stack import NavOut, NavStack

Pt = Tuple[float, float]


class LocalStack(NavStack):
    def __init__(self, cfg: Config, seed: int = 0) -> None:
        self.cfg = cfg
        self.geo: ArenaGeometry = cfg.arena
        self.pe = PoseEstimator(self.geo, seed=seed)
        self.navigator = Navigator(cfg, seed=seed)
        self.grid = self.navigator.grid
        self.field: Optional[MapLikelihood] = MapLikelihood() if cfg.nav.slam else None
        self._field_t = -1e9

    # ------------------------------------------------------------------ 상태
    @property
    def initialized(self) -> bool:                           # type: ignore[override]
        return self.pe.initialized

    @property
    def pose(self) -> Pose2D:                                # type: ignore[override]
        return self.pe.pose

    @property
    def replans(self) -> int:                                # type: ignore[override]
        return self.navigator.replans

    @property
    def recoveries(self) -> int:                             # type: ignore[override]
        return self.navigator.recoveries

    def spread(self) -> float:
        return self.pe.spread()

    def diagnostics(self) -> Dict[str, Any]:
        return {"replans": self.replans, "recoveries": self.recoveries,
                "invalidations": self.navigator.invalidations, "spread": round(self.spread(), 3),
                "map_box_cells": self.field.n_cells if self.field else 0}

    # ------------------------------------------------------------------ 시작
    def start(self, scan: Scan, odom: Pose2D, xy_guess: Pt) -> Pose2D:
        p0 = init_pose_from_scan(scan, self.geo, xy_guess[0], xy_guess[1])
        self.pe.init(p0, odom, sd_xy=0.05, sd_th=0.05)
        return p0

    # ------------------------------------------------------------------ SLAM 한 틱
    def update(self, odom: Pose2D, scan: Optional[Scan], now: float) -> None:
        self.pe.predict(odom)
        if scan is None:
            return
        if self.field is not None and now - self._field_t >= 0.5:
            self.field.refresh(self.grid)
            self._field_t = now
        self.pe.correct(scan, field=self.field)
        self.navigator.update_map(scan, self.pe.pose)

    # ------------------------------------------------------------------ 항법(내장 A*)
    def set_goal(self, goal: Pt, tol: Optional[float] = None, speed_scale: float = 1.0) -> None:
        self.navigator.set_goal(goal, tol, speed_scale)

    def clear_goal(self) -> None:
        self.navigator.clear_goal()

    def set_hint(self, turn_pref: Optional[float], allow_back: bool = True) -> None:
        self.navigator.set_hint(turn_pref, allow_back)

    def step(self, scan: Optional[Scan], now: float, dt: float) -> NavOut:
        return self.navigator.step(self.pe.pose, scan, now, dt)

    def limit(self, scan: Optional[Scan], v: float, w: float, now: float) -> Tuple[float, float]:
        return self.navigator._limit(scan, v, w, now)
