"""항법 스택 추상화 — '위치추정(SLAM) + 지도 + 경로계획·추종(Nav2)'을 한 덩어리로 Agent에 제공한다.

경기장 구조물(박스)은 경기마다, 심지어 경기 중에도 바뀔 수 있으므로 미리 만든 지도에 의존하지 않는다.
  - 위치추정·지도: SLAM이 실시간으로 만든다 (실기: Cartographer/slam_toolbox, 시뮬: 내장 격자 SLAM)
  - 경로계획·추종: 비용지도가 센서로 계속 갱신되는 Nav2 (시뮬: 같은 인터페이스의 자체 구현)
Agent는 이 인터페이스만 본다. 백엔드는 config.nav.backend 로 고른다.

좌표계: 모든 공개 값(pose, grid, set_goal)은 '경기장 좌표계'(원점=왼쪽 아래 모서리, arena.py)다.
SLAM의 map 프레임과의 변환은 Nav2Stack 내부에서 처리한다.
"""
from __future__ import annotations

import math
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple

from ..types import Pose2D, Scan
from .grid import OccupancyGrid

Pt = Tuple[float, float]


@dataclass
class NavOut:
    v: float = 0.0
    w: float = 0.0
    status: str = "idle"          # idle | moving | reached | blocked | recovering
    dist_to_goal: float = math.inf


class NavStack(ABC):
    initialized: bool = False
    pose: Pose2D
    grid: OccupancyGrid           # 경기장 좌표계 점유격자 (박스 탐색·시점 선정용; 계획은 백엔드가 자체 비용지도로)
    replans: int = 0
    recoveries: int = 0

    @abstractmethod
    def start(self, scan: Scan, odom: Pose2D, xy_guess: Pt) -> Optional[Pose2D]:
        """첫 스캔으로 경기장 좌표계의 시작 자세를 정하고 SLAM을 앵커링한다. 시작 거점 근처 xy_guess 필요.
        SLAM이 아직 준비되지 않았으면 None(Agent가 다음 틱에 다시 부른다)."""

    @abstractmethod
    def update(self, odom: Pose2D, scan: Optional[Scan], now: float) -> None:
        """매 틱 호출. scan은 '새 스캔일 때만' 넘긴다(없으면 None)."""

    @abstractmethod
    def set_goal(self, goal: Pt, tol: Optional[float] = None, speed_scale: float = 1.0) -> None: ...

    @abstractmethod
    def clear_goal(self) -> None: ...

    @abstractmethod
    def step(self, scan: Optional[Scan], now: float, dt: float) -> NavOut: ...

    @abstractmethod
    def limit(self, scan: Optional[Scan], v: float, w: float, now: float) -> Tuple[float, float]:
        """LiDAR 안전 제한(조준 모드 등 항법 밖의 구동에도 쓴다)."""

    def set_hint(self, turn_pref: Optional[float], allow_back: bool = True) -> None:
        """회피 계층의 안전 힌트(기본 구현은 아무것도 안 한다). turn_pref: 막혔을 때 도는 방향(+왼쪽/-오른쪽, None=기존),
        allow_back: False 면 복구 동작에서 후진하지 않는다. 경로계획(Nav2/A*)을 바꾸지 않고 '복구 동작의 방향'만 제약한다."""

    def set_obstacle_layer(self, mask: Optional[Any]) -> None:
        """3D 지도(깊이 카메라)에서 투영한 장애물 마스크(격자와 같은 모양, bool)를 계획에 반영한다. 기본 구현은 Agent 용 격자에만 더한다."""
        self.grid.set_layer3d(mask)

    def set_keepout_layer(self, mask: Optional[Any]) -> bool:
        """Planning-only centre keepout, distinct from perceived obstacle cells."""
        return self.grid.set_keepout(mask)

    def spread(self) -> float:
        """위치 불확실도(m). 모르면 0."""
        return 0.0

    def diagnostics(self) -> Dict[str, Any]:
        return {"replans": self.replans, "recoveries": self.recoveries}

    def close(self) -> None:
        self.clear_goal()
