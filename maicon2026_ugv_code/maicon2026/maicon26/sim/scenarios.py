"""적 UGV 회피 시나리오 빌더(시뮬 전용). bench.run_trial(setup=...) 에 넘긴다.

각 시나리오는 start_t 에 우리 로봇 기준 (거리, 방위)로 적대 상대를 *나타나게* 한다(박스에 가려지지 않는 자리로 보정).
박스 배치·경로가 시드마다 달라서 '고정 좌표 만남'은 가려져 시험이 안 되기 때문이다.

!! 상대 행동 모델은 가정이다(완벽한 위치 정보로 쫓아와 사정거리 안에서 조준·발사 등). 실기 성능이 아니다.
"""
from __future__ import annotations

import math
from typing import Callable, List, Optional, Sequence

from ..types import Pose2D
from .hostile import Blocker, Chaser, Hostile, Patroller

OUR_ID = "B-UGV-2"


def _park(world) -> Pose2D:
    cx, cy = world.geo.base_center("R-1")
    return Pose2D(cx, cy, math.pi)


def _trigger(name):
    if name == "strike":                    # 지정표적 조준(정지) 중에 등장
        return lambda m: m.agents[0].mode.value == "STRIKE" and m.agents[0]._strike.get("phase") == "aim"
    if name == "goal":                      # M5 거점 접근 중에 등장
        return lambda m: m.agents[0].mode.value == "GOAL"
    return None


def encounter(kind: str = "chaser", bearing_deg: float = 0.0, dist: float = 2.8, t_trig: float = 70.0, v_max: float = 0.35,
              shoot: bool = True, react_s: float = 0.6, n: int = 1, trigger: Optional[str] = None) -> Callable:
    """kind: chaser(추격) | blocker(정지 진로 차단) | cross(횡단). bearing_deg: 우리 헤딩 기준 방위(0 정면, +90 왼쪽, 180 뒤)."""
    def setup(m) -> Sequence[Hostile]:
        w = m.world
        out: List[Hostile] = []
        for i in range(n):
            brg = bearing_deg + 25.0 * i
            kw = dict(v_max=v_max, react_s=react_s, shoot=shoot, start_t=t_trig, spawn_rel=(dist + 0.5 * i, brg), trigger=_trigger(trigger))
            if kind == "chaser":
                h: Hostile = Chaser(w, "RED", 1 + i, _park(w), OUR_ID, **kw)
            elif kind == "blocker":
                h = Blocker(w, "RED", 1 + i, _park(w), OUR_ID, **kw)
            elif kind == "cross":
                h = Patroller(w, "RED", 1 + i, _park(w), OUR_ID, aim=True, cross=True, **kw)
            else:
                raise ValueError(kind)
            out.append(h)
        return out
    return setup


# 이름 붙은 시나리오(설계서 §8.3 의 S1~S3, S9, S10 에 해당하는 것 — 시뮬 가능한 범위)
SCENARIOS = {
    "S1_front_chaser": dict(kind="chaser", bearing_deg=0.0, t_trig=70.0),
    "S1b_front_blocker": dict(kind="blocker", bearing_deg=0.0, t_trig=70.0, dist=2.2),
    "S2_cross": dict(kind="cross", bearing_deg=90.0, t_trig=70.0),
    "S3_rear_chaser": dict(kind="chaser", bearing_deg=180.0, t_trig=70.0),
    "S3b_side_chaser": dict(kind="chaser", bearing_deg=90.0, t_trig=70.0),
    "S9_m2_mid_chaser": dict(kind="chaser", bearing_deg=60.0, dist=2.2, t_trig=1.0, trigger="strike"),     # 지정표적 조준 시작 1 s 뒤에 옆에서 접근
    "S10_m5_near_blocker": dict(kind="blocker", bearing_deg=0.0, dist=1.9, t_trig=5.0, trigger="goal"),   # M5 접근 5 s 뒤 앞에 나타나 막는다
}


def build(name: str, **over) -> Callable:
    d = dict(SCENARIOS[name])
    d.update(over)
    return encounter(**d)
