"""정체(접촉) 감지: '전진하라고 계속 명령했는데 실제로는 거의 안 움직였다'.

Agent(mission/manager.py _stall_check)와 현장 실험 F07(field/experiments.py)이 같은 로직을 쓴다.
위치는 SLAM/스캔 정합 자세처럼 '헛도는 바퀴에 속지 않는' 값이어야 한다(바퀴 오도메트리는 막혀도 이동으로 잰다).
"""
from __future__ import annotations

import math
from collections import deque
from typing import Deque, Dict, Optional, Tuple


class StallDetector:
    def __init__(self, window_s: float = 1.5, v_min: float = 0.08, ratio: float = 0.3) -> None:
        self.window_s, self.v_min, self.ratio = window_s, v_min, ratio
        self._h: Deque[Tuple[float, float, float, float, float]] = deque(maxlen=800)   # (t, x, y, v_cmd, dt)
        self.last_ratio: Optional[float] = None           # 가장 최근 창의 (실제 이동 / 기대 이동). 현장 실험이 분포를 본다

    def reset(self) -> None:
        self._h.clear()

    def update(self, now: float, dt: float, x: float, y: float, v_cmd: float) -> Optional[Dict[str, float]]:
        """매 틱 호출. 정체로 판정되면 {'expected', 'moved', 'ratio'} (이력은 비움), 아니면 None."""
        h = self._h
        h.append((now, x, y, v_cmd, dt))
        if now - h[0][0] < self.window_s:
            return None
        while now - h[0][0] > self.window_s * 1.3:
            h.popleft()
        if any(e[3] < self.v_min for e in h):             # 창 내내 전진 명령이었을 때만
            self.last_ratio = None
            return None
        expected = sum(e[3] * e[4] for e in h)
        moved = math.hypot(x - h[0][1], y - h[0][2])
        self.last_ratio = moved / expected if expected > 1e-9 else None
        if expected <= 1e-9 or moved >= self.ratio * expected:
            return None
        out = {"expected": expected, "moved": moved, "ratio": moved / expected}
        h.clear()
        return out
