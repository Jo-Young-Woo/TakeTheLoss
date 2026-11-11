"""센서 stamp 의 시계 기준 맞추기.

Agent 는 센서 데이터를 '촬영 시각의 자세'와 짝짓는다(pose_at). 자세 이력은 robot.now() 시계로 쌓이므로 스캔·프레임의 stamp 도
**같은 시계**여야 한다. 실기에서는 쉽게 어긋난다: 로봇 시계는 time.monotonic()(부팅·시작 기준)인데 ROS 메시지 header.stamp 는
에포크(1.7e9 s) 거나 카메라 자체 시계일 수 있다. 어긋나면 pose_at 이 이력 밖의 시각을 받아 한쪽 끝 자세로 조용히 고정되고,
검출 위치가 틀어져도 오류가 나지 않는다.

계약(우선): HAL(hal/wingbot.py)은 Scan·Frame 의 stamp 를 **robot.now() 시계**로 채운다. header stamp 를 쓰려면 이 모듈로 변환한다.
안전망: Agent 도 같은 변환을 한 번 더 건다(시계가 같다고 판단되면 아무것도 바꾸지 않는다).

방법: 도착 시각과 stamp 의 차 d = now − stamp 를 모은다.
  - |중앙값 d| ≤ tol(1 s)  → 같은 시계. stamp 를 그대로 쓴다(지연 몇 십~백 ms 는 d 에 이미 들어 있다).
  - 그렇지 않으면 다른 시계. stamp + min(d) 로 변환한다 — min(d) = (시계 차) + (최소 전송 지연) 이다.
한계: 다른 시계일 때는 시계 차와 최소 지연을 분리할 수 없다. 변환 결과는 [촬영, 도착] 사이이고 오차는 '최소 지연'만큼이다.
(지연이 일정한 센서면 도착 시각과 같아진다.) 시뮬 실험(에포크 시계, 지연 50~250 ms, 실제 최소 지연 0.10 s, 제자리 회전 1.8 rad/s):
같은 시계 0.33° / 다른 시계(변환만) 4.9° / **최소 지연을 맞게 넣으면(floor_s=0.10) 0.5°** / 틀리게 넣으면(0.05) 틀린 만큼 남음(2.6°).
재는 법: 로봇을 일정한 각속도로 돌리며 정지한 표적을 검출해, 검출 방위의 오차가 0 이 되는 floor_s 를 찾는다(또는 LED 점멸을
로봇 시계와 비교). 설정: camera.min_latency_s, lidar.min_latency_s. 정확히 하려면 하드웨어 시각 동기(PTP/chrony)나 드라이버의
보정이 가장 좋다.
"""
from __future__ import annotations

import statistics
from collections import deque
from typing import Deque, Optional, Tuple


class StampSync:
    def __init__(self, window: int = 240, min_samples: int = 5, tol: float = 1.0, floor_s: float = 0.0) -> None:
        self._d: Deque[float] = deque(maxlen=window)
        self.min_samples, self.tol = min_samples, tol
        self.floor_s = floor_s                   # 알려진 최소 전송 지연(s). offset 모드에서 변환 결과에서 뺀다(촬영 시각에 더 가까워짐)
        self.mode = "unknown"                    # unknown | same | offset
        self.offset = 0.0                        # offset 모드에서 더하는 값(= min d)
        self.switches = 0                        # 모드가 바뀐 횟수(잦으면 시계가 불안정하다는 신호)
        self._changed: Optional[Tuple[str, float]] = None

    def to_robot(self, stamp: float, now: float) -> float:
        """센서 stamp → robot.now() 시계의 시각. `now` 는 이 메시지를 받은 시점의 robot.now()."""
        d = now - stamp
        self._d.append(d)
        if len(self._d) < self.min_samples:               # 아직 판단 전: 같은 시계로 보이면 그대로, 아니면 도착 시각(보수적)
            return stamp if abs(d) <= self.tol else now
        mode = "same" if abs(statistics.median(self._d)) <= self.tol else "offset"
        if mode != self.mode:
            self.mode = mode
            self.switches += 1
            self._changed = (mode, min(self._d))
        if mode == "same":
            return stamp
        self.offset = min(self._d) - self.floor_s          # = 시계 차(+ 최소 지연 중 알려진 floor 를 뺀 나머지)
        return stamp + self.offset

    def consume_change(self) -> Optional[Tuple[str, float]]:
        """모드가 바뀌었으면 (mode, offset) 을 한 번만 돌려준다(로그용)."""
        c, self._changed = self._changed, None
        return c
