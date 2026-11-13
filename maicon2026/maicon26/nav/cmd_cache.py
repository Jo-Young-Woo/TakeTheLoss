"""Nav2 속도 캐시 (R06). 목표를 취소·교체한 뒤에 도착한 '이전 목표의' 속도를 쓰지 않게 한다.

Nav2 의 /cmd_vel_nav2 메시지에는 목표 ID 가 없다. 그래서
  - 목표 전송·취소 때 캐시를 비우고 문을 닫는다(invalidate) — 이후 들어오는 속도는 버린다.
  - 서버가 새 목표를 수락했을 때만 문을 연다(open_gate) — 그 뒤의 속도만 채택한다.
  - 오래된 속도는 timeout 뒤 None.
None 은 호출자(Nav2Stack)가 (0, 0)으로 해석한다.
"""
from __future__ import annotations

import threading
from typing import Optional, Tuple


class CmdCache:
    def __init__(self, timeout_s: float) -> None:
        self.timeout_s = timeout_s
        self._lock = threading.Lock()
        self._cmd: Optional[Tuple[float, float, float]] = None
        self._open = False
        self.dropped = 0                 # 문이 닫혀 있어서 버린 속도 수(시험·진단용)
        self.accepted = 0                # 문이 열린 뒤 채택한 속도 수

    def put(self, v: float, w: float, t_recv: float) -> bool:
        with self._lock:
            if not self._open:
                self.dropped += 1
                return False
            self._cmd = (v, w, t_recv)
            self.accepted += 1
            return True

    def invalidate(self) -> None:
        with self._lock:
            self._cmd, self._open = None, False

    def open_gate(self) -> None:
        with self._lock:
            self._open = True

    def get(self, now: float) -> Optional[Tuple[float, float]]:
        with self._lock:
            c = self._cmd
        if c is None or now - c[2] > self.timeout_s:
            return None
        return (c[0], c[1])
