"""상태 전이표(MAICON2026_상태전이표_경쟁상태반영.html)의 순수 로직 부품. 로봇·시뮬 없이 단위 시험이 된다.

MatchLatch  START 1회 래치 · STOP 종단 래치 · match_id/순번 검증                      (R09, 순위 00·03)
SensorWatch LiDAR 신선도 감시 — '도착 시각' 기준이라 센서 시계와 무관                  (R04, 순위 04·05)
Locks       피격 잠금을 '발사/틸트'와 '주행'으로 분리. 하드웨어 상태가 최종 권한,
            이벤트가 먼저 오면 짧은 유예(grace) 동안만 보수적으로 잠근다              (R08)
"""
from __future__ import annotations

import math
from typing import Dict, Optional, Tuple

from ..link.base import ServerMsg
from ..types import RobotStatus


class MatchLatch:
    """서버 메시지의 수용 여부를 정한다. 같은 경기에서 START는 한 번, STOP은 취소할 수 없다."""

    def __init__(self) -> None:
        self.match_id: Optional[str] = None
        self.started = False
        self.stopped = False
        self.last_seq: Dict[str, int] = {}
        self.rejected: Dict[str, int] = {}

    def _no(self, why: str) -> Tuple[bool, str]:
        self.rejected[why] = self.rejected.get(why, 0) + 1
        return False, why

    def accept(self, m: ServerMsg) -> Tuple[bool, str]:
        d = m.data or {}
        mid = d.get("match_id")
        if self.stopped:
            return self._no("after-stop")                    # 이전 경기의 늦은 이벤트·중복 START 포함
        if mid is not None and self.match_id is not None and mid != self.match_id:
            return self._no("other-match")
        seq = d.get("seq")
        if seq is not None:
            last = self.last_seq.get(m.kind)
            if last is not None and seq <= last:
                return self._no("stale-seq")
            self.last_seq[m.kind] = seq
        if m.kind == "start":
            if self.started:
                return self._no("dup-start")
            self.started = True
            if mid is not None:
                self.match_id = mid
        elif m.kind == "stop":
            self.stopped = True
        return True, "ok"


class SensorWatch:
    """새 스캔이 '도착한' 시각으로 신선도를 본다(스캔 stamp는 센서 시계라 비교하지 않고 변화만 본다)."""

    def __init__(self, stale_s: float) -> None:
        self.stale_s = stale_s
        self._stamp: Optional[float] = None
        self.t_arrival: Optional[float] = None
        self.n_new = 0

    def on_scan(self, stamp: float, now: float) -> None:
        if stamp != self._stamp:
            self._stamp, self.t_arrival = stamp, now
            self.n_new += 1

    def age(self, now: float) -> float:
        return math.inf if self.t_arrival is None else now - self.t_arrival

    def fresh(self, now: float) -> bool:
        return self.age(now) <= self.stale_s


class Locks:
    """weapon_tilt_locked / drive_locked. 하드웨어 값(disabled_left, drive_paused_left)이 있으면 그것이 권한."""

    def __init__(self, grace_s: float) -> None:
        self.grace_s = grace_s
        self._hits = 0
        self._weapon_grace = -1e9
        self._drive_grace = -1e9

    def on_hit_event(self, now: float) -> None:
        self._weapon_grace = max(self._weapon_grace, now + self.grace_s)

    def update(self, st: RobotStatus, now: float) -> int:
        """새 피격 수를 돌려준다. 첫 3회는 발사/틸트만, 4회째부터는 주행도 (현장 펌웨어 규칙으로 최종 확인)."""
        new = max(st.hits_taken - self._hits, 0)
        if new:
            self._weapon_grace = max(self._weapon_grace, now + self.grace_s)
            if st.hits_taken >= 4:
                self._drive_grace = max(self._drive_grace, now + self.grace_s)
        self._hits = st.hits_taken
        return new

    def weapon_locked(self, st: RobotStatus, now: float) -> bool:
        return st.disabled_left > 0 or now < self._weapon_grace

    def drive_locked(self, st: RobotStatus, now: float) -> bool:
        return st.drive_paused_left > 0 or now < self._drive_grace
