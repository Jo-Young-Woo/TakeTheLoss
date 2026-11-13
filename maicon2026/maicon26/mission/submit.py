"""제출 추적 (R10·R02). '계산 완료 · 전송 중 · ACK 확인'을 구분하고, 거부·유실은 기한 안에서 재시도한다.

상태: idle → pending → inflight → acked
                         └ 거부/유실 → retry → … → gave_up      cancelled(STOP 뒤 닫힌 종류)
서버가 False를 돌려줘도 '제출 완료'로 기록하지 않는다. 같은 작업의 재전송은 같은 token(= 멱등 키)을 쓴다.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Iterable, Optional

from ..link.base import ServerLink


@dataclass
class Job:
    kind: str
    payload: Dict[str, Any]
    token: str
    state: str = "pending"
    tries: int = 0
    next_t: float = 0.0
    t_sent: float = 0.0
    t_acked: Optional[float] = None
    deadline: Optional[float] = None
    max_tries: Optional[int] = None
    rejected: int = 0                    # 서버가 명시적으로 거부(False)한 횟수 — 응답 유실과 구분한다
    backoff: bool = False                # True 면 거부가 이어질수록 재시도 간격을 늘린다(최대 retry_s × 5)


class Submitter:
    """max_tries: 0 이하면 무제한(기한 deadline 이나 close_for_stop 까지 계속). M1/M3 는 STOP 전까지 계속 시도해야 한다 —
    와이파이가 몇 초 끊겼다고 영구 포기하면 그 임무 점수를 통째로 잃는다."""

    def __init__(self, server: ServerLink, retry_s: float = 1.0, ack_timeout_s: float = 3.0,
                 max_tries: int = 0) -> None:
        self.server = server
        self.retry_s, self.ack_timeout_s, self.max_tries = retry_s, ack_timeout_s, max_tries
        self.jobs: Dict[str, Job] = {}
        self.closed: set = set()
        self._n = 0

    # ------------------------------------------------------------------ 조회
    def state(self, kind: str) -> str:
        j = self.jobs.get(kind)
        return "idle" if j is None else j.state

    def started(self, kind: str) -> bool:
        return self.state(kind) != "idle"

    def acked(self, kind: str) -> bool:
        return self.state(kind) == "acked"

    def tries(self, kind: str) -> int:
        j = self.jobs.get(kind)
        return 0 if j is None else j.tries

    def rejections(self, kind: str) -> int:
        j = self.jobs.get(kind)
        return 0 if j is None else j.rejected

    # ------------------------------------------------------------------ 요청
    def request(self, kind: str, payload: Dict[str, Any], now: float, deadline: Optional[float] = None,
                max_tries: Optional[int] = None, force: bool = False, backoff: bool = False) -> bool:
        """새 제출 작업을 만든다. 이미 진행·완료된 작업이 있으면 force 일 때만 새로 만든다."""
        if kind in self.closed:
            return False
        j = self.jobs.get(kind)
        if j is not None and not force and j.state not in ("gave_up", "cancelled"):
            return False
        self._n += 1
        self.jobs[kind] = Job(kind, payload, f"{kind}-{self._n}", next_t=now, deadline=deadline,
                              max_tries=max_tries if max_tries is not None else self.max_tries, backoff=backoff)
        return True

    def revive(self, kind: str, now: float) -> bool:
        """한도(max_tries)·기한 때문에 포기한 작업을 같은 내용으로 다시 시작한다(STOP 으로 닫힌 종류는 안 된다)."""
        j = self.jobs.get(kind)
        if j is None or j.state != "gave_up" or kind in self.closed:
            return False
        return self.request(kind, j.payload, now, deadline=None, max_tries=j.max_tries, backoff=j.backoff)

    def close_for_stop(self, allowed: Iterable[str] = ("m6",)) -> None:
        """STOP 이후: 허용하지 않는 종류는 신규 제출·재시도를 모두 금지한다(전송 중인 응답은 계속 받는다)."""
        allowed = set(allowed)
        for k in ("m1", "m2", "m3", "m4", "m5", "m6"):
            if k not in allowed:
                self.close_kind(k)

    def close_kind(self, kind: str) -> None:
        """Close one mission before its receive-time deadline, preserving ACKs."""
        if kind not in self.closed:
            self.closed.add(kind)
            self.server.close_submission(kind)
        for j in self.jobs.values():
            if j.kind in self.closed and j.state in ("pending", "retry"):
                j.state = "cancelled"

    # ------------------------------------------------------------------ 한 틱
    def update(self, now: float) -> None:
        for token, ok in self.server.submit_results():
            j = next((x for x in self.jobs.values() if x.token == token), None)
            if j is None or j.state in ("acked", "cancelled", "gave_up"):
                continue
            if ok:
                j.state, j.t_acked = "acked", now
            else:
                j.rejected += 1
                gap = self.retry_s * (min(2 ** max(j.rejected - 1, 0), 5) if j.backoff else 1)
                j.state, j.next_t = "retry", now + gap
        for j in self.jobs.values():
            if j.state == "inflight" and now - j.t_sent > self.ack_timeout_s:
                j.state, j.next_t = "retry", now                   # 응답 유실 → 같은 token 으로 재전송
            if j.state not in ("pending", "retry") or now < j.next_t:
                continue
            if j.kind in self.closed:
                j.state = "cancelled"
            elif (j.deadline is not None and now >= j.deadline) or (j.max_tries and j.tries >= j.max_tries):
                j.state = "gave_up"
            elif self.server.submission_allowed(j.kind) is False:
                continue
            else:
                j.tries += 1
                j.state, j.t_sent = "inflight", now
                self.server.submit_async(j.kind, j.payload, j.token)
