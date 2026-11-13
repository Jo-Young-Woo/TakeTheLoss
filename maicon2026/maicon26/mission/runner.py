"""경기마다 새 Agent 를 만들어 연속 경기를 돌리는 러너 (코드 검토 ⑨).

한 팀이 여러 경기에 나가고 경기마다 표적·객체·거점이 재배치된다. Agent 는 DONE 뒤 재초기화를 두지 않는다 —
START 래치·지도·객체 집계·UGV 트랙·제출 상태가 모두 한 경기 분량이라, **경기마다 새 인스턴스**를 만드는 것이 가장 안전하다
(제자리 리셋은 빠뜨린 상태가 다음 경기로 새기 쉽다). 러너가 하는 일:
  1. make_agent(link) 로 새 Agent 를 만들고 DONE 까지 틱을 돌린다.
  2. 끝난 Agent 의 stack(Nav2 목표·노드)을 닫는다.
  3. 정지·보고 중에 먼저 도착한 '다음 경기' 메시지(Agent.carry_over)를 새 Agent 에 먼저 넘긴다(ReplayLink).
  4. 경기 요약을 남긴다.
하지 않는 것: SLAM(Cartographer) 재시작과 로봇을 시작 위치에 놓는 일 — 운영 절차다. 지도는 Agent 가 쓰지 않지만 Cartographer 의
map→odom 이 경기 사이에 사람이 로봇을 옮기면 어긋날 수 있으므로 경기마다 SLAM launch 를 다시 올리는 것이 안전하다.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Tuple

from ..link.base import ServerLink, ServerMsg, UavFrame
from .manager import Agent, Mode


class ReplayLink(ServerLink):
    """먼저 넘겨 받은 메시지를 첫 poll 에 돌려주고, 나머지는 원래 링크에 맡긴다."""

    def __init__(self, inner: ServerLink, msgs: List[ServerMsg]) -> None:
        self.inner = inner
        self._pending: List[ServerMsg] = list(msgs)

    def poll(self) -> List[ServerMsg]:
        out, self._pending = self._pending, []
        return out + self.inner.poll()

    def submit(self, kind: str, payload: Dict[str, Any]) -> bool:
        return self.inner.submit(kind, payload)

    def submit_async(self, kind: str, payload: Dict[str, Any], token: str) -> None:
        self.inner.submit_async(kind, payload, token)

    def submit_results(self) -> List[Tuple[str, bool]]:
        return self.inner.submit_results()

    def get_uav(self) -> Optional[UavFrame]:
        return self.inner.get_uav()

    def identity(self) -> Optional[Tuple[str, int]]:
        return self.inner.identity()

    def health(self) -> Tuple[bool, str]:
        return self.inner.health()


@dataclass
class MatchSummary:
    index: int
    match_id: Optional[str]
    stop_source: Optional[str]
    m6_result: Optional[str]
    carried: int = 0                       # 다음 경기로 넘긴 메시지 수
    modes: Dict[str, float] = field(default_factory=dict)


class MatchRunner:
    def __init__(self, make_agent: Callable[[ServerLink], Agent], link: ServerLink, tick_period: float = 0.05,
                 max_matches: Optional[int] = None, sleep: Callable[[float], None] = time.sleep,
                 clock: Callable[[], float] = time.monotonic,
                 after_tick: Optional[Callable[[Agent], None]] = None,
                 on_match_done: Optional[Callable[[MatchSummary, Agent], None]] = None) -> None:
        self.make_agent, self.link = make_agent, link
        self.period, self.max_matches = tick_period, max_matches
        self.sleep, self.clock = sleep, clock
        self.after_tick, self.on_match_done = after_tick, on_match_done
        self.results: List[MatchSummary] = []
        self.agent: Optional[Agent] = None

    def run(self, should_quit: Callable[[], bool] = lambda: False) -> List[MatchSummary]:
        carry: List[ServerMsg] = []
        while not should_quit() and (self.max_matches is None or len(self.results) < self.max_matches):
            srv = ReplayLink(self.link, carry) if carry else self.link
            agent = self.agent = self.make_agent(srv)
            while agent.mode != Mode.DONE and not should_quit():
                t0 = self.clock()
                agent.tick()
                if self.after_tick is not None:
                    self.after_tick(agent)
                self.sleep(max(0.0, self.period - (self.clock() - t0)))
            finished = agent.mode == Mode.DONE
            try:
                agent.stack.close()                    # Nav2 목표 취소·노드 정리(예외가 다음 경기를 막으면 안 된다)
            except Exception:
                pass
            carry = list(agent.carry_over)
            summary = MatchSummary(len(self.results) + 1, agent.latch.match_id, agent.stop_source, agent.m6_result,
                                   len(carry), {k: round(v, 1) for k, v in agent.stats["mode_time"].items()})
            self.results.append(summary)
            if self.on_match_done is not None:
                self.on_match_done(summary, agent)
            if not finished:
                break
        return self.results
