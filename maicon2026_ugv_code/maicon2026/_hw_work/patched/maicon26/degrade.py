"""저하 모드 제어기(1장). /ready 를 보낸 뒤 경기 시작 전까지 1 s 마다 다시 분류해 저하를 풀고, 시작 순간 플래그를 고정한다.

  - 그사이 LiDAR 가 살아나면 정상 모드로 출발한다(플래그가 줄어든다). 새로 생긴 누락은 플래그를 늘린다.
  - 경기가 시작되는 순간 고정한다. 경기 중 복귀는 이번 범위 밖이다(프롬프트 14장).
  - 저하 중이면 5 s 마다 콘솔에 누락 항목을 찍는다. OLED 표시는 Agent._show_status 가 degraded_oled_text 로 한다.
  - 어떤 메서드도 예외를 던지지 않는다(Agent 틱 안에서 불린다).
"""
from __future__ import annotations

import time
from typing import Any, Callable, FrozenSet, Optional

from .preflight import ReadyState, classify_readiness, degraded_oled_text, not_ready_reasons, FLAG_NO_DRIVE


class DegradeController:
    def __init__(self, robot, stack, state: ReadyState, *, interval_s: float = 1.0, report_every_s: float = 5.0,
                 report: Optional[Callable[[str], None]] = None, clock: Callable[[], float] = time.monotonic) -> None:
        self.robot, self.stack = robot, stack
        self.state = state
        self.interval_s, self.report_every_s = float(interval_s), float(report_every_s)
        self._report = report or (lambda m: print(m, flush=True))
        self._clock = clock
        self.frozen = False
        self.rechecks = 0
        self._next_check = -float("inf")
        self._next_report = -float("inf")
        self.apply(state.flags)

    @property
    def flags(self) -> FrozenSet[str]:
        return frozenset(self.state.flags)

    def apply(self, flags) -> None:
        """플래그를 로봇(HAL 가드)과 항법 대체물에 알린다."""
        flags = frozenset(flags)
        setter = getattr(self.robot, "set_degraded", None)
        if callable(setter):
            setter(flags)
        no_drive = getattr(self.stack, "set_no_drive", None)
        if callable(no_drive):
            no_drive(FLAG_NO_DRIVE in flags)
        oled = degraded_oled_text(flags)
        show = getattr(self.robot, "show", None)
        if oled and callable(show) and not self.frozen:
            try:
                show(oled)
            except Exception:
                pass

    def update(self, state: ReadyState) -> None:
        """핸드셰이크 중 다시 잰 wait_ready 결과를 반영한다(저하가 풀리거나 늘 수 있다)."""
        if self.frozen:
            return
        if state.flags != self.state.flags:
            self._say(f"[저하] 플래그 변경: {sorted(self.state.flags)} → {sorted(state.flags)}")
        self.state = state
        self.apply(state.flags)

    def freeze(self) -> None:
        if not self.frozen:
            self.frozen = True
            self._say(f"[저하] 경기 시작 — 플래그 고정: {sorted(self.state.flags) or '없음(정상)'}")

    def tick(self, started: bool) -> None:
        """Agent 틱마다 부른다. started = 경기 시작 래치가 걸렸는가."""
        try:
            if self.frozen:
                return
            if started:
                self.freeze()
                return
            now = self._clock()
            if now < self._next_check:
                return
            self._next_check = now + self.interval_s
            self.rechecks += 1
            st = classify_readiness(not_ready_reasons(self.robot, self.stack))
            if st.flags != self.state.flags:
                self._say(f"[저하] 재확인 결과 플래그 변경: {sorted(self.state.flags)} → {sorted(st.flags)}")
                self.state = st
                self.apply(st.flags)
            else:
                self.state = st
            if st.degraded and now >= self._next_report:
                self._next_report = now + self.report_every_s
                self._say("[저하] 누락 항목: " + st.summary() + (f" → {sorted(st.flags)}" if st.flags else " (선택 항목만)"))
        except Exception as e:                              # noqa: BLE001
            try:
                self._say(f"[저하] 재확인 중 오류(무시): {type(e).__name__}")
            except Exception:
                pass

    def _say(self, msg: str) -> None:
        try:
            self._report(msg)
        except Exception:
            pass
