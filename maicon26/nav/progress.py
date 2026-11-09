"""Mission progress and bounded recovery, independent of the navigation backend.

Only estimated route length and observed task outcomes enter this monitor. A
changed command, wheel motion, or replanning by itself is not mission progress.
Normal observation/emission/phase waits suspend elapsed time rather than erase
an existing stall. Robot-specific deterministic choices require no team link.
"""
from __future__ import annotations

import hashlib
import math
from collections import deque
from dataclasses import dataclass
from typing import Any, Optional, Tuple


@dataclass(frozen=True)
class ProgressState:
    stage: int = 0
    actions: Tuple[str, ...] = ()
    stalled_s: float = 0.0
    progressed: bool = False
    goal_available: bool = True
    lock_until: float = 0.0
    reason: str = "initial"
    paused: bool = False


class ProgressMonitor:
    def __init__(self, cfg, slot: Any = None):
        self.cfg = cfg
        self.pc = cfg.progress
        self.slot = str(slot if slot is not None else (cfg.slot or 1))
        self.goal = None
        self._episode_active = False
        self._last_target = None
        self._last_t = None
        self._paused = False
        self._remaining_anchor = None
        self._face_count = 0
        self._shot_count = 0
        self._stalled = 0.0
        self._stage = 0
        self._backoff_issued = False
        self.failures = {}
        self.retry_at = {}
        self.excluded = set()
        self.lock_until = 0.0
        self._decision = None
        self._direction = 0
        self._flips = deque()
        self._reversals = deque()
        self.state = ProgressState()

    def eligible(self, goal, now: float) -> bool:
        return goal not in self.excluded and now >= self.retry_at.get(goal, -math.inf)

    def jittered(self, base: float, key: Any = "wait") -> float:
        """Stable per-slot variation, independent of call order/Python hash seed."""
        return float(base) * (1.0 + self.pc.jitter * (2.0 * self.tie_break(key) - 1.0))

    def tie_break(self, key: Any) -> float:
        digest = hashlib.sha256((self.slot + ":" + str(key)).encode("utf-8")).digest()
        return int.from_bytes(digest[:8], "big") / float(2 ** 64)

    def note_decision(self, now: float, posture, level: int, direction: float = 0.0) -> float:
        """Return the decision commitment deadline after excessive reversals.

        direction is signed translational intent; zero does not clear the last
        nonzero direction (a stop between forward/reverse still is a reversal).
        This is a diagnostic fallback. PostureController prevents a fourth
        ordinary posture transition before it is applied.
        """
        decision = (str(getattr(posture, "value", posture)), int(level))
        if self._decision is not None and decision != self._decision:
            self._flips.append(now)
        self._decision = decision
        sign = 1 if direction > 1e-4 else -1 if direction < -1e-4 else 0
        if sign and self._direction and sign != self._direction:
            self._reversals.append(now)
        if sign:
            self._direction = sign
        cutoff = now - self.pc.oscillation_window_s
        for history in (self._flips, self._reversals):
            while history and history[0] < cutoff:
                history.popleft()
        if now >= self.lock_until and (len(self._flips) > self.pc.max_decision_flips or
                                       len(self._reversals) > self.pc.max_direction_reversals):
            self.lock_until = now + self.cfg.posture.commit_s
            self._flips.clear()
            self._reversals.clear()
        return self.lock_until

    def reset(self, now: Optional[float] = None) -> None:
        """Explicit task/abandon boundary, never an ordinary route change.

        Retains goal failure history and global observed progress counters.
        """
        self.goal = None
        self._episode_active = False
        self._last_target = None
        self._last_t = now
        self._paused = False
        self._remaining_anchor = None
        self._stalled = 0.0
        self._stage = 0
        self._backoff_issued = False
        self.state = ProgressState(lock_until=self.lock_until)

    def update(self, now: float, goal_key, remaining_m: Optional[float], *,
               face_count: int = 0, shot_count: int = 0, arrived: bool = False,
               paused: bool = False, physically_blocked: bool = False) -> ProgressState:
        now = float(now)
        if not math.isfinite(now):
            raise ValueError("progress clock must be finite")
        if self._last_t is not None and now < self._last_t:
            raise ValueError("progress clock moved backwards; reset for a new match")
        finite_remaining = (remaining_m is not None and math.isfinite(float(remaining_m)))
        remaining = max(0.0, float(remaining_m)) if finite_remaining else None
        changed = goal_key != self.goal
        dt = 0.0 if self._last_t is None else now - self._last_t
        initial = not self._episode_active and goal_key is not None
        new_attempt = (self._stage >= 4 and goal_key is not None and
                       self.eligible(goal_key, now))
        if initial or new_attempt:
            self._episode_active = True
            self._stalled = 0.0
            self._stage = 0
            self._backoff_issued = False
            self._remaining_anchor = remaining
            dt = 0.0
            if initial:
                self._face_count = max(self._face_count, face_count)
                self._shot_count = max(self._shot_count, shot_count)
        elif changed:
            # Distances to different targets cannot be compared. Everything
            # else belongs to the ongoing lack-of-progress episode: frequent
            # goal churn must still reach the recovery/abandon thresholds.
            self._remaining_anchor = remaining
        self.goal = goal_key
        if goal_key is not None:
            self._last_target = goal_key
        reason = ("new_attempt" if new_attempt else "initial" if initial else
                  "goal_changed" if changed else "no_progress")
        progressed = False
        if self._episode_active:
            if self._remaining_anchor is None and remaining is not None:
                self._remaining_anchor = remaining
            gained = (remaining is not None and self._remaining_anchor is not None and
                      self._remaining_anchor - remaining >= self.pc.min_gain_m - 1e-9)
            if arrived or face_count > self._face_count or shot_count > self._shot_count or gained:
                progressed = True
                reason = ("arrived" if arrived else "new_face" if face_count > self._face_count else
                          "shot_progress" if shot_count > self._shot_count else "route_gain")
                self._stalled, self._stage, self._backoff_issued = 0.0, 0, False
                self._remaining_anchor = remaining
            elif not paused and not self._paused and self.pc.enabled:
                self._stalled += dt
        else:
            reason = "no_goal"
        self._face_count = max(self._face_count, face_count)
        self._shot_count = max(self._shot_count, shot_count)
        self._last_t, self._paused = now, paused
        actions = []
        if not paused and not progressed and self._episode_active and self.pc.enabled:
            thresholds = ((self.pc.stall_replan_s, "replan"),
                          (self.pc.stall_engage_s, "engage"),
                          (self.pc.stall_backoff_s, "backoff"),
                          (self.pc.stall_abandon_s, "abandon"))
            for i, (threshold, action) in enumerate(thresholds, 1):
                if self._stalled + 1e-9 >= threshold and self._stage < i:
                    self._stage = i
                    if action != "backoff" or physically_blocked:
                        actions.append(action)
                    if action == "backoff" and physically_blocked:
                        self._backoff_issued = True
                    if action == "abandon":
                        # A momentary empty selection must not erase the
                        # outstanding failed target or evade its cooldown.
                        failed_goal = self._last_target
                        self.failures[failed_goal] = self.failures.get(failed_goal, 0) + 1
                        self.retry_at[failed_goal] = now + self.pc.retry_after_s
                        if self.failures[failed_goal] >= self.pc.max_goal_failures:
                            self.excluded.add(failed_goal)
            # If obstruction becomes visible after the 9-second threshold,
            # still issue its one backoff instead of missing the recovery.
            if self._stage == 3 and physically_blocked and not self._backoff_issued:
                actions.append("backoff")
                self._backoff_issued = True
        if paused:
            reason = "normal_wait"
        elif actions:
            reason = actions[-1]
        self.state = ProgressState(self._stage, tuple(actions), self._stalled, progressed,
                                   self.eligible(goal_key, now), self.lock_until, reason, paused)
        return self.state
