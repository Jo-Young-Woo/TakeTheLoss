"""Threat intensity and mission posture are separate, sensor-driven axes.

ENGAGE_OK permits accepting exposure while advancing; it does not enable UGV
hunting or bypass physical collision protection. No simulator state is read.
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from enum import Enum
from typing import Optional, Tuple


class Posture(str, Enum):
    AVOID = "AVOID"
    ENGAGE_OK = "ENGAGE_OK"
    COVER = "COVER"


@dataclass(frozen=True)
class PostureState:
    posture: Posture = Posture.AVOID
    level: int = 0
    risk_weight: float = 0.5
    allow_evade: bool = False
    cap_exposed_dwell: bool = False
    reason: str = "initial"
    transitions: Tuple[str, ...] = ()
    engage_reasons: Tuple[str, ...] = ()
    lock_until: float = 0.0


class PostureController:
    def __init__(self, cfg):
        self.cfg = cfg
        self.level = 0
        self.posture = Posture.AVOID
        self._calm_since = None
        self._changed_at = -math.inf
        self._commit_origin = None
        self._last_xy = None
        self._commit_distance = 0.0
        self._flips = deque()
        self._lock_until = 0.0
        self.state = PostureState(risk_weight=float(cfg.risk.w["L0"]))

    def update(self, now: float, *, xy=(0.0, 0.0), exposure: float = 0.0,
               hits: int = 0, visible_threat: bool = False, in_cm: bool = False,
               in_goal: bool = False, goal_distance: float = math.inf,
               invulnerable: bool = False, stall_stage: int = 0, elapsed: float = 0.0,
               unfinished_scoring: bool = False, lock_until: float = 0.0) -> PostureState:
        pc, tc, rc = self.cfg.posture, self.cfg.threat, self.cfg.risk
        before = (self.posture, self.level)
        cutoff = now - self.cfg.progress.oscillation_window_s
        while self._flips and self._flips[0] < cutoff:
            self._flips.popleft()
        self._lock_until = max(self._lock_until, lock_until)
        minimum_hits = int(self.cfg.avoid.evade_min_hits)
        e1, e2, e3 = tc.level_up
        target = (3 if exposure >= e3 and hits >= minimum_hits else
                  2 if exposure >= e2 or hits >= minimum_hits else
                  1 if exposure >= e1 or hits >= 1 or visible_threat else 0)
        transitions = []
        reason = "unchanged"
        if target > self.level:
            previous = self.level
            self.level = target
            self._calm_since = None
            transitions.append(f"level:{previous}->{self.level}:threat_rise")
        elif target < self.level:
            if self._calm_since is None:
                self._calm_since = now
            if now - self._calm_since >= tc.level_down_s - 1e-9:
                # Delay only de-escalation when the next ordinary change
                # would exceed the shared 10-second change budget. Escalation
                # above remains immediate even during a commitment: accepting
                # a more dangerous input is never traded for a prettier log.
                if now < self._lock_until:
                    reason = "level_oscillation_commit"
                elif len(self._flips) >= self.cfg.progress.max_decision_flips:
                    self._lock_until = now + pc.commit_s
                    reason = "level_oscillation_commit"
                else:
                    previous = self.level
                    self.level -= 1
                    # Each lower step needs its own continuously calm interval.
                    self._calm_since = now if target < self.level else None
                    transitions.append(f"level:{previous}->{self.level}:calm_hold")
        else:
            self._calm_since = None

        reasons = []
        if pc.engage_in_cm and in_cm:
            reasons.append("cm")
        if stall_stage >= 2:
            reasons.append("stall")
        if invulnerable:
            reasons.append("invulnerable")
        if pc.engage_in_goal and in_goal:
            reasons.append("goal")
        if (pc.engage_time_pressure_at is not None and elapsed >= pc.engage_time_pressure_at
                and unfinished_scoring):
            reasons.append("time_pressure")

        # Only net displacement counts towards the commitment distance. A
        # forward/reverse loop must not satisfy a 0.6 m advancement commitment.
        if self._commit_origin is not None:
            self._commit_distance = math.hypot(xy[0] - self._commit_origin[0],
                                               xy[1] - self._commit_origin[1])
        exception = invulnerable or (in_goal and goal_distance <= pc.goal_exception_m)
        hit_guard = hits >= pc.engage_exit_hits and not exception
        desired = (Posture.ENGAGE_OK if reasons and not hit_guard else
                   Posture.COVER if self.level >= 3 and hits >= minimum_hits else Posture.AVOID)
        safety_exit = self.posture == Posture.ENGAGE_OK and hit_guard
        if safety_exit:
            desired = Posture.AVOID
            reason = "hit_limit"
        elif desired == Posture.ENGAGE_OK:
            reason = "+".join(reasons)
        elif desired == Posture.COVER:
            reason = "level3_cover"
        elif self.posture == Posture.ENGAGE_OK:
            reason = "engage_reason_cleared"

        changing = desired != self.posture
        if changing and not safety_exit:
            dwell_done = now - self._changed_at >= pc.min_dwell_s - 1e-9
            commit_done = (now - self._changed_at >= pc.commit_s - 1e-9 or
                           self._commit_distance >= pc.commit_dist_m - 1e-9)
            if self.posture == Posture.ENGAGE_OK and not commit_done:
                desired, reason = self.posture, "commit_hold"
            elif not dwell_done:
                desired, reason = self.posture, "posture_dwell"
            elif now < self._lock_until:
                desired, reason = self.posture, "oscillation_commit"
            elif len(self._flips) >= self.cfg.progress.max_decision_flips:
                self._lock_until = now + pc.commit_s
                desired, reason = self.posture, "oscillation_commit"

        # COVER is defined only at L3. If the level settles lower, release the
        # ownership immediately; retaining its name would conceal an invalid
        # state and needlessly prolong the old EVADE maneuver.
        if self.posture == Posture.COVER and self.level < 3 and desired == Posture.COVER:
            desired, reason = Posture.AVOID, "cover_level_cleared"
        if desired != self.posture:
            previous = self.posture
            self.posture = desired
            self._changed_at = now
            self._commit_origin = (float(xy[0]), float(xy[1]))
            self._commit_distance = 0.0
            transitions.append(f"posture:{previous.value}->{desired.value}:{reason}")
        if (self.posture, self.level) != before:
            # A simultaneous posture and level change is one decision update,
            # matching the independent trace monitor's (posture, level) pair.
            self._flips.append(now)

        weight = float(rc.w[f"L{self.level}"])
        if self.posture == Posture.ENGAGE_OK:
            weight = min(weight, float(rc.w["engage"]))
        allow_evade = self.posture == Posture.COVER and self.level == 3 and hits >= minimum_hits
        cap_dwell = (visible_threat and (hits >= rc.dwell_cap_from_hits or
                                        self.posture == Posture.ENGAGE_OK))
        self.state = PostureState(self.posture, self.level, weight, allow_evade,
                                  cap_dwell, reason, tuple(transitions), tuple(reasons), self._lock_until)
        return self.state


@dataclass(frozen=True)
class FireDecision:
    allowed: bool
    reason: str


def blocker_fire_allowed(cfg, *, now: float, posture, stall_stage: int,
                         affiliation: str, confidence: float, distance_m: float,
                         dome_visible: bool, friendly_in_line: bool, weapon_locked: bool,
                         ammo: int, m2_complete: bool, last_target_shot: Optional[float] = None,
                         actually_blocking: bool = True) -> FireDecision:
    """A gate for a positively identified blocker, never a target selector.

    The caller checks the live receiver image and friendly/unknown angular
    guard, then uses the normal aim/laser path. True is permission, not a fire
    command. A fire attempt must reserve ammunition before this gate is reused.
    """
    ec = cfg.engage
    checks = (
        (ec.fire_on_blocker, "disabled"),
        (posture == Posture.ENGAGE_OK or posture == Posture.ENGAGE_OK.value, "posture"),
        (stall_stage >= 2 and actually_blocking, "not_blocked"),
        (affiliation.lower() == "enemy", "not_confirmed_enemy"),
        (math.isfinite(confidence) and confidence >= ec.enemy_confidence, "low_confidence"),
        (math.isfinite(distance_m) and ec.blocker_min_range_m <= distance_m <= ec.blocker_max_range_m, "range"),
        (dome_visible, "no_receiver"),
        (not friendly_in_line, "friendly_fire_guard"),
        (not weapon_locked, "locked"),
        (ammo > (0 if m2_complete else ec.m2_ammo_reserve), "ammo_reserve"),
        (last_target_shot is None or now - last_target_shot >= ec.target_cooldown_s, "target_cooldown"),
    )
    for passed, reason in checks:
        if not passed:
            return FireDecision(False, reason)
    return FireDecision(True, "confirmed_blocker")
