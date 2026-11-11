"""Published numerical rules (participant guide, 2026-11-04 snapshot).

These pure functions accept measurements or estimates supplied by their caller.
They expose no referee state to the robot and do not implement provisional M6
tie equivalence or an assumed physical M5 sensor dwell time.
"""

M4_POINTS = (15, 12, 8, 3, 0)
M3_SECTOR_POINTS = 7
M3_COMMON_POINTS = 9
M3_MAX_POINTS = 30


def danger_score(lives: float, ammo: float, m3_points: float) -> float:
    """S11.2: equal-weight normalized terms, each floored at zero."""
    return max(0.0, lives) / 4.0 + max(0.0, ammo) / 8.0 + max(0.0, m3_points) / M3_MAX_POINTS


def m4_points(hits_taken: int) -> int:
    return M4_POINTS[min(4, max(0, int(hits_taken)))]


def m6_points(submitted, expected) -> int:
    """S11.1: rank positions matter; no special provisional tie treatment."""
    matches = [i < len(submitted) and i < len(expected) and submitted[i] == expected[i] for i in range(3)]
    return 15 if all(matches) else 10 if matches[0] else 5 if any(matches[1:]) else 0


def alliance_bonus(blue_total: float, red_total: float):
    """S9.2: each winning-alliance team gets five; a draw gives both five."""
    return {"BLUE": 5 if blue_total >= red_total else 0, "RED": 5 if red_total >= blue_total else 0}
