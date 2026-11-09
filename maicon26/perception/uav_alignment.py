"""Conservative translation alignment of aerial and LiDAR box footprints.

The aerial image is calibrated in the arena frame. SLAM can drift relative to
that frame. This estimator aligns aerial navigation priors to independently
observed *full* LiDAR footprints; it never changes SLAM, robot pose, images, or
the original aerial measurements. Partial faces cannot establish an offset.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Optional, Tuple

import numpy as np

from ..link.base import UavBox


@dataclass(frozen=True)
class UavAlignment:
    dx: float
    dy: float
    matches: int
    rms: float
    pairs: Tuple[Tuple[int, int], ...] = ()  # indexes in filtered aerial / LiDAR inputs


def _footprints(items: Iterable, *, lidar: bool) -> np.ndarray:
    rows = []
    for item in items:
        if lidar and getattr(item, "kind", None) != "full":
            continue
        try:
            row = [float(getattr(item, key)) for key in ("cx", "cy", "sx", "sy")]
            confidence = float(getattr(item, "confidence", 1.0))
        except (AttributeError, TypeError, ValueError):
            continue
        if (all(math.isfinite(value) for value in row + [confidence]) and
                confidence >= .6 and all(.25 <= value <= 1.0 for value in row[2:])):
            rows.append(row)
    return np.asarray(rows, dtype=float).reshape(-1, 4)


def estimate_uav_translation(aerial: Iterable, lidar: Iterable, *,
                             max_offset: float = .5,
                             match_gate: float = .10,
                             size_tolerance: float = .10) -> Optional[UavAlignment]:
    """Estimate ``LiDAR centre = aerial centre + (dx, dy)`` or return None.

    Hypotheses require at least two unique, size-compatible pairs separated by
    one metre. Two-pair solutions must agree within 3 cm RMS; three or more may
    have up to 6 cm RMS. Similarly supported alternative translations are
    rejected, so repeated symmetric structures cannot invent a correction.
    Callers may retain a previously validated offset when this returns None.
    """
    if any(not math.isfinite(v) or v <= 0 for v in (max_offset, match_gate, size_tolerance)):
        raise ValueError("Alignment limits must be positive finite metres")
    a, b = _footprints(aerial, lidar=False), _footprints(lidar, lidar=True)
    if len(a) < 2 or len(b) < 2:
        return None
    compatible = np.max(np.abs(a[:, None, 2:] - b[None, :, 2:]), axis=2) <= size_tolerance
    offsets = b[None, :, :2] - a[:, None, :2]
    valid = compatible & (np.linalg.norm(offsets, axis=2) <= max_offset)
    hypotheses = offsets[valid]
    if not len(hypotheses):
        return None

    def pairs_for(offset):
        residual = np.linalg.norm(offsets - offset[None, None, :], axis=2)
        possible = np.argwhere(compatible & (residual <= match_gate))
        possible = sorted(possible, key=lambda p: residual[p[0], p[1]])
        pairs, used_a, used_b = [], set(), set()
        for ai, bi in possible:
            ai, bi = int(ai), int(bi)
            if ai not in used_a and bi not in used_b:
                pairs.append((ai, bi))
                used_a.add(ai)
                used_b.add(bi)
        return pairs

    candidates = []
    for hypothesis in hypotheses:
        pairs = pairs_for(hypothesis)
        if len(pairs) < 2:
            continue
        offset = np.median([offsets[ai, bi] for ai, bi in pairs], axis=0)
        if np.linalg.norm(offset) > max_offset:
            continue
        pairs = pairs_for(offset)
        if len(pairs) < 2:
            continue
        centres = a[[ai for ai, _ in pairs], :2]
        spread = float(np.linalg.norm(centres[:, None, :] - centres[None, :, :], axis=2).max())
        if spread < 1.0:
            continue
        errors = np.asarray([offsets[ai, bi] - offset for ai, bi in pairs])
        rms = float(np.sqrt(np.mean(np.sum(errors * errors, axis=1))))
        if rms > (.03 if len(pairs) == 2 else .06):
            continue
        candidate = UavAlignment(float(offset[0]), float(offset[1]), len(pairs), rms, tuple(pairs))
        if not any(math.hypot(candidate.dx - old.dx, candidate.dy - old.dy) < .025
                   and candidate.pairs == old.pairs for old in candidates):
            candidates.append(candidate)
    if not candidates:
        return None
    candidates.sort(key=lambda result: (-result.matches, result.rms))
    best = candidates[0]
    for alternative in candidates[1:]:
        if (alternative.matches == best.matches and alternative.rms <= best.rms + .025
                and math.hypot(alternative.dx - best.dx, alternative.dy - best.dy) > .10):
            return None
    return best


def transform_uav_boxes(boxes: Iterable[UavBox], alignment: Optional[UavAlignment]) -> list[UavBox]:
    """Return shifted copies; raw aerial positions remain available for reporting."""
    dx, dy = (alignment.dx, alignment.dy) if alignment is not None else (0.0, 0.0)
    return [UavBox(box.cx + dx, box.cy + dy, box.sx, box.sy, box.confidence) for box in boxes]
