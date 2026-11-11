"""Sensor-derived M3 arrival feasibility and one bounded refinement.

No world/target truth is consulted and this module never records an observation.
The manager may align the camera after arrival; feasibility uses that intended
heading. CoverageMap alone remains responsible for actual frame/view evidence.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

from maicon26.footprint import circle_cells_free

Pt = Tuple[float, float]


@dataclass
class ArrivalView:
    applicable: bool
    valid: bool
    heading: Optional[float] = None
    reason: str = "not_face"
    face_key: Optional[Tuple[int, int]] = None


def _planned_face(cand, coverage):
    """Preserve the planned subject; recover older/refreshed candidates safely.

    The incumbent refresh can legitimately clear primary_face while retaining
    pending faces. Choose by the ORIGINAL planned heading, not by whichever
    incidental face becomes closest after the arrival displacement.
    """
    def available(face):
        box = coverage.boxes.get(face.bid) if face is not None else None
        return face is not None and box is not None and coverage.box_available(box)
    primary = coverage.get_face(cand.primary_face) if cand.primary_face is not None else None
    if available(primary):
        return primary
    faces = [coverage.get_face(k) for k in cand.faces]
    faces = [f for f in faces if available(f)]
    if not faces:
        return None
    def planned_turn(face):
        delta = math.atan2(face.cy-cand.vp[1], face.cx-cand.vp[0])-cand.heading
        return abs(math.atan2(math.sin(delta), math.cos(delta))), face.key
    return min(faces, key=planned_turn)


def face_arrival_view(cand, pose, coverage, grid, occ=None) -> ArrivalView:
    if cand.kind != "face":
        return ArrivalView(False, True)
    face = _planned_face(cand, coverage)
    if face is None:
        return ArrivalView(True, False, reason="missing_face")
    box = coverage.boxes.get(face.bid)
    if box is None or not coverage.box_available(box):
        return ArrivalView(True, False, reason="unavailable_face")
    heading = math.atan2(face.cy - pose.y, face.cx - pose.x)
    occ = coverage.visibility_occupancy(grid) if occ is None else occ
    visible = coverage.picture_visible if coverage.picture_pending(face) else coverage.face_visible
    ok, reason = visible(pose.x, pose.y, heading, face, occ, grid)
    return ArrivalView(True, ok, heading, reason, face.key)


def refined_viewpoint(cand, pose, coverage, grid, eligible=None, tolerance=.03) -> Optional[Pt]:
    """Find one nearby, observed-free interior viewpoint; never relax visibility.

    A full rotation footprint plus configured planning margin must fit in
    LiDAR-observed free space. This deliberately defers uncertain approaches
    instead of inventing free space. Navigation still enforces dynamic safety.
    """
    from types import SimpleNamespace

    face = _planned_face(cand, coverage)
    if face is None:
        return None
    occ = coverage.visibility_occupancy(grid)
    # Unknown space is not an admissible precision-refinement footprint.
    unsafe = occ | (grid.L >= -.3)
    if grid.keepout is not None:
        unsafe |= grid.keepout
    radius = coverage.cfg.budget().plan_inflate_rotate
    guard = tolerance + .01
    iy0, ix0 = grid.to_cell(*cand.vp)
    span = int(math.ceil(.25 / grid.res))
    choices = []
    for iy in range(iy0 - span, iy0 + span + 1):
        for ix in range(ix0 - span, ix0 + span + 1):
            if not grid.in_bounds(iy, ix):
                continue
            p = grid.to_world(iy, ix)
            if math.hypot(p[0]-cand.vp[0], p[1]-cand.vp[1]) > .25:
                continue
            if math.hypot(p[0]-pose.x, p[1]-pose.y) > .4:
                continue
            if eligible is not None and not eligible(p):
                continue
            if not circle_cells_free(unsafe, grid.res, grid.ox, grid.oy, *p, radius+guard):
                continue
            valid = True
            # A small interior margin avoids repeating the .18 m stop error.
            for dx, dy in [(0., 0.)] + [(guard*math.cos(i*math.pi/4), guard*math.sin(i*math.pi/4)) for i in range(8)]:
                test_pose = SimpleNamespace(x=p[0]+dx, y=p[1]+dy)
                if not face_arrival_view(cand, test_pose, coverage, grid, occ).valid:
                    valid = False
                    break
            if not valid:
                continue
            dist = math.hypot(p[0]-face.cx, p[1]-face.cy)
            cosi = ((p[0]-face.cx)*face.nx + (p[1]-face.cy)*face.ny) / dist
            score = math.hypot(p[0]-pose.x, p[1]-pose.y) + .25*math.hypot(p[0]-cand.vp[0], p[1]-cand.vp[1]) + .05*(1-cosi)
            choices.append((score, p))
    return min(choices)[1] if choices else None
