"""Fixed MAICON infrastructure and seeded M3 cubes, shared with the referee.

Positions follow the supplied arena image.  A grouped 0.4 x 0.8 m visual
structure is one rectangular collision object, so both systems use its full
footprint.  Face indices are east, north, west, south.
"""
from __future__ import annotations

from dataclasses import dataclass
from collections import deque
from functools import lru_cache
import math
import random
from types import SimpleNamespace

import numpy as np

from ..config import CameraCfg, RobotCfg
from ..perception.camera import CameraModel


@dataclass(frozen=True)
class FixedStructure:
    name: str
    x: float
    y: float
    sx: float
    sy: float
    faces: tuple[tuple[int, str, str], ...]


STRUCTURES = (
    FixedStructure("B_upper_target", 1.60, 3.80, .40, .40,
                   ((2, "target", "GUARD_POST"),)),
    FixedStructure("B_upper_block", 2.00, 3.80, .40, .80,
                   ()),
    FixedStructure("R_upper_block", 4.12, 3.80, .40, .80,
                   ()),
    FixedStructure("R_upper_target", 4.52, 3.80, .40, .40,
                   ((0, "target", "GUARD_POST"),)),
    FixedStructure("B_middle_block", 1.70, 2.50, .80, .40,
                   ((2, "target", "COMM_FACILITY"),)),
    FixedStructure("R_middle_block", 4.42, 2.50, .80, .40,
                   ((0, "target", "COMM_FACILITY"),)),
    FixedStructure("B_lower_target", 1.60, 1.00, .40, .40,
                   ((2, "target", "RADAR_FACILITY"),)),
    FixedStructure("B_lower_block", 2.00, 1.15, .40, .80,
                   ()),
    FixedStructure("R_lower_block", 4.12, 1.15, .40, .80,
                   ()),
    FixedStructure("R_lower_target", 4.52, 1.00, .40, .40,
                   ((0, "target", "RADAR_FACILITY"),)),
    FixedStructure("common_north", 3.06, 4.40, .40, .40,
                   ()),
    FixedStructure("common_middle", 3.06, 2.50, .40, .40,
                   ()),
    FixedStructure("common_south", 3.06, .60, .40, .40,
                   ()),
)

BASE_TYPES = {
    "B-1": "COMMAND_POST", "B-2": "MAINT_DEPOT", "B-3": "SUPPLY_BASE",
    "R-1": "COMMAND_POST", "R-2": "MAINT_DEPOT", "R-3": "SUPPLY_BASE",
}

# M1 and M5 are server messages at fixed times; M2/M3/M4/M5 use the fixed
# structures / robots / bases above.  M6 ranks the observed opponent actions.
B2_MISSION = {
    "target": "COMM_FACILITY",
    "sectors": ("R-S2", "COMMON"),
    "goal_base": "SUPPLY_BASE",
    "m5_base": "COMMAND_POST",
}

# The generic stage point (W - 2.0, H / 2) lies inside R_middle_block.
FIXED_STAGE_POINT = (3.60, 3.00)

M3_CLASSES = ("AMMO_BOX", "DRUM", "MORTAR", "TRUCK", "TANK", "MISSILE")
DEFAULT_M3_COUNT = 4  # Rehearsal choice, not an official competition box count.

# 250 x 230 mm Rover corner radius (.170 m) plus the default planning
# uncertainty budget (~.049 m). A legal .40 m gap alone cannot guarantee
# an in-place turn with this margin; the audit must find a wider route.
M3_TURN_RADIUS = .22
M3_VIEW_MAX_INCIDENCE_DEG = 55.0  # Rehearsal acceptance stricter than runtime's 70 degrees.
M3_VIEW_MIN_SEPARATION = .20
M3_VIEW_NORMAL_DISTANCES = (.30, .35, .45, .6, .8, 1.0, 1.4)


def _rect(s):
    return s.x - s.sx / 2, s.y - s.sy / 2, s.x + s.sx / 2, s.y + s.sy / 2


def structure_gap(a, b):
    """Conservative axis-aligned clearance; at least one axis has a 40cm gap."""
    ax0, ay0, ax1, ay1 = _rect(a)
    bx0, by0, bx1, by1 = _rect(b)
    return max(bx0 - ax1, ax0 - bx1, by0 - ay1, ay0 - by1)


def _segment_clear(ax, ay, bx, by, rects):
    """Exact segment/AABB intersection, including grazing an obstacle edge."""
    for x0, y0, x1, y1 in rects:
        lo, hi = 0., 1.
        for origin, delta, lower, upper in ((ax, bx-ax, x0, x1), (ay, by-ay, y0, y1)):
            if abs(delta) < 1e-12:
                if not lower <= origin <= upper:
                    hi = -1.
                    break
            else:
                t0, t1 = (lower-origin)/delta, (upper-origin)/delta
                lo, hi = max(lo, min(t0, t1)), min(hi, max(t0, t1))
                if lo > hi:
                    break
        if lo <= hi:
            return False
    return True


def _complete_photo_projection(vx, vy, fx, fy, nx, ny, camera, camera_height):
    """Project all four photo corners with the current SimUGV.cam_pos model.

    SimUGV.cam_pos uses body x/y and robot.cam_height (zero forward offset),
    with the body aimed at the face, neutral pan and tilt. This is the M3
    sensing camera, not the separately mounted optional FPV recording camera.
    """
    yaw = math.atan2(fy-vy, fx-vx)
    c, s = math.cos(yaw), math.sin(yaw)
    pixels = []
    for offset in (-.18, .18):
        px, py = fx + nx*.002 - ny*offset, fy + ny*.002 + nx*offset
        dx, dy = px-vx, py-vy
        depth, left = dx*c + dy*s, -dx*s + dy*c
        if depth <= .08:
            return None
        for pz in (.02, .38):
            u = camera.cx-camera.f*left/depth
            v = camera.cy-camera.fy*(pz-camera_height)/depth
            if not (2 <= u <= camera.w-2 and 2 <= v <= camera.h-2):
                return None
            pixels.append((u, v))
    return [min(p[0] for p in pixels), min(p[1] for p in pixels),
            max(p[0] for p in pixels), max(p[1] for p in pixels)]


def audit_m3_accessibility(structures, candidates, geo):
    """Return reachable, complete-picture observation witnesses for every face.

    This is generation/referee QA only, never a source of Agent waypoints.
    Each witness has a B2-start route with room to rotate the padded Rover,
    a runtime-style NBV pose, <=55 degree incidence and unobstructed rays to
    the picture centre and both side edges. Two poses >=20 cm apart are
    required per face, so one narrow grazing ray cannot validate a layout.
    The fixed thirteen arena structures are not moved by this check.
    """
    res, radius = .05, M3_TURN_RADIUS
    camera, camera_height = CameraModel(CameraCfg()), RobotCfg().cam_height
    result = {"valid": False, "reason": "", "turn_radius_m": radius,
              "grid_resolution_m": res, "max_incidence_deg": M3_VIEW_MAX_INCIDENCE_DEG,
              "minimum_view_separation_m": M3_VIEW_MIN_SEPARATION,
              "camera": {"width": camera.w, "height": camera.h, "height_m": camera_height,
                         "forward_offset_m": 0., "pan_deg": 0., "tilt_deg": 0.}, "faces": []}
    xs = np.arange(res / 2, geo.W, res)
    ys = np.arange(res / 2, geo.H, res)
    xx, yy = np.meshgrid(xs, ys)
    # Half-cell padding makes the entire selected cell safe, including the
    # short connector from its centre to the exact NBV pose.
    raster_radius = radius + res/2
    free = ((xx >= raster_radius) & (xx <= geo.W - raster_radius)
            & (yy >= raster_radius) & (yy <= geo.H - raster_radius))
    rects = [_rect(s) for s in structures]
    for x0, y0, x1, y1 in rects:
        free &= ~((xx >= x0 - raster_radius) & (xx <= x1 + raster_radius) &
                  (yy >= y0 - raster_radius) & (yy <= y1 + raster_radius))
    def cell(x, y):
        return min(len(ys)-1, max(0, int(y/res))), min(len(xs)-1, max(0, int(x/res)))
    start = cell(.35, geo.base_cy[1])
    if not free[start]:
        result["reason"] = "B2 spawn has no padded turning space"
        return result
    reached = np.zeros_like(free)
    q = deque([start])
    parents = {start: None}
    reached[start] = True
    while q:
        iy, ix = q.popleft()
        for jy, jx in ((iy-1, ix), (iy+1, ix), (iy, ix-1), (iy, ix+1)):
            if 0 <= jy < len(ys) and 0 <= jx < len(xs) and free[jy, jx] and not reached[jy, jx]:
                reached[jy, jx] = True
                parents[(jy, jx)] = (iy, ix)
                q.append((jy, jx))
    if not reached[cell(*FIXED_STAGE_POINT)]:
        result["reason"] = "M5 stage is disconnected from B2"
        return result
    # The Rover turns in the approach lane, then enters a base straight.
    # Requiring rotation at x=.20 would incorrectly forbid legal docking:
    # padded half-length=.175 fits there, but turning radius=.22 does not.
    if not all(reached[cell(x, y)] for x in (.35, geo.W-.35) for y in geo.base_cy):
        result["reason"] = "A base approach lane is disconnected from B2"
        return result

    def route_to(x, y):
        p, path = cell(x, y), [(float(x), float(y))]
        while p is not None:
            iy, ix = p
            path.append((float(xs[ix]), float(ys[iy])))
            p = parents[p]
        path.append((.35, float(geo.base_cy[1])))
        return list(reversed(path))

    cos_limit = math.cos(math.radians(M3_VIEW_MAX_INCIDENCE_DEG))
    for s in candidates:
        for face, (nx, ny) in enumerate(((1, 0), (0, 1), (-1, 0), (0, -1))):
            fx, fy = s.x + s.sx/2*nx, s.y + s.sy/2*ny
            witnesses = []
            # Match the explorer's finite view-pose family, rather than
            # accepting a raster point the explorer can never propose.
            for distance in M3_VIEW_NORMAL_DISTANCES:
                for lateral in (0., -.3, .3, -.6, .6):
                    vx, vy = fx + nx*distance - ny*lateral, fy + ny*distance + nx*lateral
                    length = math.hypot(distance, lateral)
                    if not .40 <= length <= 1.4 or distance/length < cos_limit:
                        continue
                    # OnlineExplorer._free uses a 30 cm arena-edge margin.
                    if not (.30-1e-9 <= vx <= geo.W-.30+1e-9 and .30-1e-9 <= vy <= geo.H-.30+1e-9
                            and reached[cell(vx, vy)]):
                        continue
                    # 36 cm photo spans the face, 2 mm in front of the box.
                    if not all(_segment_clear(vx, vy, fx + nx*.002 - ny*offset,
                                               fy + ny*.002 + nx*offset, rects)
                               for offset in (-.18, 0., .18)):
                        continue
                    bbox = _complete_photo_projection(vx, vy, fx, fy, nx, ny, camera, camera_height)
                    if bbox is None:
                        continue
                    if witnesses and math.hypot(vx-witnesses[0]["x"], vy-witnesses[0]["y"]) < M3_VIEW_MIN_SEPARATION-1e-8:
                        continue
                    witnesses.append({"x": round(vx, 6), "y": round(vy, 6),
                                      "yaw": math.atan2(fy-vy, fx-vx),
                                      "distance_m": length,
                                      "incidence_deg": math.degrees(math.acos(distance/length)),
                                      "picture_bbox_px": bbox,
                                      "route": route_to(vx, vy)})
                    if len(witnesses) == 2:
                        break
                if len(witnesses) == 2:
                    break
            result["faces"].append({"cube": s.name, "face": face, "poses": witnesses})
            if len(witnesses) < 2:
                result["reason"] = f"{s.name} face {face} has fewer than two complete reachable views"
                return result
    result["valid"] = True
    return result


def _reachable_views(structures, candidates, geo):
    return audit_m3_accessibility(structures, candidates, geo)["valid"]


@lru_cache(maxsize=512)
def _single_cube_accessible(cube, width, height, base_centres):
    """Avoid repeatedly sampling locations with intrinsically unviewable sides."""
    geo = SimpleNamespace(W=width, H=height, base_cy=base_centres)
    return _reachable_views((*STRUCTURES, cube), (cube,), geo)


def _mirrored_x_lattice(width):
    """A 10 cm step on both sides of the actual 6.12 m-wide arena.

    A left-only .6 + .1*n lattice omits e.g. x=5.32, the mirror of x=.80.
    That 2 cm phase error used to remove reachable RED-side wall views.
    """
    left = {round(float(x), 2) for x in np.arange(.6, width-.599, .1)}
    return tuple(sorted(left | {round(width-x, 2) for x in left}))


def generate_m3_structures(seed, geo, count=DEFAULT_M3_COUNT):
    """Seeded M3 cubes: p13 four picture faces, p24 >=40cm new-box clearance.

    Quantities and placement are test settings. Sample free locations throughout
    the field, reject disconnected bases/staging or hidden/unreachable faces.
    """
    if not 0 <= count <= 12:
        raise ValueError("fixed_m3_count must be between 0 and 12")
    if count == 0:
        return ()
    rng = random.Random(seed)
    locations = []
    for x in _mirrored_x_lattice(geo.W):
        for y in np.arange(.6, geo.H-.599, .1):
            s = FixedStructure("candidate", round(float(x), 2), round(float(y), 2), .4, .4, ())
            if min(structure_gap(s, old) for old in STRUCTURES) < .4-1e-8:
                continue
            # Keep the base entrance axes and the shared staging point clear.
            if (s.x < 1.1 or s.x > geo.W-1.1) and min(abs(s.y-cy) for cy in geo.base_cy) < .48:
                continue
            if abs(s.x-FIXED_STAGE_POINT[0]) < .40 and abs(s.y-FIXED_STAGE_POINT[1]) < .40:
                continue
            if not _single_cube_accessible(s, geo.W, geo.H, tuple(geo.base_cy)):
                continue
            locations.append(s)
    for attempt in range(150):
        rng.shuffle(locations)
        chosen = []
        # Rehearsals with four or more cubes inspect both actual team sectors,
        # not merely opposite edges of the central COMMON strip. The fixed
        # field uses 40 cm bases; all retained side locations lie outside them.
        if count >= 4:
            blue = next((s for s in locations if s.x < geo.W/2-geo.common_half), None)
            red = next((s for s in locations if s.x > geo.W/2+geo.common_half), None)
            if blue is None or red is None:
                break
            chosen = [blue, red]
        for s in locations:
            if all(structure_gap(s, old) >= .4-1e-8 for old in chosen):
                chosen.append(s)
                if len(chosen) == count:
                    break
        if len(chosen) != count:
            continue
        # Keep upper/lower coverage as well as the BLUE/RED anchors above.
        if count >= 4 and not (min(s.x for s in chosen) < geo.W/2-geo.common_half
                               and max(s.x for s in chosen) > geo.W/2+geo.common_half
                               and min(s.y for s in chosen) < geo.H/2 < max(s.y for s in chosen)):
            continue
        if not _reachable_views((*STRUCTURES, *chosen), chosen, geo):
            continue
        return tuple(FixedStructure(f"m3_{i:02d}", s.x, s.y, .4, .4,
                                   tuple((f, "object", rng.choice(M3_CLASSES)) for f in range(4)))
                     for i, s in enumerate(chosen))
    raise ValueError(f"Cannot place {count} M3 cubes with 40cm clearance and reachable faces "
                     f"in the fixed arena (seed={seed}); reduce fixed_m3_count")
