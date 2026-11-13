"""Shared observed-map costs and outward escape rules for A*/D* comparison."""
from dataclasses import dataclass
import math
import numpy as np
from .astar_search import MOVES
from .risk_astar import PlanResult as BasePlanResult


@dataclass
class PlanResult(BasePlanResult):
    egress: bool = False


def navigation_layers(grid, blocked, cost, start, field, w_risk, w_close):
    """Build directed edges permitting only separation-increasing escape.

Only a disk already containing the robot may relax its *extra* margin. The
actual swept-body radius plus 3 cm, static walls and every other disk stay hard.
Each allowed segment has nonnegative initial distance derivative; distance to
that robot therefore cannot decrease anywhere along the segment. Once outside,
an edge cannot enter the disk again. No broad clearing around the start.
"""
    hard = np.array(blocked, dtype=bool, copy=True)
    route_cost = np.asarray(cost, dtype=float).copy()
    if field is None:
        return hard, route_cost, None, False
    route_cost += max(0., w_risk)*field.risk[0] + max(0., w_close)*field.close[0]
    body = field.body[0].copy()
    disks = getattr(field, 'clearance_disks', ())
    if not disks:
        return hard | body, route_cost, None, False
    ys, xs = np.indices(hard.shape)
    X, Y = grid.ox+(xs+.5)*grid.res, grid.oy+(ys+.5)*grid.res
    scell = grid.to_cell(*start)
    # The outgoing edge at the start uses the actual measured pose, not a
    # rounded cell centre that might be closer to the other UGV.
    X[scell], Y[scell] = start
    relaxed = np.zeros_like(hard)
    retained = np.zeros_like(hard)
    allow = np.ones((*hard.shape, 8), dtype=bool)
    escaping = False
    cell_pad = grid.res/math.sqrt(2)  # Cover the whole cell, not just its centre.
    for px, py, collision_r, margin_r in disks:
        dx, dy = X-px, Y-py
        d = np.hypot(dx, dy)
        current = math.hypot(start[0]-px, start[1]-py)
        disk = d <= margin_r
        if collision_r+cell_pad < current < margin_r:
            escaping = True
            relaxed |= disk
            retained |= d <= collision_r+cell_pad
            for k, (sy, sx, _) in enumerate(MOVES):
                ex, ey = X+sx*grid.res, Y+sy*grid.res
                # Destination is the real grid centre, including start edges.
                ex[scell] = grid.ox+(scell[1]+sx+.5)*grid.res
                ey[scell] = grid.oy+(scell[0]+sy+.5)*grid.res
                tx, ty = ex-X, ey-Y
                end = np.hypot(ex-px, ey-py)
                derivative = dx*tx + dy*ty
                near = disk | (end <= margin_r)
                allow[:, :, k] &= ~near | ((derivative >= -1e-10) & (end > collision_r))
        else:
            retained |= disk
    hard |= (body & ~relaxed) | retained
    return hard, route_cost, allow if escaping else None, escaping


def escape_waypoint(path):
    """Short first straight segment; do not shortcut an outward escape bend."""
    if len(path) < 2:
        return path[-1]
    a, target = path[0], path[1]
    vx, vy = target[0]-a[0], target[1]-a[1]
    for p in path[2:]:
        dx, dy = p[0]-target[0], p[1]-target[1]
        if abs(vx*dy-vy*dx) > 1e-5 or math.dist(a, p) > .25:
            break
        target = p
    return target
