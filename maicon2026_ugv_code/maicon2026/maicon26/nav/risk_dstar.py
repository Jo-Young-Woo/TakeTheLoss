"""Rolling observed-map D* Lite with a strictly outward clearance escape.

This is a 2-D replanner, not a space-time optimal planner. Current observed UGV
exclusion disks stay hard until a new sensor field changes them. Forecast risk
remains a soft cost; fresh sensor/control collision checks run independently.
No referee positions or ground-truth identities are accepted here.
"""
from collections import OrderedDict
import math
from weakref import WeakKeyDictionary


from .dstar_lite import DStarLite
from .grid import nearest_free
from .risk_layers import PlanResult, navigation_layers, escape_waypoint


_POOLS = WeakKeyDictionary()


def _planner(grid, goal, profile):
    pool = _POOLS.setdefault(grid, OrderedDict())
    key = (grid.to_cell(*goal), profile)
    item = pool.pop(key, None) or DStarLite()
    pool[key] = item
    while len(pool) > 24:
        pool.popitem(last=False)
    return item


def plan(grid, blocked, cost, start, goal, v_plan=.35, field=None,
         w_risk=3., w_close=1.5, risk_hit=.5, max_expand=40000, diagnostics=None):
    if not grid.in_bounds(*grid.to_cell(*start)):
        if diagnostics is not None:
            diagnostics.update(algorithm='dstar_lite', reason='no_free_endpoint', expanded=0)
        return None
    hard, route_cost, edges, escaping = navigation_layers(grid, blocked, cost, start, field, w_risk, w_close)
    # Snap only against the static mask. A dynamic body must not teleport the
    # start to free space or quietly substitute an observation point for a goal.
    s = nearest_free(blocked, *grid.to_cell(*start))
    g = nearest_free(blocked, *grid.to_cell(*goal))
    if s is None or g is None:
        if diagnostics is not None:
            diagnostics.update(algorithm='dstar_lite', reason='no_free_endpoint', expanded=0)
        return None
    if escaping and s != grid.to_cell(*start):
        if diagnostics is not None:
            diagnostics.update(algorithm='dstar_lite', reason='static_start_blocked', expanded=0)
        return None
    effective_goal = grid.to_world(*g)
    planner = _planner(grid, effective_goal, (w_risk == 0., w_close == 0.))
    d = {} if diagnostics is None else diagnostics
    pts = planner.plan(grid, hard, route_cost, grid.to_world(*s), effective_goal,
                       edge_allow=edges, max_expand=max_expand, snap_endpoints=False, diagnostics=d)
    d.update(field_present=field is not None, egress=escaping,
             field_version=getattr(field, 'version', None),
             projection='current_observed_bodies', uses_world_truth=False)
    if pts is None:
        return None
    if escaping:
        pts[0] = tuple(start)
    length = sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))
    integral, longest, run, elapsed = 0., 0., 0., 0.
    if field is not None:
        for a, b in zip(pts, pts[1:]):
            dt = math.dist(a, b)/max(v_plan, 1e-6)
            k = min(field.K-1, int(elapsed/max(field.dtau, 1e-6)))
            value = float(field.risk[k][grid.to_cell(*b)])
            integral += value*dt
            run = run+dt if value >= risk_hit else 0.
            longest = max(longest, run)
            elapsed += dt
    return PlanResult(pts, length, length/max(v_plan, 1e-6), integral, longest, escaping)
