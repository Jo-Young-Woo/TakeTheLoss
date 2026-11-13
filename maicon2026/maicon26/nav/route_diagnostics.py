"""Connectivity evidence for route failures; never relaxes a driving mask."""
from collections import deque
import numpy as np
from .grid import nearest_free


def reachable_mask(blocked, start):
    """Same eight neighbours and no-corner-cut rule as production A*."""
    seen = np.zeros(blocked.shape, dtype=bool)
    y, x = start
    ny, nx = blocked.shape
    if not (0 <= y < ny and 0 <= x < nx) or blocked[y, x]:
        return seen
    q = deque([(y, x)])
    seen[y, x] = True
    while q:
        y, x = q.popleft()
        for dy, dx in ((-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)):
            yy, xx = y+dy, x+dx
            if not (0 <= yy < ny and 0 <= xx < nx) or seen[yy,xx] or blocked[yy,xx]:
                continue
            if dy and dx and (blocked[y,xx] or blocked[yy,x]):
                continue
            seen[yy,xx] = True
            q.append((yy,xx))
    return seen


def blockage_evidence(grid, blocked, raw_occupied, start, goal, field, planner):
    s = nearest_free(blocked, *grid.to_cell(*start))
    g = nearest_free(blocked, *grid.to_cell(*goal))
    evidence = dict(planner)
    evidence.update(start=list(start), goal=list(goal), uses_world_truth=False)
    if s is None or g is None:
        evidence['classification'] = 'no_free_endpoint'
        return evidence
    static = reachable_mask(blocked, s)
    raw = reachable_mask(raw_occupied, s)
    current = blocked if field is None else blocked | field.body[0]
    dynamic = reachable_mask(current, s)
    evidence.update(static_connected=bool(static[g]), raw_connected=bool(raw[g]),
                    current_body_connected=bool(dynamic[g]),
                    static_component_cells=int(static.sum()),
                    current_component_cells=int(dynamic.sum()),
                    current_body_at_start=bool(current[s] and not blocked[s]),
                    body_cells=0 if field is None else int(field.body[0].sum()))
    if planner.get('reason') == 'expansion_limit':
        kind = 'expansion_limit'
    elif not static[g]:
        kind = 'clearance_or_policy_disconnection' if raw[g] else 'observed_map_disconnection'
    elif field is not None:
        kind = 'dynamic_forecast_failure'
    else:
        kind = 'planner_failure_on_connected_map'
    evidence['classification'] = kind
    return evidence
