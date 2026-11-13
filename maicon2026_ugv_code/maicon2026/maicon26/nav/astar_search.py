"""Fresh A* on the same directed, weighted grid used in the D* Lite trial.

Every call starts a new heap, distance map and predecessor map. Only the caller's
current masks/costs are used; no g/rhs tree or old route survives replanning.
The outward-only escape mask and rectangular-body diagonal mask are respected.
"""
from __future__ import annotations

import heapq
import math
import numpy as np

from .grid import nearest_free

MOVES = ((-1, 0, 1.), (1, 0, 1.), (0, -1, 1.), (0, 1, 1.),
         (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)),
         (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2)))


class AStar:
    def __init__(self):
        self.plans = 0
        self.total_expanded = 0

    def plan(self, grid, blocked, cost, start, goal, *, max_expand=40000,
             open_mask=None, edge_allow=None, snap_endpoints=True, diagnostics=None):
        ny, nx = grid.ny, grid.nx
        if blocked.shape != (ny, nx) or cost.shape != blocked.shape:
            raise ValueError('A* map shape differs from grid')
        if not np.isfinite(cost).all() or np.any(cost < 1):
            raise ValueError('A* cell costs must be finite and >= 1')
        self.plans += 1
        expanded = 0

        def record(reason):
            self.total_expanded += expanded
            if diagnostics is not None:
                diagnostics.update(algorithm='astar', reason=reason, expanded=expanded,
                    reused=False, plans=self.plans, total_expanded=self.total_expanded,
                    max_expand=max_expand, grid_cells=int(blocked.size))

        s, target = grid.to_cell(*start), grid.to_cell(*goal)
        if snap_endpoints:
            s, target = nearest_free(blocked, *s), nearest_free(blocked, *target)
        if (s is None or target is None or not grid.in_bounds(*s) or not grid.in_bounds(*target)
                or blocked[s] or blocked[target]):
            record('no_free_endpoint')
            return None
        bl, co = blocked.ravel().tolist(), cost.ravel().tolist()
        om = open_mask.ravel().tolist() if open_mask is not None else None
        allowed = edge_allow.reshape(ny*nx, 8).tolist() if edge_allow is not None else None
        start_id, goal_id = s[0]*nx+s[1], target[0]*nx+target[1]
        gy, gx = target

        def heuristic(y, x):
            dx, dy = abs(x-gx), abs(y-gy)
            return max(dx, dy)+(math.sqrt(2)-1)*min(dx, dy)

        scores, previous, closed = {start_id: 0.}, {}, set()
        heap = [(heuristic(*s), 0., start_id)]
        while heap:
            _, value, u = heapq.heappop(heap)
            if u in closed or value != scores.get(u):
                continue
            if u == goal_id:
                ids = [u]
                while ids[-1] != start_id:
                    ids.append(previous[ids[-1]])
                ids.reverse()
                record('found')
                return [grid.to_world(*divmod(i, nx)) for i in ids]
            if expanded >= max_expand:
                record('expansion_limit')
                return None
            closed.add(u)
            expanded += 1
            y, x = divmod(u, nx)
            for k, (dy, dx, length) in enumerate(MOVES):
                yy, xx = y+dy, x+dx
                if not (0 <= yy < ny and 0 <= xx < nx):
                    continue
                v = yy*nx+xx
                if bl[v] or v in closed or (allowed is not None and not allowed[u][k]):
                    continue
                if dy and dx:
                    ca, cb = y*nx+xx, yy*nx+x
                    if bl[ca] or bl[cb]:
                        continue
                    if om is not None and not (om[u] and om[v] and om[ca] and om[cb]):
                        continue
                candidate = value+length*co[v]
                if candidate < scores.get(v, math.inf):
                    scores[v], previous[v] = candidate, u
                    heapq.heappush(heap, (candidate+heuristic(yy, xx), candidate, v))
        record('frontier_exhausted')
        return None
