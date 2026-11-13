"""Persistent, directed eight-neighbour D* Lite (Koenig & Likhachev, 2002).

g/rhs and the priority queue survive map updates and motion of the start.
Only vertices whose outgoing edges changed are repaired. A different goal or
grid geometry starts a new search. Costs are >= 1; diagonals never cut corners.
Reference: https://idm-lab.org/bib/abstracts/Koen02e.html
"""
from __future__ import annotations

import heapq
import math
import numpy as np

from .grid import nearest_free

MOVES = ((-1, 0, 1.), (1, 0, 1.), (0, -1, 1.), (0, 1, 1.),
         (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)),
         (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2)))
INF = math.inf


class DStarLite:
    def __init__(self):
        self.geometry = None
        self.goal = None
        self.resets = 0
        self.repairs = 0
        self.total_expanded = 0

    def _geometry(self, grid):
        geometry = (grid.ny, grid.nx, grid.res, grid.ox, grid.oy)
        if geometry == self.geometry:
            return
        self.geometry, self.goal = geometry, None
        self.ny, self.nx = grid.ny, grid.nx
        self.size = grid.ny * grid.nx
        ys, xs = np.indices((grid.ny, grid.nx))
        self.neighbours = np.full((self.size, 8), -1, dtype=np.int32)
        self.corner_a = np.zeros_like(self.neighbours)
        self.corner_b = np.zeros_like(self.neighbours)
        for k, (dy, dx, _) in enumerate(MOVES):
            yy, xx = ys + dy, xs + dx
            valid = (yy >= 0) & (yy < grid.ny) & (xx >= 0) & (xx < grid.nx)
            self.neighbours[:, k] = np.where(valid, yy * grid.nx + xx, -1).ravel()
            self.corner_a[:, k] = np.clip(ys * grid.nx + xx, 0, self.size - 1).ravel()
            self.corner_b[:, k] = np.clip(yy * grid.nx + xs, 0, self.size - 1).ravel()
        self.adj = [tuple(v for v in row if v >= 0) for row in self.neighbours.tolist()]
        self.neighbour_rows = self.neighbours.tolist()

    def _heuristic(self, a, b):
        ay, ax = divmod(a, self.nx)
        by, bx = divmod(b, self.nx)
        dx, dy = abs(ax-bx), abs(ay-by)
        return max(dx, dy) + (math.sqrt(2)-1) * min(dx, dy)

    def _key(self, u):
        value = min(self.g[u], self.rhs[u])
        return value + self._heuristic(self.start, u) + self.km, value

    @staticmethod
    def _equal(a, b):
        return a == b or math.isclose(a, b, rel_tol=1e-12, abs_tol=1e-9)

    def _enqueue(self, u):
        self.ticket += 1
        key = self._key(u)
        self.queued[u] = (key, self.ticket)
        heapq.heappush(self.heap, (key, self.ticket, u))

    def _update_vertex(self, u):
        if u != self.goal:
            self.rhs[u] = min((c + self.g[v] for v, c in
                              zip(self.neighbour_rows[u], self.edges[u]) if v >= 0), default=INF)
        self.queued.pop(u, None)
        if not self._equal(self.g[u], self.rhs[u]):
            self._enqueue(u)

    def _top(self):
        while self.heap:
            key, ticket, u = self.heap[0]
            if self.queued.get(u) == (key, ticket):
                return key
            heapq.heappop(self.heap)
        return (INF, INF)

    def _edge_costs(self, blocked, cost, open_mask, edge_allow):
        b, c = blocked.ravel(), cost.ravel()
        dest = np.maximum(self.neighbours, 0)
        allowed = (self.neighbours >= 0) & ~b[:, None] & ~b[dest]
        allowed[:, 4:] &= ~b[self.corner_a[:, 4:]] & ~b[self.corner_b[:, 4:]]
        if open_mask is not None:
            om = open_mask.ravel()
            allowed[:, 4:] &= (om[:, None] & om[dest[:, 4:]] &
                               om[self.corner_a[:, 4:]] & om[self.corner_b[:, 4:]])
        if edge_allow is not None:
            allowed &= edge_allow.reshape(self.size, 8)
        return np.where(allowed, c[dest] * np.array([m[2] for m in MOVES]), INF)

    def plan(self, grid, blocked, cost, start, goal, *, max_expand=40000,
             open_mask=None, edge_allow=None, snap_endpoints=True, diagnostics=None):
        self._geometry(grid)
        if blocked.shape != (self.ny, self.nx) or cost.shape != blocked.shape:
            raise ValueError('D* Lite map shape differs from grid')
        if not np.isfinite(cost).all() or np.any(cost < 1):
            raise ValueError('D* Lite cell costs must be finite and >= 1')
        s, target = grid.to_cell(*start), grid.to_cell(*goal)
        if snap_endpoints:
            s, target = nearest_free(blocked, *s), nearest_free(blocked, *target)
        expanded, changed, reused = 0, 0, False

        def record(reason):
            if diagnostics is not None:
                diagnostics.update(algorithm='dstar_lite', reason=reason, expanded=expanded,
                    changed_vertices=changed, reused=reused, resets=self.resets,
                    repairs=self.repairs, total_expanded=self.total_expanded,
                    max_expand=max_expand, grid_cells=self.size)

        if (s is None or target is None or not grid.in_bounds(*s) or not grid.in_bounds(*target)
                or blocked[s] or blocked[target]):
            record('no_free_endpoint')
            return None
        start_id, goal_id = s[0]*self.nx+s[1], target[0]*self.nx+target[1]
        edges = self._edge_costs(blocked, np.asarray(cost, dtype=float), open_mask, edge_allow)
        if self.goal != goal_id:
            self.start, self.goal, self.km = start_id, goal_id, 0.
            self.g, self.rhs = [INF]*self.size, [INF]*self.size
            self.rhs[goal_id] = 0.
            self.heap, self.queued, self.ticket = [], {}, 0
            self.edge_array, self.edges = edges, edges.tolist()
            self.resets += 1
            self._enqueue(goal_id)
        else:
            reused = True
            self.km += self._heuristic(self.start, start_id)
            self.start = start_id
            updates = np.flatnonzero(np.any(edges != self.edge_array, axis=1))
            changed = len(updates)
            self.edge_array = edges
            for u in updates:
                self.edges[int(u)] = edges[u].tolist()
            for u in updates:
                self._update_vertex(int(u))
            self.repairs += 1
        while self._top() < self._key(self.start) or not self._equal(self.rhs[self.start], self.g[self.start]):
            if not self.heap:
                break
            if expanded >= max_expand:
                self.total_expanded += expanded
                record('expansion_limit')
                return None  # Preserve unfinished queue for the next call; never return a stale route.
            old_key, ticket, u = heapq.heappop(self.heap)
            self.queued.pop(u, None)
            new_key = self._key(u)
            if old_key < new_key:
                self._enqueue(u)
            elif self.g[u] > self.rhs[u]:
                self.g[u] = self.rhs[u]
                for v in self.adj[u]:
                    self._update_vertex(v)
            else:
                self.g[u] = INF
                self._update_vertex(u)
                for v in self.adj[u]:
                    self._update_vertex(v)
            expanded += 1
        self.total_expanded += expanded
        # Lazy invalid entries must not grow without bound during a long match.
        if len(self.heap) > max(128, 3*len(self.queued)):
            self.heap = [(key, ticket, u) for u, (key, ticket) in self.queued.items()]
            heapq.heapify(self.heap)
        if not math.isfinite(self.rhs[self.start]):
            record('frontier_exhausted')
            return None
        ids, seen = [self.start], {self.start}
        while ids[-1] != self.goal:
            u = ids[-1]
            candidates = [(c+self.g[v], self._heuristic(v, self.goal), v)
                          for v, c in zip(self.neighbour_rows[u], self.edges[u]) if v >= 0]
            value, _, nxt = min(candidates, default=(INF, INF, -1))
            if not math.isfinite(value) or nxt in seen:
                record('inconsistent_route')
                return None
            ids.append(nxt)
            seen.add(nxt)
        record('found')
        return [grid.to_world(*divmod(i, self.nx)) for i in ids]
