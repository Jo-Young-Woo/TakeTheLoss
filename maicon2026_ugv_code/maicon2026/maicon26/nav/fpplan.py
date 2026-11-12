"""직사각형 차체를 고려한 경로계획 한 번 — 내장 항법(Navigator)과 경기장 유효성 검사(sim/arena_check.py)가 **같은 함수**를 쓴다.

흐름: ① 반폭 + 계획 여유 만큼 키운 장애물 위에서 A*(기존 `grid.astar`) ② 경로 단순화 ③ 직사각형 검증(`footprint.path_violations`):
진행 방향으로 놓은 차체가 점유 칸과 겹치거나, 방향이 크게 바뀌는 꼭짓점에서 제자리 회전 공간이 없으면 그 칸을 막고 ①부터 다시.
경로를 못 찾았다고 인플레이션을 줄이지 않는다.
"""
from __future__ import annotations

import math
from typing import List, Optional, Sequence, Tuple

import numpy as np

from ..footprint import ClearanceBudget, path_violations
from .grid import OccupancyGrid, astar, simplify

Pt = Tuple[float, float]


def _no_turn_cells(grid: OccupancyGrid, path: Sequence[Pt], v: Pt, rotate_r: float, lateral: float = 0.08) -> List[Tuple[int, int]]:
    """꼭짓점 v 에서 제자리 회전 공간이 없을 때, v 둘레(rotate_r)에서 *들어오던 방향의 직선 띠(±lateral)를 뺀* 칸을 막는다 →
    다음 계획은 직진으로 통과하고 트인 곳에서 꺾는다(꼭짓점 칸 하나만 막으면 한 칸 옆에서 같은 위반이 8번 반복된다)."""
    i = min(range(len(path)), key=lambda k: (path[k][0] - v[0]) ** 2 + (path[k][1] - v[1]) ** 2)
    if i == 0:
        return [grid.to_cell(*v)]
    px, py = path[i - 1]
    th = math.atan2(v[1] - py, v[0] - px)
    c, s = math.cos(th), math.sin(th)
    out: List[Tuple[int, int]] = []
    k = int(math.ceil(rotate_r / grid.res))
    iy0, ix0 = grid.to_cell(*v)
    for dy in range(-k, k + 1):
        for dx in range(-k, k + 1):
            wx, wy = dx * grid.res, dy * grid.res
            if math.hypot(wx, wy) > rotate_r:
                continue
            lat = -wx * s + wy * c                                # 들어오던 방향에 대한 옆 거리
            if abs(lat) > lateral:
                out.append((iy0 + dy, ix0 + dx))
    return out


_N4 = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0)]
_ND = [(-1, -1, 1.4142), (-1, 1, 1.4142), (1, -1, 1.4142), (1, 1, 1.4142)]


def _astar_open_diag(grid: OccupancyGrid, blocked: np.ndarray, cost: np.ndarray, open_mask: np.ndarray, start: Pt, goal: Pt,
                     max_expand: int = 40000) -> Optional[List[Pt]]:
    """기존 `grid.astar` 와 같은 8방향 A* 인데, **대각 이동은 양 끝이 '어느 방향으로 서도 차체가 들어가는' 트인 칸(open_mask)일 때만** 허용한다.
    장애물 가까운 좁은 곳은 축 방향(상하좌우)으로만 지나간다 — 박스·벽이 축 정렬이라 이때 차체 폭이 최소가 되고, 비스듬히 틈에 끼는 경로가 처음부터 안 나온다."""
    from .grid import nearest_free
    import heapq
    s = nearest_free(blocked, *grid.to_cell(*start))
    g = nearest_free(blocked, *grid.to_cell(*goal))
    if s is None or g is None:
        return None
    ny, nx = blocked.shape
    bl, co, om = blocked.reshape(-1).tolist(), cost.reshape(-1).tolist(), open_mask.reshape(-1).tolist()
    sidx, gidx = s[0] * nx + s[1], g[0] * nx + g[1]
    gy, gx = g
    gs, came, closed = {sidx: 0.0}, {}, set()
    heap = [(0.0, sidx)]
    expands = 0
    while heap:
        _f, cur = heapq.heappop(heap)
        if cur in closed:
            continue
        if cur == gidx:
            break
        closed.add(cur)
        expands += 1
        if expands > max_expand:
            return None
        cy, cx = divmod(cur, nx)
        gc = gs[cur]
        for dy, dx, step in (_N4 + _ND if om[cur] else _N4):
            y, x = cy + dy, cx + dx
            if y < 0 or y >= ny or x < 0 or x >= nx:
                continue
            ni = y * nx + x
            if bl[ni] or ni in closed:
                continue
            if dy and dx and (not om[ni] or bl[cy * nx + x] or bl[y * nx + cx] or not om[cy * nx + x] or not om[y * nx + cx]):
                continue
            ng = gc + step * co[ni]
            if ng < gs.get(ni, 1e18):
                gs[ni] = ng
                came[ni] = cur
                ddx, ddy = abs(x - gx), abs(y - gy)
                heapq.heappush(heap, (ng + (ddx + ddy) + (1.4142 - 2) * min(ddx, ddy), ni))
    else:
        return None
    path = [gidx]
    while path[-1] != sidx:
        path.append(came[path[-1]])
    path.reverse()
    return [grid.to_world(*divmod(i, nx)) for i in path]


def _simplify_open(grid: OccupancyGrid, blocked: np.ndarray, open_mask: np.ndarray, path: List[Pt]) -> List[Pt]:
    """가시선 단순화. 축에 맞지 않는 지름길은 선분 위 모든 칸이 트인 칸일 때만 허용(좁은 곳의 비스듬한 지름길 금지)."""
    from .grid import line_free
    if len(path) <= 2:
        return path
    out = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1:
            a, b = path[i], path[j]
            ok = line_free(grid, blocked, a, b)
            if ok and abs(b[0] - a[0]) > 1e-6 and abs(b[1] - a[1]) > 1e-6:                     # 비스듬한 선분
                n = int(max(math.hypot(b[0] - a[0], b[1] - a[1]) / (grid.res * 0.5), 1))
                for k in range(n + 1):
                    t = k / n
                    iy, ix = grid.to_cell(a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)
                    if not (grid.in_bounds(iy, ix) and open_mask[iy, ix]):
                        ok = False
                        break
            if ok:
                break
            j -= 1
        out.append(path[j])
        i = j
    return out


def _axis_detour(grid: OccupancyGrid, budget: ClearanceBudget, path: Sequence[Pt], p: Pt, lead: float = 0.45):
    """좁은 틈 입구에서 비스듬히 들어오다 걸린 경우의 보정: 박스·벽이 모두 축 정렬이므로, 위반 지점 p 에서 가장 가까운 축 방향으로
    '트인 곳에서 먼저 정렬(회전) → 그 방향으로 직진해 통과 → 통과 뒤 꺾기'로 바꾼다. → (W1, Q, 축 각도) 또는 None."""
    best_i, bd = 0, 1e18
    for i in range(len(path) - 1):
        ax, ay = path[i]
        bx, by = path[i + 1]
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        t = 0.0 if L2 < 1e-12 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / L2))
        d = (ax + t * dx - p[0]) ** 2 + (ay + t * dy - p[1]) ** 2
        if d < bd:
            best_i, bd = i, d
    th = math.atan2(path[best_i + 1][1] - path[best_i][1], path[best_i + 1][0] - path[best_i][0])
    axis = round(th / (math.pi / 2)) * (math.pi / 2)
    c, s = math.cos(axis), math.sin(axis)
    occ = grid.planning_occupied()
    from ..footprint import circle_cells_free
    rr = budget.plan_inflate_rotate

    def run(sign: float) -> Optional[Pt]:
        # p 에서 축 방향으로 가며 '제자리 회전이 되는 첫 지점'(최소 lead 이상 떨어진)을 찾는다. 틈 안은 회전이 안 되므로 틈을 지나 트인 곳까지 직진한다.
        d = lead
        while d <= 1.6:
            x, y = p[0] + sign * d * c, p[1] + sign * d * s
            if circle_cells_free(occ, grid.res, grid.ox, grid.oy, x, y, rr):
                return (x, y)
            d += grid.res
        return None

    w1, q = run(-1.0), run(+1.0)
    if w1 is None or q is None:
        return None
    if path_violations(occ, grid.res, grid.ox, grid.oy, [w1, q], budget):
        return None
    return w1, q, axis


def plan_footprint(grid: OccupancyGrid, inflate_hard: float, inflate_soft: float, soft_weight: float, budget: ClearanceBudget,
                   start: Pt, goal: Pt, start_heading: Optional[float] = None, extra_block: Optional[Sequence[Pt]] = None,
                   check: bool = True, max_iter: int = 8, max_expand: int = 40000, _depth: int = 0):
    """→ (path, blocked, n_rejects) 또는 None. check=False 면 직사각형 검증을 건너뛴다(레거시)."""
    bad_cells: List[Tuple[int, int]] = []
    rejects = 0
    for _ in range(max_iter if check else 1):
        blocked, cost = grid.cost_maps(inflate_hard, inflate_soft, soft_weight, extra_block)
        for (by, bx) in bad_cells:
            blocked[max(by - 1, 0):by + 2, max(bx - 1, 0):bx + 2] = True
        if check:
            # 트인 칸(어느 방향으로 서도 직사각형이 들어가는 칸) = 장애물 중심까지 거리 > 회전 소거 반경. 좁은 칸은 축 방향으로만 지나간다.
            open_mask = (grid.distance_field(grid.planning_occupied(), budget.plan_inflate_rotate + 0.05) > budget.plan_inflate_rotate) & ~blocked
            p = _astar_open_diag(grid, blocked, cost, open_mask, start, goal, max_expand)
            path = _simplify_open(grid, blocked, open_mask, p) if p else None
        else:
            p = astar(grid, blocked, cost, start, goal, max_expand)
            path = simplify(grid, blocked, p) if p else None
        if not p:
            return None
        if check:
            viol = path_violations(grid.planning_occupied(), grid.res, grid.ox, grid.oy, path, budget, start_heading=start_heading)
            if viol:
                rejects += 1
                body = [v for v in viol if v[2] == "body"]
                if body and _depth < 3:
                    det = _axis_detour(grid, budget, path, (body[0][0], body[0][1]))
                    if det is not None:
                        w1, q, axis = det
                        a = plan_footprint(grid, inflate_hard, inflate_soft, soft_weight, budget, start, w1, start_heading, extra_block, True, max_iter, max_expand, _depth + 1)
                        b = plan_footprint(grid, inflate_hard, inflate_soft, soft_weight, budget, q, goal, axis, extra_block, True, max_iter, max_expand, _depth + 1)
                        if a is not None and b is not None:
                            return a[0] + b[0], blocked, rejects + a[2] + b[2]
                for (x, y, kind) in viol:
                    if kind == "rotate":
                        bad_cells += _no_turn_cells(grid, path, (x, y), rotate_r=budget.plan_inflate_rotate)
                    else:
                        bad_cells.append(grid.to_cell(x, y))
                continue
        return path, blocked, rejects
    return None

