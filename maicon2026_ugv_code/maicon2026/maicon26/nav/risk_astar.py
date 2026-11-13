"""임무 계층용 '위험을 보는 경로 추정기'. **기존 A*(nav/grid.py astar)·Nav2 전역 계획기를 대체하지 않는다.**

쓰임: (1) 탐색 시점의 이동 시간·노출 시간을 재고, (2) 위협이 있을 때 안전 경유지(carrot)를 고른다. 실제 주행 경로는 여전히
Nav2(NavigateToPose)나 내장 Navigator 의 A* 가 그 경유지/시점까지 계획한다.

field=None 이면 grid.astar 와 같은 결과(비용·차단은 입력 그대로)를 낸다. field(ThreatField)가 있으면 시간 층 k 마다
  blocked_k = blocked | body_k          (적 차체의 예측 점유 — 하드)
  cost_k    = cost + w_risk·risk_k + w_close·close_k   (피격 위험 — 소프트)
를 쓰고, 칸 도착 시각 t = (경로 길이)/v_plan 으로 층을 고른다(A* 휴리스틱은 그대로: 추가 항이 모두 ≥ 0 이라 허용).
"""
from __future__ import annotations

import heapq
import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

from .grid import OccupancyGrid, nearest_free

Pt = Tuple[float, float]
_N8 = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
       (-1, -1, 1.4142), (-1, 1, 1.4142), (1, -1, 1.4142), (1, 1, 1.4142)]


@dataclass
class PlanResult:
    path: List[Pt]
    length: float            # m
    time_s: float            # length / v_plan
    risk_int: float          # ∑ risk · (칸 길이 / v_plan)  [risk·초]
    max_exposure_s: float    # risk ≥ risk_hit 인 구간의 최대 연속 시간(s)


def plan(grid: OccupancyGrid, blocked: np.ndarray, cost: np.ndarray, start: Pt, goal: Pt, v_plan: float = 0.35,
         field=None, w_risk: float = 3.0, w_close: float = 1.5, risk_hit: float = 0.5, max_expand: int = 40000,
         diagnostics: Optional[dict] = None) -> Optional[PlanResult]:
    def record(reason, expanded):
        if diagnostics is not None:
            diagnostics.update(reason=reason, expanded=expanded, max_expand=max_expand,
                               grid_cells=int(blocked.size), field_present=field is not None)
    s = nearest_free(blocked, *grid.to_cell(*start))
    g = nearest_free(blocked, *grid.to_cell(*goal))
    if s is None or g is None:
        record("no_free_endpoint", 0)
        return None
    ny, nx = blocked.shape
    K = 1 if field is None else int(field.K)
    dtau = 1e9 if field is None else float(field.dtau)
    bl = [blocked.reshape(-1).tolist()] if field is None else [(blocked | field.body[k]).reshape(-1).tolist() for k in range(K)]
    static_bl = blocked.reshape(-1).tolist()
    body_horizon = math.inf if field is None else getattr(field, 'body_horizon_s', math.inf)
    if field is None:
        co = [cost.reshape(-1).tolist()]
    else:
        co = [(cost + w_risk * field.risk[k] + w_close * field.close[k]).astype(np.float32).reshape(-1).tolist() for k in range(K)]
    sidx, gidx = s[0] * nx + s[1], g[0] * nx + g[1]
    gy, gx = g
    INF = 1e18
    gs, gl, came, closed = {sidx: 0.0}, {sidx: 0.0}, {}, set()
    heap = [(0.0, sidx)]
    expands = 0
    res = grid.res
    while heap:
        f, cur = heapq.heappop(heap)
        if cur in closed:
            continue
        if cur == gidx:
            break
        closed.add(cur)
        expands += 1
        if expands > max_expand:
            record("expansion_limit", expands)
            return None
        cy, cx = divmod(cur, nx)
        gc, lc = gs[cur], gl[cur]
        for dy, dx, step in _N8:
            y, x = cy + dy, cx + dx
            if y < 0 or y >= ny or x < 0 or x >= nx:
                continue
            ni = y * nx + x
            if ni in closed:
                continue
            ln = lc + step * res
            k = 0 if K == 1 else min(K - 1, int(ln / v_plan / dtau))
            hard = bl[k] if ln / v_plan <= body_horizon else static_bl
            if hard[ni]:
                continue
            if dy and dx and (hard[cy * nx + x] or hard[y * nx + cx]):
                continue
            ng = gc + step * co[k][ni]
            if ng < gs.get(ni, INF):
                gs[ni], gl[ni], came[ni] = ng, ln, cur
                ddx, ddy = abs(x - gx), abs(y - gy)
                h = (ddx + ddy) + (1.4142 - 2) * min(ddx, ddy)
                heapq.heappush(heap, (ng + h, ni))
    else:
        record("frontier_exhausted", expands)
        return None
    idxs = [gidx]
    while idxs[-1] != sidx:
        idxs.append(came[idxs[-1]])
    idxs.reverse()
    pts = [grid.to_world(*divmod(i, nx)) for i in idxs]
    length = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts[:-1], pts[1:]))
    risk_int, run, mx, t = 0.0, 0.0, 0.0, 0.0
    if field is not None:
        for (ax, ay), (bx, by) in zip(pts[:-1], pts[1:]):
            seg = math.hypot(bx - ax, by - ay)
            dt = seg / v_plan
            k = min(K - 1, int(t / dtau))
            iy, ix = grid.to_cell(bx, by)
            r = float(field.risk[k][iy, ix]) if grid.in_bounds(iy, ix) else 0.0
            risk_int += r * dt
            if r >= risk_hit:
                run += dt
                mx = max(mx, run)
            else:
                run = 0.0
            t += dt
    record("found", expands)
    return PlanResult(pts, length, length / max(v_plan, 1e-6), risk_int, mx)
