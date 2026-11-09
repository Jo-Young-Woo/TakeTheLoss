"""점유 격자(로그오즈) + 비용 격자 + A* + 경로 단순화.

좌표: 월드(m). 격자 (iy, ix) = ((y-oy)/res, (x-ox)/res). 경기장 밖은 점유.
미지 영역은 '자유'로 본다(경기장이 작고 박스는 LiDAR에 곧 드러남 → 계획 후 재계획).
"""
from __future__ import annotations

import heapq
import math
from typing import List, Optional, Sequence, Tuple

import numpy as np

from ..arena import ArenaGeometry
from ..types import Pose2D, Scan

Pt = Tuple[float, float]


class OccupancyGrid:
    L_FREE, L_OCC, L_MIN, L_MAX, L_THR = -0.45, 0.95, -2.0, 4.0, 0.7

    def __init__(self, geo: ArenaGeometry, res: float = 0.05, margin: float = 0.1) -> None:
        self.geo, self.res = geo, res
        self.ox, self.oy = -margin, -margin
        self.nx = int(math.ceil((geo.W + 2 * margin) / res))
        self.ny = int(math.ceil((geo.H + 2 * margin) / res))
        self.L = np.zeros((self.ny, self.nx), dtype=np.float32)
        self.static = np.zeros((self.ny, self.nx), dtype=bool)
        xs = self.ox + (np.arange(self.nx) + 0.5) * res
        ys = self.oy + (np.arange(self.ny) + 0.5) * res
        X, Y = np.meshgrid(xs, ys)
        self.static |= (X < 0) | (X > geo.W) | (Y < 0) | (Y > geo.H)
        self._off_cache = {}
        # 3D 지도(깊이 카메라)에서 투영한 키 높이대 장애물. LiDAR 평면 밖의 낮은/높은 물체를 항법에 더한다. None 이면 없음.
        self.layer3d: Optional[np.ndarray] = None
        # Policy is not a sensed obstacle: keep it out of localization and cube
        # extraction, and apply it only to planning centre-cell costs.
        self.keepout: Optional[np.ndarray] = None
        self.keepout_version = 0

    def set_keepout(self, mask: Optional[np.ndarray]) -> bool:
        if mask is not None and mask.shape != self.L.shape:
            raise ValueError("keepout mask shape differs from planning grid")
        same = (mask is None and self.keepout is None) or (mask is not None and self.keepout is not None
                and np.array_equal(mask, self.keepout))
        if same:
            return False
        self.keepout = None if mask is None else np.asarray(mask, dtype=bool).copy()
        self.keepout_version += 1
        return True

    def set_layer3d(self, mask: Optional[np.ndarray]) -> None:
        if mask is not None and mask.shape != self.L.shape:
            raise ValueError(f"3D 층 모양 {mask.shape} 이 격자 {self.L.shape} 와 다르다")
        self.layer3d = mask

    # ------------------------------------------------------------------ 변환
    def to_cell(self, x: float, y: float) -> Tuple[int, int]:
        return int((y - self.oy) / self.res), int((x - self.ox) / self.res)

    def to_world(self, iy: int, ix: int) -> Pt:
        return (self.ox + (ix + 0.5) * self.res, self.oy + (iy + 0.5) * self.res)

    def in_bounds(self, iy: int, ix: int) -> bool:
        return 0 <= iy < self.ny and 0 <= ix < self.nx

    # ------------------------------------------------------------------ 갱신
    def add_static_box(self, rect: Tuple[float, float, float, float]) -> None:
        x0, y0, x1, y1 = rect
        iy0, ix0 = self.to_cell(x0, y0)
        iy1, ix1 = self.to_cell(x1, y1)
        self.static[max(iy0, 0):iy1 + 1, max(ix0, 0):ix1 + 1] = True

    def update(self, scan: Scan, pose: Pose2D, free_range: float = 4.0, min_r: float = 0.12) -> None:
        # NaN includes directions outside the sensor FOV and empty rebin bins.
        # These rays, -inf (too close) and sub-minimum values provide no evidence
        # of free space. Only explicit +inf / finite maximum-range returns may
        # clear to the sensor limit without adding an occupied endpoint.
        observed = ((np.isfinite(scan.ranges) & (scan.ranges > min_r))
                    | np.isposinf(scan.ranges))
        if not observed.any():
            return
        ang = (scan.angles + pose.th)[observed]
        r = scan.ranges[observed]
        finite = np.isfinite(r)
        hit = finite & (r < scan.range_max - 0.05)
        rr = np.where(finite, np.minimum(r, scan.range_max), scan.range_max)
        step = self.res * 0.6
        nsteps = int(free_range / step)
        d = (np.arange(1, nsteps + 1) * step)[None, :]
        lim = (np.minimum(rr, free_range) - self.res * 0.8)[:, None]
        m = d < lim
        px = pose.x + np.cos(ang)[:, None] * d
        py = pose.y + np.sin(ang)[:, None] * d
        ix = ((px - self.ox) / self.res).astype(int)
        iy = ((py - self.oy) / self.res).astype(int)
        ok = m & (ix >= 0) & (ix < self.nx) & (iy >= 0) & (iy < self.ny)
        # 같은 셀 중복 방지를 위해 unique 후 한 번만 감소
        free_idx = np.unique(iy[ok] * self.nx + ix[ok])
        flat = self.L.reshape(-1)
        flat[free_idx] = np.maximum(flat[free_idx] + self.L_FREE, self.L_MIN)
        # 끝점(점유)
        ex = pose.x + np.cos(ang[hit]) * rr[hit]
        ey = pose.y + np.sin(ang[hit]) * rr[hit]
        eix = ((ex - self.ox) / self.res).astype(int)
        eiy = ((ey - self.oy) / self.res).astype(int)
        ok2 = (eix >= 0) & (eix < self.nx) & (eiy >= 0) & (eiy < self.ny)
        occ_idx = np.unique(eiy[ok2] * self.nx + eix[ok2])
        flat[occ_idx] = np.minimum(flat[occ_idx] + self.L_OCC, self.L_MAX)

    # ------------------------------------------------------------------ 비용
    def occupied(self) -> np.ndarray:
        occ = self.static | (self.L > self.L_THR)
        if self.layer3d is not None:
            occ |= self.layer3d
        return occ

    def _offsets(self, rad_cells: int):
        if rad_cells not in self._off_cache:
            offs = [(dy, dx, math.hypot(dy, dx)) for dy in range(-rad_cells, rad_cells + 1)
                    for dx in range(-rad_cells, rad_cells + 1) if math.hypot(dy, dx) <= rad_cells + 1e-9]
            self._off_cache[rad_cells] = offs
        return self._off_cache[rad_cells]

    def distance_field(self, occ: np.ndarray, max_dist: float) -> np.ndarray:
        """점유 셀까지의 거리(m), max_dist 이상은 max_dist."""
        rc = int(math.ceil(max_dist / self.res))
        dist = np.full(occ.shape, max_dist, dtype=np.float32)
        pad = np.pad(occ, rc, constant_values=False)
        ny, nx = occ.shape
        for dy, dx, dd in self._offsets(rc):
            sh = pad[rc + dy:rc + dy + ny, rc + dx:rc + dx + nx]
            v = dd * self.res
            np.minimum(dist, np.where(sh, v, max_dist), out=dist)
        return dist

    def cost_maps(self, hard: float, soft: float, soft_w: float,
                  extra_block: Optional[Sequence[Pt]] = None, extra_r: float = 0.3):
        """(blocked bool, cost float) — cost는 이동 1칸당 가중."""
        occ = self.occupied().copy()
        if extra_block:
            for (x, y) in extra_block:
                iy, ix = self.to_cell(x, y)
                rc = int(extra_r / self.res)
                occ[max(iy - rc, 0):iy + rc + 1, max(ix - rc, 0):ix + rc + 1] = True
        dist = self.distance_field(occ, soft)
        blocked = dist <= hard
        if self.keepout is not None:
            blocked |= self.keepout  # Already expanded for the full rotating footprint.
        cost = 1.0 + soft_w * np.clip((soft - dist) / max(soft - hard, 1e-6), 0, 1) ** 2
        return blocked, cost.astype(np.float32)


# ====================================================================== A*
_N8 = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
       (-1, -1, 1.4142), (-1, 1, 1.4142), (1, -1, 1.4142), (1, 1, 1.4142)]


def nearest_free(blocked: np.ndarray, iy: int, ix: int, max_r: int = 30) -> Optional[Tuple[int, int]]:
    ny, nx = blocked.shape
    iy, ix = min(max(iy, 0), ny - 1), min(max(ix, 0), nx - 1)
    if not blocked[iy, ix]:
        return iy, ix
    for r in range(1, max_r):
        best, bd = None, 1e9
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if max(abs(dy), abs(dx)) != r:
                    continue
                y, x = iy + dy, ix + dx
                if 0 <= y < ny and 0 <= x < nx and not blocked[y, x]:
                    d = dy * dy + dx * dx
                    if d < bd:
                        best, bd = (y, x), d
        if best:
            return best
    return None


def astar(grid: OccupancyGrid, blocked: np.ndarray, cost: np.ndarray, start: Pt, goal: Pt,
          max_expand: int = 40000) -> Optional[List[Pt]]:
    s = nearest_free(blocked, *grid.to_cell(*start))
    g = nearest_free(blocked, *grid.to_cell(*goal))
    if s is None or g is None:
        return None
    ny, nx = blocked.shape
    bl = blocked.reshape(-1).tolist()
    co = cost.reshape(-1).tolist()
    sidx, gidx = s[0] * nx + s[1], g[0] * nx + g[1]
    gy, gx = g
    INF = 1e18
    gs = {sidx: 0.0}
    came = {}
    heap = [(0.0, sidx)]
    closed = set()
    expands = 0
    while heap:
        f, cur = heapq.heappop(heap)
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
        for dy, dx, step in _N8:
            y, x = cy + dy, cx + dx
            if y < 0 or y >= ny or x < 0 or x >= nx:
                continue
            ni = y * nx + x
            if bl[ni] or ni in closed:
                continue
            if dy and dx and (bl[cy * nx + x] or bl[y * nx + cx]):   # 모서리 가로지르기 금지
                continue
            ng = gc + step * co[ni]
            if ng < gs.get(ni, INF):
                gs[ni] = ng
                came[ni] = cur
                ddx, ddy = abs(x - gx), abs(y - gy)
                h = (ddx + ddy) + (1.4142 - 2) * min(ddx, ddy)
                heapq.heappush(heap, (ng + h, ni))
    else:
        return None
    path = [gidx]
    while path[-1] != sidx:
        path.append(came[path[-1]])
    path.reverse()
    return [grid.to_world(*divmod(i, nx)) for i in path]


def line_free(grid: OccupancyGrid, blocked: np.ndarray, a: Pt, b: Pt) -> bool:
    n = int(max(math.hypot(b[0] - a[0], b[1] - a[1]) / (grid.res * 0.5), 1))
    for i in range(n + 1):
        t = i / n
        iy, ix = grid.to_cell(a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)
        if not grid.in_bounds(iy, ix) or blocked[iy, ix]:
            return False
    return True


def simplify(grid: OccupancyGrid, blocked: np.ndarray, path: List[Pt]) -> List[Pt]:
    """가시선 기반 경로 단순화(그리디)."""
    if len(path) <= 2:
        return path
    out = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and not line_free(grid, blocked, path[i], path[j]):
            j -= 1
        out.append(path[j])
        i = j
    return out
