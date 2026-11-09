"""탐색 계획: 격자 시점(viewpoint) 순회. 각 시점에서 제자리 360° 회전으로 사방을 본다.

간단하지만 안정적인 방식: 박스 배치를 몰라도 되고(미지 배치 대응), 방문 이력만 관리한다.
고도화 여지: 박스를 격자에서 검출해 면별 시점을 만들면 더 짧아진다(사전 테스트 경기장 지도가 있으면 효과 큼).
"""
from __future__ import annotations

import math
from typing import List, Optional, Set, Tuple

import numpy as np

from ..arena import ArenaGeometry
from ..nav.grid import OccupancyGrid

Pt = Tuple[float, float]


class ViewpointTour:
    def __init__(self, geo: ArenaGeometry, nx: int = 5, ny: int = 5) -> None:
        self.geo = geo
        x0, x1 = geo.base_depth + 0.45, geo.W - geo.base_depth - 0.45
        y0, y1 = 0.50, geo.H - 0.50
        xs = np.linspace(x0, x1, nx) if nx > 1 else [0.5 * (x0 + x1)]
        ys = np.linspace(y0, y1, ny) if ny > 1 else [0.5 * (y0 + y1)]
        self.points: List[Pt] = [(float(x), float(y)) for x in xs for y in ys]
        self.done: Set[int] = set()
        self.cur: Optional[int] = None

    def remaining(self) -> int:
        return len(self.points) - len(self.done)

    def mark_done(self, i: Optional[int] = None) -> None:
        i = self.cur if i is None else i
        if i is not None:
            self.done.add(i)
        self.cur = None

    def pick(self, pos: Pt, blocked: Optional[np.ndarray], grid: OccupancyGrid,
             prefer: Optional[Set[str]] = None) -> Optional[Pt]:
        """가장 가까운(선호 섹터 가중) 미방문 시점."""
        best, bc = None, 1e9
        for i, p in enumerate(self.points):
            if i in self.done:
                continue
            if blocked is not None:
                iy, ix = grid.to_cell(*p)
                if grid.in_bounds(iy, ix) and blocked[iy, ix]:
                    self.done.add(i)          # 박스 위/벽 밀착 시점은 영구 제외
                    continue
            c = math.hypot(p[0] - pos[0], p[1] - pos[1])
            if prefer and self.geo.sector_of(*p) in prefer:
                c *= 0.7
            if c < bc:
                best, bc = i, c
        if best is None:
            return None
        self.cur = best
        return self.points[best]


class CubeFinder:
    """점유 격자에서 박스(40 cm 정육면체)를 찾는다: 연결 성분의 외접 사각형이 약 40x40 cm 이면 박스.
    LiDAR는 보통 박스의 1~2개 면만 보므로, 로봇이 돌아다니며 L자·ㅁ자가 채워질수록 정확해진다.
    (박스를 한 면만 본 단편은 크기가 안 맞아 제외 — 더 관측되면 자연히 포함된다.)"""

    def __init__(self, geo: ArenaGeometry, size: float = 0.40, tol: float = 0.10) -> None:
        self.geo, self.size, self.tol = geo, size, tol

    def find(self, grid: OccupancyGrid) -> List[Pt]:
        occ = (grid.L > grid.L_THR) & ~grid.static
        ys, xs = np.nonzero(occ)
        cells = set()
        wm = 0.14
        for iy, ix in zip(ys.tolist(), xs.tolist()):
            x, y = grid.to_world(iy, ix)
            if wm < x < self.geo.W - wm and wm < y < self.geo.H - wm:       # 외벽 끝점 제외
                cells.add((iy, ix))
        out: List[Pt] = []
        seen = set()
        for c in cells:
            if c in seen:
                continue
            comp, stack = [], [c]
            seen.add(c)
            while stack:
                cy, cx = stack.pop()
                comp.append((cy, cx))
                for dy in (-1, 0, 1):
                    for dx in (-1, 0, 1):
                        n = (cy + dy, cx + dx)
                        if n in cells and n not in seen:
                            seen.add(n)
                            stack.append(n)
            if len(comp) < 6:
                continue
            iy = [p[0] for p in comp]
            ix = [p[1] for p in comp]
            w = (max(ix) - min(ix) + 1) * grid.res
            h = (max(iy) - min(iy) + 1) * grid.res
            if abs(w - self.size) <= self.tol and abs(h - self.size) <= self.tol:
                cx = grid.ox + (min(ix) + max(ix) + 1) / 2 * grid.res
                cy = grid.oy + (min(iy) + max(iy) + 1) / 2 * grid.res
                out.append((cx, cy))
        return out


FACE_DIRS = [(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0)]


class FaceTour:
    """아직 관측하지 못한 박스 면마다 '정면 0.9 m' 지점을 시점으로 만들어 가까운 순으로 방문."""

    def __init__(self, geo: ArenaGeometry) -> None:
        self.geo = geo
        self.attempts: dict = {}

    @staticmethod
    def key(cx: float, cy: float, f: int) -> Tuple[int, int, int]:
        return (int(round(cx * 10)), int(round(cy * 10)), f)

    def pick(self, cubes: List[Pt], seen_faces: list, pos: Pt, blocked: np.ndarray, grid: OccupancyGrid):
        """seen_faces: [(x, y, vx, vy)] — 면 위치와 관측 방향. → (viewpoint, face_center, key) 또는 None."""
        best, bc = None, 1e9
        for (cx, cy) in cubes:
            for fi, (nx, ny) in enumerate(FACE_DIRS):
                k = self.key(cx, cy, fi)
                if self.attempts.get(k, 0) >= 2:
                    continue
                fc = (cx + 0.2 * nx, cy + 0.2 * ny)
                # 같은 박스의 인접 면은 0.28 m 거리라 위치만으로는 구분 불가 → 관측 방향이 이 면의 법선과 맞는 것만 '본 면'
                if any(math.hypot(fc[0] - sx, fc[1] - sy) < 0.22 and
                       (nx * vx + ny * vy) / (math.hypot(vx, vy) or 1.0) > 0.5
                       for sx, sy, vx, vy in seen_faces):
                    continue
                vp = None
                for d in (0.95, 1.2, 0.75, 1.5):
                    p = (fc[0] + nx * d, fc[1] + ny * d)
                    if not self.geo.inside(p[0], p[1], 0.30):
                        continue
                    iy, ix = grid.to_cell(*p)
                    if not grid.in_bounds(iy, ix) or blocked[iy, ix]:
                        continue
                    from ..nav.grid import line_free
                    if not line_free(grid, blocked, p, (fc[0] + nx * 0.28, fc[1] + ny * 0.28)):
                        continue
                    vp = p
                    break
                if vp is None:
                    self.attempts[k] = 2          # 접근 불가 면(벽 쪽 좁은 틈 등)은 포기
                    continue
                c = math.hypot(vp[0] - pos[0], vp[1] - pos[1])
                if c < bc:
                    best, bc = (vp, fc, k), c
        return best

    def mark(self, k) -> None:
        self.attempts[k] = self.attempts.get(k, 0) + 1
