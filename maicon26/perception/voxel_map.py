"""3D 복셀 점유 지도(로그오즈). 깊이 영상의 점들을 '카메라 자세 × 로봇 자세'로 월드에 등록해 쌓는다.

2D 점유격자(nav/grid.py)와 같은 해상도·원점이라 층(z)만 더 있다. 로그오즈라 움직인 물체는 광선이 그 자리를 다시 통과하며 지워진다.
- integrate()  광선마다 [카메라 → 끝점) 구간을 '비움', 끝점을 '점유'로 갱신 (바닥 점은 비움만 하고 점유로 쓰지 않는다)
- column_obstacles()  키 높이대 [z_lo, z_hi] 안의 점유를 (y, x) 2D 마스크로 투영 — LiDAR 평면 밖의 낮은/높은 장애물을 항법에 더한다
- segment_blocked()  두 3D 점 사이에 점유가 끼어 있나 — 레이저 사선 확인
- forward_clearance()  차체 앞 통로의 가장 가까운 장애물까지 거리 — 깊이 기반 전방 안전 정지
- box_clusters()  점유 열(column)의 연결 성분 → 위에서 본 외접 사각형·높이 (박스 40 cm, UGV 32 cm 구분의 근거)
- save_npz / save_ply  지도를 파일로 (오프라인 확인·RViz/MeshLab 에서 열기)
"""
from __future__ import annotations

import math
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

from ..arena import ArenaGeometry
from ..config import Map3DCfg


class VoxelMap3D:
    def __init__(self, geo: ArenaGeometry, cfg: Map3DCfg, margin: float = 0.1) -> None:
        self.geo, self.cfg = geo, cfg
        self.res = cfg.res
        self.ox = self.oy = -margin
        self.nx = int(math.ceil((geo.W + 2 * margin) / cfg.res))     # OccupancyGrid 와 같은 계산 → 같은 모양
        self.ny = int(math.ceil((geo.H + 2 * margin) / cfg.res))
        self.nz = int(math.ceil(cfg.z_max / cfg.res))
        self.L = np.zeros((self.nz, self.ny, self.nx), dtype=np.float32)
        self.seen = np.zeros((self.nz, self.ny, self.nx), dtype=bool)
        self.version = 0                  # integrate 마다 증가 — 파생 마스크의 캐시 키
        self.frames = 0
        self._col_cache: Dict[Tuple, Tuple[int, np.ndarray]] = {}

    # ------------------------------------------------------------------ 변환
    def _cells(self, pts: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        ix = np.floor((pts[..., 0] - self.ox) / self.res).astype(np.int64)
        iy = np.floor((pts[..., 1] - self.oy) / self.res).astype(np.int64)
        iz = np.floor(pts[..., 2] / self.res).astype(np.int64)
        return ix, iy, iz

    def _inb(self, ix: np.ndarray, iy: np.ndarray, iz: np.ndarray) -> np.ndarray:
        return (ix >= 0) & (ix < self.nx) & (iy >= 0) & (iy < self.ny) & (iz >= 0) & (iz < self.nz)

    def _flat(self, ix: np.ndarray, iy: np.ndarray, iz: np.ndarray) -> np.ndarray:
        return (iz * self.ny + iy) * self.nx + ix

    def voxel_center(self, ix: int, iy: int, iz: int) -> Tuple[float, float, float]:
        return (self.ox + (ix + 0.5) * self.res, self.oy + (iy + 0.5) * self.res, (iz + 0.5) * self.res)

    # ------------------------------------------------------------------ 갱신
    def integrate(self, origin: np.ndarray, pts: np.ndarray, max_free: Optional[float] = None) -> int:
        """origin (3,) 에서 월드 점 pts (N,3) 으로 쏜 광선들을 반영. 반환: 점유로 반영한 끝점 수."""
        c = self.cfg
        self.frames += 1
        self.version += 1
        if len(pts) == 0:
            return 0
        origin = np.asarray(origin, dtype=float)
        vec = pts - origin
        d = np.linalg.norm(vec, axis=1)
        keep = d > 0.05
        pts, vec, d = pts[keep], vec[keep], d[keep]
        if len(pts) == 0:
            return 0
        dirs = vec / d[:, None]
        step = self.res * 0.9
        mf = c.max_free_range if max_free is None else max_free
        S = max(int(mf / step), 1)
        t = (np.arange(1, S + 1) * step)[None, :]
        lim = (np.minimum(d, mf) - self.res * 0.8)[:, None]
        m = t < lim
        P = origin[None, None, :] + dirs[:, None, :] * t[..., None]
        ix, iy, iz = self._cells(P)
        ok = m & self._inb(ix, iy, iz)
        Lf = self.L.reshape(-1)
        Sf = self.seen.reshape(-1)
        mark = np.zeros(Lf.size, dtype=bool)                 # 프레임당 셀마다 한 번만 갱신(정렬 없이 표시 배열로 중복 제거)
        mark[self._flat(ix[ok], iy[ok], iz[ok])] = True
        Lf[mark] = np.maximum(Lf[mark] + c.free, c.l_min)
        Sf |= mark
        # 끝점: 바닥(z < floor_z)과 지도 높이 밖은 점유로 쓰지 않는다
        eix, eiy, eiz = self._cells(pts)
        ok2 = self._inb(eix, eiy, eiz) & (pts[:, 2] >= c.floor_z) & (pts[:, 2] < c.z_max)
        mark = np.zeros(Lf.size, dtype=bool)
        mark[self._flat(eix[ok2], eiy[ok2], eiz[ok2])] = True
        Lf[mark] = np.minimum(Lf[mark] + c.hit, c.l_max)
        Sf |= mark
        return int(ok2.sum())

    # ------------------------------------------------------------------ 질의
    def occupied(self, thr: Optional[float] = None) -> np.ndarray:
        return self.L > (self.cfg.occ_thr if thr is None else thr)

    def _neighbor_count(self, occ: np.ndarray) -> np.ndarray:
        n = np.zeros(occ.shape, dtype=np.uint8)
        n[1:] += occ[:-1]
        n[:-1] += occ[1:]
        n[:, 1:] += occ[:, :-1]
        n[:, :-1] += occ[:, 1:]
        n[:, :, 1:] += occ[:, :, :-1]
        n[:, :, :-1] += occ[:, :, 1:]
        return n

    def column_obstacles(self, z_lo: float, z_hi: float, thr: Optional[float] = None, min_neighbors: int = 0) -> np.ndarray:
        """높이 [z_lo, z_hi) 안에 점유 복셀이 하나라도 있는 (y, x) 열. 고립 복셀(이웃 < min_neighbors)은 허상으로 보고 뺀다."""
        key = (round(z_lo, 3), round(z_hi, 3), thr, min_neighbors)
        hit = self._col_cache.get(key)
        if hit is not None and hit[0] == self.version:
            return hit[1]
        k0, k1 = max(int(math.floor(z_lo / self.res)), 0), min(int(math.ceil(z_hi / self.res)), self.nz)
        if k1 <= k0:
            return np.zeros((self.ny, self.nx), dtype=bool)
        if min_neighbors > 0:                        # 이웃 수는 관심 층 ±1 만 계산한다(전체 부피를 훑지 않는다)
            a, b = max(k0 - 1, 0), min(k1 + 1, self.nz)
            occ = self.L[a:b] > (self.cfg.occ_thr if thr is None else thr)
            occ = (occ & (self._neighbor_count(occ) >= min_neighbors))[k0 - a:k1 - a]
        else:
            occ = self.occupied(thr)[k0:k1]
        col = occ.any(axis=0)
        self._col_cache = {k: v for k, v in self._col_cache.items() if v[0] == self.version}
        self._col_cache[key] = (self.version, col)
        return col

    def height_map(self, thr: Optional[float] = None) -> np.ndarray:
        """열마다 가장 높은 점유 복셀의 윗면 높이(m). 점유가 없으면 0."""
        occ = self.occupied(thr)
        top = np.where(occ.any(axis=0), self.nz - 1 - np.argmax(occ[::-1], axis=0), -1)
        return np.where(top >= 0, (top + 1) * self.res, 0.0).astype(np.float32)

    def _sample_line(self, a: np.ndarray, b: np.ndarray, skip_start: float, skip_end: float) -> np.ndarray:
        L = float(np.linalg.norm(b - a))
        if L < 1e-6:
            return np.zeros((0, 3))
        n = max(int((L - skip_start - skip_end) / (self.res * 0.5)), 0)
        if n <= 0:
            return np.zeros((0, 3))
        s = skip_start + (np.arange(n) + 0.5) * (L - skip_start - skip_end) / n
        return a[None, :] + (b - a)[None, :] / L * s[:, None]

    def segment_blocked(self, a: Sequence[float], b: Sequence[float], skip_start: float = 0.05, skip_end: float = 0.10,
                        thr: Optional[float] = None, min_neighbors: int = 0) -> int:
        """a→b 선분이 지나는 점유 복셀 수(끝 구간 skip_end 는 표적 자신의 면이라 뺀다)."""
        a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
        P = self._sample_line(a, b, skip_start, skip_end)
        if len(P) == 0:
            return 0
        ix, iy, iz = self._cells(P)
        ok = self._inb(ix, iy, iz)
        idx = np.unique(self._flat(ix[ok], iy[ok], iz[ok]))
        thr_v = self.cfg.occ_thr if thr is None else thr
        occ = (self.L > thr_v).reshape(-1)
        hit = idx[occ[idx]]
        if min_neighbors > 0 and len(hit):            # 선분 위 점유 복셀만 이웃 6칸을 직접 확인(전체 부피를 훑지 않는다)
            hz, hy, hx = hit // (self.ny * self.nx), (hit // self.nx) % self.ny, hit % self.nx
            nb = np.zeros(len(hit), dtype=np.int32)
            for dz, dy, dx in ((1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)):
                z2, y2, x2 = hz + dz, hy + dy, hx + dx
                inb = (z2 >= 0) & (z2 < self.nz) & (y2 >= 0) & (y2 < self.ny) & (x2 >= 0) & (x2 < self.nx)
                nb[inb] += self.L[z2[inb], y2[inb], x2[inb]] > thr_v
            hit = hit[nb >= min_neighbors]
        return int(len(hit))

    def ray_first_hit(self, origin: Sequence[float], direction: Sequence[float], max_d: float,
                      thr: Optional[float] = None) -> Optional[float]:
        o, dv = np.asarray(origin, dtype=float), np.asarray(direction, dtype=float)
        dv = dv / max(float(np.linalg.norm(dv)), 1e-9)
        s = np.arange(self.res * 0.5, max_d, self.res * 0.5)
        P = o[None, :] + dv[None, :] * s[:, None]
        ix, iy, iz = self._cells(P)
        ok = self._inb(ix, iy, iz)
        occ = np.zeros(len(s), dtype=bool)
        occ[ok] = self.occupied(thr)[iz[ok], iy[ok], ix[ok]]
        return float(s[np.argmax(occ)]) if occ.any() else None

    def lidar_blind_columns(self, z_lo: float, z_hi: float, plane_z: float, plane_half: float = 0.05,
                            thr: Optional[float] = None, min_neighbors: int = 0, plane_thr: Optional[float] = None,
                            grow: int = 3) -> np.ndarray:
        """키 높이대 [z_lo, z_hi] 에 점유가 있지만 **2D LiDAR 평면(plane_z ± plane_half)에는 점유가 없는** 열 — LiDAR 가 못 보는 장애물(낮은 물체,
        평면 위로 걸린 물체). LiDAR 가 보는 박스·UGV·벽은 LiDAR 쪽(클리어링 포함)에 맡기고, 깊이 카메라는 못 보는 것만 더한다.
        박스·벽 둘레로 번진 깊이 잡음(스테레오 가장자리 번짐)이 '못 보는 것'으로 새지 않게 평면 쪽 마스크를 grow 칸(기본 3칸 = 15 cm) 부풀린다 —
        어차피 박스 둘레 인플레이션(17 cm) 안이라 항법에는 손해가 없다."""
        key = ("blind", round(z_lo, 3), round(z_hi, 3), round(plane_z, 3), round(plane_half, 3), thr, min_neighbors, plane_thr, grow)
        hit = self._col_cache.get(key)
        if hit is not None and hit[0] == self.version:
            return hit[1]
        band = self.column_obstacles(z_lo, z_hi, thr, min_neighbors)
        plane = self.column_obstacles(plane_z - plane_half, plane_z + plane_half, plane_thr, 0)
        grown = plane.copy()
        for _ in range(max(grow, 0)):
            g2 = grown.copy()
            g2[1:] |= grown[:-1]
            g2[:-1] |= grown[1:]
            g2[:, 1:] |= grown[:, :-1]
            g2[:, :-1] |= grown[:, 1:]
            grown = g2
        out = band & ~grown
        self._col_cache[key] = (self.version, out)
        return out

    def forward_clearance(self, x: float, y: float, th: float, half_w: float, z_lo: float, z_hi: float, max_d: float,
                          thr: Optional[float] = None, min_neighbors: int = 0, col: Optional[np.ndarray] = None) -> float:
        """차체 앞쪽 폭 2·half_w 통로 안, 높이 [z_lo, z_hi] 점유까지의 가장 가까운 앞 거리(로봇 중심 기준). 없으면 max_d.
        col 을 주면 그 (y, x) 마스크를 쓴다(예: LiDAR 가 못 보는 열만)."""
        if col is None:
            col = self.column_obstacles(z_lo, z_hi, thr, min_neighbors)
        r = max_d + half_w + self.res
        ix0, iy0 = int((x - r - self.ox) / self.res), int((y - r - self.oy) / self.res)
        ix1, iy1 = int((x + r - self.ox) / self.res) + 1, int((y + r - self.oy) / self.res) + 1
        ix0, iy0, ix1, iy1 = max(ix0, 0), max(iy0, 0), min(ix1, self.nx), min(iy1, self.ny)
        if ix1 <= ix0 or iy1 <= iy0:
            return max_d
        ys, xs = np.nonzero(col[iy0:iy1, ix0:ix1])
        if len(xs) == 0:
            return max_d
        wx = self.ox + (xs + ix0 + 0.5) * self.res - x
        wy = self.oy + (ys + iy0 + 0.5) * self.res - y
        c, s = math.cos(th), math.sin(th)
        fwd, lat = c * wx + s * wy, -s * wx + c * wy
        m = (fwd > -0.02) & (fwd < max_d) & (np.abs(lat) < half_w)
        return float(fwd[m].min()) if m.any() else max_d

    # ------------------------------------------------------------------ 요약
    def box_clusters(self, thr: Optional[float] = None, min_cells: int = 6, z_lo: float = 0.10, margin: float = 0.14
                     ) -> List[Dict[str, Any]]:
        """점유 열의 8-연결 성분. 외벽 근처(margin)는 벽 끝점이라 뺀다. 각 성분: 중심 (x, y), 폭·깊이(m), 최대 높이(m), 칸 수."""
        occ = self.occupied(thr)
        k0 = int(math.floor(z_lo / self.res))
        col = occ[k0:].any(axis=0)
        H = self.height_map(thr)
        g = self.geo
        ys, xs = np.nonzero(col)
        cells = set()
        for iy, ix in zip(ys.tolist(), xs.tolist()):
            x, y = self.ox + (ix + 0.5) * self.res, self.oy + (iy + 0.5) * self.res
            if margin < x < g.W - margin and margin < y < g.H - margin:
                cells.add((iy, ix))
        out, seen = [], set()
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
            if len(comp) < min_cells:
                continue
            iy = [p[0] for p in comp]
            ix = [p[1] for p in comp]
            out.append({"x": self.ox + (min(ix) + max(ix) + 1) / 2 * self.res,
                        "y": self.oy + (min(iy) + max(iy) + 1) / 2 * self.res,
                        "w": (max(ix) - min(ix) + 1) * self.res, "d": (max(iy) - min(iy) + 1) * self.res,
                        "height": float(max(H[p] for p in comp)), "cells": len(comp)})
        return out

    def summary(self) -> Dict[str, Any]:
        occ = self.occupied()
        seen_col = self.seen.any(axis=0)
        inside = np.zeros((self.ny, self.nx), dtype=bool)
        x0, y0 = int((0 - self.ox) / self.res), int((0 - self.oy) / self.res)
        x1, y1 = int((self.geo.W - self.ox) / self.res), int((self.geo.H - self.oy) / self.res)
        inside[y0:y1, x0:x1] = True
        return {"frames": self.frames, "occupied_voxels": int(occ.sum()), "seen_voxels": int(self.seen.sum()),
                "coverage": round(float((seen_col & inside).sum() / max(inside.sum(), 1)), 3),
                "max_height_m": round(float(self.height_map().max()), 2)}

    # ------------------------------------------------------------------ 저장
    def occupied_points(self, thr: Optional[float] = None) -> np.ndarray:
        iz, iy, ix = np.nonzero(self.occupied(thr))
        return np.stack([self.ox + (ix + 0.5) * self.res, self.oy + (iy + 0.5) * self.res, (iz + 0.5) * self.res], axis=1)

    def save_npz(self, path: str) -> None:
        np.savez_compressed(path, L=self.L, seen=self.seen, origin=np.array([self.ox, self.oy, 0.0]), res=self.res)

    def save_ply(self, path: str, thr: Optional[float] = None) -> int:
        """점유 복셀 중심을 ASCII PLY 로. 색은 높이(낮음 파랑 → 높음 빨강)."""
        P = self.occupied_points(thr)
        z = np.clip(P[:, 2] / max(self.cfg.z_max, 1e-6), 0, 1) if len(P) else np.zeros(0)
        with open(path, "w", encoding="utf-8") as f:
            f.write("ply\nformat ascii 1.0\nelement vertex %d\nproperty float x\nproperty float y\nproperty float z\n"
                    "property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n" % len(P))
            for (x, y, zz), h in zip(P.tolist(), z.tolist()):
                f.write("%.3f %.3f %.3f %d %d %d\n" % (x, y, zz, int(255 * h), 60, int(255 * (1 - h))))
        return len(P)
