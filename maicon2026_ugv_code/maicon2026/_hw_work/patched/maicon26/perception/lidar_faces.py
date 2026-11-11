"""2D LiDAR 점유격자에서 장애물의 '면'을 기록한다(M3 식별 미션의 바탕 지도). 카메라·깊이·UAV 를 쓰지 않는다.

왜 새로 만드나. 기존 군집(coverage._clusters)은 40 cm 한 칸·80 cm 두 칸 모양만 박스로 인정해서, 정육면체 장애물이 이어 붙어
L 자·T 자·세 칸 줄이 되면 박스가 하나도 안 만들어졌다. 여기서는 모양을 가정하지 않고 면(직선 선분)부터 찾는다.

절차
  1. 점유 셀 = (L > L_THR, 두 번 이상 맞은 셀) − 정적 벽 − UGV 주변.  UGV 는 면·장애물 기록에 넣지 않는다(아래 UGV 항목).
  2. 연결된 점유 셀 덩어리(그룹)마다 축 정렬 직선 구간(런)을 찾는다. 길이 ≥ 40 cm(× 0.9 허용)만 '면'으로 기록한다.
  3. 면의 바깥 쪽 = 광선이 지나가 자유 증거(L ≤ −0.3)가 쌓인 쪽. 양쪽이 다 자유면 얇은 물체(울타리·막대), 어느 쪽도 아니면 아직 모른다 → 기록하지 않는다.
  4. 면을 40 cm 단위로 쪼개 정육면체 가설(Unit)을 만든다. 면 중심에서 법선 반대로 20 cm 들어간 곳이 칸 중심이다.
     이웃 칸과 맞닿은 면·벽에 붙은 면은 '안쪽 면'이라 볼 수도 식별할 수도 없다(internal).
  5. 닫힌 도형 검사: 그룹의 점유 셀 + 기록한 면 + 벽을 막으로 놓고 바깥에서 4방향으로 물을 채운다. 안쪽에 물이 안 닿는 영역이 있으면 닫힌 도형이다.
     닫히지 않았으면 뒷면을 아직 못 본 것이다 → 물이 샌 쪽에 가까운 '미관측 칸' 무리의 중심을 정찰 지점으로 내놓는다.

UGV 와 장애물 구분(LiDAR 높이 한계)
  2D LiDAR 는 UGV 차체의 윗부분만 본다(궤도 부분은 LiDAR 평면 아래라 안 보일 수 있다). 그래서 '차체 크기'로 UGV 를 가르지 않는다.
  ① 외부에서 받은 UGV 위치(트래커·움직이는 LiDAR 덩어리·카메라 UGV 검출)의 반경 안 셀은 지운다.
  ② 40 cm 가 넘는 정적 직선 면만 기록하므로, 25 cm 급 UGV 는 면을 만들 수 없다(위치를 놓쳐도 면으로 기록되지 않는다).
  ③ UGV 가 장애물 옆에 붙어 있으면 그 그룹은 '닫힘 판정 보류'(ugv_near)로 두고 정찰하지 않는다(가려진 쪽을 열린 것으로 오인하지 않게).
  항법에서는 UGV 를 계속 피한다 — 이 모듈은 면 기록만 다룬다."""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional, Sequence, Set, Tuple

import numpy as np

from ..config import Config
from ..nav.grid import OccupancyGrid

FACE_N = [(1, 0), (0, 1), (-1, 0), (0, -1)]            # E N W S — coverage.FACE_N 과 같은 순서
FACE_IDX = {n: i for i, n in enumerate(FACE_N)}
FREE_THR = -0.3                                         # 이보다 낮으면 '광선이 지나간 자유 공간'
WALL_MARGIN = 0.14                                      # coverage._clusters 와 같다: 벽에 붙은 셀은 박스 증거로 안 쓴다
Pt = Tuple[float, float]


@dataclass
class LidarFace:
    fid: int
    gid: int
    axis: str                  # "x": 선분이 x 방향(법선 ±y) | "y": 선분이 y 방향(법선 ±x)
    coord: float               # 선분이 놓인 좌표(axis=x 면 y, axis=y 면 x)
    a0: float                  # 선분 범위(axis 방향 좌표)
    a1: float
    nx: int
    ny: int
    support: int               # 선분을 받치는 점유 셀 수
    outside_L: float           # 바깥 쪽 평균 L(낮을수록 자유 증거가 강하다)

    @property
    def length(self) -> float:
        return self.a1 - self.a0

    @property
    def center(self) -> Pt:
        m = 0.5 * (self.a0 + self.a1)
        return (m, self.coord) if self.axis == "x" else (self.coord, m)


@dataclass
class Unit:
    """40 cm 정육면체 가설 하나. faces: 면 인덱스(E N W S) → 그 칸에서 본 선분."""
    cx: float
    cy: float
    gid: int
    faces: Dict[int, LidarFace] = field(default_factory=dict)
    internal: Set[int] = field(default_factory=set)      # 이웃 칸·벽에 가려진 면(볼 수 없다)
    n: int = 1
    closed: Optional[bool] = None
    kind: str = "face"                                   # "full": 두 면 이상 또는 닫힌 그룹 | "face": 한 면만 본 칸


@dataclass
class Group:
    gid: int
    cx: float
    cy: float
    n_cells: int
    faces: List[LidarFace] = field(default_factory=list)
    units: List[Unit] = field(default_factory=list)
    closed: Optional[bool] = None          # True 닫힘 | False 열림(뒷면 미관측) | None 판정 보류(면 없음·UGV 근처)
    ugv_near: bool = False
    touches_wall: bool = False
    scout_points: List[Tuple[float, float, float, float]] = field(default_factory=list)   # (x, y, 바깥 방향 dx, dy) 미관측 쪽
    cells: List[Tuple[int, int]] = field(default_factory=list)


@dataclass
class FaceMapResult:
    groups: List[Group]
    faces: List[LidarFace]
    units: List[Unit]
    stats: Dict[str, int]

    def group(self, gid: int) -> Optional[Group]:
        return next((g for g in self.groups if g.gid == gid), None)


def _runs_along_rows(work: np.ndarray, min_cells: int, gap: int = 1):
    """work(행=직선이 놓인 방향과 수직, 열=직선 방향) 의 가로 직선 구간을 찾는다. 띠(세 행)를 한 번에 소비하며 반복.
    반환: [(행 평균, 열 시작, 열 끝, 지지 셀 수)] — 행 평균은 실수 인덱스."""
    work = work.copy()
    out = []
    n_rows = work.shape[0]
    while True:
        cnt = work.sum(axis=1).astype(int)
        band = cnt.copy()
        band[1:] += cnt[:-1]
        band[:-1] += cnt[1:]
        r = int(np.argmax(band))
        if band[r] < min_cells:
            break
        r0, r1 = max(r - 1, 0), min(r + 2, n_rows)
        cols = np.nonzero(work[r0:r1].any(axis=0))[0]
        if len(cols) == 0:
            break
        splits = np.nonzero(np.diff(cols) > gap + 1)[0]
        starts = np.concatenate(([0], splits + 1))
        ends = np.concatenate((splits, [len(cols) - 1]))
        for s, e in zip(starts, ends):
            c0, c1 = int(cols[s]), int(cols[e])
            if c1 - c0 + 1 < min_cells:
                continue
            sub = work[r0:r1, c0:c1 + 1]
            # 면의 위치(행 평균)는 양 끝 두 열을 뺀 안쪽 열로 낸다 — 끝 열에는 직각으로 만나는 이웃 면의 셀이 섞여 위치가 안쪽으로 끌린다.
            inner = sub[:, 2:-2] if sub.shape[1] >= 8 else sub
            rr, _ = np.nonzero(inner if inner.any() else sub)
            out.append((float(r0 + rr.mean()), c0, c1, int(sub.sum())))
        work[r0:r1, :] = False
    return out


class LidarFaceMap:
    """점유격자 → 그룹·면·칸. 호출은 1 Hz 안팎(coverage.refresh 주기)이면 충분하다. 상태는 그룹 번호 유지뿐이다."""

    def __init__(self, cfg: Config) -> None:
        self.cfg, self.ec, self.geo = cfg, cfg.explore, cfg.arena
        self._prev: List[Tuple[int, float, float]] = []       # (gid, cx, cy)
        self._next_gid = 1
        self._next_fid = 1
        self.last: Optional[FaceMapResult] = None

    # ------------------------------------------------------------------ 입력 정리
    def _occupied(self, grid: OccupancyGrid, ugv_points: Sequence[Tuple[float, float, float]]) -> Tuple[np.ndarray, int]:
        ec = self.ec
        occ = (grid.L > max(grid.L_THR, ec.lidar_face_min_L)) & ~grid.static
        ys, xs = np.mgrid[0:grid.ny, 0:grid.nx]
        X = grid.ox + (xs + 0.5) * grid.res
        Y = grid.oy + (ys + 0.5) * grid.res
        wm = WALL_MARGIN
        occ &= (X > wm) & (X < self.geo.W - wm) & (Y > wm) & (Y < self.geo.H - wm)
        removed = 0
        for px, py, pr in ugv_points:
            m = occ & ((X - px) ** 2 + (Y - py) ** 2 <= max(pr, 0.0) ** 2)
            removed += int(m.sum())
            occ &= ~m
        return occ, removed

    @staticmethod
    def _components(occ: np.ndarray, reach: int = 2) -> List[List[Tuple[int, int]]]:
        """8방향에 한 칸 틈까지 이어 붙인 덩어리(가는 선의 한 칸 끊김을 메운다)."""
        cells = set(zip(*[a.tolist() for a in np.nonzero(occ)]))
        offs = [(dy, dx) for dy in range(-reach, reach + 1) for dx in range(-reach, reach + 1) if (dy or dx)]
        out, seen = [], set()
        for c in cells:
            if c in seen:
                continue
            comp, st = [], [c]
            seen.add(c)
            while st:
                y, x = st.pop()
                comp.append((y, x))
                for dy, dx in offs:
                    n = (y + dy, x + dx)
                    if n in cells and n not in seen:
                        seen.add(n)
                        st.append(n)
            out.append(comp)
        return out

    # ------------------------------------------------------------------ 면 찾기
    def _outside(self, grid: OccupancyGrid, f: LidarFace) -> Optional[Tuple[int, int, float]]:
        """선분 양쪽의 자유 증거로 바깥 방향을 정한다. (nx, ny, 바깥 쪽 평균 L) 또는 None(얇은 물체·미관측)."""
        pts = np.linspace(f.a0 + 0.05, f.a1 - 0.05, 7) if f.length > 0.12 else np.array([0.5 * (f.a0 + f.a1)])
        vals = {}
        for sgn in (+1, -1):
            acc = []
            for a in pts:
                for d in (0.12, 0.18):
                    x, y = (a, f.coord + sgn * d) if f.axis == "x" else (f.coord + sgn * d, a)
                    iy, ix = grid.to_cell(x, y)
                    acc.append(float(grid.L[iy, ix]) if grid.in_bounds(iy, ix) else 0.0)
            vals[sgn] = float(np.mean(acc))
        free = [s for s in (+1, -1) if vals[s] <= FREE_THR]
        if len(free) != 1:
            return None                                       # 양쪽 다 자유 = 얇은 물체 / 어느 쪽도 아님 = 바깥을 모른다
        s = free[0]
        nx, ny = (0, s) if f.axis == "x" else (s, 0)
        return nx, ny, vals[s]

    def _find_faces(self, grid: OccupancyGrid, gid: int, comp: List[Tuple[int, int]]) -> Tuple[List[LidarFace], Dict[str, int]]:
        res = grid.res
        min_len = self.ec.lidar_face_min_len_m
        min_cells = max(int(math.ceil(min_len * 0.9 / res - 1e-9)), 3)
        ys = np.array([c[0] for c in comp])
        xs = np.array([c[1] for c in comp])
        iy0, ix0 = int(ys.min()), int(xs.min())
        sub = np.zeros((int(ys.max()) - iy0 + 1, int(xs.max()) - ix0 + 1), dtype=bool)
        sub[ys - iy0, xs - ix0] = True
        stats = {"runs": 0, "thin_or_unknown_outside": 0}
        faces: List[LidarFace] = []
        for axis, arr in (("x", sub), ("y", sub.T)):
            for r_mean, c0, c1, support in _runs_along_rows(arr, min_cells):
                stats["runs"] += 1
                if axis == "x":                                  # 행 = y, 열 = x
                    coord = grid.oy + (iy0 + r_mean + 0.5) * res
                    a0, a1 = grid.ox + (ix0 + c0) * res, grid.ox + (ix0 + c1 + 1) * res
                else:                                            # 전치: 행 = x, 열 = y
                    coord = grid.ox + (ix0 + r_mean + 0.5) * res
                    a0, a1 = grid.oy + (iy0 + c0) * res, grid.oy + (iy0 + c1 + 1) * res
                f = LidarFace(self._next_fid, gid, axis, float(coord), float(a0), float(a1), 0, 0, support, 0.0)
                out = self._outside(grid, f)
                if out is None:
                    stats["thin_or_unknown_outside"] += 1
                    continue
                f.nx, f.ny, f.outside_L = out
                self._next_fid += 1
                faces.append(f)
        return faces, stats

    # ------------------------------------------------------------------ 칸(40 cm) 만들기
    def _units(self, gid: int, faces: List[LidarFace]) -> List[Unit]:
        size = self.geo.box_size
        units: List[Unit] = []
        for f in faces:
            n = max(1, int(round(f.length / size)))
            step = f.length / n
            idx = FACE_IDX[(f.nx, f.ny)]
            for i in range(n):
                a = f.a0 + (i + 0.5) * step
                mx, my = (a, f.coord) if f.axis == "x" else (f.coord, a)
                cx, cy = mx - f.nx * size / 2, my - f.ny * size / 2
                for u in units:
                    if math.hypot(u.cx - cx, u.cy - cy) < 0.16:
                        u.cx, u.cy = (u.cx * u.n + cx) / (u.n + 1), (u.cy * u.n + cy) / (u.n + 1)
                        u.n += 1
                        u.faces.setdefault(idx, f)
                        break
                else:
                    units.append(Unit(cx, cy, gid, {idx: f}))
        size_tol = 0.14
        for u in units:                                          # 안쪽 면: 이웃 칸과 맞닿은 면
            for v in units:
                if v is u:
                    continue
                dx, dy = v.cx - u.cx, v.cy - u.cy
                if abs(abs(dx) - size) < size_tol and abs(dy) < size_tol:
                    u.internal.add(FACE_IDX[(1 if dx > 0 else -1, 0)])
                elif abs(abs(dy) - size) < size_tol and abs(dx) < size_tol:
                    u.internal.add(FACE_IDX[(0, 1 if dy > 0 else -1)])
            W, H = self.geo.W, self.geo.H                        # 벽에 붙은 면
            h = size / 2 + 0.08
            if W - u.cx < h:
                u.internal.add(0)
            if H - u.cy < h:
                u.internal.add(1)
            if u.cx < h:
                u.internal.add(2)
            if u.cy < h:
                u.internal.add(3)
            for i in list(u.faces):
                if i in u.internal:
                    u.faces.pop(i)                               # 이웃·벽에 가려진 면으로 잡힌 선분은 잡음이다
        return units

    # ------------------------------------------------------------------ 닫힌 도형
    def _closure(self, grid: OccupancyGrid, comp: List[Tuple[int, int]], faces: List[LidarFace]):
        """(닫힘?, 벽에 닿음?, 미관측 쪽 점 목록). 막 = 점유 셀 + 기록한 면 + 벽, 1칸 팽창, 바깥에서 4방향 물채우기."""
        res, pad = grid.res, 6
        ys = np.array([c[0] for c in comp])
        xs = np.array([c[1] for c in comp])
        y0, x0 = int(ys.min()) - pad, int(xs.min()) - pad
        y1, x1 = int(ys.max()) + pad + 1, int(xs.max()) + pad + 1
        h, w = y1 - y0, x1 - x0
        barrier = np.zeros((h, w), dtype=bool)
        barrier[ys - y0, xs - x0] = True
        for f in faces:
            ia, ib = (int((f.a0 - grid.ox) / res), int((f.a1 - grid.ox) / res)) if f.axis == "x" else (int((f.a0 - grid.oy) / res), int((f.a1 - grid.oy) / res))
            ic = int(((f.coord - grid.oy) if f.axis == "x" else (f.coord - grid.ox)) / res)
            for k in range(ia, ib + 1):
                iy, ix = (ic, k) if f.axis == "x" else (k, ic)
                if 0 <= iy - y0 < h and 0 <= ix - x0 < w:
                    barrier[iy - y0, ix - x0] = True
        # 벽(경기장 밖·벽 띠)
        yy, xx = np.mgrid[y0:y1, x0:x1]
        X = grid.ox + (xx + 0.5) * res
        Y = grid.oy + (yy + 0.5) * res
        wall = (X < WALL_MARGIN) | (X > self.geo.W - WALL_MARGIN) | (Y < WALL_MARGIN) | (Y > self.geo.H - WALL_MARGIN)
        inb = (yy >= 0) & (yy < grid.ny) & (xx >= 0) & (xx < grid.nx)
        st = np.zeros((h, w), dtype=bool)
        st[inb] = grid.static[yy[inb], xx[inb]]
        wall |= st | ~inb
        touches_wall = bool((wall & _dilate(barrier, 2)).any())
        d = _dilate(barrier, 1) | wall
        reached = np.zeros((h, w), dtype=bool)
        dq = deque()
        for iy in range(h):
            for ix in (0, w - 1):
                if not d[iy, ix] and not reached[iy, ix]:
                    reached[iy, ix] = True
                    dq.append((iy, ix))
        for ix in range(w):
            for iy in (0, h - 1):
                if not d[iy, ix] and not reached[iy, ix]:
                    reached[iy, ix] = True
                    dq.append((iy, ix))
        while dq:
            iy, ix = dq.popleft()
            for ny_, nx_ in ((iy - 1, ix), (iy + 1, ix), (iy, ix - 1), (iy, ix + 1)):
                if 0 <= ny_ < h and 0 <= nx_ < w and not d[ny_, nx_] and not reached[ny_, nx_]:
                    reached[ny_, nx_] = True
                    dq.append((ny_, nx_))
        interior = ~d & ~reached
        closed = int(interior.sum()) >= self.ec.lidar_closure_min_cells
        scout: List[Tuple[float, float, float, float]] = []
        if not closed:
            L = np.zeros((h, w), dtype=np.float32)
            L[inb] = grid.L[yy[inb], xx[inb]]
            near = _dilate(barrier, 4) & reached & ~d
            unknown = near & (L > FREE_THR) & ~wall                # 그룹 가까이에 있으면서 한 번도 자유로 확인된 적 없는 칸 = 가려진 뒷면
            scout = self._cluster_points(unknown, grid, y0, x0, (float(np.mean(xs)) * res + grid.ox, float(np.mean(ys)) * res + grid.oy))
        return closed, touches_wall, scout

    @staticmethod
    def _cluster_points(mask: np.ndarray, grid: OccupancyGrid, y0: int, x0: int, centroid: Pt, max_n: int = 3):
        cells = set(zip(*[a.tolist() for a in np.nonzero(mask)]))
        out, seen = [], set()
        for c in sorted(cells):
            if c in seen:
                continue
            comp, st = [], [c]
            seen.add(c)
            while st:
                y, x = st.pop()
                comp.append((y, x))
                for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
                    n = (y + dy, x + dx)
                    if n in cells and n not in seen:
                        seen.add(n)
                        st.append(n)
            if len(comp) < 6:
                continue
            my = float(np.mean([p[0] for p in comp])) + y0
            mx = float(np.mean([p[1] for p in comp])) + x0
            px, py = grid.ox + (mx + 0.5) * grid.res, grid.oy + (my + 0.5) * grid.res
            dx, dy = px - centroid[0], py - centroid[1]
            n = math.hypot(dx, dy) or 1.0
            out.append((px, py, dx / n, dy / n, len(comp)))
        out.sort(key=lambda t: -t[4])
        return [(p[0], p[1], p[2], p[3]) for p in out[:max_n]]

    # ------------------------------------------------------------------ 한 번 갱신
    def update(self, grid: OccupancyGrid, ugv_points: Sequence[Tuple[float, float, float]] = (), now: float = 0.0) -> FaceMapResult:
        ec = self.ec
        occ, removed = self._occupied(grid, ugv_points)
        comps = self._components(occ)
        stats = {"components": len(comps), "ugv_cells_removed": removed, "faces": 0, "units": 0, "closed": 0, "open": 0,
                 "deferred": 0, "no_face": 0, "runs": 0, "thin_or_unknown_outside": 0}
        groups: List[Group] = []
        used: Set[int] = set()
        for comp in comps:
            if len(comp) < 5:
                continue
            cx = grid.ox + (float(np.mean([c[1] for c in comp])) + 0.5) * grid.res
            cy = grid.oy + (float(np.mean([c[0] for c in comp])) + 0.5) * grid.res
            gid = None
            best = 0.35
            for pg, px, py in self._prev:
                d = math.hypot(px - cx, py - cy)
                if pg not in used and d < best:
                    gid, best = pg, d
            if gid is None:
                gid = self._next_gid
                self._next_gid += 1
            used.add(gid)
            faces, fs = self._find_faces(grid, gid, comp)
            for k in ("runs", "thin_or_unknown_outside"):
                stats[k] += fs[k]
            g = Group(gid, cx, cy, len(comp), faces=faces, cells=comp)
            near_ugv = any(math.hypot(px - cx, py - cy) <= max(pr, 0.0) + 0.55 for px, py, pr in ugv_points)
            g.ugv_near = near_ugv
            if faces:
                g.units = self._units(gid, faces)
                g.faces = [f for f in faces if any(f in u.faces.values() for u in g.units)] or faces
                closed, wall, scout = self._closure(grid, comp, g.faces)
                g.touches_wall = wall
                if near_ugv:
                    g.closed = None                                  # UGV 가 가린 쪽을 '열림'으로 오인하지 않는다
                    stats["deferred"] += 1
                else:
                    g.closed = closed
                    g.scout_points = [] if closed else scout
                    stats["closed" if closed else "open"] += 1
                for u in g.units:
                    u.closed = g.closed                              # True 닫힘 | False 열림 | None 판정 보류(UGV 근처)
                    u.kind = "full" if (g.closed is True or len(u.faces) >= 2) else "face"
            else:
                stats["no_face"] += 1
            groups.append(g)
        self._prev = [(g.gid, g.cx, g.cy) for g in groups]
        all_faces = [f for g in groups for f in g.faces]
        all_units = [u for g in groups for u in g.units]
        stats["faces"], stats["units"] = len(all_faces), len(all_units)
        self.last = FaceMapResult(groups, all_faces, all_units, stats)
        return self.last


def _dilate(a: np.ndarray, r: int) -> np.ndarray:
    """정사각 팽창(반경 r 칸)."""
    out = a.copy()
    for _ in range(r):
        n = out.copy()
        n[1:] |= out[:-1]
        n[:-1] |= out[1:]
        m = n.copy()
        m[:, 1:] |= n[:, :-1]
        m[:, :-1] |= n[:, 1:]
        out = m
    return out


def ugv_points_from(tracks: Iterable, blobs: Iterable, now: float, radius: float, max_track_age_s: float = 4.0,
                    extra: Iterable[Tuple[float, float]] = ()) -> List[Tuple[float, float, float]]:
    """UGV 로 알려진 위치(트래커 트랙 — 팀 무관, 움직이는 LiDAR 덩어리, 카메라 UGV 검출 위치)를 (x, y, 반경) 목록으로.
    UGV 는 장애물·면 기록에서 빼야 하지만 항법에서는 계속 피해야 하므로 이 목록은 기록 쪽에서만 쓴다."""
    out: List[Tuple[float, float, float]] = []
    for tr in tracks:
        if now - getattr(tr, "t", -1e9) > max_track_age_s:
            continue
        try:
            x, y = tr.predicted(now)
        except Exception:
            x, y = tr.x, tr.y
        out.append((float(x), float(y), radius))
    for b in blobs:
        out.append((float(b.x), float(b.y), radius))
    for x, y in extra:
        out.append((float(x), float(y), radius))
    return out
