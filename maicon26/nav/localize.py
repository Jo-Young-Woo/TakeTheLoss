"""자세 추정(내장 SLAM): 오도메트리 + 파티클 필터, LiDAR를 '경기장 외벽 + 지금까지 만든 지도'에 매칭.

HAL 오도메트리는 '시작 기준 (0,0,0)'이므로 월드 자세로 바꾸는 일이 필요하다.
시작 거점(팀/슬롯)을 알면 초기 자세를 알 수 있다.

두 종류의 지형지물을 쓴다.
  - 외벽 4면: 항상 같다 → 앵커. 드리프트를 묶어 준다.
  - SLAM 지도의 박스(MapLikelihood): 경기마다, 경기 중에도 바뀌므로 '확실히 점유된(여러 번 관측된)'
    셀만 쓰고, 벽보다 약한 가중으로 쓴다. 박스가 벽을 가리는 구간에서도 위치가 유지되고,
    박스가 옮겨져도 (지도가 갱신될 때까지) 바닥 확률 덕에 위치가 무너지지 않는다.
    박스에 맞은 빔이 지도에 없는 경우도 바닥 확률(floor)로 강건하다.

정직한 한계: 직사각형 외벽은 180° 대칭이라 '전역 위치추정'은 못 한다. 반드시 알려진 시작
자세에서 추적한다. ROS2에서는 SLAM(Cartographer/slam_toolbox)이 이 역할을 한다(nav2_stack.py).
"""
from __future__ import annotations

import math
from typing import Optional

import numpy as np

from ..arena import ArenaGeometry
from ..types import Pose2D, Scan
from ..util import wrap
from .grid import OccupancyGrid


class MapLikelihood:
    """SLAM 지도에서 '확실히 점유된 비벽 셀'까지의 거리장(우도장). 0.5 s 이하 주기로 갱신한다."""

    def __init__(self, max_dist: float = 0.30, conf_logodds: float = 2.0, wall_margin: float = 0.14) -> None:
        self.max_dist, self.conf, self.wm = max_dist, conf_logodds, wall_margin
        self.dist: Optional[np.ndarray] = None
        self._grid: Optional[OccupancyGrid] = None
        self.n_cells = 0

    def refresh(self, grid: OccupancyGrid) -> None:
        occ = (grid.L > self.conf) & ~grid.static
        # 외벽 끝점은 PoseEstimator가 해석적 거리로 이미 쓰므로 제외(중복 가중 방지)
        xs = grid.ox + (np.arange(grid.nx) + 0.5) * grid.res
        ys = grid.oy + (np.arange(grid.ny) + 0.5) * grid.res
        W, H, wm = grid.geo.W, grid.geo.H, self.wm
        inner_x = (xs > wm) & (xs < W - wm)
        inner_y = (ys > wm) & (ys < H - wm)
        occ &= inner_y[:, None] & inner_x[None, :]
        self.n_cells = int(occ.sum())
        self._grid = grid
        self.dist = grid.distance_field(occ, self.max_dist) if self.n_cells else None

    def lookup(self, ex: np.ndarray, ey: np.ndarray) -> Optional[np.ndarray]:
        """끝점 배열(입자×빔)의 지도 거리. 지도에 박스가 없으면 None."""
        g = self._grid
        if self.dist is None or g is None:
            return None
        ix = ((ex - g.ox) / g.res).astype(int)
        iy = ((ey - g.oy) / g.res).astype(int)
        ok = (ix >= 0) & (ix < g.nx) & (iy >= 0) & (iy < g.ny)
        d = np.full(ex.shape, self.max_dist, dtype=np.float32)
        d[ok] = self.dist[iy[ok], ix[ok]]
        return d


class PoseEstimator:
    def __init__(self, geo: ArenaGeometry, n: int = 160, seed: int = 0,
                 sigma_hit: float = 0.07, floor: float = 0.30) -> None:
        self.geo = geo
        self.n = n
        self.rng = np.random.default_rng(seed)
        self.sigma, self.floor = sigma_hit, floor
        self.p = np.zeros((n, 3))
        self.wt = np.full(n, 1.0 / n)
        self._odom0: Optional[Pose2D] = None
        self.initialized = False
        self.pose = Pose2D()
        self.updates = 0

    # ------------------------------------------------------------------ 초기화
    def init(self, pose: Pose2D, odom: Pose2D, sd_xy: float = 0.06, sd_th: float = 0.10) -> None:
        self.p[:, 0] = pose.x + self.rng.normal(0, sd_xy, self.n)
        self.p[:, 1] = pose.y + self.rng.normal(0, sd_xy, self.n)
        self.p[:, 2] = pose.th + self.rng.normal(0, sd_th, self.n)
        self.wt[:] = 1.0 / self.n
        self._odom0 = odom.copy()
        self.pose = pose.copy()
        self.initialized = True

    # ------------------------------------------------------------------ 예측(오도메트리 증분)
    def predict(self, odom: Pose2D) -> None:
        if not self.initialized:
            return
        o = self._odom0
        dx, dy = odom.x - o.x, odom.y - o.y
        dth = wrap(odom.th - o.th)
        # 증분을 이전 오도메트리 프레임의 로봇 좌표로
        c, s = math.cos(o.th), math.sin(o.th)
        fx, fy = c * dx + s * dy, -s * dx + c * dy
        trans = math.hypot(fx, fy)
        self._odom0 = odom.copy()
        n = self.n
        nx = self.rng.normal(0, 0.012 + 0.04 * trans, n)
        ny = self.rng.normal(0, 0.012 + 0.04 * trans, n)
        nth = self.rng.normal(0, 0.004 + 0.06 * abs(dth) + 0.02 * trans, n)
        th = self.p[:, 2]
        self.p[:, 0] += np.cos(th) * (fx + nx) - np.sin(th) * (fy + ny)
        self.p[:, 1] += np.sin(th) * (fx + nx) + np.cos(th) * (fy + ny)
        self.p[:, 2] = th + dth + nth
        self._estimate()

    # ------------------------------------------------------------------ 보정(LiDAR)
    def correct(self, scan: Scan, n_rays: int = 36, field: Optional[MapLikelihood] = None) -> None:
        if not self.initialized:
            return
        v = scan.valid() & (scan.ranges > 0.15) & (scan.ranges < 6.0)
        idx = np.flatnonzero(v)
        if len(idx) < 8:
            return
        sel = idx[np.linspace(0, len(idx) - 1, min(n_rays, len(idx))).astype(int)]
        ang = scan.angles[sel]
        r = scan.ranges[sel]
        th = self.p[:, 2:3] + ang[None, :]
        ex = self.p[:, 0:1] + r[None, :] * np.cos(th)
        ey = self.p[:, 1:2] + r[None, :] * np.sin(th)
        W, H = self.geo.W, self.geo.H
        d = np.minimum(np.minimum(ex, W - ex), np.minimum(ey, H - ey))
        d = np.abs(d)                                  # 벽 밖으로 나간 끝점도 거리로 처리
        if field is not None:                          # SLAM 지도의 박스에 맞은 빔도 '맞는 빔'으로 인정
            dm = field.lookup(ex, ey)
            if dm is not None:
                d = np.minimum(d, dm + 0.02)           # +0.02: 박스는 벽보다 약간 불확실하게 취급
        like = self.floor + (1 - self.floor) * np.exp(-0.5 * (d / self.sigma) ** 2)
        ll = np.log(like).sum(axis=1)
        # 벽 밖으로 나간 입자는 강하게 벌점
        out = (self.p[:, 0] < 0) | (self.p[:, 0] > W) | (self.p[:, 1] < 0) | (self.p[:, 1] > H)
        ll[out] -= 30.0
        ll -= ll.max()
        w = np.exp(ll * 0.5) * self.wt             # 0.5: 빔 간 상관을 감안해 우도를 눌러줌
        s = w.sum()
        if not np.isfinite(s) or s <= 1e-300:
            return
        self.wt = w / s
        neff = 1.0 / np.sum(self.wt ** 2)
        if neff < 0.5 * self.n:
            self._resample()
        self.updates += 1
        self._estimate()

    def _resample(self) -> None:
        pos = (self.rng.random() + np.arange(self.n)) / self.n
        cs = np.cumsum(self.wt)
        cs[-1] = 1.0
        idx = np.searchsorted(cs, pos)
        self.p = self.p[idx]
        self.p[:, 0] += self.rng.normal(0, 0.006, self.n)
        self.p[:, 1] += self.rng.normal(0, 0.006, self.n)
        self.p[:, 2] += self.rng.normal(0, 0.004, self.n)
        self.wt[:] = 1.0 / self.n

    def _estimate(self) -> None:
        w = self.wt
        self.pose = Pose2D(float(np.sum(w * self.p[:, 0])), float(np.sum(w * self.p[:, 1])),
                           float(math.atan2(np.sum(w * np.sin(self.p[:, 2])), np.sum(w * np.cos(self.p[:, 2])))))

    def spread(self) -> float:
        """위치 불확실도(입자 표준편차 m)."""
        m = self.pose
        return float(math.sqrt(np.sum(self.wt * ((self.p[:, 0] - m.x) ** 2 + (self.p[:, 1] - m.y) ** 2))))


def heading_from_nearest_wall(scan: Scan, geo: ArenaGeometry, team: str, x: float, y: float) -> float:
    """시작 자세의 방향을 모를 때: 가장 가까운 벽이 자기 진영 뒤쪽 벽(BLUE=왼쪽 x=0, RED=오른쪽)이라고
    가정해 θ를 구한다. 로봇이 거점 안에서 벽을 등지고 있지 않아도, 거점은 그 벽에 붙어 있다는 사실을 이용."""
    v = scan.valid()
    if not v.any():
        return 0.0 if team == "BLUE" else math.pi
    k = int(np.argmin(np.where(v, scan.ranges, np.inf)))
    phi = float(scan.angles[k])                   # 로봇 기준 가장 가까운 점의 방향
    wall_dir = math.pi if team == "BLUE" else 0.0  # 월드 기준 그 벽의 방향
    return wrap(wall_dir - phi)


def init_pose_from_scan(scan: Scan, geo: ArenaGeometry, x0: float, y0: float, search: float = 0.25,
                        step: float = 0.05, sigma: float = 0.05) -> Pose2D:
    """시작 자세 추정: 대략의 시작 위치(x0,y0 ± search)와 모든 방향 θ 후보 중, LiDAR 끝점이
    경기장 외벽 위에 가장 많이 놓이는 조합을 고른다. 박스가 앞을 가려도(가까운 점이 벽이 아니어도)
    벽에 맞은 빔이 다수라서 안정적이다. 직사각형이라 180° 대칭이지만 시작 위치가 한쪽 벽에 붙어 있어
    (거점 = 벽 쪽) 대칭이 깨진다."""
    v = scan.valid() & (scan.ranges > 0.15) & (scan.ranges < 7.5)
    idx = np.flatnonzero(v)
    if len(idx) < 12:
        return Pose2D(x0, y0, 0.0)
    sel = idx[np.linspace(0, len(idx) - 1, min(120, len(idx))).astype(int)]
    ang, r = scan.angles[sel], scan.ranges[sel]
    W, H = geo.W, geo.H

    def search_grid(cx: float, cy: float, half: float, st: float, ths: np.ndarray, sg: float):
        best, bp = -1.0, None
        offs = np.arange(-half, half + 1e-9, st)
        for dx in offs:
            for dy in offs:
                x, y = cx + dx, cy + dy
                if not (0.1 < x < W - 0.1 and 0.1 < y < H - 0.1):
                    continue
                a = ths[:, None] + ang[None, :]
                ex, ey = x + r[None, :] * np.cos(a), y + r[None, :] * np.sin(a)
                d = np.minimum(np.minimum(ex, W - ex), np.minimum(ey, H - ey))
                sc = np.exp(-0.5 * (d / sg) ** 2).sum(axis=1)
                k = int(np.argmax(sc))
                if sc[k] > best:
                    best, bp = float(sc[k]), Pose2D(x, y, float(ths[k]))
        return bp

    bp = search_grid(x0, y0, search, step, np.deg2rad(np.arange(-180, 180, 1.5)), sigma)
    if bp is None:
        return Pose2D(x0, y0, 0.0)
    # 2단계 미세 정렬: SLAM 앵커가 한 번의 추정에 평생 의존하므로 격자 양자화 오차(최대 ~3.5 cm)를 줄인다
    fine = search_grid(bp.x, bp.y, 0.04, 0.01, bp.th + np.deg2rad(np.arange(-1.5, 1.5 + 1e-9, 0.25)), 0.03)
    return fine or bp
