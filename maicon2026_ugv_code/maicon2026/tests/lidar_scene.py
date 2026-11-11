"""LiDAR 면 지도 시험용 합성 장면: numpy 광선 투사로 만든 스캔을 실제 OccupancyGrid 에 먹인다. 시뮬을 쓰지 않는다."""
import math

import numpy as np

from maicon26.config import Config
from maicon26.nav.grid import OccupancyGrid
from maicon26.perception.lidar_faces import LidarFaceMap
from maicon26.types import Pose2D, Scan

C = 0.40


def cube(cx, cy, s=C):
    return (cx - s / 2, cy - s / 2, cx + s / 2, cy + s / 2)


def _ray_aabb(ox, oy, dx, dy, r):
    x0, y0, x1, y1 = r
    tmin, tmax = 0.0, 1e9
    for o, d, lo, hi in ((ox, dx, x0, x1), (oy, dy, y0, y1)):
        if abs(d) < 1e-12:
            if o < lo or o > hi:
                return None
            continue
        t1, t2 = (lo - o) / d, (hi - o) / d
        if t1 > t2:
            t1, t2 = t2, t1
        tmin, tmax = max(tmin, t1), min(tmax, t2)
        if tmin > tmax:
            return None
    return tmin if tmin > 0 else None


def scan_at(pose, rects, W, H, rng, noise=0.0, inc_deg=0.5, range_max=8.0):
    angs = np.deg2rad(np.arange(-180, 180, inc_deg))
    out = np.full(len(angs), np.inf)
    for i, a in enumerate(angs):
        dx, dy = math.cos(a + pose.th), math.sin(a + pose.th)
        t = 1e9
        for r in rects:
            h = _ray_aabb(pose.x, pose.y, dx, dy, r)
            if h is not None:
                t = min(t, h)
        for tw in ((0 - pose.x) / dx if dx < -1e-12 else None, (W - pose.x) / dx if dx > 1e-12 else None,
                   (0 - pose.y) / dy if dy < -1e-12 else None, (H - pose.y) / dy if dy > 1e-12 else None):
            if tw is not None and tw > 0:
                t = min(t, tw)
        if t < range_max - 0.1:
            out[i] = t + (rng.normal(0, noise) if noise else 0.0)
    return Scan(float(angs[0]), float(angs[1] - angs[0]), out, range_max=range_max)


def make_world(cubes, poses, ugv=(), scans=3, noise=0.0, seed=1):
    cfg = Config()
    cfg.explore.lidar_faces = True
    g = OccupancyGrid(cfg.arena, res=0.05)
    W, H = cfg.arena.W, cfg.arena.H
    for r in ((-0.1, -0.1, 0.0, H + 0.1), (W, -0.1, W + 0.1, H + 0.1), (-0.1, -0.1, W + 0.1, 0.0), (-0.1, H, W + 0.1, H + 0.1)):
        g.add_static_box(r)
    rects = [cube(*c) if len(c) == 2 else c for c in cubes] + list(ugv)
    rng = np.random.default_rng(seed)
    for p in poses:
        pose = Pose2D(*p)
        for _ in range(scans):
            g.update(scan_at(pose, rects, W, H, rng, noise), pose)
    return cfg, g


def around(cx, cy, d=1.0, n=8, extra=()):
    """중심 둘레를 도는 로봇 자세(+ 추가 자세)."""
    return [(cx + d * math.cos(2 * math.pi * k / n), cy + d * math.sin(2 * math.pi * k / n), 0.0) for k in range(n)] + list(extra)


def run(cubes, poses, ugv=(), ugv_points=(), **kw):
    cfg, g = make_world(cubes, poses, ugv=ugv, **kw)
    m = LidarFaceMap(cfg)
    return m.update(g, ugv_points, 0.0), cfg, g, m
