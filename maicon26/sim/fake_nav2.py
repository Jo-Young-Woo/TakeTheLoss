"""Nav2/SLAM 대역(시험용). ROS 없이 Nav2Stack의 '번역·중재 로직'을 시뮬로 검증한다.

실제 Nav2와 같은 경계(Nav2Client)를 구현하되, 안쪽은 내장 Navigator를 쓴다:
  - SLAM 대역: map 프레임 = 로봇 '시작 자세'가 원점인 좌표계(실제 SLAM과 같은 관례). TF(map→base)는
    시뮬의 참 자세를 그 프레임으로 옮긴 값 + 느린 드리프트(선택). /map 은 참 자세로 스캔을 쌓은 지도를
    map 프레임으로 다시 그려 낸다 → Nav2Stack의 앵커링·좌표 변환·격자 투영이 틀리면 로봇이 엉뚱한 곳으로 간다.
  - Nav2 대역: 목표는 map 프레임으로 받고, 속도는 '/cmd_vel_nav2' 처럼 읽어 가게만 한다(구동은 Agent가).
이 대역은 '인터페이스 계약'을 시험할 뿐 Nav2의 성능(DWB/RPP/SmacPlanner 거동)을 보여주지 않는다.
"""
from __future__ import annotations

import math
import random
from typing import Optional, Tuple

import numpy as np

from ..config import Config
from ..nav.nav2_stack import MapData, Nav2Client
from ..nav.navigator import Navigator
from ..types import Pose2D
from ..util import wrap
from .robot import SimRobot


class FakeNav2Client(Nav2Client):
    MAP_HALF = 8.0                       # m: map 프레임 지도 한 변의 절반(시작 자세 기준)

    def __init__(self, robot: SimRobot, cfg: Config, drift_rate: float = 0.0, seed: int = 0) -> None:
        self.robot, self.cfg = robot, cfg
        self.rng = random.Random(seed)
        p = robot.u.pose
        self.T = Pose2D(p.x, p.y, p.th)                 # arena ← map (SLAM은 시작 자세를 map 원점으로 삼는다)
        self._nav = Navigator(cfg, seed=seed)           # '지도'와 '플래너·컨트롤러'를 겸한다(참 자세로 갱신)
        self._goal_map: Optional[Tuple[float, float, float]] = None
        self._state = "none"
        self._cmd = (0.0, 0.0)
        self._cmd_t = -1e9
        self._last_t: Optional[float] = None
        self._scan_stamp = -1.0
        self._drift = [0.0, 0.0, 0.0]
        self.drift_rate = drift_rate
        self._recovering = False
        self.goals_received = 0

    # ------------------------------------------------------------------ 좌표
    def _arena_to_map(self, x: float, y: float, th: float) -> Tuple[float, float, float]:
        c, s = math.cos(self.T.th), math.sin(self.T.th)
        dx, dy = x - self.T.x, y - self.T.y
        return (c * dx + s * dy, -s * dx + c * dy, wrap(th - self.T.th))

    def _map_to_arena(self, mx: float, my: float) -> Tuple[float, float]:
        c, s = math.cos(self.T.th), math.sin(self.T.th)
        return (self.T.x + c * mx - s * my, self.T.y + s * mx + c * my)

    # ------------------------------------------------------------------ SLAM 대역
    def lookup_map_to_base(self) -> Optional[Tuple[float, float, float]]:
        self._sync(self.robot.now())                    # SLAM은 목표와 무관하게 항상 돈다
        p = self.robot.u.pose
        mx, my, mth = self._arena_to_map(p.x, p.y, p.th)
        d = self._drift
        return (mx + d[0], my + d[1], wrap(mth + d[2]))

    def _sync(self, now: float) -> float:
        """한 틱에 한 번: 새 스캔으로 지도 갱신 + 드리프트 진행. dt 반환."""
        dt = 0.05 if self._last_t is None else max(1e-3, min(0.25, now - self._last_t))
        if self._last_t is not None and now == self._last_t:
            return dt
        self._last_t = now
        for i, k in enumerate((0.01, 0.01, 0.002)):
            self._drift[i] += self.rng.gauss(0, k * self.drift_rate * math.sqrt(dt))
        sc = self.robot.get_scan()
        if sc is not None and sc.stamp != self._scan_stamp:
            self._scan_stamp = sc.stamp
            self._nav.update_map(sc, self.robot.u.pose)
        return dt

    def latest_map(self) -> Optional[MapData]:
        g = self._nav.grid
        res = g.res
        n = int(2 * self.MAP_HALF / res)
        xs = -self.MAP_HALF + (np.arange(n) + 0.5) * res
        MX, MY = np.meshgrid(xs, xs)
        c, s = math.cos(self.T.th), math.sin(self.T.th)
        X, Y = self.T.x + c * MX - s * MY, self.T.y + s * MX + c * MY
        ix = ((X - g.ox) / res).astype(int)
        iy = ((Y - g.oy) / res).astype(int)
        ok = (ix >= 0) & (ix < g.nx) & (iy >= 0) & (iy < g.ny)
        L = np.zeros((n, n), dtype=np.float32)
        L[ok] = g.L[iy[ok], ix[ok]]
        data = np.full((n, n), -1, dtype=np.int8)
        data[L < -0.3] = 0
        data[L > g.L_THR] = 100
        return MapData(data=data, resolution=res, origin_x=-self.MAP_HALF, origin_y=-self.MAP_HALF,
                       stamp=float(self._scan_stamp))

    def set_depth_obstacles(self, pts_map: np.ndarray, height: float = 0.1) -> None:
        """실제 Nav2 의 depth_obstacles 마킹 소스 대역: 점군(map 프레임)을 '플래너 격자'의 장애물 층으로 표시한다.
        (실제 Nav2 는 마킹만 하고 지우지 않지만, 대역은 최신 점군으로 통째로 바꾼다 — 정적 물체 시험엔 같다.)"""
        g = self._nav.grid
        mask = np.zeros(g.L.shape, dtype=bool)
        for mx, my in np.asarray(pts_map).reshape(-1, 2):
            iy, ix = g.to_cell(*self._map_to_arena(float(mx), float(my)))
            if g.in_bounds(iy, ix):
                mask[iy, ix] = True
        g.set_layer3d(mask if mask.any() else None)

    def set_keepout_mask(self, mask: MapData) -> bool:
        g = self._nav.grid
        xs, ys = g.ox + (np.arange(g.nx)+.5)*g.res, g.oy + (np.arange(g.ny)+.5)*g.res
        x, y = np.meshgrid(xs, ys)
        c, s = math.cos(self.T.th), math.sin(self.T.th)
        mx, my = c*(x-self.T.x)+s*(y-self.T.y), -s*(x-self.T.x)+c*(y-self.T.y)
        ix = np.floor((mx-mask.origin_x)/mask.resolution).astype(int)
        iy = np.floor((my-mask.origin_y)/mask.resolution).astype(int)
        h, w = mask.data.shape
        valid = (ix >= 0) & (ix < w) & (iy >= 0) & (iy < h)
        result = np.zeros(g.L.shape, dtype=bool)
        result[valid] = mask.data[iy[valid], ix[valid]] > 0
        g.set_keepout(result if result.any() else None)
        return True

    # ------------------------------------------------------------------ Nav2 대역
    def send_goal(self, x: float, y: float, yaw: float) -> None:
        self._goal_map = (x, y, yaw)
        ax, ay = self._map_to_arena(x, y)
        self._nav.set_goal((ax, ay), tol=0.10)
        self._state = "active"
        self.goals_received += 1

    def cancel(self) -> None:
        self._nav.clear_goal()
        self._goal_map = None
        self._state = "none"
        self._cmd = (0.0, 0.0)

    def goal_state(self) -> str:
        return self._state

    def recovering(self) -> bool:
        return self._recovering

    def latest_cmd(self, now: float) -> Optional[Tuple[float, float]]:
        dt = self._sync(now)
        if self._state != "active" or self._goal_map is None:
            return None
        out = self._nav.step(self.robot.u.pose, self.robot.get_scan(), now, dt)
        self._recovering = out.status == "recovering"
        if out.status == "reached":
            self._state = "succeeded"
            return None
        if out.status == "blocked":
            self._state = "aborted"
            return None
        return (out.v, out.w)
