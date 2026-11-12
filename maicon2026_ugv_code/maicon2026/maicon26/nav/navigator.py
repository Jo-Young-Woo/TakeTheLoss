"""내장 항법기(Nav2 대체/폴백): 점유격자 → A* → 가시선 단순화 → Pure-Pursuit식 추종 + 안전 제한 + 끼임 복구.

Nav2와 같은 역할(전역 계획 + 지역 추종 + 복구)을 ROS 없이 하는 구현이다. 시뮬 회귀시험과
Nav2 불가 시 폴백에 쓴다. 실기 기본 경로는 nav/nav2_stack.py 다.

임무 관리자는 set_goal(x, y)만 하고, 매 틱 step()이 돌려주는 (v, w)를 로봇에 보낸다.
안전 제한(전방 정지/감속, 측면 회피)은 목표와 무관하게 항상 적용된다 — "LiDAR 안전 정지는
임무 관리자보다 우선한다".

구조물이 실시간으로 바뀌는 경기장 대응:
  - 격자는 로그오즈라 박스가 사라지면 (LiDAR가 그 자리를 통과해 보면) 수 회 관측 안에 비워진다.
  - 경로 근처에 '새로 점유된' 셀이 생기면 1 s 주기를 기다리지 않고 즉시 재계획한다(_path_invalidated).
"""
from __future__ import annotations

import math
import random
from typing import List, Optional, Tuple

import numpy as np

from ..config import Config
from .fpplan import plan_footprint
from ..types import Pose2D, Scan
from ..util import clamp, wrap
from .grid import OccupancyGrid
from .safety import SafetyLimiter
from .stack import NavOut  # noqa: F401  (재노출)

Pt = Tuple[float, float]


class Navigator:
    INVAL_LOGODDS = 1.8

    def __init__(self, cfg: Config, seed: int = 0) -> None:
        self.cfg = cfg
        self.n = cfg.nav
        self.grid = OccupancyGrid(cfg.arena, cfg.nav.res)
        self.safety = SafetyLimiter(cfg)
        self._occ_snap: Optional[np.ndarray] = None
        self._last_inval_check = -1e9
        self.invalidations = 0
        self.goal: Optional[Pt] = None
        self.tol = self.n.goal_tol
        self.speed_scale = 1.0
        self.path: List[Pt] = []
        self._last_plan = -1e9
        self._plan_fail = 0
        self._stuck_ref: Optional[Tuple[float, float, float]] = None
        self._recover_until = -1.0
        self._recover_turn = 1.0
        self._extra_block: List[Pt] = []
        self.goal_eff: Optional[Pt] = None
        self.rng = random.Random(seed)
        self.replans = 0
        self.recoveries = 0
        self.budget = cfg.budget()                   # 차체 외곽·여유거리 예산(footprint.py) — 경로 검증에 쓴다
        self.footprint_rejects = 0                   # 직사각형 검증에서 걸러진 중심점 경로 수(진단)
        self.turn_pref: Optional[float] = None       # 회피 힌트: 복구 회전 방향(+왼쪽/-오른쪽). None 이면 무작위(기존)
        self.allow_back = True                       # False 면 복구 때 후진하지 않는다(적이 뒤에 있을 때)
        self._keepout_version = self.grid.keepout_version

    def set_hint(self, turn_pref: Optional[float], allow_back: bool = True) -> None:
        self.turn_pref, self.allow_back = turn_pref, allow_back
        self.safety.turn_pref = turn_pref

    # ------------------------------------------------------------------ 입력
    def update_map(self, scan: Scan, pose: Pose2D) -> None:
        self.grid.update(scan, pose)

    def set_goal(self, goal: Pt, tol: Optional[float] = None, speed_scale: float = 1.0) -> None:
        if self.goal is None or math.hypot(goal[0] - self.goal[0], goal[1] - self.goal[1]) > 0.05:
            self.path = []
            self._last_plan = -1e9
            self._extra_block = []
            self.goal_eff = None
        self.goal = goal
        self.tol = tol if tol is not None else self.n.goal_tol
        self.speed_scale = speed_scale

    def clear_goal(self) -> None:
        self.goal = None
        self.path = []

    # ------------------------------------------------------------------ 계획
    def _plan(self, pose: Pose2D, now: float) -> bool:
        n = self.n
        # 인플레이션 완화(0.8·0.6배)는 차체(반폭 0.115 m)보다 작은 여유를 허용하므로 기본 금지(n.relax_inflation=False). 레거시 비교용으로만 남겼다.
        for scale in ((1.0, 0.8, 0.6) if n.relax_inflation else (1.0,)):
            # 중심점 경로는 직사각형 차체가 지나간다는 증거가 아니다 → plan_footprint 가 진행 방향 직사각형 + 꼭짓점 회전 공간을 검증한다(scale=1.0 만)
            res = plan_footprint(self.grid, n.inflate_hard * scale, n.inflate_soft, n.soft_weight, self.budget, (pose.x, pose.y), self.goal,
                                 pose.th, self._extra_block, check=(n.footprint_check and scale == 1.0))
            if res is not None:
                path, blocked, rej = res
                self.footprint_rejects += rej
                self.path = path
                if self._goal_free(blocked):
                    self.path[-1] = self.goal
                    self.goal_eff = None
                else:                                   # 목표가 장애물 안: 가장 가까운 자유 지점이 실질 목표
                    self.goal_eff = self.path[-1]
                self._last_plan = now
                self._occ_snap = self.grid.planning_occupied().copy()
                self.replans += 1
                self._plan_fail = 0
                return True
        self._plan_fail += 1
        self._last_plan = now
        return False

    def _goal_free(self, blocked: np.ndarray) -> bool:
        iy, ix = self.grid.to_cell(*self.goal)
        return self.grid.in_bounds(iy, ix) and not blocked[iy, ix]

    def _path_invalidated(self, now: float) -> bool:
        """계획 이후 '새로 점유된' 셀이 경로 위(또는 인플레이션 거리 안)에 생겼는가 — 구조물이 바뀐 신호."""
        if (not self.n.replan_on_change or not self.path or self._occ_snap is None
                or now - self._last_inval_check < 0.2):
            return False
        self._last_inval_check = now
        # 정적 구조물만: 로그오즈 ≥ 1.8(같은 셀에 ≥2~3회 연속 관측)인 새 셀. 움직이는 다른 UGV의 표면은 같은 셀에
        # 1~2회만 맞고 지나가므로 걸러진다(시뮬 3v3: 이 조건이 없으면 UGV끼리 재계획 진동으로 충돌이 늘었다).
        new = (self.grid.L > self.INVAL_LOGODDS) & ~self._occ_snap
        if self.grid.planning_prior is not None:
            new |= self.grid.planning_prior & ~self._occ_snap
        if self.grid.layer3d is not None:                  # 깊이 카메라가 새로 확정한 장애물(이미 임계 ≥ 3회 관측)
            new |= self.grid.layer3d & ~self._occ_snap
        if not new.any():
            return False
        ys, xs = np.nonzero(new)
        if len(ys) > 300:
            sel = np.linspace(0, len(ys) - 1, 300).astype(int)
            ys, xs = ys[sel], xs[sel]
        g = self.grid
        qx, qy = g.ox + (xs + 0.5) * g.res, g.oy + (ys + 0.5) * g.res
        thr = self.n.inflate_hard + 0.03
        for a, b in zip(self.path[:-1], self.path[1:]):
            dx, dy = b[0] - a[0], b[1] - a[1]
            L2 = dx * dx + dy * dy
            t = np.clip(((qx - a[0]) * dx + (qy - a[1]) * dy) / L2, 0.0, 1.0) if L2 > 1e-12 else np.zeros_like(qx)
            if np.min(np.hypot(qx - (a[0] + t * dx), qy - (a[1] + t * dy))) < thr:
                return True
        return False

    # ------------------------------------------------------------------ 한 틱
    def step(self, pose: Pose2D, scan: Optional[Scan], now: float, dt: float) -> NavOut:
        if self._keepout_version != self.grid.keepout_version:
            self._keepout_version = self.grid.keepout_version
            self.path = []
            self.goal_eff = None
            self._last_plan = -1e9
            self._recover_until = -1.0
        if self.goal is None:
            return NavOut(status="idle")
        n = self.n
        g = self.goal_eff or self.goal
        dg = math.hypot(g[0] - pose.x, g[1] - pose.y)
        if dg <= self.tol:
            return NavOut(0.0, 0.0, "reached", dg)

        # 끼임 복구 중
        if now < self._recover_until:
            v, w = -0.12, 0.7 * self._recover_turn
            if scan is not None and scan.min_in_sector(math.pi, 0.6) < 0.20:
                v = 0.0
            if not self.allow_back:
                v = 0.0
            return NavOut(v, w, "recovering", dg)

        # 끼임 판정
        if self._stuck_ref is None:
            self._stuck_ref = (now, pose.x, pose.y)
        elif not self.cfg.progress.enabled and now - self._stuck_ref[0] >= n.stuck_s:
            moved = math.hypot(pose.x - self._stuck_ref[1], pose.y - self._stuck_ref[2])
            self._stuck_ref = (now, pose.x, pose.y)
            if moved < n.stuck_move:
                self.recoveries += 1
                self._recover_until = now + 1.3
                self._recover_turn = self.rng.choice([-1.0, 1.0]) if self.turn_pref is None else (1.0 if self.turn_pref > 0 else -1.0)
                c, s = math.cos(pose.th), math.sin(pose.th)
                self._extra_block = [(pose.x + 0.40 * c, pose.y + 0.40 * s)]
                self._last_plan = -1e9
                return NavOut(0.0, 0.0, "recovering", dg)

        if self._path_invalidated(now):
            self._last_plan = -1e9
            self.invalidations += 1
        if not self.path or now - self._last_plan >= n.replan_s:
            if not self._plan(pose, now) and not self.path:
                return NavOut(0.0, 0.0, "blocked", dg)

        v, w = self._follow(pose, dg)
        v, w = self._limit(scan, v, w, now)
        return NavOut(v * self.speed_scale if v > 0 else v, w, "moving", dg)

    # ------------------------------------------------------------------ 추종
    def _follow(self, pose: Pose2D, dg: float) -> Tuple[float, float]:
        n, r = self.n, self.cfg.robot
        pts = self.path
        # 경로 위 가장 가까운 선분 찾기 → 그 앞 lookahead 지점
        best_i, best_d = 0, 1e9
        for i in range(len(pts) - 1):
            d = _seg_dist((pose.x, pose.y), pts[i], pts[i + 1])
            if d < best_d:
                best_i, best_d = i, d
        tgt = pts[-1]
        remain = n.lookahead
        px, py = pose.x, pose.y
        # 가장 가까운 선분의 투영점에서 출발해 lookahead만큼 전진
        a, b = pts[best_i], pts[min(best_i + 1, len(pts) - 1)]
        qx, qy = _project((px, py), a, b)
        cx, cy = qx, qy
        for j in range(best_i, len(pts) - 1):
            e = pts[j + 1]
            seg = math.hypot(e[0] - cx, e[1] - cy)
            if seg >= remain:
                t = remain / max(seg, 1e-9)
                tgt = (cx + (e[0] - cx) * t, cy + (e[1] - cy) * t)
                break
            remain -= seg
            cx, cy = e
        else:
            tgt = pts[-1]
        alpha = wrap(math.atan2(tgt[1] - py, tgt[0] - px) - pose.th)
        if abs(alpha) > math.radians(65):
            return 0.0, clamp(2.5 * alpha, -r.w_max * 0.8, r.w_max * 0.8)
        v = n.v_cruise * max(0.2, math.cos(alpha)) ** 1.5
        v = min(v, 0.9 * dg + 0.05, r.v_max)
        w = clamp(2.4 * alpha, -r.w_max * 0.8, r.w_max * 0.8)
        return v, w

    # ------------------------------------------------------------------ 안전 제한
    def _limit(self, scan: Optional[Scan], v: float, w: float, now: float) -> Tuple[float, float]:
        v, w = self.safety.limit(scan, v, w, now)
        if self.safety.blocked_long:                       # 오래 막히면 복구 모드로
            self._recover_until = now + 1.2
            self._recover_turn = self.safety.blocked_turn
        return v, w


def _seg_dist(p: Pt, a: Pt, b: Pt) -> float:
    q = _project(p, a, b)
    return math.hypot(p[0] - q[0], p[1] - q[1])


def _project(p: Pt, a: Pt, b: Pt) -> Pt:
    ax, ay, bx, by = a[0], a[1], b[0], b[1]
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    if L2 < 1e-12:
        return a
    t = clamp(((p[0] - ax) * dx + (p[1] - ay) * dy) / L2, 0.0, 1.0)
    return (ax + dx * t, ay + dy * t)
