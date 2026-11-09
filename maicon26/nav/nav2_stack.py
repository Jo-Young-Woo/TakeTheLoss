"""SLAM + Nav2 스택(backend="nav2"). 실기(ROS2 Humble, Jetson Docker)의 기본 경로.

역할 분담
  SLAM (Cartographer 2D / slam_toolbox)  : /map, TF(map→odom→base_link) — 구조물이 바뀌어도 실시간으로 지도·위치 갱신
  Nav2  (planner/controller/bt_navigator): 센서로 계속 갱신되는 전역·지역 비용지도로 계획·추종·복구
  이 파일(Nav2Stack)                      : Agent(경기장 좌표계)와 ROS(map 프레임) 사이의 번역기 + 안전 제한 + 속도 중재

속도 중재(중요)
  Nav2는 /cmd_vel 이 아니라 /cmd_vel_nav2 로 속도를 낸다(ros2/launch 에서 리맵). Agent가 그 값을 읽어
  LiDAR 안전 제한과 가속 제한을 거친 뒤 로봇에 보낸다. 그래서 조준·교전 모드가 구동을 가져와도 Nav2와
  명령이 충돌하지 않고, "LiDAR 안전 정지는 임무 관리자·Nav2 보다 우선한다"가 지켜진다.

ROS 의존은 Nav2Client 뒤에 숨겼다 — 이 파일은 ROS 없이 import·시험 가능하다.
  RosNav2Client (nav/ros_nav2.py)  : 실기
  FakeNav2Client (sim/fake_nav2.py): 시뮬 시험용 대역
"""
from __future__ import annotations

import math
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple

import numpy as np

from ..arena import ArenaGeometry
from ..config import Config
from ..types import Pose2D, Scan
from ..util import wrap
from .grid import nearest_free
from .localize import init_pose_from_scan
from .navigator import Navigator
from .safety import SafetyLimiter
from .stack import NavOut, NavStack

Pt = Tuple[float, float]


# ====================================================================== ROS 경계
@dataclass
class MapData:
    """nav_msgs/OccupancyGrid 의 요약. data[iy, ix]: -1 미지, 0..100 점유확률. 원점 방향(yaw)은 0으로 가정."""
    data: np.ndarray
    resolution: float
    origin_x: float
    origin_y: float
    stamp: float = 0.0


class Nav2Client(ABC):
    @abstractmethod
    def lookup_map_to_base(self) -> Optional[Tuple[float, float, float]]:
        """map 프레임에서의 base_link 자세 (x, y, θ). TF가 아직 없으면 None."""

    @abstractmethod
    def latest_map(self) -> Optional[MapData]: ...

    @abstractmethod
    def latest_cmd(self, now: float) -> Optional[Tuple[float, float]]:
        """Nav2가 낸 최신 (v, w). 오래됐거나 없으면 None. now는 Agent 시간축(구현이 무시해도 됨)."""

    @abstractmethod
    def send_goal(self, x: float, y: float, yaw: float) -> None:
        """map 프레임 목표. 진행 중인 목표가 있으면 대체(선점)한다."""

    @abstractmethod
    def cancel(self) -> None: ...

    @abstractmethod
    def goal_state(self) -> str:
        """none | active | succeeded | aborted | canceled | rejected"""

    def recovering(self) -> bool:
        """Nav2가 복구 동작(spin/backup/wait) 중인가."""
        return False

    def available(self) -> bool:
        """Nav2 액션 서버가 응답 가능한가(미지원 구현은 항상 True)."""
        return True

    def set_arena_walls(self, pts_map: np.ndarray) -> None:
        """경기장 외벽 점들(map 프레임, N×2)을 Nav2 전역 비용지도에 마킹 전용으로 알린다(구현이 주기적으로 재발행)."""

    def set_depth_obstacles(self, pts_map: np.ndarray, height: float = 0.1) -> None:
        """깊이 3D 지도에서 'LiDAR 가 못 보는' 장애물 셀 중심들(map 프레임, N×2)을 Nav2 비용지도에 마킹 전용으로 알린다.
        N=0 이면 빈 점군(구현이 최신 상태로 계속 재발행). 기본 구현은 아무것도 하지 않는다."""

    def set_keepout_mask(self, mask: MapData) -> bool:
        """Replace a planning-only KeepoutFilter grid in map frame (axis aligned).

        Return False when unsupported; the stack then uses its constrained
        built-in planner instead of silently sending unconstrained Nav2 goals.
        """
        return False

    def shutdown(self) -> None:
        pass


# ====================================================================== 좌표 변환
@dataclass
class Anchor:
    """경기장 좌표 = Anchor ∘ map 좌표. SLAM의 map 원점은 보통 '시작 자세'라 시작 때 한 번 정한다."""
    x: float
    y: float
    th: float

    def to_arena(self, mx: float, my: float, mth: float) -> Tuple[float, float, float]:
        c, s = math.cos(self.th), math.sin(self.th)
        return (self.x + c * mx - s * my, self.y + s * mx + c * my, wrap(self.th + mth))

    def to_map(self, ax: float, ay: float, ath: float = 0.0) -> Tuple[float, float, float]:
        c, s = math.cos(self.th), math.sin(self.th)
        dx, dy = ax - self.x, ay - self.y
        return (c * dx + s * dy, -s * dx + c * dy, wrap(ath - self.th))

    def points_to_map(self, pts: np.ndarray) -> np.ndarray:
        """경기장 좌표 점들(N×2) → map 프레임."""
        c, s = math.cos(self.th), math.sin(self.th)
        d = pts - np.array([self.x, self.y])
        return np.stack([c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1]], axis=1)

    @staticmethod
    def from_pair(arena: Pose2D, m: Tuple[float, float, float]) -> "Anchor":
        th = wrap(arena.th - m[2])
        c, s = math.cos(th), math.sin(th)
        return Anchor(arena.x - (c * m[0] - s * m[1]), arena.y - (s * m[0] + c * m[1]), th)


# ====================================================================== 스택
class Nav2Stack(NavStack):
    def __init__(self, cfg: Config, client: Nav2Client, seed: int = 0) -> None:
        self.cfg = cfg
        self.geo: ArenaGeometry = cfg.arena
        self.n, self.c2 = cfg.nav, cfg.nav2
        self.client = client
        # 폴백: Nav2 서버가 없거나 한 목표에서 연속 실패하면 내장 A* 항법기가 같은 SLAM 자세로 대신 간다.
        # 격자는 폴백 항법기의 것을 공유한다(Agent 용 격자 = 폴백 계획용 격자).
        self._fb = Navigator(cfg, seed=seed)
        self.grid = self._fb.grid
        self._fallback = False
        self._unavail_since: Optional[float] = None
        self.fallbacks = 0
        self.fallback_reason = ""
        self.safety = SafetyLimiter(cfg)
        self.anchor: Optional[Anchor] = None
        self.pose = Pose2D()
        self.goal: Optional[Pt] = None
        self.goal_eff: Optional[Pt] = None
        self.tol = self.n.goal_tol
        self.speed_scale = 1.0
        self._sent_goal: Optional[Pt] = None
        self._sent_t = -1e9
        self._eff_t = -1e9
        self._resends = 0
        self._fails = 0
        self._map_stamp = None
        self._map_t = -1e9
        self.replans = 0
        self.recoveries = 0
        self.map_updates = 0
        self.tf_misses = 0
        self._was_recovering = False
        self._progress_ref: Optional[Tuple[float, float, float]] = None
        self._last_step_t = -1e9
        self._stall_fallback = False
        self._keepout_external = False
        self._keepout_retry_t = -1e9

    @property
    def initialized(self) -> bool:                             # type: ignore[override]
        return self.anchor is not None

    # ------------------------------------------------------------------ 시작: SLAM 앵커링
    def start(self, scan: Scan, odom: Pose2D, xy_guess: Pt) -> Optional[Pose2D]:
        m0 = self.client.lookup_map_to_base()
        if m0 is None:
            return None                                        # SLAM/TF 준비 전 — 다음 틱에 재시도
        p0 = (Pose2D(*self.c2.start_pose_override) if self.c2.start_pose_override is not None
              else init_pose_from_scan(scan, self.geo, xy_guess[0], xy_guess[1]))
        self.anchor = Anchor.from_pair(p0, m0)
        self.pose = p0.copy()
        if self.c2.publish_walls:
            self.client.set_arena_walls(self.anchor.points_to_map(self._perimeter(0.04)))
        self._publish_keepout_mask()  # Clear any previous match's retained policy.
        return p0

    def set_keepout_layer(self, mask) -> bool:
        changed = super().set_keepout_layer(mask)
        if changed:
            self.client.cancel()
            self._sent_goal = self.goal_eff = None
            self._publish_keepout_mask()
        return changed

    def keepout_map(self) -> Optional[MapData]:
        """Conservatively rerasterize to map axes; Humble ignores origin yaw."""
        if self.anchor is None:
            return None
        g, a, res = self.grid, self.anchor, self.grid.res
        bounds = a.points_to_map(np.array([[g.ox, g.oy], [g.ox+g.nx*res, g.oy],
                                          [g.ox, g.oy+g.ny*res], [g.ox+g.nx*res, g.oy+g.ny*res]]))
        lo, hi = bounds.min(axis=0), bounds.max(axis=0)
        nx, ny = np.ceil((hi-lo)/res).astype(int) + 1
        result = np.zeros((ny, nx), dtype=np.int8)
        if g.keepout is not None:
            ys, xs = np.nonzero(g.keepout)
            for iy, ix in zip(ys, xs):
                x0, y0 = g.ox+ix*res, g.oy+iy*res
                corners = a.points_to_map(np.array([[x0, y0], [x0+res, y0], [x0, y0+res], [x0+res, y0+res]]))
                low = np.floor((corners.min(axis=0)-lo)/res + 1e-9).astype(int)
                high = np.ceil((corners.max(axis=0)-lo)/res - 1e-9).astype(int)
                result[max(0, low[1]):min(ny, high[1]), max(0, low[0]):min(nx, high[0])] = 100
        return MapData(result, res, float(lo[0]), float(lo[1]))

    def _publish_keepout_mask(self) -> None:
        message = self.keepout_map()
        if message is not None:
            self._keepout_external = bool(self.client.set_keepout_mask(message))

    def set_obstacle_layer(self, mask) -> None:
        """3D 층을 Agent 용 격자(=폴백 계획 격자)에 더하고, 같은 셀들을 Nav2 비용지도에도 마킹 전용 점군으로 알린다."""
        super().set_obstacle_layer(mask)
        if self.anchor is None or not self.c2.publish_depth_obstacles:
            return
        g = self.grid
        if mask is None or not mask.any():
            self.client.set_depth_obstacles(np.zeros((0, 2)))
            return
        ys, xs = np.nonzero(mask & ~g.static)
        if len(xs) > 4000:
            sel = np.linspace(0, len(xs) - 1, 4000).astype(int)
            ys, xs = ys[sel], xs[sel]
        pts = np.stack([g.ox + (xs + 0.5) * g.res, g.oy + (ys + 0.5) * g.res], axis=1)
        self.client.set_depth_obstacles(self.anchor.points_to_map(pts))

    def _perimeter(self, step: float) -> np.ndarray:
        """경기장 안쪽 둘레 점(경기장 좌표). 외벽 안쪽 면 = 로봇이 갈 수 있는 영역의 경계."""
        W, H = self.geo.W, self.geo.H
        xs, ys = np.arange(0.0, W + 1e-9, step), np.arange(0.0, H + 1e-9, step)
        return np.concatenate([np.stack([xs, np.zeros_like(xs)], 1), np.stack([xs, np.full_like(xs, H)], 1),
                               np.stack([np.zeros_like(ys), ys], 1), np.stack([np.full_like(ys, W), ys], 1)])

    # ------------------------------------------------------------------ 매 틱: 위치·지도 동기화
    def update(self, odom: Pose2D, scan: Optional[Scan], now: float) -> None:
        if self.anchor is None:
            return
        m = self.client.lookup_map_to_base()
        if m is None:
            self.tf_misses += 1                                # 직전 자세 유지
        else:
            x, y, th = self.anchor.to_arena(*m)
            self.pose = Pose2D(x, y, th)
        if scan is not None and self.c2.grid_source == "scan":
            # SLAM이 교정한 자세로 LiDAR를 쌓는다. 로그오즈라 옮겨진 박스는 몇 번 관측에 사라지고 새 박스는 즉시 나타난다.
            self._fb.update_map(scan, self.pose)
        if self.c2.grid_source == "slam_map" and now - self._map_t >= self.c2.map_refresh_s:
            self._map_t = now
            mp = self.client.latest_map()
            if mp is not None and mp.stamp != self._map_stamp:
                self._map_stamp = mp.stamp
                self._project_map(mp)

    def _project_map(self, mp: MapData) -> None:
        """SLAM 지도(map 프레임) → 경기장 좌표 점유격자. 박스 탐색·시점 선정·목표 보정에 쓴다."""
        g, a = self.grid, self.anchor
        xs = g.ox + (np.arange(g.nx) + 0.5) * g.res
        ys = g.oy + (np.arange(g.ny) + 0.5) * g.res
        X, Y = np.meshgrid(xs, ys)
        c, s = math.cos(a.th), math.sin(a.th)
        dx, dy = X - a.x, Y - a.y
        mx, my = c * dx + s * dy, -s * dx + c * dy
        ix = np.floor((mx - mp.origin_x) / mp.resolution).astype(int)
        iy = np.floor((my - mp.origin_y) / mp.resolution).astype(int)
        h, w = mp.data.shape
        ok = (ix >= 0) & (ix < w) & (iy >= 0) & (iy < h)
        v = np.full(X.shape, -1, dtype=np.int16)
        v[ok] = mp.data[iy[ok], ix[ok]]
        L = np.zeros(X.shape, dtype=np.float32)
        L[v >= self.c2.occ_thresh] = g.L_MAX
        L[(v >= 0) & (v <= self.c2.free_thresh)] = g.L_MIN
        g.L[:] = L
        self.map_updates += 1

    # ------------------------------------------------------------------ 목표
    def set_goal(self, goal: Pt, tol: Optional[float] = None, speed_scale: float = 1.0) -> None:
        if self.goal is None or math.hypot(goal[0] - self.goal[0], goal[1] - self.goal[1]) > 0.05:
            self._sent_goal = None
            self._resends = self._fails = 0
            self.goal_eff = None
            self._fallback = self._stall_fallback              # 무진행 복구 중 목표 변경으로 실패 이력을 지우지 않는다
        self.goal = goal
        self.tol = tol if tol is not None else self.n.goal_tol
        self.speed_scale = speed_scale

    def clear_goal(self) -> None:
        if self.goal is not None or self._sent_goal is not None:
            self.client.cancel()
        self.goal = None
        self.goal_eff = None
        self._sent_goal = None
        self._fallback = self._stall_fallback
        self._fb.clear_goal()

    def set_hint(self, turn_pref: Optional[float], allow_back: bool = True) -> None:
        self.safety.turn_pref = turn_pref
        self._fb.set_hint(turn_pref, allow_back)

    def _effective_goal(self) -> Pt:
        """목표가 장애물/인플레이션 안이면 가장 가까운 자유 지점으로(내장 항법기와 같은 의미)."""
        blocked, _ = self.grid.cost_maps(self.n.inflate_hard, self.n.inflate_soft, self.n.soft_weight)
        iy, ix = self.grid.to_cell(*self.goal)
        if self.grid.in_bounds(iy, ix) and not blocked[iy, ix]:
            return self.goal
        nf = nearest_free(blocked, iy, ix)
        return self.goal if nf is None else self.grid.to_world(*nf)

    def _send(self, g: Pt, now: float) -> None:
        mx, my, _ = self.anchor.to_map(g[0], g[1])
        # 목표 자세 방향 = 지금 위치에서 목표를 보는 방향 (거점 진입은 방향 무관 — yaw 허용오차는 nav2 yaml에서 크게)
        yaw = math.atan2(g[1] - self.pose.y, g[0] - self.pose.x)
        _, _, myaw = self.anchor.to_map(g[0], g[1], yaw)
        self.client.send_goal(mx, my, myaw)
        self._sent_goal, self._sent_t = g, now
        self.replans += 1

    # ------------------------------------------------------------------ 한 틱
    def step(self, scan: Optional[Scan], now: float, dt: float) -> NavOut:
        if self.goal is None or self.anchor is None:
            return NavOut(status="idle")
        if self.goal_eff is None or (self.goal_eff != self.goal and now - self._eff_t > 2.0):
            self.goal_eff = self._effective_goal()               # 지도가 바뀌면 보정 목표도 갱신(2 s 주기)
            self._eff_t = now
        g = self.goal_eff
        keepout_active = self.grid.keepout is not None and self.grid.keepout.any()
        if keepout_active and not self._keepout_external:
            if now - self._keepout_retry_t >= 1.0:
                self._keepout_retry_t = now
                self._publish_keepout_mask()
            if not self._keepout_external:
                return self._step_fallback(g, scan, now, dt)
        dg = math.hypot(g[0] - self.pose.x, g[1] - self.pose.y)
        if dg <= self.tol:
            self.client.cancel()
            self._sent_goal = None
            return NavOut(0.0, 0.0, "reached", dg)

        # Action feedback can stay active through repeated controller recovery
        # loops. New viewpoint goals must not reset the physical-motion clock.
        # Gaps mean the mission deliberately stopped navigating (e.g. camera
        # observation/aiming), so those pauses do not count as failed motion.
        if self._progress_ref is None or now - self._last_step_t > 1.0:
            self._progress_ref = (now, self.pose.x, self.pose.y)
            self._stall_fallback = False
        self._last_step_t = now
        t0, px, py = self._progress_ref
        if math.hypot(self.pose.x - px, self.pose.y - py) >= self.c2.progress_min_m:
            self._progress_ref = (now, self.pose.x, self.pose.y)
            self._stall_fallback = False
        elif not self._fallback and now - t0 >= self.c2.progress_timeout_s:
            self._stall_fallback = True
            self._enter_fallback("Nav2 active without translational progress")

        # ---- Nav2 가 죽었는지: 액션 서버가 1.5 s 넘게 안 보이면 폴백
        if self.client.available():
            self._unavail_since = None
        else:
            self._unavail_since = self._unavail_since if self._unavail_since is not None else now
            if not self._fallback and now - self._unavail_since > 1.5:
                self._enter_fallback("Nav2 액션 서버 없음")
        if self._fallback:
            return self._step_fallback(g, scan, now, dt)

        if self._sent_goal is None or math.hypot(g[0] - self._sent_goal[0], g[1] - self._sent_goal[1]) > 0.15:
            self._send(g, now)
            return NavOut(0.0, 0.0, "moving", dg)

        st = self.client.goal_state()
        if st == "succeeded":                                    # Nav2는 도착했다고 보지만 우리 허용오차 밖
            if self._resends < self.c2.max_resend:
                self._resends += 1
                self._send(g, now)
                return NavOut(0.0, 0.0, "moving", dg)
            return NavOut(0.0, 0.0, "reached" if dg <= self.tol + 0.10 else "blocked", dg)
        if st in ("aborted", "rejected", "canceled"):            # Nav2 복구까지 실패 → 한 번은 다시, 그래도 안 되면 막힘
            self._fails += 1
            if self._fails <= 1:
                self.goal_eff = self._effective_goal()
                self._send(self.goal_eff, now)
                return NavOut(0.0, 0.0, "recovering", dg)
            self._enter_fallback(f"Nav2 목표 연속 실패({st})")       # 복구까지 실패 → 내장 A* 로 이 목표만 한 번 더
            return self._step_fallback(g, scan, now, dt)

        rec = self.client.recovering()
        if rec and not self._was_recovering:
            self.recoveries += 1
        self._was_recovering = rec
        cmd = self.client.latest_cmd(now)
        v, w = cmd if cmd is not None else (0.0, 0.0)
        v, w = self.limit(scan, v, w, now)
        if v > 0:
            v = min(v * self.speed_scale, self.cfg.robot.v_max * max(1.0, self.speed_scale))
        return NavOut(v, w, "recovering" if rec else "moving", dg)

    def _enter_fallback(self, why: str) -> None:
        self._fallback = True
        self.fallbacks += 1
        self.client.cancel()
        self._sent_goal = None
        self.fallback_reason = why

    def _step_fallback(self, g: Pt, scan: Optional[Scan], now: float, dt: float) -> NavOut:
        self._fb.set_goal(g, self.tol, self.speed_scale)
        out = self._fb.step(self.pose, scan, now, dt)
        self.replans = max(self.replans, self._fb.replans)
        return out

    def limit(self, scan: Optional[Scan], v: float, w: float, now: float) -> Tuple[float, float]:
        return self.safety.limit(scan, v, w, now)

    def diagnostics(self) -> Dict[str, Any]:
        return {"replans": self.replans, "recoveries": self.recoveries, "fallbacks": self.fallbacks,
                "keepout_external": self._keepout_external,
                "fallback_reason": self.fallback_reason,
                "nav2_cmds(accepted,gated)": getattr(self.client, "cmd_stats", lambda: None)(),
                "map_updates": self.map_updates,
                "tf_misses": self.tf_misses, "anchor": None if self.anchor is None else
                (round(float(self.anchor.x), 3), round(float(self.anchor.y), 3), round(float(self.anchor.th), 3))}

    def close(self) -> None:
        self.client.cancel()
        self.client.shutdown()
