"""LiDAR 만으로 하는 '움직이는 작은 물체' 검출 — 카메라(YOLO)·UAV 가 없어도 적 UGV 후보를 잡는 최소 감지.

원리: 정적 구조물(박스·벽)이 있는 칸은 '예전에 자유로 확인된 적이 없다'(박스는 처음부터 거기 있었다).
      예전에 자유로 확인된 칸(free_seen)에 새로 반사가 생기고, 그 칸이 아직 몇 번밖에 안 맞았으면(H < h_static) 움직이는 물체다.
      → 새 반사점을 연속 빔 군집으로 묶어 크기(arc 길이)가 UGV 만 한 것만 남긴다 → 위치·속도 추적.
한계(설계서 §2.4): ① 멈춰 선 적은 h_static 번 맞은 뒤 정적으로 흡수된다(카메라/UAV 확인에 맡긴다). ② 자기 회전이 빠르면(|w|>w_skip)
      자세 오차로 정적 모서리가 움직이는 것처럼 보이므로 건너뛴다. ③ 경기 중 구조물을 옮기면 옛 칸이 자유로 알려져 있어 한동안
      오탐이 난다(그래도 크기·속도 조건으로 일부 걸러짐). ④ 시뮬 LiDAR 에서만 검증했다 — 실기 LiDAR 잡음·SLAM 지터에서는 문턱 재측정 필요.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np

from ..config import Config
from ..types import Pose2D, Scan


@dataclass
class Blob:
    bid: int
    x: float
    y: float
    vx: float
    vy: float
    size: float
    n: int                    # 연속으로 잡힌 스캔 수
    t: float                  # 마지막으로 잡힌 시각
    speed_est: float = 0.0
    moved: bool = True        # 실제로 움직인 것이 확인됐다(정적 모서리 오탐 거름)


@dataclass
class _BT:
    bid: int
    x: float
    y: float
    t: float
    n: int = 1
    vx: float = 0.0
    vy: float = 0.0
    size: float = 0.0
    moved: bool = False
    x0: float = 0.0
    y0: float = 0.0
    hist: List[Tuple[float, float, float]] = field(default_factory=list)
    covariance: Optional[np.ndarray] = None


class BlobDetector:
    def __init__(self, cfg: Config, h_static: int = 6, w_skip: float = 0.8, gate: float = 0.5, lost_s: float = 0.6) -> None:
        self.cfg = cfg
        self.ac = cfg.avoid
        self.geo = cfg.arena
        self.h_static, self.w_skip, self.gate, self.lost_s = h_static, w_skip, gate, lost_s
        self.free_seen: Optional[np.ndarray] = None
        self.H: Optional[np.ndarray] = None
        self._interior_free: Optional[np.ndarray] = None
        self._birth_grid = None
        self._scan_n = 0
        self._t_first: Optional[float] = None
        self._tracks: List[_BT] = []
        self._next = 1
        self.stats: Dict[str, int] = {"scans": 0, "skipped_rot": 0, "candidates": 0, "clusters": 0, "confirmed": 0}
        self.stats.update(birth_support_checks=0, birth_accepted=0,
                          birth_rejected_no_prior_interior=0, guarded_free_cells=0)
        self.latency_ms: List[float] = []
        self._last_stamp = None

    @property
    def fast_enabled(self) -> bool:
        return bool(getattr(getattr(self.cfg, "progress", None), "enabled", False))

    def reset(self) -> None:
        self._tracks.clear()
        self.free_seen = None
        self.H = None
        self._interior_free = None
        self._birth_grid = None
        self._last_stamp = None
        self.stats["guarded_free_cells"] = 0

    # ------------------------------------------------------------------
    def update(self, scan: Optional[Scan], pose: Pose2D, grid, now: float, w_cmd: float = 0.0,
               static_boxes=()) -> List[Blob]:
        if scan is None or not self.ac.enable:
            return self.blobs(now)
        if self.fast_enabled:
            started = time.perf_counter()
            result = self._fast_update(scan, pose, grid, now, w_cmd, static_boxes)
            self.latency_ms.append((time.perf_counter() - started) * 1000.)
            self.latency_ms = self.latency_ms[-2000:]
            return result
        if self.free_seen is None or self.free_seen.shape != grid.L.shape:
            self.free_seen = np.zeros(grid.L.shape, dtype=bool)
            self.H = np.zeros(grid.L.shape, dtype=np.uint8)
        self.stats["scans"] += 1
        if self._t_first is None:
            self._t_first = now
        skip = abs(w_cmd) > self.w_skip or now - self._t_first < 3.0        # 시작 직후(SLAM 자세가 굳기 전)에도 건너뛴다
        if skip:
            self.stats["skipped_rot"] += 1
        else:
            self._detect(scan, pose, grid, now)
        # 자유 확인 칸 갱신(이번 스캔의 반사를 포함한 격자 — 적이 있던 칸은 점유라 자유로 안 들어간다)
        self.free_seen |= (grid.L < -0.4)
        self._scan_n += 1
        if self._scan_n % 10 == 0:                           # H 서서히 감쇠: 적이 떠난 칸이 영원히 '정적'으로 남지 않게
            np.subtract(self.H, 1, out=self.H, where=self.H > 0)
        self._tracks = [b for b in self._tracks if now - b.t <= self.lost_s]
        return self.blobs(now)

    def blobs(self, now: float) -> List[Blob]:
        out = []
        for b in self._tracks:
            minimum = getattr(getattr(self.cfg, "threat", None), "lidar_confirm_scans", 1) if self.fast_enabled else self.ac.lidar_blob_scans
            if b.n >= minimum and now - b.t <= self.lost_s and (b.moved or self.fast_enabled):
                sp = math.hypot(b.vx, b.vy)
                out.append(Blob(b.bid, b.x, b.y, b.vx, b.vy, b.size, b.n, b.t, sp))
        return out

    def _fast_update(self, scan, pose, grid, now, w_cmd, static_boxes):
        """CPU scan path. Unknown structure is not free-space evidence.

        A prior free observation (or an already tracked body) admits returns.
        Static subtraction uses immutable walls and *observed* confirmed boxes;
        live occupancy includes this scan and must never be used as background.
        This detects a stationary new blocker immediately in surveyed space,
        without waiting for YOLO or absorbing it after six scans.
        """
        tc = getattr(self.cfg, "threat", None)
        self.lost_s = float(getattr(tc, "lidar_stale_s", 1.0))
        if self.free_seen is None or self.free_seen.shape != grid.L.shape:
            self.free_seen = np.zeros(grid.L.shape, dtype=bool)
            self.H = np.zeros(grid.L.shape, dtype=np.uint8)
        if self._interior_free is None or self._interior_free.shape != grid.L.shape:
            self._interior_free = np.zeros(grid.L.shape, dtype=bool)
        self._birth_grid = grid
        self.stats["scans"] += 1
        if abs(w_cmd) > self.w_skip:
            self.stats["skipped_rot"] += 1
        else:
            r = np.asarray(scan.ranges)
            ok = np.isfinite(r) & (r > .20) & (r < min(scan.range_max - .05, 4.5))
            a = scan.angles + pose.th
            px = pose.x + np.cos(a) * np.where(ok, r, 0.)
            py = pose.y + np.sin(a) * np.where(ok, r, 0.)
            ix = np.floor((px - grid.ox) / grid.res).astype(int)
            iy = np.floor((py - grid.oy) / grid.res).astype(int)
            valid = ok & (ix >= 0) & (ix < grid.nx) & (iy >= 0) & (iy < grid.ny)
            margin = float(getattr(tc, "lidar_static_margin_m", .06))
            valid &= (px > margin) & (px < self.geo.W-margin) & (py > margin) & (py < self.geo.H-margin)
            background = np.zeros(len(r), dtype=bool)
            inds = np.flatnonzero(valid)
            # Long walls are not necessarily a confirmed 0.4m cube. Learn
            # persistent occupied returns independently from free_seen, then
            # reject a tolerance band around them before forming candidates.
            # A live tracked body stays dynamic even when SLAM marks its points
            # occupied; after it leaves, free rays erase that support again.
            learned_static=(self.H>=3)&(grid.L>2.0)
            for tr in self._tracks:
                iy0,ix0=grid.to_cell(tr.x,tr.y)
                rr=int(math.ceil(.25/grid.res))
                learned_static[max(0,iy0-rr):min(grid.ny,iy0+rr+1),
                               max(0,ix0-rr):min(grid.nx,ix0+rr+1)]=False
            known_static=grid.static|learned_static
            # Dilate only known static geometry, not the current robot returns.
            rad = max(1, int(math.ceil(margin/grid.res)))
            for dy in range(-rad, rad+1):
                for dx in range(-rad, rad+1):
                    yy = np.clip(iy[inds]+dy, 0, grid.ny-1)
                    xx = np.clip(ix[inds]+dx, 0, grid.nx-1)
                    background[inds] |= known_static[yy, xx]
            for box in static_boxes:
                background |= ((np.abs(px-box.cx) <= box.sx/2+margin)
                               & (np.abs(py-box.cy) <= box.sy/2+margin))
            cand = np.zeros(len(r), dtype=bool)
            cand[inds] = self.free_seen[iy[inds], ix[inds]]
            for tr in self._tracks:
                dt = max(0., min(now-tr.t, self.lost_s))
                # Retain a measured blocker when the occupancy map learns it.
                cand |= np.hypot(px-(tr.x+tr.vx*dt), py-(tr.y+tr.vy*dt)) < .24
            cand &= valid & ~background
            self.stats["candidates"] += int(cand.sum())
            clusters, cur, prev = [], [], -10
            # Cluster the complete measured surface, not only the fraction
            # that flickered into historical free space after a small PF shift.
            # Otherwise a long wall corner can be trimmed into a .2m "UGV".
            for i in np.flatnonzero(valid):
                if cur and (i-prev > 2 or math.hypot(px[i]-px[prev], py[i]-py[prev]) > .12):
                    clusters.append(cur); cur = []
                cur.append(int(i)); prev = int(i)
            if cur:
                clusters.append(cur)
            dets = []
            for c in clusters:
                if len(c) < 2 or np.mean(cand[c])<.60:
                    continue
                first, last = c[0], c[-1]
                width = math.hypot(px[last]-px[first], py[last]-py[first])
                if not (float(getattr(tc, "lidar_min_width_m", .12)) <= width
                        <= float(getattr(tc, "lidar_max_width_m", .40))):
                    continue
                if not (self._edge(r, first-1, r[first]) and self._edge(r, last+1, r[last])):
                    continue
                x, y = float(np.mean(px[c])), float(np.mean(py[c]))
                d = math.hypot(x-pose.x, y-pose.y)
                if d > 1e-6:
                    x += (x-pose.x)*(.7*self.ac.r_body/d)
                    y += (y-pose.y)*(.7*self.ac.r_body/d)
                dets.append((x, y, width))
            self.stats["clusters"] += len(dets)
            self._kalman_update(dets, now)
            if inds.size:
                flat=np.unique(iy[inds]*grid.nx+ix[inds])
                history=self.H.reshape(-1)
                history[flat]=np.minimum(history[flat].astype(np.int16)+1,250).astype(np.uint8)
            self.H[grid.L<-.4]=0
            if getattr(tc,"lidar_birth_require_prior_interior",True):
                # After birth decisions: this scan cannot establish its own
                # prior evidence. Current occupancy is never free support.
                self._remember_guarded_free(scan,pose,grid)
        self.free_seen |= grid.L < -.4
        self._tracks = [b for b in self._tracks if now-b.t <= self.lost_s]
        self.stats["confirmed"] = len(self._tracks)
        return self.blobs(now)

    def _remember_guarded_free(self, scan, pose, grid):
        """Surveyed ray interiors, excluding noisy surface endpoints.

        This contains only previous sensor rays, never a world-layout lookup.
        Unknown stationary objects remain ordinary occupancy until another
        sensor identifies them or they enter surveyed free space.
        """
        tc=self.cfg.threat
        r=np.asarray(scan.ranges,dtype=float)
        valid=(np.isfinite(r)&(r>.20))|np.isposinf(r)
        angles=scan.angles[valid]+pose.th
        limit=min(scan.range_max,4.5)
        guard=max(0.,float(getattr(tc,"lidar_birth_guard_m",.12)))
        reach=np.minimum(np.where(np.isfinite(r[valid]),r[valid],scan.range_max),limit)-guard
        samples=np.arange(.20,limit,grid.res*.5)
        live=samples[None,:]<=reach[:,None]
        x=pose.x+np.cos(angles)[:,None]*samples
        y=pose.y+np.sin(angles)[:,None]*samples
        ix=np.floor((x-grid.ox)/grid.res).astype(int)
        iy=np.floor((y-grid.oy)/grid.res).astype(int)
        live&=(ix>=0)&(ix<grid.nx)&(iy>=0)&(iy<grid.ny)
        self._interior_free[iy[live],ix[live]]=True
        self.stats["guarded_free_cells"]=int(np.count_nonzero(self._interior_free))

    def _birth_supported(self, x, y):
        tc=self.cfg.threat
        if not getattr(tc,"lidar_birth_require_prior_interior",True):
            return True
        self.stats["birth_support_checks"]+=1
        grid=self._birth_grid
        if grid is None or self._interior_free is None:
            return False
        iy,ix=grid.to_cell(x,y)
        radius=max(1,int(math.ceil(float(getattr(tc,"lidar_birth_support_radius_m",.05))/grid.res)))
        if iy<radius or ix<radius or iy>=grid.ny-radius or ix>=grid.nx-radius:
            return False
        patch=self._interior_free[iy-radius:iy+radius+1,ix-radius:ix+radius+1]
        fraction=min(1.,max(0.,float(getattr(tc,"lidar_birth_min_free_fraction",.75))))
        return bool(patch[radius,radius] and np.count_nonzero(patch)>=math.ceil(fraction*patch.size))

    def _kalman_update(self, measurements, now):
        """Constant-velocity Kalman filter; one-to-one gated assignment."""
        predictions = {}
        for b in self._tracks:
            dt = max(.001, min(now-b.t, self.lost_s))
            F = np.eye(4); F[0,2] = dt; F[1,3] = dt
            G = np.array([[dt*dt/2,0], [0,dt*dt/2], [dt,0], [0,dt]])
            P = b.covariance if b.covariance is not None else np.diag([.004,.004,.25,.25])
            predictions[b.bid] = (F @ np.array([b.x,b.y,b.vx,b.vy]), F @ P @ F.T + .8**2*(G@G.T))
        used = set()
        for x, y, width in measurements:
            available = [(math.hypot(predictions[b.bid][0][0]-x, predictions[b.bid][0][1]-y), b)
                         for b in self._tracks if b.bid not in used and b.bid in predictions]
            best = min(available, key=lambda pair: pair[0], default=None)
            if best is None or best[0] > self.gate:
                if not self._birth_supported(x,y):
                    self.stats["birth_rejected_no_prior_interior"]+=1
                    continue
                self.stats["birth_accepted"]+=1
                b = _BT(self._next, x, y, now, size=width,
                        covariance=np.diag([.0036,.0036,.25,.25]))
                self._next += 1; self._tracks.append(b); used.add(b.bid)
                continue
            b = best[1]; state, P = predictions[b.bid]
            S = P[:2,:2] + np.eye(2)*.0036
            K = np.linalg.solve(S, P[:2,:]).T
            state += K @ (np.array([x,y])-state[:2])
            I_KH = np.eye(4); I_KH[:,:2] -= K
            b.covariance = I_KH @ P @ I_KH.T + .0036*(K@K.T)
            speed = math.hypot(state[2], state[3])
            if speed > self.ac.v_enemy_max*1.5:
                state[2:] *= self.ac.v_enemy_max*1.5/speed
            b.x,b.y,b.vx,b.vy = map(float,state)
            b.t,b.size,b.n = now,width,b.n+1
            b.moved = speed >= self.ac.lidar_blob_min_speed
            used.add(b.bid)

    @staticmethod
    def _edge(r: np.ndarray, j: int, r_end: float, jump: float = 0.25) -> bool:
        """빔 j(군집 바깥 이웃)가 군집 끝 빔과 깊이 차이가 jump 이상이거나 유효하지 않으면 True(깊이 불연속)."""
        if j < 0 or j >= len(r):
            return True
        rj = r[j]
        return (not np.isfinite(rj)) or abs(float(rj) - float(r_end)) > jump

    # ------------------------------------------------------------------
    def _detect(self, scan: Scan, pose: Pose2D, grid, now: float) -> None:
        r = scan.ranges
        ok = np.isfinite(r) & (r > 0.25) & (r < min(scan.range_max - 0.05, 4.5))
        if not np.any(ok):
            return
        ang = scan.angles + pose.th
        px = pose.x + np.cos(ang) * np.where(ok, r, 0.0)
        py = pose.y + np.sin(ang) * np.where(ok, r, 0.0)
        ix = ((px - grid.ox) / grid.res).astype(int)
        iy = ((py - grid.oy) / grid.res).astype(int)
        inb = ok & (ix >= 0) & (ix < grid.nx) & (iy >= 0) & (iy < grid.ny)
        wm = 0.15                                          # 벽에 맞은 반사(측정·자세 오차로 벽 안팎에 흩어짐)와 벽에 붙은 물체는 제외
        inb &= (px > wm) & (px < self.geo.W - wm) & (py > wm) & (py < self.geo.H - wm)
        cand = np.zeros(len(r), dtype=bool)
        idx = np.where(inb)[0]
        if idx.size:
            yy, xx = iy[idx], ix[idx]
            hh = self.H[yy, xx]
            # A newly exposed static edge is not a moving-object observation.
            # Require free-space evidence from BEFORE this scan; free_seen is
            # updated after _detect, even when grid already contains this scan.
            cand[idx] = self.free_seen[yy, xx] & (hh < self.h_static) & ~grid.static[yy, xx]
            # H 갱신(이번 스캔에서 맞은 칸, 칸당 한 번)
            flat = np.unique(yy * grid.nx + xx)
            hv = self.H.reshape(-1)
            hv[flat] = np.minimum(hv[flat].astype(np.int16) + 1, 250).astype(np.uint8)
        self.stats["candidates"] += int(cand.sum())
        # 연속 빔 군집(점 사이 0.12 m 이하, 빔 1개 건너뛰기 허용)
        clusters: List[List[int]] = []
        cur: List[int] = []
        last_i = -10
        for i in np.where(cand)[0]:
            if cur and (i - last_i > 2 or math.hypot(px[i] - px[last_i], py[i] - py[last_i]) > 0.12):
                clusters.append(cur)
                cur = []
            cur.append(int(i))
            last_i = int(i)
        if cur:
            clusters.append(cur)
        dets: List[Tuple[float, float, float]] = []
        for c in clusters:
            if len(c) < 2:
                continue
            a, b = c[0], c[-1]
            size = math.hypot(px[b] - px[a], py[b] - py[a])
            if not (self.ac.lidar_blob_min <= size <= self.ac.lidar_blob_max):
                continue
            if not (self._edge(r, a - 1, r[a]) and self._edge(r, b + 1, r[b])):
                continue                                      # 양쪽이 모두 깊이 불연속이어야 '배경 앞의 고립 물체'다(박스·벽면의 일부는 한쪽이 이어진다)
            mx, my = float(np.mean(px[c])), float(np.mean(py[c]))
            d = math.hypot(mx - pose.x, my - pose.y)
            if d > 1e-6:                                      # 보이는 호의 중심은 실제 차체 중심보다 가깝다 → 시선 방향으로 보정
                k = 0.7 * self.ac.r_body / d
                mx, my = mx + (mx - pose.x) * k, my + (my - pose.y) * k
            dets.append((mx, my, size))
        self.stats["clusters"] += len(dets)
        # 추적 연관
        used = set()
        for (mx, my, size) in dets:
            best, bd = None, self.gate
            for t in self._tracks:
                if id(t) in used:
                    continue
                dd = math.hypot(t.x - mx, t.y - my)
                if dd < bd:
                    best, bd = t, dd
            if best is None:
                bt = _BT(self._next, mx, my, now, size=size)
                bt.hist.append((now, mx, my))
                self._next += 1
                self._tracks.append(bt)
                used.add(id(bt))
            else:
                best.x, best.y, best.t, best.n, best.size = mx, my, now, best.n + 1, size
                best.hist.append((now, mx, my))
                best.hist = [h for h in best.hist if now - h[0] <= 1.0]
                if best.hist[-1][0] - best.hist[0][0] >= 0.3:
                    h0 = best.hist[0]
                    dt = best.hist[-1][0] - h0[0]
                    vx, vy = (mx - h0[1]) / dt, (my - h0[2]) / dt
                    sp = math.hypot(vx, vy)
                    if sp > self.ac.v_enemy_max * 1.5:           # 잡음으로 튄 속도는 클램프
                        vx, vy = vx * self.ac.v_enemy_max / sp, vy * self.ac.v_enemy_max / sp
                    best.vx, best.vy = 0.5 * best.vx + 0.5 * vx, 0.5 * best.vy + 0.5 * vy
                    path_len = sum(math.hypot(b2[1] - b1[1], b2[2] - b1[2]) for b1, b2 in zip(best.hist[:-1], best.hist[1:]))
                    net = math.hypot(mx - h0[1], my - h0[2])
                    if dt >= 0.6 and math.hypot(vx, vy) >= self.ac.lidar_blob_min_speed and net >= 0.15 and net >= 0.7 * path_len:
                        best.moved = True                         # 속도·순변위·직진성이 모두 있어야 '움직인다'(새로 드러난 정적 박스의 자세 오차성 겉보기 이동 거름)
                used.add(id(best))
        self.stats["confirmed"] = sum(1 for t in self._tracks if t.n >= self.ac.lidar_blob_scans)


