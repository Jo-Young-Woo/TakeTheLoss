"""위협장(ThreatField): 같은 격자 위에 시간 층 K 장의 (물리 점유 body, 피격 위험 risk, 근접 close)를 만든다. 순수 numpy.

두 영역을 따로 둔다(설계서 §3.1)
  body_k  적 차체가 *있을 수 있는* 공간 — 하드 차단. 예측 위치(속도×τ_k) 주변 ρ(τ_k)=r_body+k_σ·σ(τ_k) 에 우리 차체 반경(inflate_hard)을 더한 원판의 합집합.
  risk_k  적이 *맞힐 수 있는* 공간 — 소프트 비용. w_team · f_range(거리) · V(시야). 거리 r_core 안은 1, r_out 까지 제곱 감쇠, 박스가 가리면 V=0.15.
  close_k 근접 압박 항(정면에서 붙는 것을 더 싫어하게).
기존 risk_astar는 도착 시간층을 읽는다. 현재 risk_astar_realtime은 현재 관측 점유·비용을 2D 지도에 반영하고
새 센서 필드마다 A*를 새로 계산한다. 미래 층은 노출 시간 평가에 남기며, 시간 확장 최적 경로를 주장하지 않는다.

!! 시야 V 는 '박스가 레이저를 막는다'는 시뮬 규칙과 같은 가정이다. 돔(전방 ±105°) 노출 계수는 2단계로 미룬다(use_dome=False).
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from functools import lru_cache
from typing import Optional, Sequence, Tuple

import numpy as np

from ..perception.threat_board import ThreatTrack


@dataclass
class ThreatField:
    K: int
    dtau: float
    body: np.ndarray                  # (K, ny, nx) bool
    risk: np.ndarray                  # (K, ny, nx) float32  0..1
    close: np.ndarray                 # (K, ny, nx) float32  0..1
    mind: np.ndarray                  # (K, ny, nx) float32  가장 가까운 위협 중심까지 거리(m)
    version: int
    n_tracks: int
    t_built: float
    tracks: Tuple = ()
    body_horizon_s: float = math.inf  # Legacy predictions remain unchanged.
    clearance_disks: Tuple = ()  # Observed (x,y,physical+stop margin,preferred separation), current layer only.

    def layer(self, t_arr: float) -> int:
        return min(self.K - 1, max(0, int(t_arr / max(self.dtau, 1e-6))))

    def body_at(self, t_arr: float, iy: int, ix: int) -> bool:
        # A short velocity forecast cannot be frozen into a permanent wall.
        # The controller replans as fresh scans arrive; immediate clearance is
        # still enforced independently before every drive command.
        return t_arr <= self.body_horizon_s and bool(self.body[self.layer(t_arr)][iy, ix])


_XY_CACHE: dict = {}


@lru_cache(maxsize=16)
def _ray_offsets(step, r_out, n_rays):
    """Cache immutable ray geometry, never map state or sensor observations."""
    s = np.arange(.25,r_out+step,step,dtype=np.float32)
    th = np.linspace(-math.pi,math.pi,n_rays,endpoint=False,dtype=np.float32)
    dx,dy = np.cos(th)[:,None]*s[None,:],np.sin(th)[:,None]*s[None,:]
    for item in (s,dx,dy):
        item.flags.writeable = False
    return s,dx,dy


def _xy(grid) -> Tuple[np.ndarray, np.ndarray]:
    key = (grid.ny, grid.nx, grid.res, grid.ox, grid.oy)
    if key not in _XY_CACHE:
        xs = grid.ox + (np.arange(grid.nx) + 0.5) * grid.res
        ys = grid.oy + (np.arange(grid.ny) + 0.5) * grid.res
        X, Y = np.meshgrid(xs, ys)
        _XY_CACHE[key] = (X.astype(np.float32), Y.astype(np.float32))
    return _XY_CACHE[key]


def _visibility(grid, occ: np.ndarray, px: float, py: float, r_out: float, X: np.ndarray, Y: np.ndarray,
                n_rays: int = 72, v_hidden: float = 0.15, *, geometry=None) -> np.ndarray:
    """(px,py)에서 정적 점유(occ)에 가려지지 않은 칸 = 1, 가려진 칸 = v_hidden."""
    step = grid.res
    s,rx,ry = _ray_offsets(step,r_out,n_rays)
    sx,sy = px+rx,py+ry
    ix = ((sx - grid.ox) / grid.res).astype(int)
    iy = ((sy - grid.oy) / grid.res).astype(int)
    inb = (ix >= 0) & (ix < grid.nx) & (iy >= 0) & (iy < grid.ny)
    hit = np.zeros(sx.shape, dtype=bool)
    hit[inb] = occ[iy[inb], ix[inb]]
    hit |= ~inb
    first = np.where(hit.any(axis=1), hit.argmax(axis=1), len(s))            # 각 광선의 첫 점유 샘플 번호
    hit_d = np.where(first < len(s), s[np.minimum(first, len(s) - 1)], 1e9)  # 첫 점유까지 거리
    dx,dy,d = (X-px,Y-py,None) if geometry is None else geometry
    ang = np.arctan2(dy,dx)
    a_idx = np.clip(((ang + math.pi) / (2 * math.pi) * n_rays).astype(int), 0, n_rays - 1)
    if d is None:
        d = np.hypot(dx,dy)
    vis = d <= (hit_d[a_idx] + 2 * step)
    return np.where(vis, 1.0, v_hidden).astype(np.float32)


def build_field(tracks: Sequence[ThreatTrack], grid, ac, laser_range: float, now: float, r_hard: float,
                version: int = 0, use_visibility: bool = True, *, risk_cfg=None,
                min_clearance_m: float = .30, hit_history=(), static_boxes=(),
                ego_velocity=(0., 0.), body_radius_m=None) -> Optional[ThreatField]:
    """위협이 없으면 None."""
    if risk_cfg is not None:
        return _cost_field(tracks, grid, ac, now, r_hard, version, use_visibility,
                           risk_cfg, min_clearance_m, hit_history, static_boxes, ego_velocity,body_radius_m)
    if not tracks:
        return None
    K, dtau = int(ac.layers), float(ac.layer_dt)
    X, Y = _xy(grid)
    ny, nx = grid.ny, grid.nx
    body = np.zeros((K, ny, nx), dtype=bool)
    risk = np.zeros((K, ny, nx), dtype=np.float32)
    close = np.zeros((K, ny, nx), dtype=np.float32)
    mind = np.full((K, ny, nx), 99.0, dtype=np.float32)
    r_out = laser_range + ac.r_laser_extra
    r_cc = 1.2
    occ = grid.occupied() if use_visibility else None
    for tt in tracks:
        vis_first: Optional[np.ndarray] = None
        vis_last: Optional[np.ndarray] = None
        for k in range(K):
            px, py, rho = tt.predict(now, k * dtau, ac)
            d = np.hypot(X - px, Y - py)
            np.minimum(mind[k], d, out=mind[k])
            body[k] |= d <= (rho + r_hard)
            f = np.where(d <= ac.r_core, 1.0, np.clip((r_out - d) / max(r_out - ac.r_core, 1e-6), 0.0, 1.0) ** 2).astype(np.float32)
            f = np.where(d >= r_out, 0.0, f)
            vis = 1.0
            if use_visibility:                                      # 시야는 첫 층과 마지막 층에서만 계산하고 중간 층은 첫 층을 쓴다(예측 이동이 작다)
                if k == K - 1 and K > 1:
                    if vis_last is None:
                        vis_last = _visibility(grid, occ, px, py, r_out, X, Y)
                    vis = vis_last
                else:
                    if vis_first is None:
                        vis_first = _visibility(grid, occ, px, py, r_out, X, Y)
                    vis = vis_first
            risk[k] += tt.weight * f * vis
            close[k] = np.maximum(close[k], tt.weight * np.clip((r_cc - d) / r_cc, 0.0, 1.0) ** 2)
    np.minimum(risk, 1.0, out=risk)
    return ThreatField(K, dtau, body, risk, close, mind, version, len(tracks), now, tuple(tracks))


def observed_static_mask(grid, tracks=(), static_boxes=()):
    """Observed map geometry, excluding tracked movable bodies; no world truth."""
    occ = grid.occupied().copy()
    X, Y = _xy(grid)
    for tt in tracks:
        occ[np.hypot(X-tt.x, Y-tt.y) <= .26] = False
    occ |= grid.static
    for box in static_boxes:
        occ |= (np.abs(X-box.cx) <= box.sx/2) & (np.abs(Y-box.cy) <= box.sy/2)
    return occ


def bottleneck_mask(occ, res, max_width=.60):
    """Free cells flanked by observed solids with a gap <= max_width.

    Only cardinal local gaps are estimated. This is a preference cost, never a
    geometric guarantee and never a hard keepout.
    """
    cap = int(math.ceil(max_width/res))+1
    left = np.full(occ.shape, cap, dtype=np.int16)
    right, up, down = left.copy(), left.copy(), left.copy()
    for i in range(1, min(cap, occ.shape[1])):
        left[:,i:] = np.minimum(left[:,i:], np.where(occ[:,:-i],i,cap))
        right[:,:-i] = np.minimum(right[:,:-i], np.where(occ[:,i:],i,cap))
    for i in range(1, min(cap, occ.shape[0])):
        down[i:,:] = np.minimum(down[i:,:], np.where(occ[:-i,:],i,cap))
        up[:-i,:] = np.minimum(up[:-i,:], np.where(occ[i:,:],i,cap))
    return ~occ & (((left+right-1)*res <= max_width+1e-6)
                   | ((up+down-1)*res <= max_width+1e-6))


def _cost_field(tracks, grid, ac, now, r_hard, version, use_visibility,
                rc, clearance, hit_history, static_boxes, ego_velocity,body_radius_m=None):
    if not tracks and not hit_history:
        return None
    K, dtau = int(ac.layers), float(ac.layer_dt)
    X, Y = _xy(grid)
    shape = (K, grid.ny, grid.nx)
    body = np.zeros(shape, dtype=bool)
    risk = np.zeros(shape, dtype=np.float32)
    close = np.zeros(shape, dtype=np.float32)
    mind = np.full(shape, 99., dtype=np.float32)
    radius = max(.01, float(rc.range_m))
    costs = {"exposure":1., "hit_history":2., "bottleneck":1.5,
             "closing":.5, "ally_fire_lane":1.}
    costs.update(getattr(rc, "cost_table", {}))
    # A typo or imported negative table must not create negative A* edges.
    costs = {key:max(0.,float(value)) for key,value in costs.items()}
    occ = observed_static_mask(grid, tracks, static_boxes)
    narrow = bottleneck_mask(occ, grid.res) if tracks else np.zeros(occ.shape,dtype=bool)
    fresh_body_s = max(.2, float(getattr(rc, "body_stale_s", 1.6)))
    physical_radius=max(0.,float(body_radius_m or 0.))
    center_clearance=(max(ac.r_body,physical_radius)+max(r_hard,physical_radius)
                      +max(0.,clearance))
    collision_radius = max(ac.r_body,physical_radius)+max(r_hard,physical_radius)+.03
    clearance_disks = []
    for tt in tracks:
        # Uncertainty changes risk spread; it must not grow a metre-sized hard
        # ghost. Only recently observed physical bodies get a safety disk.
        # Stationary tracks and capped (>3 s) predictions repeat exactly. Reuse
        # their geometry only inside this one build/track. Freshness and body
        # eligibility are still evaluated separately for every time layer.
        previous_position = None
        previous_terms = None
        for k in range(K):
            px, py, _ = tt.predict(now, k*dtau, ac)
            if previous_position != (px,py):
                dx,dy = X-px,Y-py
                d = np.hypot(dx,dy)
                vis = _visibility(grid,occ,px,py,radius,X,Y,n_rays=180,v_hidden=0.,
                                  geometry=(dx,dy,d)) if use_visibility else 1.
                distance_factor = np.clip((radius-d)/radius,0.,1.)
                closing = (dx*(tt.vx-ego_velocity[0])+dy*(tt.vy-ego_velocity[1]))/np.maximum(d,.05)
                approach = 1.+costs["closing"]*np.clip(closing/max(ac.v_enemy_max,.05),0.,1.)
                exposure = costs["exposure"]*max(0.,tt.weight)*distance_factor*vis*approach
                proximity = max(0.,tt.weight)*np.clip((1.2-d)/1.2,0.,1.)**2
                bottleneck = costs["bottleneck"]*narrow*np.exp(-.5*(d/.65)**2)
                previous_position = (px,py)
                previous_terms = (d,exposure,proximity,bottleneck)
            d,exposure,proximity,bottleneck = previous_terms
            np.minimum(mind[k],d,out=mind[k])
            if tt.age(now) + k*dtau <= fresh_body_s:
                body[k] |= d <= center_clearance
                if k == 0:
                    clearance_disks.append((float(px),float(py),collision_radius,center_clearance))
            risk[k] += exposure
            close[k] = np.maximum(close[k],proximity)
            # Penalise an occupied narrow passage and both entrances, without
            # expecting a teammate to cooperate or declaring it forbidden.
            risk[k] += bottleneck
    for hit in hit_history:
        # Public input contract is (x, y, timestamp); history decays in 30 s.
        hx, hy, ht = map(float,hit[:3])
        age = max(0.,now-ht)
        if age <= 120.:
            bump = costs["hit_history"]*math.exp(-age/30.)*np.exp(-.5*((X-hx)**2+(Y-hy)**2)/.35**2)
            risk += bump[None,:,:]
    return ThreatField(K,dtau,body,risk,close,mind,version,len(tracks),now,tuple(tracks),fresh_body_s,tuple(clearance_disks))


def path_exposure(path: Sequence[Tuple[float, float]], v_plan: float, field: ThreatField, grid, risk_hit: float = 0.5,
                  t0: float = 0.0) -> Tuple[float, float]:
    """(위험 적분 [risk·s], 최대 연속 노출 시간 [s]) — risk ≥ risk_hit 인 구간의 연속 시간."""
    if field is None or len(path) < 2:
        return 0.0, 0.0
    t, ri, run, mx = t0, 0.0, 0.0, 0.0
    for (ax, ay), (bx, by) in zip(path[:-1], path[1:]):
        dt = math.hypot(bx - ax, by - ay) / max(v_plan, 1e-6)
        k = field.layer(t)
        iy, ix = grid.to_cell(bx, by)
        r = float(field.risk[k][iy, ix]) if grid.in_bounds(iy, ix) else 0.0
        ri += r * dt
        if r >= risk_hit:
            run += dt
            mx = max(mx, run)
        else:
            run = 0.0
        t += dt
    return ri, mx


def risk_at(field: Optional[ThreatField], grid, x: float, y: float, t_arr: float = 0.0) -> float:
    if field is None:
        return 0.0
    iy, ix = grid.to_cell(x, y)
    if not grid.in_bounds(iy, ix):
        return 0.0
    return float(field.risk[field.layer(t_arr)][iy, ix])


def body_hit(field: Optional[ThreatField], grid, x: float, y: float, t_arr: float = 0.0) -> bool:
    if field is None:
        return False
    iy, ix = grid.to_cell(x, y)
    if not grid.in_bounds(iy, ix):
        return True
    return field.body_at(t_arr, iy, ix)
