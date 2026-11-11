"""자세 감시(PoseGuard): 충격 감지, LiDAR 장착 틀어짐, SLAM 도약, 경기장(UAV) 좌표 연결 점검.

왜 필요한가
  - Nav2Stack 은 시작 때 한 번 정한 Anchor(경기장 = Anchor ∘ map)로 SLAM 자세를 경기장(UAV) 좌표로 옮긴다. 그 뒤로는 연결이 맞는지 다시 보지 않는다.
  - 충돌로 LiDAR 가 차체에 대해 돌아가면 SLAM 은 '틀어진 LiDAR'를 정확히 따라간다. 그래서 SLAM 자세의 헤딩에 상수 편차 δ 가 생기고,
    차체 기준의 바퀴·IMU 헤딩과는 어긋난다 — δ = (SLAM 헤딩 − 차체 헤딩) 의 충격 전후 차이.
  - 이 파일은 ROS·Gazebo 없이 시험할 수 있는 순수 계산이다. 연결은 Nav2Stack.update / Agent._sense 가 한다.

네 가지 일
  1. ImpactDetector : IMU 가속도(+자이로)로 충돌을 IMU 주기에서 잡는다. 방향(차체 기준 접촉 방위)·세기를 낸다.
  2. 틀어짐 추정   : 충격 전 기준 오프셋(SLAM 헤딩 − 차체 헤딩)과 충격 후 정지 상태 오프셋의 차이 = LiDAR 장착 yaw 틀어짐.
                      확정되면 자세 헤딩에서 빼고(pose.th -= δ) 스캔 각도에 더한다(scan.angles += δ). 크면 Nav2 대신 내장 A* 로 간다
                      (Nav2 의 TF 는 Agent 가 못 고친다).
  3. SLAM 도약 감시 : 한 틱에 예측(이전 자세 + 오도메트리 변화)에서 크게 벗어난 SLAM 자세는 바로 믿지 않고 몇 표본 동안 확인한다.
                      TF 가 끊겨도 제한된 시간 동안 IMU/바퀴 추측항법으로 자세를 이어 간다. 그 시간을 넘으면 'lost'.
  4. 경기장 연결 점검: LiDAR 끝점을 경기장 외벽·고정 구조물 13개(+UAV 상자 윤곽)에 맞춰 본다. 시작 때의 맞음 정도를 기준선으로 삼고(경기장 도면
                      치수 오차는 기준선에 흡수된다) 그 뒤 크게 나빠졌을 때만 '복구용' 소폭 보정을 제안한다. 평상시 절대 보정은 하지 않는다.

모든 각도는 내부에서 rad. 설정(PoseGuardCfg)은 deg·m·s.
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import Any, Callable, Deque, Dict, List, Optional, Sequence, Tuple

import numpy as np

from ..types import Pose2D, Scan
from ..util import wrap

Rect = Tuple[float, float, float, float]          # (cx, cy, sx, sy) 경기장 좌표 — UAV 상자 윤곽


# ====================================================================== 1. 충돌 감지
@dataclass
class ImpactEvent:
    t: float
    peak_dg: float                       # 기준선에서 벗어난 최대 가속도 [g]
    bearing: Optional[float]             # 접촉 방위(차체 기준 rad, 0 = 앞, 왼쪽 +). 모르면 None
    gyro_dps: float = 0.0                # 충격 순간 yaw 각속도 크기 [deg/s] (없으면 0)
    src: str = "accel"                   # accel | gyro | yaw_step
    major_dg: float = 1.2
    major_gyro_dps: float = 400.0

    @property
    def severity(self) -> str:
        return "major" if (self.peak_dg >= self.major_dg or self.gyro_dps >= self.major_gyro_dps) else "minor"

    @property
    def sector(self) -> str:
        if self.bearing is None:
            return "unknown"
        a = abs(wrap(self.bearing))
        if a <= math.radians(50):
            return "front"
        if a >= math.radians(130):
            return "rear"
        return "left" if self.bearing > 0 else "right"


class ImpactDetector:
    """IMU 표본마다 부르는 충돌 감지기. 느리게 변하는 기준선(중력·기울기·가속)에서 갑자기 벗어난 가속도를 충돌로 본다.

    - 기준선은 지수이동평균(시정수 baseline_tau_s). 충격 표본은 기준선에 섞지 않는다.
    - 문턱 = max(trigger_dg, noise_k × 최근 평균 편차) — 거친 바닥의 모터 진동이 충돌로 오인되지 않게 한다.
    - 편차가 sustain_s 넘게 이어지면 충돌이 아니라 기울기 변화(경사로)로 보고 기준선을 현재 값으로 옮긴다.
    - ax, ay(차체 기준 수평 가속도 [g])가 있으면 방향을 낸다: 몸이 받은 가속도의 반대쪽이 접촉 방위.
    - 한 번의 충돌은 refractory_s 안에 한 번만 낸다(그 안의 더 큰 값은 같은 사건의 peak 에 반영).
    스레드: IMU 콜백 스레드에서 add(), Agent 스레드에서 pop() — 내부 deque 가 한 방향이라 별도 잠금은 쓰지 않는다."""

    def __init__(self, trigger_dg: float = 0.45, major_dg: float = 1.2, gyro_dps: float = 180.0, major_gyro_dps: float = 400.0,
                 refractory_s: float = 0.35, baseline_tau_s: float = 1.5, noise_k: float = 5.0, warmup_s: float = 0.5,
                 sustain_s: float = 0.6) -> None:
        self.trigger_dg, self.major_dg, self.gyro_dps, self.major_gyro_dps = trigger_dg, major_dg, gyro_dps, major_gyro_dps
        self.refractory_s, self.tau, self.noise_k, self.warmup_s, self.sustain_s = refractory_s, baseline_tau_s, noise_k, warmup_s, sustain_s
        self._t0: Optional[float] = None
        self._tp: Optional[float] = None
        self._bg = self._bx = self._by = 0.0
        self._noise = 0.03
        self._n_noise = 0
        self._until = -1e18
        self._cur: Optional[ImpactEvent] = None
        self._cur_h = 0.0
        self._trig_since: Optional[float] = None
        self.events: Deque[ImpactEvent] = deque(maxlen=32)
        self.count = 0

    def add(self, t: float, g: float, ax: Optional[float] = None, ay: Optional[float] = None,
            gz_dps: Optional[float] = None) -> Optional[ImpactEvent]:
        if not (math.isfinite(t) and math.isfinite(g)):
            return None
        if self._tp is not None and t <= self._tp:
            return None                                              # 시각이 거꾸로 가는 표본은 버린다
        has_xy = ax is not None and ay is not None and math.isfinite(ax) and math.isfinite(ay)
        if self._t0 is None:
            self._t0, self._tp, self._bg = t, t, g
            if has_xy:
                self._bx, self._by = float(ax), float(ay)
            return None
        dt = t - self._tp
        self._tp = t
        alpha = 1.0 - math.exp(-min(dt, 0.5) / max(self.tau, 1e-3))
        dev_g = g - self._bg
        hx, hy = (ax - self._bx, ay - self._by) if has_xy else (0.0, 0.0)
        dh = math.hypot(hx, hy)
        mag = max(abs(dev_g), dh)
        gyro = abs(gz_dps) if gz_dps is not None and math.isfinite(gz_dps) else 0.0
        thr = max(self.trigger_dg, self.noise_k * self._noise)
        warm = (t - self._t0) < self.warmup_s
        trig = (not warm) and (mag >= thr or gyro >= self.gyro_dps)
        if not trig:
            self._trig_since = None
            self._bg += alpha * dev_g
            if has_xy:
                self._bx += alpha * hx
                self._by += alpha * hy
            self._n_noise += 1
            a_noise = max(alpha, 1.0 / self._n_noise) if warm else alpha          # 시작 구간은 단순 평균으로 잡음 수준을 빨리 잡는다
            self._noise += a_noise * (min(mag, 0.5) - self._noise)
            return None
        rising = self._trig_since is None
        if rising:
            self._trig_since = t
        else:
            if self._cur is not None:                                              # 같은 덩어리의 후속 표본: 더 큰 값은 같은 사건에 반영
                if mag > self._cur.peak_dg:
                    self._cur.peak_dg = mag
                if gyro > self._cur.gyro_dps:
                    self._cur.gyro_dps = gyro
                if has_xy and dh > self._cur_h and dh >= 0.5 * thr:
                    self._cur_h, self._cur.bearing = dh, math.atan2(-hy, -hx)
            if t - self._trig_since >= self.sustain_s:
                self._bg, self._trig_since = g, None                                # 이어지는 편차 = 기울기 변화: 기준선을 옮긴다
                if has_xy:
                    self._bx, self._by = float(ax), float(ay)
            return None
        if self._cur is not None and t < self._until:                              # refractory 안의 두 번째 덩어리 = 같은 충돌
            if mag > self._cur.peak_dg:
                self._cur.peak_dg = mag
            if gyro > self._cur.gyro_dps:
                self._cur.gyro_dps = gyro
            return None
        ev = ImpactEvent(t, mag, math.atan2(-hy, -hx) if (has_xy and dh >= 0.5 * mag and dh >= 0.5 * thr) else None, gyro,
                         "gyro" if (gyro >= self.gyro_dps and mag < thr) else "accel", self.major_dg, self.major_gyro_dps)
        self._cur, self._cur_h, self._until = ev, dh, t + self.refractory_s
        self.events.append(ev)
        self.count += 1
        return ev

    def pop(self) -> List[ImpactEvent]:
        out: List[ImpactEvent] = []
        while self.events:
            out.append(self.events.popleft())
        return out


# ====================================================================== 2. 경기장 맞춤 (벽·구조물·UAV 상자)
def wall_heading_residual(points: np.ndarray, max_gap: float = 0.12) -> Optional[Tuple[float, float, int]]:
    """경기장 좌표 점들(스캔 각도 순서, N×2)에서 '축 정렬 선분'의 방향 편차 ψ [rad, (-π/4, π/4]]와 집중도 R(0..1), 쌍 수.
    경기장 벽·구조물·박스는 모두 축에 정렬(20 cm 격자)이라, 올바른 자세라면 ψ ≈ 0 이다. 도면 치수 오차와 무관한 헤딩 점검이다.
    4θ 원형 평균(90° 대칭)을 쓴다. 이어진 점(간격 < max_gap)만 선분으로 센다."""
    if points is None or len(points) < 8:
        return None
    d = points[1:] - points[:-1]
    L = np.hypot(d[:, 0], d[:, 1])
    ok = (L > 0.008) & (L < max_gap)
    if ok.sum() < 6:
        return None
    th = np.arctan2(d[ok, 1], d[ok, 0])
    w = L[ok]
    z = np.sum(w * np.exp(4j * th))
    tot = float(np.sum(w))
    return float(np.angle(z) / 4.0), float(abs(z) / max(tot, 1e-9)), int(ok.sum())


class ArenaFit:
    """LiDAR 끝점 ↔ 외벽 + 고정 구조물(+UAV 상자 윤곽) 거리장. ArenaEdgeMap 의 거리장을 재사용한다."""

    def __init__(self, width: float, height: float, rectangles=None, resolution: float = 0.025) -> None:
        from .arena_localization import ArenaEdgeMap
        if rectangles is None:
            try:
                from .arena_localization import fixed_infrastructure_rectangles
                rectangles = fixed_infrastructure_rectangles()
            except Exception:                                        # noqa: BLE001 — 구조물 목록이 없으면 외벽만
                rectangles = ()
        self.map = ArenaEdgeMap(width, height, rectangles, resolution)
        self.width, self.height = width, height

    @staticmethod
    def rect_distance(points: np.ndarray, rects: Sequence[Rect]) -> np.ndarray:
        """각 점에서 가장 가까운 사각형 '윤곽'까지 거리(안쪽도 윤곽까지)."""
        shape = points.shape[:-1]
        p = points.reshape(-1, 2)
        best = np.full(len(p), np.inf)
        for cx, cy, sx, sy in rects:
            hx, hy = sx / 2.0, sy / 2.0
            dx, dy = np.abs(p[:, 0] - cx) - hx, np.abs(p[:, 1] - cy) - hy
            outside = np.hypot(np.maximum(dx, 0.0), np.maximum(dy, 0.0))
            inside = -np.maximum(dx, dy)
            d = np.where((dx > 0) | (dy > 0), outside, inside)
            best = np.minimum(best, d)
        return best.reshape(shape)

    def distance(self, points: np.ndarray, rects: Sequence[Rect] = ()) -> np.ndarray:
        d = self.map.distances(points)
        return np.minimum(d, self.rect_distance(points, rects)) if len(rects) else d

    @staticmethod
    def scan_points(scan: Scan, max_pts: int = 140, rmin: float = 0.15, rmax: float = 7.5,
                    exclude: Optional[Tuple[Sequence[float], Sequence[Tuple[float, float, float]]]] = None) -> np.ndarray:
        """스캔 → 차체 기준 끝점(N×2, 각도 순서 유지). max_pts 개로 균등 솎는다.
        exclude=(pose, [(x, y, 반경)…]): 경기장 좌표에서 그 원 안에 드는 끝점은 뺀다(UGV 같은 움직이는 것 — 솎기 전에 빼서 남은 점 수를 지킨다)."""
        valid = scan.valid() & (scan.ranges > rmin) & (scan.ranges < rmax)
        if exclude is not None and len(exclude[1]):
            pose, circles = exclude
            a = scan.angles + pose[2]
            px = pose[0] + scan.ranges * np.cos(a)
            py = pose[1] + scan.ranges * np.sin(a)
            with np.errstate(invalid="ignore"):
                for cx, cy, r in circles:
                    valid &= ~(np.hypot(px - cx, py - cy) < r)
        idx = np.flatnonzero(valid)
        if len(idx) < 12:
            return np.zeros((0, 2))
        if len(idx) > max_pts:
            idx = idx[np.linspace(0, len(idx) - 1, max_pts).astype(int)]
        a, r = scan.angles[idx], scan.ranges[idx]
        return np.column_stack([r * np.cos(a), r * np.sin(a)])

    @staticmethod
    def to_arena(points_base: np.ndarray, pose: Sequence[float]) -> np.ndarray:
        c, s = math.cos(pose[2]), math.sin(pose[2])
        return np.column_stack([pose[0] + c * points_base[:, 0] - s * points_base[:, 1],
                                pose[1] + s * points_base[:, 0] + c * points_base[:, 1]])

    def score(self, points_base: np.ndarray, pose: Sequence[float], rects: Sequence[Rect] = (),
              sigma: float = 0.05, inlier: float = 0.06) -> Tuple[float, float]:
        """(가우시안 점수 0..1, 설명된 비율 0..1). 설명된 비율 = 윤곽에서 inlier 안에 든 끝점 비율."""
        if len(points_base) == 0:
            return 0.0, 0.0
        d = self.distance(self.to_arena(points_base, pose), rects)
        return float(np.mean(np.exp(-0.5 * (d / sigma) ** 2))), float(np.mean(d < inlier))

    def _batch(self, points_base: np.ndarray, base_pose: Sequence[float], offs: np.ndarray, rects: Sequence[Rect],
               sigma: float) -> np.ndarray:
        th = base_pose[2] + offs[:, 2:3]
        c, s = np.cos(th), np.sin(th)
        ex = base_pose[0] + offs[:, 0:1] + c * points_base[None, :, 0] - s * points_base[None, :, 1]
        ey = base_pose[1] + offs[:, 1:2] + s * points_base[None, :, 0] + c * points_base[None, :, 1]
        d = self.distance(np.stack([ex, ey], axis=-1), rects)
        return np.mean(np.exp(-0.5 * (d / sigma) ** 2), axis=1)

    def search(self, points_base: np.ndarray, pose: Sequence[float], rects: Sequence[Rect] = (),
               xy: float = 0.15, th: float = math.radians(6.0), sigma: float = 0.05) -> Dict[str, Any]:
        """자세 주변(±xy m, ±th rad)에서 점수가 가장 높은 오프셋을 굵은→가는 격자로 찾는다.
        반환: off=(dx,dy,dth), s0(0 오프셋 점수), sb(최고 점수), obs=(x,y,th 가 관측 가능한가) — 관측 불가 축(벽 하나만 보일 때 벽을 따르는 방향)은
        점수가 평평해 최고점 위치가 임의이므로 호출자가 0 으로 취급해야 한다."""
        if len(points_base) < 20:
            return {"ok": False}
        coarse_x = np.linspace(-xy, xy, 7)
        coarse_t = np.linspace(-th, th, 7)
        offs = np.array([(a, b, c) for a in coarse_x for b in coarse_x for c in coarse_t])
        sc = self._batch(points_base, pose, offs, rects, sigma)
        best = offs[int(np.argmax(sc))]
        fx, ft = (coarse_x[1] - coarse_x[0]) * 0.5, (coarse_t[1] - coarse_t[0]) * 0.5
        fine = np.array([(best[0] + a, best[1] + b, best[2] + c) for a in np.linspace(-fx, fx, 5)
                         for b in np.linspace(-fx, fx, 5) for c in np.linspace(-ft, ft, 5)])
        sc2 = self._batch(points_base, pose, fine, rects, sigma)
        k = int(np.argmax(sc2))
        best, sb = fine[k], float(sc2[k])
        s0 = float(self._batch(points_base, pose, np.zeros((1, 3)), rects, sigma)[0])
        probe_xy, probe_th = 0.05, math.radians(2.5)
        probes = np.array([best + np.array(v) for v in
                           ((probe_xy, 0, 0), (-probe_xy, 0, 0), (0, probe_xy, 0), (0, -probe_xy, 0), (0, 0, probe_th), (0, 0, -probe_th))])
        sp = self._batch(points_base, pose, probes, rects, sigma)
        drop = np.array([sb - max(sp[0], sp[1]), sb - max(sp[2], sp[3]), sb - max(sp[4], sp[5])])
        obs = tuple(bool(v >= 0.035) for v in drop)
        # 격자 가장자리에서 끝난 최적은 '더 멀리 있을 수 있다' — 신뢰하지 않는다
        edge = bool(abs(best[0]) >= xy - 1e-6 or abs(best[1]) >= xy - 1e-6 or abs(best[2]) >= th - 1e-6)
        return {"ok": True, "off": (float(best[0]), float(best[1]), float(best[2])), "s0": s0, "sb": sb, "obs": obs, "edge": edge}

    def search_wide(self, points_base: np.ndarray, pose: Sequence[float], rects: Sequence[Rect] = (),
                    center: Sequence[float] = (0.0, 0.0, 0.0), xy: float = 0.5, th: float = math.radians(20.0),
                    sigma: float = 0.05, step_xy: float = 0.05, step_th: float = math.radians(1.5)) -> Dict[str, Any]:
        """앵커 재설정용 넓은 탐색: center(기준선 오프셋) 둘레 ±xy m, ±th rad 를 거친 격자로 훑고, 서로 떨어진 상위 후보 몇 개를 좁게 다듬는다.
        반환: off(pose 기준 전체 오프셋), sb(최고 점수), s_center(center 의 점수), ratio(최고 후보에서 설명되는 끝점 비율), margin(최고와 '다른 후보'의 점수 차 —
        벽 하나만 보여 따라 미끄러지는 방향이 있거나 대칭·반복 구조면 0 에 가깝다), obs(x,y,θ 가 모두 점수에 영향을 주는가), edge(탐색 가장자리 최적).
        탐색 가장자리에서 끝난 최적은 더 먼 곳에 진짜가 있을 수 있어 호출자가 신뢰하지 않는다."""
        if len(points_base) < 30:
            return {"ok": False, "why": "few_points"}
        cx, cy, ct = (float(v) for v in center)
        nx, nt = max(1, int(round(xy / step_xy))), max(1, int(round(th / step_th)))
        gx = np.arange(-nx, nx + 1) * step_xy
        gt = np.arange(-nt, nt + 1) * step_th
        offs = np.stack(np.meshgrid(gx, gx, gt, indexing="ij"), axis=-1).reshape(-1, 3) + np.array([cx, cy, ct])
        sc = np.empty(len(offs))
        for i in range(0, len(offs), 1200):
            sc[i:i + 1200] = self._batch(points_base, pose, offs[i:i + 1200], rects, sigma)
        order = np.argsort(-sc)
        cands: List[Tuple[float, Tuple[float, float, float]]] = []
        for k in order[:400]:
            o = offs[k]
            if all(math.hypot(o[0] - q[0], o[1] - q[1]) > 0.12 or abs(o[2] - q[2]) > math.radians(4.0) for _, q in cands):
                cands.append((float(sc[k]), (float(o[0]), float(o[1]), float(o[2]))))
                if len(cands) >= 3:
                    break
        refined: List[Tuple[float, Tuple[float, float, float], Dict[str, Any]]] = []
        for _, o in cands:
            shifted = (pose[0] + o[0], pose[1] + o[1], pose[2] + o[2])
            r = self.search(points_base, shifted, rects, xy=step_xy, th=step_th, sigma=sigma)
            if not r.get("ok"):
                continue
            tot = (o[0] + r["off"][0], o[1] + r["off"][1], o[2] + r["off"][2])
            if all(math.hypot(tot[0] - q[0], tot[1] - q[1]) > 0.12 or abs(tot[2] - q[2]) > math.radians(4.0) for _, q, _ in refined):
                refined.append((float(r["sb"]), tot, r))
        if not refined:
            return {"ok": False, "why": "no_peak"}
        refined.sort(key=lambda x: -x[0])
        sb, tot, r = refined[0]
        margin = sb - refined[1][0] if len(refined) > 1 else sb
        s_center = float(self._batch(points_base, pose, np.array([[cx, cy, ct]]), rects, sigma)[0])
        _, ratio = self.score(points_base, (pose[0] + tot[0], pose[1] + tot[1], pose[2] + tot[2]), rects, sigma=sigma)
        edge = bool(max(abs(tot[0] - cx), abs(tot[1] - cy)) >= xy - 1e-6 or abs(tot[2] - ct) >= th - 1e-6)
        return {"ok": True, "off": tot, "sb": sb, "s_center": s_center, "ratio": float(ratio), "margin": float(margin),
                "obs": r["obs"], "edge": edge}


# ====================================================================== 3. 자세 감시
def _circ_median(vals: Sequence[float]) -> float:
    """각도 값들의 원형 중앙값(첫 값 기준으로 펼친 뒤 중앙값)."""
    v = np.asarray(vals, dtype=float)
    ref = v[0]
    return float(wrap(ref + float(np.median([wrap(x - ref) for x in v]))))


def _circ_std(vals: Sequence[float]) -> float:
    v = np.asarray(vals, dtype=float)
    m = _circ_median(vals)
    return float(np.sqrt(np.mean([wrap(x - m) ** 2 for x in v])))


class PoseGuard:
    """SLAM 자세 → 경기장(UAV) 좌표 자세를 감시·보정한다. 설정 키는 config.PoseGuardCfg 참고."""

    def __init__(self, cfg: Any, geo: Any, *, rectangles=None, fit: Optional[ArenaFit] = None,
                 log: Optional[Callable[[float, str], None]] = None) -> None:
        self.c, self.geo = cfg, geo
        self._log = log or (lambda t, m: None)
        self.fit: Optional[ArenaFit] = fit
        self._fit_rects = rectangles
        self.pose: Optional[Pose2D] = None
        self._odom_prev: Optional[Pose2D] = None
        self._t_prev: Optional[float] = None
        self.state = "ok"                                   # ok | degraded | lost
        self.reasons: List[str] = []
        self.last_fix_t: Optional[float] = None
        self.rot = 0.0                                      # 평활한 yaw 변화율 [rad/s] (오도메트리)
        self.lin = 0.0                                      # 평활한 선속도 [m/s]
        self.fixes = 0
        self.dead_reckoned = 0
        self.jumps_held = 0
        self.jumps_accepted = 0
        # ---- LiDAR 장착 yaw 틀어짐
        self.shift = 0.0                                    # 확정된 누적 틀어짐 δ [rad]
        self.shift_events: List[Dict[str, Any]] = []
        self._d_hist: Deque[Tuple[float, float]] = deque(maxlen=400)
        self._d_anchor: Optional[float] = None              # 마지막으로 확인된 (SLAM 헤딩 − 차체 헤딩) 기준 오프셋
        self._v: Optional[Dict[str, Any]] = None            # 진행 중인 충격 후 검증
        self.verify_results: List[Dict[str, Any]] = []
        # ---- SLAM 도약
        self._jump: Optional[Dict[str, Any]] = None
        # ---- 경기장 연결
        self.uav_rects: List[Rect] = []
        self._link_t = -1e9
        self._link_base: Optional[Dict[str, Any]] = None
        self.link: Dict[str, Any] = {"state": "unchecked"}
        self._link_bad: List[Tuple[float, float, float]] = []
        self._trim_req: Optional[Tuple[float, float, float]] = None
        self._trim_total = [0.0, 0.0, 0.0]
        self.trims_applied = 0
        self.link_checks = 0
        self._moving_t = -1e9
        self._link_disabled = False
        self._shift_unverified = False                      # 충격 후 틀어짐을 판정하지 못했거나 보정 한도를 넘었다 → 내장 A* 로 간다
        # ---- 앵커 재설정(reanchor, 기본 꺼짐)
        self.uav_rects_raw: List[Rect] = []                 # UAV 정렬(dx,dy)을 빼기 전의 UAV 상자 윤곽 — 앵커 재설정 적합에만 쓴다(정렬이 앵커 오차를 품고 있다)
        self._ra: Optional[Dict[str, Any]] = None           # 진행 중인 시도(트리거·표본)
        self._ra_req: Optional[Dict[str, Any]] = None       # 스택이 적용할 요청
        self._ra_ver: Optional[Dict[str, Any]] = None       # 적용 뒤 확인
        self._ra_total = [0.0, 0.0, 0.0]
        self._ra_count = 0
        self._ra_last_t = -1e9
        self._ra_reverts = 0
        self._ra_locked = False                             # 원복이 거듭되면 더 하지 않는다
        self._ra_weak_n = 0
        self._now = -1e9
        self._resid: List[Tuple[float, float, float, float, float]] = []     # (x, y, 기준선 대비 잔차 dx, dy, dth) — 연결이 좋은 동안 자리마다 잰 도면 잔차
        self._resid_t = -1e9
        self.ugv_points: List[Tuple[float, float, float]] = []   # 알려진 UGV 위치(경기장 좌표 x, y, 반경) — 앵커 재설정 적합에서 이 주변 끝점을 뺀다
        self.ugv_excluded = 0                                    # 마지막 적합에서 뺀 UGV 후보 수(보고용)
        self._plan_warned = False
        self.reanchors: List[Dict[str, Any]] = []
        self.reanchor_rejects: Dict[str, int] = {}

    def set_log(self, fn: Callable[[float, str], None]) -> None:
        self._log = fn

    # ------------------------------------------------------------------ 외부에서 쓰는 값
    @property
    def verifying(self) -> bool:
        return self._v is not None

    @property
    def shift_applied(self) -> float:
        return self.shift if getattr(self.c, "shift_compensate", True) else 0.0

    @property
    def needs_builtin_nav(self) -> bool:
        """LiDAR 틀어짐이 Nav2 로 계속 가기엔 크다(Nav2 의 TF 는 Agent 가 못 고친다) → 내장 A* 로 간다."""
        return self._shift_unverified or abs(math.degrees(self.shift)) >= float(self.c.fallback_deg)

    @property
    def speed_scale(self) -> float:
        return float(self.c.degraded_speed_scale) if self.state == "degraded" else 1.0

    def set_uav_rects(self, rects: Sequence[Rect], raw: Optional[Sequence[Rect]] = None) -> None:
        ok = lambda rs: [tuple(map(float, r)) for r in rs if len(r) == 4 and all(math.isfinite(float(v)) for v in r)]   # noqa: E731
        self.uav_rects = ok(rects)
        if raw is not None:
            self.uav_rects_raw = ok(raw)

    def set_ugv_points(self, points: Sequence[Tuple[float, float, float]]) -> None:
        """알려진 UGV 위치(트래커·움직이는 LiDAR 덩어리; 경기장 좌표). 앵커 재설정에서 UGV 를 구조물로 착각하지 않도록 그 주변 끝점을 적합에서 뺀다."""
        self.ugv_points = [(float(x), float(y), float(r)) for x, y, r in points
                           if math.isfinite(float(x)) and math.isfinite(float(y)) and math.isfinite(float(r))]

    def _structure_points(self, scan: Scan, pose: Sequence[float], max_pts: int) -> np.ndarray:
        """앵커 재설정용 끝점: 알려진 UGV 주변(반경 + 여유 — UGV 위치는 UAV 틀이라 앵커가 틀어져 있으면 그만큼 어긋난다)을 뺀 구조물 후보만."""
        c = self.c
        circles: List[Tuple[float, float, float]] = []
        if getattr(c, "reanchor_ugv_exclude", True) and self.ugv_points:
            m = float(c.reanchor_ugv_margin_m)
            circles = [(x, y, max(r, float(c.reanchor_ugv_radius_m)) + m) for x, y, r in self.ugv_points]
        self.ugv_excluded = len(circles)
        return self.fit.scan_points(scan, max_pts=max_pts, exclude=(tuple(pose), circles) if circles else None)

    def correct_scan(self, scan: Optional[Scan]) -> Optional[Scan]:
        """차체 기준 스캔 각도 보정: LiDAR 가 +δ 돌아갔으면 같은 빔은 실제로 차체 기준 +δ 방향이다. 스캔을 새로 만든다(원본은 그대로)."""
        d = self.shift_applied
        if scan is None or abs(d) < 1e-9:
            return scan
        import dataclasses
        return dataclasses.replace(scan, angle_min=scan.angle_min + d)

    def take_trim(self) -> Optional[Tuple[float, float, float]]:
        t, self._trim_req = self._trim_req, None
        return t

    def take_reanchor(self) -> Optional[Dict[str, Any]]:
        """스택이 적용할 앵커 재설정(또는 원복) 요청 {trim:(dx,dy,dth), why, ...}. 한 번 가져가면 비운다."""
        r, self._ra_req = self._ra_req, None
        return r

    @property
    def wants_still(self) -> bool:
        """앵커 재설정 표본을 모으려고 잠깐(reanchor_hold_s) 멈춰 서 있고 싶다 — 큰 충격·도약 뒤에만. 임무 관리자가 정지 사유로 쓴다."""
        ra = self._ra
        return bool(self.c.reanchor and ra is not None and ra.get("hold") and self._now - ra["t0"] < float(self.c.reanchor_hold_s))

    def mark_impact(self, now: float, severity: str = "minor", bearing: Optional[float] = None, src: str = "accel") -> None:
        """충격이 있었다(now = 충격 시각) → 그 직전의 기준 오프셋을 굳히고 충격 후 검증을 시작한다.
        src='yaw_step' 은 충격 센서가 못 잡은 오프셋 계단이라, 계단 이전에 학습해 둔 기준(_d_anchor)을 그대로 쓴다."""
        if src == "yaw_step" and self._d_anchor is not None:
            ref = (self._d_anchor, math.radians(self.c.pre_std_deg), 0)
        else:
            ref = self._reference(now - 0.05)
        self._v = {"t": now, "ref": ref, "post": [], "severity": severity, "bearing": bearing,
                   "t_end": now + float(self.c.verify_timeout_s), "result": None}
        self._log(now, f"PoseGuard: 충격({severity}) — 검증 시작, 기준 오프셋 "
                       f"{'없음' if ref is None else f'{math.degrees(ref[0]):+.1f}° (±{math.degrees(ref[1]):.1f}°, n={ref[2]})'}")

    def verify_done(self) -> bool:
        return self._v is None

    # ------------------------------------------------------------------ 내부: 기준 오프셋
    def _reference(self, t_before: float) -> Optional[Tuple[float, float, int]]:
        c = self.c
        vals = [d for (t, d) in self._d_hist if t_before - c.pre_window_s <= t <= t_before]
        if len(vals) >= 5:
            std = _circ_std(vals)
            if std <= math.radians(c.pre_std_deg):
                return _circ_median(vals), std, len(vals)
        if self._d_anchor is not None:                               # 최근 표본이 모자라면 마지막으로 확인된 기준
            return self._d_anchor, math.radians(c.pre_std_deg), 0
        return None

    # ------------------------------------------------------------------ 메인
    def process(self, now: float, raw: Optional[Pose2D], odom: Pose2D, scan: Optional[Scan] = None) -> Pose2D:
        """raw: SLAM(TF) 에서 온 경기장 좌표 자세(LiDAR 틀어짐이 섞인 것) 또는 None(TF 없음). odom: 차체 오도메트리(IMU 융합일 수 있다).
        scan: 새 스캔일 때만(이미 correct_scan 을 거친 것). 반환: 보정·검증을 거친 자세."""
        c = self.c
        self._now = now
        dt = None if self._t_prev is None else max(now - self._t_prev, 1e-3)
        body = None
        if self._odom_prev is not None and dt is not None:
            dx, dy = odom.x - self._odom_prev.x, odom.y - self._odom_prev.y
            ct, st = math.cos(self._odom_prev.th), math.sin(self._odom_prev.th)
            body = (ct * dx + st * dy, -st * dx + ct * dy, wrap(odom.th - self._odom_prev.th))
            if math.hypot(body[0], body[1]) > 0.5 or abs(body[2]) > 1.0:
                body = None                                          # 오도메트리 리셋·도약은 이동량으로 쓰지 않는다
        if body is not None and dt is not None:
            k = min(1.0, dt / 0.3)
            self.rot += k * (abs(body[2]) / dt - self.rot)
            self.lin += k * (math.hypot(body[0], body[1]) / dt - self.lin)
        self._odom_prev, self._t_prev = odom.copy(), now
        pred = None
        if body is not None and self.pose is not None:
            p = self.pose
            cp, sp = math.cos(p.th), math.sin(p.th)
            pred = Pose2D(p.x + cp * body[0] - sp * body[1], p.y + sp * body[0] + cp * body[1], wrap(p.th + body[2]))
        if self.rot >= math.radians(c.quiet_rot_dps):
            self._moving_t = now
        if raw is None:
            return self._no_fix(now, pred)
        fixed = Pose2D(raw.x, raw.y, wrap(raw.th - self.shift_applied))
        # ---- SLAM 도약 점검
        if pred is not None and self.pose is not None:
            far = math.hypot(fixed.x - pred.x, fixed.y - pred.y) > c.jump_m or abs(wrap(fixed.th - pred.th)) > math.radians(c.jump_deg)
            if far:
                accepted = self._handle_jump(now, fixed, pred, scan)
                if not accepted:
                    return self._hold(now, pred)
            else:
                self._jump = None
        elif self.pose is None:
            self._jump = None
        self.pose = fixed
        self.last_fix_t = now
        self.fixes += 1
        # ---- 충격 전 기준/충격 후 검증에 쓰는 오프셋 표본 (SLAM 헤딩 − 차체 헤딩)
        d = wrap(raw.th - odom.th)
        if self.rot < math.radians(c.quiet_rot_dps):
            self._d_hist.append((now, d))
        self._verify_step(now, d, scan)
        if scan is not None and c.link_check and not self._link_disabled:
            self._link_step(now, scan)
        if c.reanchor:
            self._reanchor_step(now, scan)
        reasons = self._degraded_reasons(now)
        self._set_state(now, "degraded" if reasons else "ok", reasons)
        return self.pose

    # ------------------------------------------------------------------ TF 끊김 · 도약
    def _no_fix(self, now: float, pred: Optional[Pose2D]) -> Pose2D:
        c = self.c
        age = None if self.last_fix_t is None else now - self.last_fix_t
        if self.pose is None:
            return Pose2D()
        if pred is not None and age is not None and age <= c.dead_reckon_s:
            self.pose = pred
            self.dead_reckoned += 1
            self._set_state(now, "degraded", ["tf_miss_dead_reckoning"])
        elif age is not None and age > c.dead_reckon_s:
            self._set_state(now, "lost", ["tf_lost"])
        return self.pose

    def _hold(self, now: float, pred: Pose2D) -> Pose2D:
        c = self.c
        self.pose = pred
        self.jumps_held += 1
        t0 = self._jump["t0"] if self._jump else now
        if now - t0 > c.dead_reckon_s:
            self._set_state(now, "lost", ["slam_jump_unresolved"])
        else:
            self._set_state(now, "degraded", ["slam_jump_hold"])
        return pred

    def _handle_jump(self, now: float, fixed: Pose2D, pred: Pose2D, scan: Optional[Scan]) -> bool:
        """예측에서 크게 벗어난 SLAM 자세: 연속 표본이 한 곳에 모이고(재국소화) 경기장 맞음이 나빠지지 않을 때만 받아들인다."""
        c = self.c
        j = self._jump
        if j is None or math.hypot(fixed.x - j["p"].x, fixed.y - j["p"].y) > 0.10 or abs(wrap(fixed.th - j["p"].th)) > math.radians(6.0):
            self._jump = {"t0": now if j is None else j["t0"], "n": 1, "p": fixed.copy()}
            return False
        j["n"] += 1
        if j["n"] < c.jump_confirm_n:
            return False
        forced = now - j["t0"] > 2.0 * c.dead_reckon_s
        if self.fit is None and c.link_check and not self._link_disabled:
            self._ensure_fit()
        if self.fit is not None and scan is not None and not forced:
            pts = self.fit.scan_points(scan)
            if len(pts) >= 30:
                sf, rf = self.fit.score(pts, (fixed.x, fixed.y, fixed.th), self.uav_rects)
                sp, rp = self.fit.score(pts, (pred.x, pred.y, pred.th), self.uav_rects)
                if rp > rf + 0.10:
                    if not j.get("rejected"):
                        self._log(now, f"PoseGuard: SLAM 도약 거부(경기장 맞음 예측 {rp:.2f} > SLAM {rf:.2f}) — 추측항법 유지")
                    j["rejected"] = True                              # 예측 자리가 훨씬 잘 맞는다 — 시간 상한(forced)까지 거부한다
                    return False
                j["rejected"] = False
        if forced:
            self._log(now, "PoseGuard: SLAM 도약이 오래 이어져 SLAM 자세를 최선으로 받아들인다(추측항법은 그동안 누적 오차)")
        self.jumps_accepted += 1
        self._jump = None
        self._d_hist.clear()                                           # 재국소화로 헤딩 오프셋이 바뀌는 것은 LiDAR 틀어짐이 아니다
        self._d_anchor = None
        self._log(now, f"PoseGuard: SLAM 도약 수용 ({pred.x:.2f},{pred.y:.2f})→({fixed.x:.2f},{fixed.y:.2f})")
        if c.reanchor and c.reanchor_on_jump:
            self._ra_arm(now, "jump", hold=True)
        return True

    # ------------------------------------------------------------------ 충격 후 틀어짐 검증
    def _verify_step(self, now: float, d: float, scan: Optional[Scan]) -> None:
        v, c = self._v, self.c
        if v is None:
            self._learn_anchor(d)
            return
        if now >= v["t_end"]:
            self._finish_verify(now, "timeout")
            return
        if scan is None or now - v["t"] < c.settle_s:
            return
        if self.rot >= math.radians(c.quiet_rot_dps) or self.lin > c.quiet_lin:
            return                                                     # 정지 상태의 표본만 쓴다
        v["post"].append(d)
        if len(v["post"]) < c.shift_confirm_scans:
            return
        std = _circ_std(v["post"])
        if std > math.radians(c.post_std_deg):
            if len(v["post"]) >= 3 * c.shift_confirm_scans:
                self._finish_verify(now, "noisy")
            else:
                v["post"].pop(0)
            return
        self._finish_verify(now, "measured")

    def _finish_verify(self, now: float, how: str) -> None:
        v, c = self._v, self.c
        self._v = None
        res: Dict[str, Any] = {"t": v["t"], "severity": v["severity"], "how": how, "delta_deg": None, "action": "none"}
        if how == "measured" and v["ref"] is not None:
            self._shift_unverified = False
            post = _circ_median(v["post"])
            delta = wrap(post - v["ref"][0])
            res["delta_deg"] = round(math.degrees(delta), 2)
            res["post_deg"] = round(math.degrees(post), 2)
            if abs(delta) >= math.radians(c.shift_detect_deg):
                if abs(delta) <= math.radians(c.shift_max_deg):
                    self.shift = wrap(self.shift + delta)
                    res["action"] = "shift_confirmed"
                    self.shift_events.append({"t": v["t"], "delta_deg": res["delta_deg"], "total_deg": round(math.degrees(self.shift), 2)})
                    self._log(now, f"PoseGuard: LiDAR 장착 틀어짐 확정 δ={math.degrees(delta):+.1f}° (누적 {math.degrees(self.shift):+.1f}°)"
                                   f" → 자세 헤딩·스캔 각도 보정{' · 내장 A* 로 전환' if self.needs_builtin_nav else ''}")
                else:
                    res["action"] = "too_large"
                    self._shift_unverified = True
                    self._log(now, f"PoseGuard: 충격 후 헤딩 오프셋 {math.degrees(delta):+.1f}° — 보정 한도({c.shift_max_deg:.0f}°) 초과, 보정하지 않고 내장 A* 로 간다")
            self._d_anchor = post
            self._d_hist.clear()
            self._d_hist.extend((now, x) for x in v["post"][-5:])
        elif how == "measured":
            self._d_anchor = _circ_median(v["post"])
            res["action"] = "no_reference"
            self._log(now, "PoseGuard: 충격 후 오프셋을 쟀지만 충격 전 기준이 없어 틀어짐을 판정하지 못했다 — 이 값을 새 기준으로")
        else:
            res["action"] = "unverified"
            self._shift_unverified = v["severity"] == "major"      # 큰 충격인데 확인하지 못했다 → Nav2 헤딩을 믿지 않는다. 작은 충격은 의심만 하고 계속
            self._d_anchor = None
            self._log(now, f"PoseGuard: 충격 후 검증 {how} — 틀어짐 여부를 확인하지 못했다(자세를 의심하며 계속)")
        self.verify_results.append(res)
        del self.verify_results[:-8]
        if c.reanchor and c.reanchor_on_impact:
            hold = v["severity"] == "major" or res["action"] in ("shift_confirmed", "too_large", "unverified", "no_reference")
            self._ra_arm(now, "impact", hold=hold)

    def _learn_anchor(self, d: float) -> None:
        """충격이 없을 때도 기준 오프셋을 천천히 따라간다(IMU 헤딩 드리프트·바퀴 미끄럼을 흡수). 급변은 따라가지 않는다."""
        if self.rot >= math.radians(self.c.quiet_rot_dps):
            return
        if self._d_anchor is None:
            if len(self._d_hist) >= 10:
                recent = [x for _, x in list(self._d_hist)[-10:]]
                if _circ_std(recent) <= math.radians(self.c.pre_std_deg):
                    self._d_anchor = _circ_median(recent)
            return
        err = wrap(d - self._d_anchor)
        if abs(err) <= math.radians(self.c.shift_detect_deg):
            self._d_anchor = wrap(self._d_anchor + 0.02 * err)

    def step_detected(self, now: float) -> Optional[ImpactEvent]:
        """충격 센서가 못 잡은 '오프셋 계단'(정지 상태에서 SLAM−차체 헤딩이 갑자기 한도 이상 변함)을 가상 충격으로 본다. 호출자가 mark_impact 한다."""
        c = self.c
        if self._v is not None or self._d_anchor is None or len(self._d_hist) < 4:
            return None
        recent = [x for (t, x) in list(self._d_hist)[-4:]]
        if _circ_std(recent) > math.radians(c.pre_std_deg):
            return None
        err = wrap(_circ_median(recent) - self._d_anchor)
        if abs(err) >= math.radians(c.step_trigger_deg) and now - self._moving_t >= 0.3:
            return ImpactEvent(now, 0.0, None, 0.0, "yaw_step")
        return None

    # ------------------------------------------------------------------ 경기장(UAV) 좌표 연결 점검
    def _ensure_fit(self) -> None:
        if self.fit is None:
            try:
                self.fit = ArenaFit(self.geo.W, self.geo.H, self._fit_rects)
            except Exception as e:                                       # noqa: BLE001 — 점검 보조가 제어 루프를 죽이면 안 된다
                self.link = {"state": "unavailable", "why": f"{type(e).__name__}: {e}"}
                self._link_disabled = True

    def _link_step(self, now: float, scan: Scan) -> None:
        c = self.c
        if now - self._link_t < c.link_period_s or self.pose is None:
            return
        if self.rot > math.radians(c.link_rot_dps) or self.lin > c.link_lin or self._v is not None:
            return                                                       # 움직임 왜곡이 큰 동안은 점검하지 않는다
        self._link_t = now
        self._ensure_fit()
        if self.fit is None or self._link_disabled:
            return
        p = self.pose
        pose = (p.x, p.y, p.th)
        pts = self._structure_points(scan, pose, 140) if c.reanchor else self.fit.scan_points(scan)
        if len(pts) < 40:
            return
        s0, r0 = self.fit.score(pts, pose, self.uav_rects)
        arena_pts = self.fit.to_arena(pts, pose)
        hd = wall_heading_residual(arena_pts)
        st_static = self.fit.map.distances(arena_pts)
        unexplained = st_static > 0.08
        uav_support = None
        if self.uav_rects and unexplained.sum() >= 10:
            uav_support = float(np.mean(self.fit.rect_distance(arena_pts[unexplained], self.uav_rects) < 0.06))
        self.link_checks += 1
        rep: Dict[str, Any] = {"t": round(now, 2), "ratio": round(r0, 3), "score": round(s0, 3),
                               "wall_psi_deg": None if hd is None else round(math.degrees(hd[0]), 2),
                               "wall_R": None if hd is None else round(hd[1], 2),
                               "uav_support": None if uav_support is None else round(uav_support, 3),
                               "uav_boxes": len(self.uav_rects), "n_pts": int(len(pts))}
        need_search = self._link_base is None or r0 < self._link_base["ratio"] - c.link_drop
        found = self.fit.search(pts, pose, self.uav_rects, xy=c.link_search_xy, th=math.radians(c.link_search_deg)) if need_search else {"ok": False}
        if found.get("ok"):
            rep["best_off"] = [round(found["off"][0], 3), round(found["off"][1], 3), round(math.degrees(found["off"][2]), 2)]
            rep["best_gain"] = round(found["sb"] - found["s0"], 3)
            rep["obs"] = list(found["obs"])
        if self._link_base is None:
            if r0 >= c.link_baseline_min and found.get("ok") and not found["edge"]:
                self._link_base = {"ratio": r0, "off": found["off"], "psi": None if hd is None else hd[0], "t": now}
                rep["state"] = "baseline"
                self._log(now, f"PoseGuard: 경기장 연결 기준선 — 벽·구조물 맞음 {r0:.2f}, 도면과의 잔차 "
                               f"({found['off'][0]:+.3f},{found['off'][1]:+.3f}) m {math.degrees(found['off'][2]):+.1f}° "
                               f"(이 값은 도면 치수·시작 자세 오차로 보고 기준선에 흡수한다)")
            else:
                rep["state"] = "poor_start" if r0 < c.link_baseline_min else "searching"
        else:
            base = self._link_base
            rep["base_ratio"] = round(base["ratio"], 3)
            if r0 >= max(c.link_lost_ratio, base["ratio"] - c.link_drop):
                rep["state"] = "ok"
                self._link_bad.clear()
                if c.reanchor and now - self._resid_t >= 5.0:
                    self._note_residual(now, pts, pose, base)
            else:
                rep["state"] = "bad" if r0 < c.link_lost_ratio else "weak"
                if not c.reanchor:                                         # 앵커 재설정이 켜져 있으면 그쪽이 같은 일을 넓은 창·검증으로 한다
                    self._maybe_trim(now, r0, found, rep)
        self.link = rep
        if c.reanchor and c.reanchor_on_link:
            if rep.get("state") in ("weak", "bad"):
                self._ra_weak_n += 1
                if self._ra_weak_n >= c.reanchor_link_n and self._ra is None:
                    self._ra_arm(now, "link", hold=False)
            elif rep.get("state") == "ok":
                self._ra_weak_n = 0

    def _maybe_trim(self, now: float, r0: float, found: Dict[str, Any], rep: Dict[str, Any]) -> None:
        """연결이 기준선보다 크게 나빠졌을 때만(SLAM 도약·재국소화 실패·앵커 어긋남) 소폭 '복구 보정'을 제안한다.
        관측 가능한 축만, 연속 3회 같은 결과, 점수 회복 조건, 누적 한도를 모두 만족해야 한다."""
        c = self.c
        base = self._link_base
        if not getattr(c, "link_trim", False) or not found.get("ok") or found["edge"]:
            return
        if found["sb"] - found["s0"] < c.link_trim_gain:
            self._link_bad.clear()
            return
        dx, dy, dth = found["off"]
        ox, oy, oth = base["off"]
        rel = [dx - ox if found["obs"][0] else 0.0, dy - oy if found["obs"][1] else 0.0, wrap(dth - oth) if found["obs"][2] else 0.0]
        if not any(found["obs"]):
            self._link_bad.clear()
            return
        self._link_bad.append((rel[0], rel[1], rel[2]))
        del self._link_bad[:-3]
        if len(self._link_bad) < 3:
            return
        a = np.array(self._link_bad)
        if np.ptp(a[:, 0]) > 0.04 or np.ptp(a[:, 1]) > 0.04 or np.ptp(a[:, 2]) > math.radians(1.5):
            return
        m = a.mean(axis=0)
        tot = [self._trim_total[0] + m[0], self._trim_total[1] + m[1], self._trim_total[2] + m[2]]
        if math.hypot(tot[0], tot[1]) > c.link_trim_max_m or abs(tot[2]) > math.radians(c.link_trim_max_deg):
            rep["trim"] = "limit"
            self._log(now, "PoseGuard: 연결 복구 보정이 누적 한도를 넘는다 — 더 보정하지 않는다(연결 불량 상태로 계속)")
            self._link_bad.clear()
            return
        self._trim_req = (float(m[0]), float(m[1]), float(m[2]))
        self._trim_total = tot
        self._link_bad.clear()
        rep["trim"] = [round(float(m[0]), 3), round(float(m[1]), 3), round(math.degrees(float(m[2])), 2)]
        self._log(now, f"PoseGuard: 연결 복구 보정 제안 ({m[0]:+.3f},{m[1]:+.3f}) m {math.degrees(m[2]):+.1f}° (맞음 {r0:.2f} → 회복 예상 {found['sb']:.2f})")

    def note_trim_applied(self, trim: Tuple[float, float, float]) -> None:
        """스택이 앵커에 보정을 적용했다 — 보정 후의 자세와 기준 오프셋을 맞춘다."""
        if self.pose is not None:
            self.pose = Pose2D(self.pose.x + trim[0], self.pose.y + trim[1], wrap(self.pose.th + trim[2]))
        self.trims_applied += 1
        self._d_hist.clear()
        self._d_anchor = None
        self._link_t = -1e9

    # ------------------------------------------------------------------ 앵커 재설정(reanchor)
    def _still(self, now: float) -> bool:
        c = self.c
        return self.rot < math.radians(c.quiet_rot_dps) and self.lin <= c.quiet_lin and now - self._moving_t >= 0.3

    def _ra_reject(self, why: str) -> None:
        self.reanchor_rejects[why] = self.reanchor_rejects.get(why, 0) + 1

    def _ra_arm(self, now: float, why: str, hold: bool = False) -> None:
        """충격·도약·연결 불량 뒤 앵커를 다시 맞춰 볼 시도를 시작한다(정지 상태 표본을 모아 판정). 조건이 안 되면 이유만 세고 끝."""
        c = self.c
        if not c.reanchor or self._ra_locked:
            return
        if self._ra_count >= c.reanchor_max_count:
            self._ra_reject("max_count")
            return
        if self._link_base is None:
            self._ra_reject("no_baseline")                         # 도면과의 기준 관계를 모르면 '무엇으로 되돌릴지' 알 수 없다
            return
        if now - self._ra_last_t < c.reanchor_cooldown_s:
            self._ra_reject("cooldown")
            return
        sp = self.plan_spread()
        if c.reanchor_plan_spread_m > 0 and sp is None and getattr(c, "reanchor_require_plan_check", False):
            self._ra_reject("plan_unchecked")                          # 아직 여러 자리에서 도면 잔차를 못 쟀다 — 도면을 믿을 근거가 없으면 앵커를 건드리지 않는다
            return
        if c.reanchor_plan_spread_m > 0 and sp is not None and sp["xy"] > c.reanchor_plan_spread_m:
            self._ra_reject("plan_inconsistent")                       # 자리마다 도면 잔차가 다르다 — 도면(구조물 좌표)이 틀렸을 수 있어 거기에 맞추면 앵커를 망친다
            if not self._plan_warned:
                self._plan_warned = True
                self._log(now, f"WARN PoseGuard: 자리마다 도면 잔차가 {sp['xy'] * 100:.0f} cm 씩 달라진다(표본 {sp['n']}) — 경기장 도면·구조물 좌표를 확인할 것. 앵커 재설정은 하지 않는다")
            return
        if self._ra_ver is not None:
            self._ra_reject("verifying")
            return
        if self._ra is not None:                                    # 진행 중에 또 사건 — 표본을 비우고 다시 시작
            self._ra.update(t0=now, samples=[], hold=bool(self._ra["hold"] or hold), why=why)
            return
        self._ra = {"why": why, "t0": now, "hold": hold, "samples": [], "t_last": -1e9, "logged": set()}
        self._log(now, f"PoseGuard: 앵커 재설정 시도 시작({why}) — 정지 상태에서 벽·구조물에 다시 맞춰 본다{' (잠깐 정지)' if hold else ''}")

    def _note_residual(self, now: float, pts: np.ndarray, pose: Tuple[float, float, float], base: Dict[str, Any]) -> None:
        """연결이 좋은 동안(앵커가 맞다고 볼 때) 5 s 마다 이 자리의 도면 잔차를 잰다. 도면이 자리마다 다르게 틀려 있으면(구조물 좌표 오차) 잔차가 자리에 따라 달라진다 —
        앵커 오차는 자리와 무관하게 같은 양으로 나타나므로 구분된다. 퍼짐이 크면 도면을 믿을 수 없어 앵커 재설정을 막는다."""
        self._resid_t = now
        f = self.fit.search(pts, pose, self._ra_rects(), xy=0.15, th=math.radians(6.0)) if self.fit is not None else {"ok": False}
        if not f.get("ok") or f["edge"] or not all(f["obs"]):
            return
        rel = (f["off"][0] - base["off"][0], f["off"][1] - base["off"][1], wrap(f["off"][2] - base["off"][2]))
        self._resid.append((pose[0], pose[1], rel[0], rel[1], rel[2]))
        del self._resid[:-24]

    def plan_spread(self) -> Optional[Dict[str, float]]:
        """자리별 도면 잔차의 퍼짐(최대−최소). 서로 1.5 m 이상 떨어진 자리에서 잰 표본이 4 개 미만이면 None(아직 모른다)."""
        r = self._resid
        if len(r) < 4:
            return None
        a = np.array(r)
        if np.ptp(a[:, 0]) < 1.5 and np.ptp(a[:, 1]) < 1.5:
            return None
        return {"xy": float(max(np.ptp(a[:, 2]), np.ptp(a[:, 3]))), "deg": float(math.degrees(np.ptp(a[:, 4]))), "n": len(r)}

    def _ra_end(self, now: float, how: str, msg: str) -> None:
        self._ra = None
        self._ra_reject("end_" + how)
        self._log(now, f"PoseGuard: 앵커 재설정 종료({how}) — {msg}")

    def _ra_rects(self) -> Sequence[Rect]:
        return self.uav_rects_raw if getattr(self.c, "reanchor_use_uav_boxes", False) else ()

    def _reanchor_step(self, now: float, scan: Optional[Scan]) -> None:
        c = self.c
        self._ra_verify_step(now, scan)
        ra = self._ra
        if ra is None:
            return
        if now - ra["t0"] > c.reanchor_window_s:
            self._ra_end(now, "window", f"{c.reanchor_window_s:.0f} s 안에 믿을 만한 일치 적합을 얻지 못했다 — 앵커를 그대로 둔다")
            return
        if scan is None or self.pose is None or self._v is not None:
            return
        if not self._still(now):
            ra["samples"].clear()                                   # 움직이면 연속 일치가 끊긴다
            return
        if now - ra["t_last"] < 0.4:
            return
        ra["t_last"] = now
        base = self._link_base
        if base is None:
            self._ra_end(now, "no_baseline", "기준선이 없다")
            return
        self._ensure_fit()
        if self.fit is None or self._link_disabled:
            self._ra_end(now, "no_fit", "경기장 적합을 쓸 수 없다")
            return
        p = self.pose
        pose = (p.x, p.y, p.th)
        pts = self._structure_points(scan, pose, 120)          # 넓은 탐색은 후보 1만여 개 × 끝점 수라 끝점을 줄인다(Jetson 에서도 제어 주기를 오래 막지 않게)
        f = self.fit.search_wide(pts, pose, self._ra_rects(), center=base["off"], xy=c.reanchor_search_xy,
                                 th=math.radians(c.reanchor_search_deg))

        def reject(why: str, detail: str = "") -> None:
            self._ra_reject(why)
            ra["samples"].clear()
            if why not in ra["logged"]:
                ra["logged"].add(why)
                self._log(now, f"PoseGuard: 앵커 재설정 후보 거부({why}) {detail}".rstrip())

        if not f.get("ok"):
            return reject(str(f.get("why", "no_fit")))
        if f["edge"]:
            return reject("edge", "— 최적이 탐색 가장자리 — 더 먼 곳에 있을 수 있다")
        if not all(f["obs"]):
            return reject("unobservable", f"obs={f['obs']} — 한 방향은 점수가 평평해 위치를 정할 수 없다")
        if f["margin"] < c.reanchor_ambiguity:
            return reject("ambiguous", f"— 다른 후보와의 점수 차 {f['margin']:.3f} < {c.reanchor_ambiguity:.3f}")
        if f["ratio"] < c.reanchor_min_ratio:
            return reject("low_ratio", f"— 새 자리 맞음 {f['ratio']:.2f} < {c.reanchor_min_ratio:.2f}")
        bx, by, bt = base["off"]
        rel = (f["off"][0] - bx, f["off"][1] - by, wrap(f["off"][2] - bt))
        mv, md = math.hypot(rel[0], rel[1]), abs(rel[2])
        if mv < c.reanchor_min_move_m and md < math.radians(c.reanchor_min_move_deg):
            self._ra = None
            self._log(now, f"PoseGuard: 앵커 재설정 불필요 — 기준선 관계가 그대로다(차이 {mv * 100:.1f} cm, {math.degrees(md):.1f}°)")
            return
        if f["sb"] - f["s_center"] < c.reanchor_min_gain:
            return reject("low_gain", f"— 점수 향상 {f['sb'] - f['s_center']:.3f} < {c.reanchor_min_gain:.3f}")
        if mv > c.reanchor_max_m or md > math.radians(c.reanchor_max_deg):
            return reject("too_large", f"— 보정량 {mv:.2f} m {math.degrees(rel[2]):+.1f}° 가 한 번 한도를 넘는다(지도가 망가졌을 수 있다)")
        tot = (self._ra_total[0] + rel[0], self._ra_total[1] + rel[1], self._ra_total[2] + rel[2])
        if math.hypot(tot[0], tot[1]) > c.reanchor_total_m or abs(tot[2]) > math.radians(c.reanchor_total_deg):
            return reject("total_limit", f"— 누적 보정 {math.hypot(tot[0], tot[1]):.2f} m {math.degrees(tot[2]):+.1f}° 가 한도를 넘는다")
        smp = ra["samples"]
        smp.append((rel[0], rel[1], rel[2], f["ratio"], f["sb"], f["s_center"]))
        del smp[:-int(c.reanchor_confirm_n)]
        if len(smp) < c.reanchor_confirm_n:
            return
        a = np.array(smp)
        if np.ptp(a[:, 0]) > c.reanchor_ptp_m or np.ptp(a[:, 1]) > c.reanchor_ptp_m or np.ptp(a[:, 2]) > math.radians(c.reanchor_ptp_deg):
            smp.pop(0)                                              # 서로 일치하지 않는다 — 가장 오래된 표본을 버리고 더 모은다
            self._ra_reject("inconsistent")
            return
        m = a.mean(axis=0)
        self._ra_req = {"trim": (float(m[0]), float(m[1]), float(m[2])), "why": ra["why"], "t": now, "revert": False,
                        "ratio_after": float(m[3]), "s_after": float(m[4]), "s_before": float(m[5]),
                        "pose_before": [round(p.x, 3), round(p.y, 3), round(p.th, 3)], "samples": len(smp)}
        self._ra = None
        self._log(now, f"PoseGuard: 앵커 재설정 결정({ra['why']}) 이동 ({m[0]:+.3f},{m[1]:+.3f}) m {math.degrees(m[2]):+.1f}° — "
                       f"점수 {m[5]:.2f}→{m[4]:.2f}, 맞음 {m[3]:.2f} (연속 {len(smp)}회 일치)")

    def note_reanchor_applied(self, req: Dict[str, Any]) -> None:
        """스택이 앵커를 바꿨다(또는 원복했다) — 자세·기준 오프셋·연결 상태를 맞추고 적용 뒤 확인을 시작한다."""
        now, trim = self._now, req["trim"]
        if self.pose is not None:
            self.pose = Pose2D(self.pose.x + trim[0], self.pose.y + trim[1], wrap(self.pose.th + trim[2]))
        self._d_hist.clear()
        self._d_anchor = None
        self._jump = None
        self._link_t = -1e9
        self._link_bad.clear()
        self._ra_weak_n = 0
        self.link = {"state": "unchecked", "why": "reanchored"}
        ev = {"t": round(now, 2), "why": req.get("why"), "trim": [round(trim[0], 3), round(trim[1], 3), round(math.degrees(trim[2]), 2)],
              "revert": bool(req.get("revert"))}
        if req.get("revert"):
            self._ra_total = [self._ra_total[i] + trim[i] for i in range(3)]      # trim 이 이미 부호 반전된 값이다
            self._ra_reverts += 1
            self._ra_ver = None
            if self._ra_reverts >= 2:
                self._ra_locked = True
                self._log(now, "PoseGuard: 앵커 재설정이 두 번 원복됐다 — 적합을 믿을 수 없다고 보고 더 시도하지 않는다")
        else:
            self._ra_total = [self._ra_total[i] + trim[i] for i in range(3)]
            self._ra_count += 1
            self._ra_last_t = now
            self._ra_ver = {"t": now, "t_last": -1e9, "vals": [], "expect": float(req.get("ratio_after", 0.0)), "trim": tuple(trim)}
            ev["ratio_after"] = round(float(req.get("ratio_after", 0.0)), 3)
        self.reanchors.append(ev)
        del self.reanchors[:-8]
        self.trims_applied += 1
        self._log(now, f"PoseGuard: 앵커 {'원복' if req.get('revert') else '재설정'} 적용 — 누적 ({self._ra_total[0]:+.3f},{self._ra_total[1]:+.3f}) m "
                       f"{math.degrees(self._ra_total[2]):+.1f}°")

    def _ra_verify_step(self, now: float, scan: Optional[Scan]) -> None:
        """적용 뒤 정지 상태의 표본 몇 개로 맞음 비율을 다시 재어, 기대보다 나빠졌으면 앞 앵커로 되돌리도록 요청한다."""
        v, c = self._ra_ver, self.c
        if v is None:
            return
        if now - v["t"] > c.reanchor_window_s:
            self._ra_ver = None
            self._log(now, "PoseGuard: 앵커 재설정 뒤 확인 표본이 모자라 확인 없이 둔다(움직이는 중)")
            return
        if scan is None or self.pose is None or self._v is not None or not self._still(now) or now - v["t_last"] < 0.4:
            return
        self._ensure_fit()
        if self.fit is None:
            return
        p = self.pose
        pts = self._structure_points(scan, (p.x, p.y, p.th), 140)
        if len(pts) < 40:
            return
        v["t_last"] = now
        _, r = self.fit.score(pts, (p.x, p.y, p.th), self._ra_rects())
        v["vals"].append(r)
        if len(v["vals"]) < c.reanchor_verify_n:
            return
        rm = float(np.mean(v["vals"]))
        self._ra_ver = None
        if rm < v["expect"] - c.reanchor_verify_drop:
            tr = v["trim"]
            self._ra_req = {"trim": (-tr[0], -tr[1], -tr[2]), "why": "revert", "revert": True, "t": now}
            self._log(now, f"PoseGuard: 앵커 재설정 뒤 맞음 {rm:.2f} < 기대 {v['expect']:.2f}-{c.reanchor_verify_drop:.2f} — 앞 앵커로 되돌린다")
        else:
            self._log(now, f"PoseGuard: 앵커 재설정 확인 — 맞음 {rm:.2f} (기대 {v['expect']:.2f})")

    # ------------------------------------------------------------------ 상태
    def _degraded_reasons(self, now: float) -> List[str]:
        out: List[str] = []
        if self._shift_unverified:
            out.append("lidar_shift_unverified")
        if abs(self.shift) >= math.radians(self.c.shift_detect_deg):
            out.append("lidar_shift_compensated")
        if self.link.get("state") in ("weak", "bad", "poor_start") and self.c.link_degrades:
            out.append("link_" + str(self.link["state"]))
        if self._v is not None:
            out.append("impact_verifying")
        return out

    def _set_state(self, now: float, state: str, reasons: List[str]) -> None:
        if state == "ok" and self.state == "lost":
            self._log(now, "PoseGuard: lost → ok (새 SLAM 자세 수신)")
        self.state, self.reasons = state, reasons

    def report(self) -> Dict[str, Any]:
        p = self.pose
        return {"state": self.state, "reasons": list(self.reasons),
                "pose": None if p is None else [round(p.x, 3), round(p.y, 3), round(p.th, 3)],
                "shift_deg": round(math.degrees(self.shift), 2), "shift_events": list(self.shift_events[-4:]),
                "needs_builtin_nav": self.needs_builtin_nav, "verifying": self.verifying,
                "last_verify": self.verify_results[-1] if self.verify_results else None,
                "fixes": self.fixes, "dead_reckoned": self.dead_reckoned, "jumps_held": self.jumps_held,
                "jumps_accepted": self.jumps_accepted, "link": dict(self.link), "link_checks": self.link_checks,
                "trims_applied": self.trims_applied,
                "reanchor": None if not self.c.reanchor else {
                    "count": self._ra_count, "reverts": self._ra_reverts, "locked": self._ra_locked,
                    "armed": None if self._ra is None else self._ra["why"], "verifying": self._ra_ver is not None,
                    "total": [round(self._ra_total[0], 3), round(self._ra_total[1], 3), round(math.degrees(self._ra_total[2]), 2)],
                    "events": list(self.reanchors[-3:]), "rejects": dict(self.reanchor_rejects), "plan_spread": self.plan_spread(),
                    "ugv_excluded": self.ugv_excluded},
                "trim_total": [round(self._trim_total[0], 3), round(self._trim_total[1], 3), round(math.degrees(self._trim_total[2]), 2)]}
