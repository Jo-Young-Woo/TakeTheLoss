"""차체 외곽(footprint)과 여유거리 예산 — 충돌 판정·경로계획·안전 정지가 **같은 기준**을 쓰도록 한 곳에 모은다.

값의 출처 구분
  [PDF p21]  UGV 외형 250(L) × 230(W) × 320(H) mm  → config.robot.length / width / height   (사양서 값)
  [PDF p24]  장애물(박스) 사이 최소 이격 40 cm     → config.arena.min_gap (경기장 생성 규칙)
  [가정]     LiDAR 오차(config.lidar_err.*) — PDF 에 LiDAR 정확도 수치가 없다. 실측 전까지 '가정값'이고 실측값으로 표시하지 않는다.

외곽 모델
  · 기본은 방향이 있는 직사각형(250 × 230 mm). 중심에서 모서리까지 외접 반경 = √(0.125²+0.115²) = 0.1698 m, 내접 반경(반폭) = 0.115 m.
  · 원이 필요한 곳(A* 의 거리장 인플레이션, 제자리 회전 소거 공간)은 **용도별로 다른 반경**을 쓴다.
      - 직진 통과(A* 하드 인플레이션) : 반폭 + 계획 여유   → 좁은 통로를 지나갈 수 있어야 하므로 내접 기준
      - 제자리 회전                  : 외접 반경 + 계획 여유 → 회전 중 모서리가 쓸고 지나가는 원
      - 경로 확정 전 검증              : 실제 직사각형을 경로의 진행 방향으로 놓고 점유 칸과 겹치는지(`path_violations`)
    작은 원(예전 0.14 m)으로 모서리 충돌을 숨기지 않는다: 직진에는 반폭 인플레이션을 쓰되, 경로의 모든 점에서 직사각형 검증을 통과해야 확정된다.

여유거리 예산 (같은 오차를 여러 계층에 두 번 더하지 않는다)
  항목                      계획(A*/Nav2 footprint)   안전 정지(LiDAR 제한)   근거
  LiDAR 거리 편향                    ✔                      ✔               센서 측정값 자체의 오차 → 두 계층이 같은 측정을 쓰므로 *각자 한 번*
  LiDAR 거리 잡음 k·σ                ✔                      ✔
  각도 오차 × 기준 거리              ✔                      ✔
  장착 위치 오차(xy, yaw)            ✔                      ✔
  지연 × 속도                        —                      ✔               정지 판단은 '지금' 움직이는 중에 실시간으로 하므로. 계획은 속도를 가정하지 않는다
  자기 위치추정 오차 k·σ_pose        ✔                      —               안전 정지는 로봇 좌표계의 LiDAR 상대 거리를 직접 쓰므로 위치추정 오차가 들어가지 않는다
  → plan_margin = 센서 항 + 위치추정 항,  stop_margin = 센서 항 + 지연 항.
  Nav2 는 footprint 를 plan_margin 만큼 키운 다각형을 쓰고 inflation_radius 는 *비용 기울기*용이라 하드 여유에 더하지 않는다.
  누락(dropout)·이상치(outlier)는 거리 여유로 메우지 않고 '지속성 필터'(여러 스캔)로 다룬다 — 시험으로 확인한다.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

Pt = Tuple[float, float]


# ====================================================================== 외곽
@dataclass(frozen=True)
class Footprint:
    length: float = 0.25          # [PDF p21] 250 mm (진행 방향)
    width: float = 0.23           # [PDF p21] 230 mm
    height: float = 0.32          # [PDF p21] 320 mm

    @property
    def half_l(self) -> float:
        return 0.5 * self.length

    @property
    def half_w(self) -> float:
        return 0.5 * self.width

    @property
    def circumradius(self) -> float:
        """중심에서 가장 먼 모서리까지 (≈0.1698 m)."""
        return math.hypot(self.half_l, self.half_w)

    @property
    def inscribed(self) -> float:
        """중심에서 가장 가까운 변까지 = 반폭 (0.115 m)."""
        return min(self.half_l, self.half_w)

    def corners(self, x: float, y: float, th: float, pad: float = 0.0) -> np.ndarray:
        c, s = math.cos(th), math.sin(th)
        hl, hw = self.half_l + pad, self.half_w + pad
        pts = np.array([[hl, hw], [hl, -hw], [-hl, -hw], [-hl, hw]], dtype=float)
        R = np.array([[c, -s], [s, c]])
        return pts @ R.T + np.array([x, y])


# ====================================================================== 여유거리 예산
@dataclass
class ClearanceBudget:
    """`Config` 에서 계산한 여유거리와 그 구성(항목별 값·출처). 한 번 계산해 모든 계층이 읽는다."""
    fp: Footprint
    sensor_bias: float
    sensor_random: float
    latency_term: float
    pose_term: float
    items: List[Tuple[str, float, str, str]] = field(default_factory=list)    # (이름, 값 m, 사용처, 출처)

    @property
    def sensor_margin(self) -> float:
        return self.sensor_bias + self.sensor_random

    @property
    def plan_margin(self) -> float:
        return self.sensor_margin + self.pose_term

    @property
    def stop_margin(self) -> float:
        return self.sensor_margin + self.latency_term

    # --- 계층별로 쓰는 값
    @property
    def plan_inflate_straight(self) -> float:
        """A* 하드 인플레이션 반경(직진 통과 기준): 반폭 + 계획 여유."""
        return self.fp.inscribed + self.plan_margin

    @property
    def plan_inflate_rotate(self) -> float:
        """제자리 회전에 필요한 자유 반경: 외접 반경 + 계획 여유."""
        return self.fp.circumradius + self.plan_margin

    @property
    def stop_front(self) -> float:
        """LiDAR 정지거리(센서→장애물, 전방): 앞 반길이 + 정지 여유."""
        return self.fp.half_l + self.stop_margin

    @property
    def stop_side(self) -> float:
        """측면 접근 보정 시작 거리: 반폭 + 정지 여유."""
        return self.fp.half_w + self.stop_margin

    @property
    def nav2_footprint(self) -> List[Tuple[float, float]]:
        """Nav2 footprint 다각형(계획 여유만큼 키운 직사각형), 앞-왼쪽부터 시계 반대."""
        hl, hw = self.fp.half_l + self.plan_margin, self.fp.half_w + self.plan_margin
        return [(hl, hw), (-hl, hw), (-hl, -hw), (hl, -hw)]

    def corridor_clearance(self, gap: float = 0.40) -> Dict[str, float]:
        """간격 gap 인 직선 통로에서 한쪽에 남는 여유와 회전 가능 여부."""
        side = 0.5 * (gap - self.fp.width)
        return {"side_free_m": side, "side_after_plan_margin_m": side - self.plan_margin,
                "can_pass_straight": side - self.plan_margin > 0.0,
                "can_rotate_in_place": 0.5 * gap > self.plan_inflate_rotate}

    def table(self) -> str:
        rows = [f"{n:28s} {v * 1000:7.1f} mm  {use:12s} {src}" for n, v, use, src in self.items]
        rows += [f"{'plan_margin':28s} {self.plan_margin * 1000:7.1f} mm", f"{'stop_margin':28s} {self.stop_margin * 1000:7.1f} mm",
                 f"{'A* 인플레이션(직진, 반폭+plan)':28s} {self.plan_inflate_straight * 1000:7.1f} mm",
                 f"{'회전 소거 반경(외접+plan)':28s} {self.plan_inflate_rotate * 1000:7.1f} mm"]
        return "\n".join(rows)


def budget_from(robot, lerr, v_ref: float) -> ClearanceBudget:
    """robot: RobotCfg(length/width/height), lerr: LidarErrCfg, v_ref: 정지 판단에 쓸 기준 속도(m/s)."""
    fp = Footprint(robot.length, robot.width, robot.height)
    k = lerr.k_sigma
    ang = math.radians(lerr.angle_sigma_deg) * lerr.ref_range_m
    yaw = math.radians(lerr.mount_yaw_deg) * lerr.ref_range_m
    bias = lerr.range_bias_m + lerr.mount_xy_m
    rnd = k * math.sqrt(lerr.range_sigma_m ** 2 + ang ** 2 + yaw ** 2)
    lat = max(v_ref, 0.0) * lerr.latency_s
    pose = k * lerr.pose_sigma_m
    src = lerr.source
    items = [("LiDAR 거리 편향", lerr.range_bias_m, "계획+정지", src.get("range_bias_m", "가정")),
             ("LiDAR 장착 위치(xy)", lerr.mount_xy_m, "계획+정지", src.get("mount_xy_m", "가정")),
             ("LiDAR 거리 잡음 k·σ", k * lerr.range_sigma_m, "계획+정지(RSS)", src.get("range_sigma_m", "가정")),
             ("LiDAR 각도 오차×기준거리", k * ang, "계획+정지(RSS)", src.get("angle_sigma_deg", "가정")),
             ("LiDAR 장착 yaw×기준거리", k * yaw, "계획+정지(RSS)", src.get("mount_yaw_deg", "가정")),
             ("전달 지연×기준속도", lat, "정지만", src.get("latency_s", "가정")),
             ("자기 위치추정 k·σ_pose", pose, "계획만", src.get("pose_sigma_m", "가정"))]
    return ClearanceBudget(fp, bias, rnd, lat, pose, items)


# ====================================================================== 기하 (SAT)
def _axes(c: np.ndarray) -> List[np.ndarray]:
    e1, e2 = c[1] - c[0], c[2] - c[1]
    return [e1 / (np.linalg.norm(e1) + 1e-12), e2 / (np.linalg.norm(e2) + 1e-12)]


def sat_push(a: np.ndarray, b: np.ndarray) -> Optional[np.ndarray]:
    """볼록 사각형 a, b 의 SAT. 겹치면 a 를 b 에서 떼어내는 최소 이동 벡터(길이 = 침투 깊이), 안 겹치면 None."""
    best_depth, best_axis = 1e18, None
    for ax in _axes(a) + _axes(b):
        pa, pb = a @ ax, b @ ax
        lo, hi = max(pa.min(), pb.min()), min(pa.max(), pb.max())
        depth = hi - lo
        if depth <= 0:
            return None
        if depth < best_depth:
            best_depth, best_axis = depth, ax
            if (pa.min() + pa.max()) < (pb.min() + pb.max()):
                best_axis = -ax
    return best_axis * best_depth


def aabb_corners(x0: float, y0: float, x1: float, y1: float) -> np.ndarray:
    return np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]], dtype=float)


def ray_rect(ox: float, oy: float, dx: float, dy: float, corners: np.ndarray, tmax: float) -> Optional[float]:
    """반직선 (o + t·d) 이 볼록 사각형과 처음 만나는 t (없으면 None)."""
    best = None
    for i in range(4):
        ax, ay = corners[i]
        bx, by = corners[(i + 1) % 4]
        ex, ey = bx - ax, by - ay
        den = dx * ey - dy * ex
        if abs(den) < 1e-12:
            continue
        t = ((ax - ox) * ey - (ay - oy) * ex) / den
        u = ((ax - ox) * dy - (ay - oy) * dx) / den
        if t > 1e-9 and 0.0 <= u <= 1.0 and t <= tmax and (best is None or t < best):
            best = t
    return best


# ====================================================================== 격자 위 검사
def rect_cells_free(occ: np.ndarray, res: float, ox: float, oy: float, x: float, y: float, th: float,
                    hl: float, hw: float) -> bool:
    """직사각형(반길이 hl, 반폭 hw)을 (x, y, th) 에 놓았을 때 점유 칸과 안 겹치는가. 칸 중심이 사각형 안(칸 반크기만큼 확장)에 있으면 겹침."""
    ny, nx = occ.shape
    c, s = abs(math.cos(th)), abs(math.sin(th))
    ex, ey = hl * c + hw * s + res, hl * s + hw * c + res
    ix0, ix1 = int((x - ex - ox) / res), int((x + ex - ox) / res) + 1
    iy0, iy1 = int((y - ey - oy) / res), int((y + ey - oy) / res) + 1
    if ix0 < 0 or iy0 < 0 or ix1 >= nx or iy1 >= ny:
        return False                                      # 경기장 밖으로 걸치면 불가
    sub = occ[iy0:iy1 + 1, ix0:ix1 + 1]
    if not sub.any():
        return True
    yy, xx = np.nonzero(sub)
    px = ox + (xx + ix0 + 0.5) * res - x
    py = oy + (yy + iy0 + 0.5) * res - y
    cth, sth = math.cos(th), math.sin(th)
    lx = px * cth + py * sth
    ly = -px * sth + py * cth
    half = 0.5 * res
    return not bool(np.any((np.abs(lx) <= hl + half * 0.999) & (np.abs(ly) <= hw + half * 0.999)))


def circle_cells_free(occ: np.ndarray, res: float, ox: float, oy: float, x: float, y: float, r: float) -> bool:
    ny, nx = occ.shape
    k = int(math.ceil(r / res)) + 1
    ix, iy = int((x - ox) / res), int((y - oy) / res)
    if ix - k < 0 or iy - k < 0 or ix + k >= nx or iy + k >= ny:
        return False
    sub = occ[iy - k:iy + k + 1, ix - k:ix + k + 1]
    if not sub.any():
        return True
    yy, xx = np.nonzero(sub)
    dx = (xx - k) * res
    dy = (yy - k) * res
    return not bool(np.any(dx * dx + dy * dy <= (r + 0.5 * res * 0.999) ** 2))


def path_violations(occ: np.ndarray, res: float, ox: float, oy: float, path: Sequence[Pt], bud: ClearanceBudget,
                    step: float = 0.05, turn_deg: float = 25.0, start_heading: Optional[float] = None) -> List[Tuple[float, float, str]]:
    """경로를 따라 직사각형(+계획 여유)을 진행 방향으로 놓고 점유와 겹치는 점, 그리고 방향이 크게 바뀌는 꼭짓점에서
    제자리 회전에 필요한 외접원(+계획 여유)이 막힌 점을 모은다. [(x, y, 사유)]  — 비어 있으면 통과."""
    out: List[Tuple[float, float, str]] = []
    hl = bud.fp.half_l + bud.plan_margin
    hw = bud.fp.half_w + bud.plan_margin
    rr = bud.plan_inflate_rotate
    pts = list(path)
    if len(pts) < 2:
        return out
    heading_prev = start_heading
    for i in range(len(pts) - 1):
        ax, ay = pts[i]
        bx, by = pts[i + 1]
        L = math.hypot(bx - ax, by - ay)
        if L < 1e-9:
            continue
        th = math.atan2(by - ay, bx - ax)
        if heading_prev is not None and i > 0:
            d = abs(math.atan2(math.sin(th - heading_prev), math.cos(th - heading_prev)))
            if math.degrees(d) > turn_deg and not circle_cells_free(occ, res, ox, oy, ax, ay, rr):
                out.append((ax, ay, "rotate"))               # 꼭짓점에서 방향을 바꿔야 하는데 회전 공간이 없다
        heading_prev = th
        n = max(int(L / step), 1)
        for k in range(n + 1):
            t = k / n
            x, y = ax + (bx - ax) * t, ay + (by - ay) * t
            if not rect_cells_free(occ, res, ox, oy, x, y, th, hl, hw):
                out.append((x, y, "body"))
                break                                          # 한 선분에서는 첫 위반만
    return out
