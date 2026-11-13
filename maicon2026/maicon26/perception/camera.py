"""카메라 기하. 핀홀(기본)과 어안(OpenCV fisheye 등거리 다항식, 7장) 두 모델.

로봇 카메라는 팬/틸트 위에 있으므로, 월드 방위 = 차체 θ + 팬 + 영상 방위.
"""
from __future__ import annotations

import logging
import math
from dataclasses import dataclass
from typing import Optional, Tuple


from ..config import CameraCfg
from ..types import Detection, Pose2D, Scan
from ..validate import check_camera_matrix, check_fisheye_D


_WARNED: set = set()                                   # 같은 경고를 CameraModel 을 만들 때마다 되풀이하지 않는다


def _warn_once(reason: str) -> None:
    if reason not in _WARNED:
        _WARNED.add(reason)
        logging.getLogger("maicon26.camera").warning("[camera] %s — 핀홀(hfov)로 돌아간다", reason)


class CameraModel:
    """카메라 모델 둘: 핀홀(기본)과 어안(OpenCV fisheye 등거리 다항식, 7장).

    핀홀: cfg.K(체커보드 보정값)가 있으면 fx, fy, cx, cy 를 그대로 쓰고, 없으면 hfov 로 근사한다(주점 = 영상 중심, fx = fy).
    어안: cfg.model == "fisheye" 이고 K 와 D(k1..k4, 4개)가 있을 때만. 하나라도 없거나 이상하면 **경고 후 핀홀로 돌아간다**(예외 없음, 0장 원칙).

    각도 약속: 방위(bearing)는 왼쪽 +, 고각(elevation)은 위 +. 카메라 좌표는 X 오른쪽, Y 아래, Z 앞.
        bearing = atan2(-X, Z),  elevation = atan2(-Y, sqrt(X² + Z²))
    핀홀의 고각은 지금까지처럼 u 를 무시하는 분리형(atan2(cy - v, fy))이다. 어안은 방향 벡터의 실제 고각이다.
    """

    def __init__(self, cfg: CameraCfg) -> None:
        self.w, self.h = cfg.width, cfg.height
        K = getattr(cfg, "K", None)
        if K:
            if len(K) != 9:
                raise ValueError(f"camera.K 는 9개 값(fx,0,cx,0,fy,cy,0,0,1)이어야 한다: {K}")
            self.f, self.fy, self.cx, self.cy = float(K[0]), float(K[4]), float(K[2]), float(K[5])
        else:
            self.f = (self.w / 2) / math.tan(math.radians(cfg.hfov_deg) / 2)
            self.fy = self.f
            self.cx, self.cy = self.w / 2, self.h / 2
        self.model = "pinhole"
        self.D: Optional[Tuple[float, ...]] = None
        self.fallback_reason: Optional[str] = None            # 어안을 요청했는데 핀홀로 돌아갔다면 그 이유(진단·시험용)
        self.usable_hfov_deg = getattr(cfg, "usable_hfov_deg", None)
        self.theta_max = math.pi / 2
        self._thd_max = math.pi / 2
        want = str(getattr(cfg, "model", "pinhole") or "pinhole").strip().lower()
        if want == "fisheye":
            self._init_fisheye(cfg, K)
        elif want != "pinhole":
            self.fallback_reason = f"camera.model={want!r} 는 모르는 값(pinhole | fisheye)"
            _warn_once(self.fallback_reason)

    # ------------------------------------------------------------------ 어안 초기화
    def _init_fisheye(self, cfg: CameraCfg, K) -> None:
        D = getattr(cfg, "D", None)
        why = None
        if not K:
            why = "camera.model=fisheye 인데 camera.K 가 없다"
        elif D is None or check_fisheye_D(D) == "D_count":                     # 9장 V8: 점검은 공통 함수(maicon26/validate.py)
            why = f"camera.model=fisheye 인데 camera.D 가 4개(k1..k4)가 아니다: {D!r}"
        elif check_fisheye_D(D) is not None:
            why = f"camera.D 에 유한하지 않은 값이 있다: {D!r}"
        elif not (self.f > 0 and self.fy > 0):
            why = f"camera.K 의 fx, fy 가 양수가 아니다: ({self.f}, {self.fy})"
        elif check_camera_matrix(K, self.w, self.h) is not None:               # 유한하지 않은 값 · 주점이 영상 밖
            why = f"camera.K 를 쓸 수 없다({check_camera_matrix(K, self.w, self.h)}): {tuple(K)!r}"
        if why is None:
            self.D = tuple(float(x) for x in D)
            self.model = "fisheye"
            self._setup_domain()
            if not self.theta_max > 0.0:
                self.model, self.D = "pinhole", None
                why = "camera.D 로는 (보정 영역 안에서) 단조 증가하는 어안 투영이 나오지 않는다"
        if why is not None:
            self.fallback_reason = why
            _warn_once(why)

    def _thd(self, th: float) -> float:
        k1, k2, k3, k4 = self.D
        t2 = th * th
        return th * (1.0 + t2 * (k1 + t2 * (k2 + t2 * (k3 + t2 * k4))))

    def _dthd(self, th: float) -> float:
        k1, k2, k3, k4 = self.D
        t2 = th * th
        return 1.0 + t2 * (3.0 * k1 + t2 * (5.0 * k2 + t2 * (7.0 * k3 + t2 * 9.0 * k4)))

    def _setup_domain(self) -> None:
        """보정 최대각 theta_max: (a) 영상의 가장 먼 모서리가 보는 각, (b) θd(θ) 가 단조 증가하는 한계, 둘 중 작은 쪽."""
        corner = max(math.hypot((x - self.cx) / self.f, (y - self.cy) / self.fy)
                     for x in (0.0, float(self.w)) for y in (0.0, float(self.h)))
        lim = math.pi                                          # (b) 단조 한계: dθd/dθ > 0 인 가장 큰 구간
        n = 720
        for i in range(1, n + 1):
            th = lim * i / n
            if self._dthd(th) <= 1e-9:
                lim = lim * (i - 1) / n
                break
        thd_lim = self._thd(lim)
        if corner >= thd_lim:
            self.theta_max, self._thd_max = lim, thd_lim
        else:
            self.theta_max, self._thd_max = self._solve_theta(corner, clamp=False), corner

    def _solve_theta(self, thd: float, clamp: bool = True) -> float:
        """θd → θ 뉴턴법(최대 10회). clamp 면 보정 영역 밖은 theta_max 로 자른다."""
        if clamp and thd >= self._thd_max:
            return self.theta_max
        th = thd
        for _ in range(10):
            err = self._thd(th) - thd
            if abs(err) < 1e-15:
                break
            d = self._dthd(th)
            if d <= 1e-12:
                break
            th -= err / d
        return th

    # ------------------------------------------------------------------ 픽셀 → 각도
    def angles(self, u: float, v: float) -> Tuple[float, float]:
        """픽셀 (u, v) → (방위, 고각) [rad]. 핀홀은 지금의 분리형 식, 어안은 방향 벡터에서 구한다.
        어안에서 보정 영역 밖의 픽셀은 가장자리 각도(theta_max)로 자른다."""
        if self.model != "fisheye":
            return math.atan2(self.cx - u, self.f), math.atan2(self.cy - v, self.fy)
        mx, my = (u - self.cx) / self.f, (v - self.cy) / self.fy
        thd = math.hypot(mx, my)
        if thd < 1e-15:
            return 0.0, 0.0
        th = self._solve_theta(thd)
        s = math.sin(th)
        X, Y, Z = s * mx / thd, s * my / thd, math.cos(th)
        return math.atan2(-X, Z), math.atan2(-Y, math.hypot(X, Z))

    def bearing(self, u: float, v: Optional[float] = None) -> float:
        """픽셀 x → 카메라 축 기준 방위(rad, 왼쪽 +). 어안은 v 에도 의존한다(v 없으면 주점 높이)."""
        if self.model != "fisheye":
            return math.atan2(self.cx - u, self.f)
        return self.angles(u, self.cy if v is None else v)[0]

    def elevation(self, v: float, u: Optional[float] = None) -> float:
        """픽셀 y → 카메라 축 기준 고각(rad, 위 +). 어안은 u 에도 의존한다(u 없으면 주점 가로 위치)."""
        if self.model != "fisheye":
            return math.atan2(self.cy - v, self.fy)
        return self.angles(self.cx if u is None else u, v)[1]

    # ------------------------------------------------------------------ 각도 → 픽셀
    def project(self, bearing: float, elevation: float) -> Optional[Tuple[float, float]]:
        """(방위, 고각)[rad] → 픽셀 (u, v). 닫힌 식(반복 없음). 모델 영역 밖(핀홀: 앞쪽 반구 밖, 어안: theta_max 밖)이면 None.
        핀홀: u = cx - f·tan(b), v = cy - fy·tan(e)."""
        if self.model != "fisheye":
            if abs(bearing) >= math.pi / 2 or abs(elevation) >= math.pi / 2:
                return None
            return self.cx - self.f * math.tan(bearing), self.cy - self.fy * math.tan(elevation)
        ce = math.cos(elevation)
        X, Y, Z = -math.sin(bearing) * ce, -math.sin(elevation), math.cos(bearing) * ce
        return self._project_dir(X, Y, Z)

    def _project_dir(self, X: float, Y: float, Z: float) -> Optional[Tuple[float, float]]:
        rho = math.hypot(X, Y)
        th = math.atan2(rho, Z)
        if th > self.theta_max + 1e-12:
            return None
        if rho < 1e-15:
            return self.cx, self.cy
        thd = self._thd(th)
        return self.f * thd * X / rho + self.cx, self.fy * thd * Y / rho + self.cy

    def project_point(self, forward: float, left: float, up: float) -> Optional[Tuple[float, float]]:
        """카메라 기준 점(앞 forward, 왼쪽 left, 위 up [m]) → 픽셀. 핀홀은 지금의 식 그대로(cx - f·left/forward, cy - fy·up/forward),
        어안은 방향 벡터의 투영. 카메라 뒤(forward ≤ 0)는 핀홀에서 None."""
        if self.model != "fisheye":
            if forward <= 0.0:
                return None
            return self.cx - self.f * left / forward, self.cy - self.fy * up / forward
        return self._project_dir(-left, -up, forward)

    # ------------------------------------------------------------------ 크기·화각
    def angular_extent(self, u: float, v: float, half_px: float) -> float:
        """(u, v) 에서 가로로 half_px 픽셀 떨어진 점까지의 각도(반각, rad). 핀홀은 atan2(half_px, f) — 위치와 무관(지금의 식).
        어안은 (u ± half_px, v) 두 방향 사이 각의 절반이다(같은 픽셀 수가 덮는 각이 위치에 따라 달라진다 — 핀홀 식은 위치와 무관)."""
        if self.model != "fisheye":
            return math.atan2(half_px, self.f)
        return 0.5 * self._angle_between(u - half_px, v, u + half_px, v)

    def _angle_between(self, u1: float, v1: float, u2: float, v2: float) -> float:
        d1, d2 = self._dir(u1, v1), self._dir(u2, v2)
        dot = d1[0] * d2[0] + d1[1] * d2[1] + d1[2] * d2[2]
        cr = (d1[1] * d2[2] - d1[2] * d2[1], d1[2] * d2[0] - d1[0] * d2[2], d1[0] * d2[1] - d1[1] * d2[0])
        return math.atan2(math.sqrt(cr[0] ** 2 + cr[1] ** 2 + cr[2] ** 2), dot)

    def _dir(self, u: float, v: float) -> Tuple[float, float, float]:
        mx, my = (u - self.cx) / self.f, (v - self.cy) / self.fy
        thd = math.hypot(mx, my)
        if thd < 1e-15:
            return 0.0, 0.0, 1.0
        th = self._solve_theta(thd)
        s = math.sin(th)
        return s * mx / thd, s * my / thd, math.cos(th)

    def half_fov_h(self) -> float:
        """커버리지가 쓰는 수평 반화각(rad). usable_hfov_deg 가 있으면 그 절반, 없으면 모델 전체
        (핀홀: 지금의 atan2(w/2, f), 어안: 주점 높이에서 양쪽 영상 가장자리의 방위각 차이의 절반).
        두 방향 사이의 각(내적)으로 구하면 시야가 180° 를 넘을 때 반대쪽으로 접혀 작아지므로, 방위각(부호 있음)의 차이로 구한다."""
        if self.usable_hfov_deg is not None:
            return math.radians(float(self.usable_hfov_deg)) / 2.0
        if self.model != "fisheye":
            return math.atan2(self.w / 2.0, self.f)
        return 0.5 * (self.bearing(0.0, self.cy) - self.bearing(float(self.w), self.cy))

    def range_from_size(self, size_px: float, real_m: float, u: Optional[float] = None, v: Optional[float] = None) -> float:
        """겉보기 높이(px)로 거리 추정. 핀홀에서 크기는 '광축 방향 깊이'에 반비례하므로,
        화면 가장자리에 있을수록 실제 거리 = 깊이 / cos(방위)로 보정해야 한다.
        어안은 (u, v) 에서 세로 size_px 가 덮는 각 α 로 바로 광선 거리를 구한다: r = (real/2) / tan(α/2)."""
        if self.model == "fisheye":
            uu, vv = (self.cx if u is None else u), (self.cy if v is None else v)
            s = max(size_px, 1.0)
            alpha = self._angle_between(uu, vv - s / 2.0, uu, vv + s / 2.0)
            return (real_m / 2.0) / max(math.tan(alpha / 2.0), 1e-6)
        depth = self.f * real_m / max(size_px, 1.0)
        if u is None:
            return depth
        return depth / max(math.cos(self.bearing(u)), 0.3)

    def world_ray(self, pose: Pose2D, pan_deg: float, u: float, v: Optional[float] = None) -> float:
        """픽셀 (u, v) 의 월드 방위각(rad). v 는 어안에서만 쓴다."""
        return pose.th + math.radians(pan_deg) + self.bearing(u, v)


@dataclass(frozen=True)
class ObsCamera:
    """관측 하나가 어느 카메라에서 왔는지: 카메라 모델 + 차체에 대한 장착(yaw, 위치) + 출처 이름.

    object_map·tracks 의 관측 함수에 cam= 으로 넘기면 팬틸트 카메라 대신 이 카메라로 방위·거리를 계산한다. 넘기지 않으면(None) 기존과
    같이 self.cam 과 frame.pan 을 쓴다. 고정 카메라(OAK)는 팬이 없으므로 '팬 대신 장착 yaw'가 방위에 더해진다:
      월드 방위 = 차체 θ + 장착 yaw + 영상 방위(왼쪽 +).   위치(x, y)는 차체 원점 기준 [m] — 크기로 잰 거리의 시작점에만 쓴다
      (LiDAR 거리는 차체 원점에서 잰 값이므로 차체 원점에서 쏜다)."""
    model: CameraModel
    yaw: float = 0.0
    x: float = 0.0
    y: float = 0.0
    source: str = "oak"

    @property
    def pan_equiv_deg(self) -> float:
        """estimate_range / world_ray 의 pan_deg 자리에 넣을 값 — 장착 yaw(+ frame.pan 은 고정 카메라라 0)."""
        return math.degrees(self.yaw)

    def origin(self, pose: Pose2D, how: str) -> Tuple[float, float]:
        """거리 근거(how)에 따른 광선 시작점. 'size'·'depth'(카메라에서 잰 거리)는 카메라 위치, 'lidar' 등은 차체 원점."""
        if how not in ("size", "depth"):        # 'depth' 도 카메라 광축 기준 거리 — 카메라 위치에서 쏜다
            return pose.x, pose.y
        c, s = math.cos(pose.th), math.sin(pose.th)
        return pose.x + c * self.x - s * self.y, pose.y + s * self.x + c * self.y


def oak_obs_camera(oak_cfg, rig, K, width: int, height: int) -> Optional[ObsCamera]:
    """OAK 컬러 카메라 → ObsCamera. K 는 /oak/rgb/camera_info 의 3×3(9개 값)이다.
    OAK-D Lite 의 컬러 화각은 69° 로 왜곡이 작아 핀홀로 두고 D(왜곡 계수)는 쓰지 않는다(어안 보정이 필요한 팬틸트 카메라와 다르다).
    K 가 없으면 None → 호출자는 OAK 관측을 쓰지 않는다(화각 추측으로 방위를 만들지 않는다)."""
    if not K or len(K) != 9 or not all(math.isfinite(v) for v in K) or K[0] <= 0 or K[4] <= 0 or width <= 0 or height <= 0:
        return None
    cam_cfg = CameraCfg(width=int(width), height=int(height), K=tuple(float(v) for v in K))
    return ObsCamera(CameraModel(cam_cfg), yaw=float(rig.oak_rgb_rpy[2]), x=float(rig.oak_rgb_xyz[0]), y=float(rig.oak_rgb_xyz[1]),
                     source="oak")


def lidar_range_at(scan: Optional[Scan], body_bearing: float, half_width: float = 0.04) -> Optional[float]:
    """차체 기준 방위(rad)에서의 LiDAR 거리. 카메라가 가리키는 방향의 물체까지 거리 추정용.
    인접한 두 빔 사이를 선형 보간한다(비스듬한 면에서 창(window) 최솟값을 쓰면 가까운 쪽으로 편향됨).
    두 빔 거리가 12cm 넘게 다르면(모서리/가장자리) 보간하지 않고 가까운 빔 하나만 쓴다."""
    if scan is None or len(scan.ranges) < 3:
        return None
    n = len(scan.ranges)
    x = ((body_bearing - scan.angle_min) / scan.angle_inc) % n       # 실수 인덱스
    i0 = int(math.floor(x)) % n
    i1 = (i0 + 1) % n
    f = x - math.floor(x)
    r0, r1 = float(scan.ranges[i0]), float(scan.ranges[i1])
    ok0 = math.isfinite(r0) and 0.05 < r0 < scan.range_max
    ok1 = math.isfinite(r1) and 0.05 < r1 < scan.range_max
    if ok0 and ok1:
        if abs(r0 - r1) < 0.12:
            return r0 * (1 - f) + r1 * f
        return r0 if f < 0.5 else r1
    if ok0:
        return r0
    if ok1:
        return r1
    return None


def estimate_range(cam: CameraModel, det: Detection, real_h: float, scan: Optional[Scan],
                   pan_deg: float, dth: float = 0.0) -> Tuple[Optional[float], str]:
    """검출 하나까지의 거리(m)와 근거('lidar'|'size'|'none').
    LiDAR가 있으면 우선하되, 크기 기반 추정과 40% 넘게 어긋나면(가림·다른 물체) 크기 기반을 쓴다."""
    r_size = cam.range_from_size(det.h, real_h, det.cx, det.cy) if det.h > 4 else None
    # dth = (프레임 촬영 시 차체 헤딩) - (스캔 촬영 시 차체 헤딩): 회전 중 시간차 보정
    body_bearing = math.radians(pan_deg) + cam.bearing(det.cx, det.cy) + dth
    r_lidar = lidar_range_at(scan, body_bearing)
    if r_lidar is not None and (r_size is None or abs(r_lidar - r_size) <= 0.4 * max(r_size, 0.3)):
        return r_lidar, "lidar"
    if r_size is not None:
        return r_size, "size"
    return None, "none"
