"""카메라 기하. 핀홀 근사(왜곡 보정된 영상 가정).

로봇 카메라는 팬/틸트 위에 있으므로, 월드 방위 = 차체 θ + 팬 + 영상 방위.
"""
from __future__ import annotations

import math
from typing import Optional, Tuple


from ..config import CameraCfg
from ..types import Detection, Pose2D, Scan


class CameraModel:
    """핀홀 모델. cfg.K(체커보드 보정값)가 있으면 fx, fy, cx, cy 를 그대로 쓰고, 없으면 hfov 로 근사한다(주점 = 영상 중심, fx = fy)."""

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

    def bearing(self, u: float) -> float:
        """픽셀 x → 카메라 축 기준 방위(rad, 왼쪽 +)."""
        return math.atan2(self.cx - u, self.f)

    def elevation(self, v: float) -> float:
        """픽셀 y → 카메라 축 기준 고각(rad, 위 +)."""
        return math.atan2(self.cy - v, self.fy)

    def range_from_size(self, size_px: float, real_m: float, u: Optional[float] = None) -> float:
        """겉보기 높이(px)로 거리 추정. 핀홀에서 크기는 '광축 방향 깊이'에 반비례하므로,
        화면 가장자리에 있을수록 실제 거리 = 깊이 / cos(방위)로 보정해야 한다."""
        depth = self.f * real_m / max(size_px, 1.0)
        if u is None:
            return depth
        return depth / max(math.cos(self.bearing(u)), 0.3)

    def world_ray(self, pose: Pose2D, pan_deg: float, u: float) -> float:
        """픽셀 x의 월드 방위각(rad)."""
        return pose.th + math.radians(pan_deg) + self.bearing(u)


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
    r_size = cam.range_from_size(det.h, real_h, det.cx) if det.h > 4 else None
    # dth = (프레임 촬영 시 차체 헤딩) - (스캔 촬영 시 차체 헤딩): 회전 중 시간차 보정
    body_bearing = math.radians(pan_deg) + cam.bearing(det.cx) + dth
    r_lidar = lidar_range_at(scan, body_bearing)
    if r_lidar is not None and (r_size is None or abs(r_lidar - r_size) <= 0.4 * max(r_size, 0.3)):
        return r_lidar, "lidar"
    if r_size is not None:
        return r_size, "size"
    return None, "none"
