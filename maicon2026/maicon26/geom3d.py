"""3D 기하: 강체 변환, 핀홀 내부 파라미터, 차체→팬→틸트→센서 기구학(= URDF/TF 체인).

좌표 약속
  월드     경기장 좌표(arena.py): x 오른쪽, y 위쪽, z 위(바닥 0). 차체 자세는 Pose2D(x, y, θ) — 바닥이 평평하다고 본다.
  link     ROS REP-103 센서 링크: x 앞, y 왼쪽, z 위.
  optical  카메라 광학 프레임: z 앞(광축), x 오른쪽, y 아래. 영상 (u, v) = (fx·x/z + cx, fy·y/z + cy).
팬(+ 왼쪽, z 축 회전), 틸트(+ 위, y 축 회전)는 HAL 약속(hal/base.py)을 따른다.

TF 체인(ros2/urdf/wingbot_ugv.urdf 와 같다):
  RGB 카메라  world ← base_link(차체 자세, 높이 base_z) ← pan_link(pan_xyz, 팬 회전) ← tilt_link(틸트 회전) ← camera_link(xyz, rpy) ← optical
  깊이 카메라 world ← base_link ← depth_link(xyz, rpy) ← optical      (팬/틸트 '아래'에 고정 — 팬/틸트와 같이 움직이지 않는다)
URDF 에서는 팬/틸트가 고정 관절(0°)이라 RGB 카메라의 동적 각도는 여기서 곱한다 — 이것이 '카메라 자세(TF)'를 코드에서 쓰는 지점이다.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

import numpy as np

# optical → link 회전(열 = optical 축을 link 좌표로): x_opt=-y_link, y_opt=-z_link, z_opt=x_link
R_LINK_OPT = np.array([[0.0, 0.0, 1.0],
                       [-1.0, 0.0, 0.0],
                       [0.0, -1.0, 0.0]])


def rot_x(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=float)


def rot_y(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=float)


def rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=float)


def rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """URDF 규약: R = Rz(yaw)·Ry(pitch)·Rx(roll)."""
    return rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)


@dataclass
class Rt:
    """강체 변환 p' = R·p + t."""
    R: np.ndarray
    t: np.ndarray

    @staticmethod
    def identity() -> "Rt":
        return Rt(np.eye(3), np.zeros(3))

    @staticmethod
    def from_xyz_rpy(xyz: Sequence[float], rpy: Sequence[float] = (0.0, 0.0, 0.0)) -> "Rt":
        return Rt(rpy_matrix(*rpy), np.asarray(xyz, dtype=float))

    def __matmul__(self, o: "Rt") -> "Rt":
        return Rt(self.R @ o.R, self.R @ o.t + self.t)

    def inv(self) -> "Rt":
        return Rt(self.R.T, -self.R.T @ self.t)

    def apply(self, pts: np.ndarray) -> np.ndarray:
        """(N,3) 또는 (3,) 점들을 변환."""
        p = np.asarray(pts, dtype=float)
        return p @ self.R.T + self.t


@dataclass
class Intrinsics:
    """핀홀 내부 파라미터(왜곡 보정된 영상 기준). 깊이 영상은 보통 이미 정류(rectified)돼 있어 왜곡을 따로 다루지 않는다."""
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int

    @staticmethod
    def from_hfov(width: int, height: int, hfov_deg: float) -> "Intrinsics":
        f = (width / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
        return Intrinsics(f, f, width / 2.0, height / 2.0, width, height)

    @staticmethod
    def from_K(K: Sequence[float], width: int, height: int) -> "Intrinsics":
        """K = [fx,0,cx, 0,fy,cy, 0,0,1] (sensor_msgs/CameraInfo.k 와 같은 순서)."""
        k = [float(x) for x in K]
        if len(k) != 9 or k[0] <= 0 or k[4] <= 0:
            raise ValueError(f"카메라 행렬 K 가 올바르지 않다: {K}")
        return Intrinsics(k[0], k[4], k[2], k[5], int(width), int(height))

    def scaled(self, width: int, height: int) -> "Intrinsics":
        """같은 광학계로 해상도만 바뀐 영상용(예: 640x400 보정값을 160x100 으로 줄여 쓸 때)."""
        sx, sy = width / self.width, height / self.height
        return Intrinsics(self.fx * sx, self.fy * sy, self.cx * sx, self.cy * sy, int(width), int(height))

    @property
    def hfov_deg(self) -> float:
        return math.degrees(2 * math.atan2(self.width / 2.0, self.fx))

    @property
    def vfov_deg(self) -> float:
        return math.degrees(2 * math.atan2(self.height / 2.0, self.fy))

    def K(self) -> list:
        return [self.fx, 0.0, self.cx, 0.0, self.fy, self.cy, 0.0, 0.0, 1.0]

    def pixel_grid(self, stride: int = 1) -> Tuple[np.ndarray, np.ndarray]:
        """(v 인덱스, u 인덱스) 배열 — 픽셀 중심 기준(+0.5 없이 정수 좌표를 쓴다)."""
        vs = np.arange(0, self.height, stride)
        us = np.arange(0, self.width, stride)
        return np.meshgrid(vs, us, indexing="ij")

    def rays(self, u: np.ndarray, v: np.ndarray) -> np.ndarray:
        """픽셀 → optical 프레임 단위 z=1 광선 (…,3): ((u-cx)/fx, (v-cy)/fy, 1)."""
        return np.stack([(u - self.cx) / self.fx, (v - self.cy) / self.fy, np.ones_like(u, dtype=float)], axis=-1)

    def project(self, p_opt: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """optical 점 (N,3) → (u, v, z). z ≤ 0 이면 u, v 는 nan."""
        p = np.asarray(p_opt, dtype=float).reshape(-1, 3)
        z = p[:, 2]
        with np.errstate(divide="ignore", invalid="ignore"):
            u = np.where(z > 1e-6, self.fx * p[:, 0] / z + self.cx, np.nan)
            v = np.where(z > 1e-6, self.fy * p[:, 1] / z + self.cy, np.nan)
        return u, v, z


class RigModel:
    """차체→팬→틸트→센서 기구학. RigCfg(config.py)에서 만든다. 센서 이름: 'camera' | 'depth' | 'lidar'."""

    def __init__(self, rig) -> None:
        self.base_z = float(rig.base_z)
        self.T_base_pan = Rt.from_xyz_rpy(rig.pan_xyz)
        # RGB 카메라는 팬/틸트 위에 달려 있다(tilt_link 기준). 깊이 카메라는 팬/틸트 '아래'에 붙은 고정 센서라 base_link 기준이고 팬/틸트와 무관하다.
        self.T_tilt = {"camera": Rt.from_xyz_rpy(rig.camera_xyz, rig.camera_rpy)}
        self.T_base_depth = Rt.from_xyz_rpy(rig.depth_xyz, rig.depth_rpy)
        self.lidar_xyz = tuple(float(x) for x in rig.lidar_xyz)

    @staticmethod
    def base_to_world(x: float, y: float, th: float, base_z: float) -> Rt:
        return Rt(rot_z(th), np.array([x, y, base_z], dtype=float))

    def optical_in_world(self, x: float, y: float, th: float, pan_deg: float, tilt_deg: float,
                         sensor: str = "depth", extra: Optional[Rt] = None) -> Rt:
        """센서 광학 프레임 → 월드 변환. extra 는 '숨은 설치 오차'(시뮬 시험용)를 link 프레임에서 덧붙일 때 쓴다.
        sensor='depth' 는 팬/틸트 아래 차체에 고정이라 pan_deg, tilt_deg 를 무시한다."""
        if sensor == "depth":
            T = self.base_to_world(x, y, th, self.base_z) @ self.T_base_depth
            if extra is not None:
                T = T @ extra
            return T @ Rt(R_LINK_OPT, np.zeros(3))
        T = self.base_to_world(x, y, th, self.base_z) @ self.T_base_pan
        T = T @ Rt(rot_z(math.radians(pan_deg)), np.zeros(3))            # 팬: 왼쪽 + (z 축 반시계)
        T = T @ Rt(rot_y(-math.radians(tilt_deg)), np.zeros(3))         # 틸트: 위 + (y 축 회전은 부호 반대)
        T = T @ self.T_tilt[sensor]
        if extra is not None:
            T = T @ extra
        return T @ Rt(R_LINK_OPT, np.zeros(3))

    def position(self, x: float, y: float, th: float, pan_deg: float, tilt_deg: float, sensor: str = "camera") -> np.ndarray:
        return self.optical_in_world(x, y, th, pan_deg, tilt_deg, sensor).t

    def lidar_origin(self, x: float, y: float, th: float) -> np.ndarray:
        return self.base_to_world(x, y, th, self.base_z).apply(np.asarray(self.lidar_xyz))


def backproject(depth_m: np.ndarray, intr: Intrinsics, T_world_opt: Rt, stride: int = 1,
                min_range: float = 0.0, max_range: float = 1e9, phase: int = 0) -> Tuple[np.ndarray, np.ndarray]:
    """깊이 영상(z 깊이, m; 0/nan = 무효) → (월드 점 (N,3), 그 점의 카메라 기준 직선거리 (N,)).
    stride 로 격자를 솎아 쓰되, phase(프레임마다 0, 1, 2, … 로 돌려 준다)로 솎는 위치를 순환시켜 stride² 프레임이면 모든 픽셀이 한 번씩 쓰이게 한다
    (영상에서 몇 줄밖에 안 되는 낮은 물체가 늘 같은 줄 사이로 빠지지 않도록). 무효·범위 밖은 제외한다."""
    h, w = depth_m.shape[:2]
    oy, ox = (phase // stride) % stride, phase % stride
    vv, uu = np.meshgrid(np.arange(oy, h, stride), np.arange(ox, w, stride), indexing="ij")
    z = depth_m[oy::stride, ox::stride].astype(float)
    ok = np.isfinite(z) & (z > 0) & (z >= min_range) & (z <= max_range)
    z, uu, vv = z[ok], uu[ok].astype(float), vv[ok].astype(float)
    p_opt = np.stack([(uu - intr.cx) / intr.fx * z, (vv - intr.cy) / intr.fy * z, z], axis=1)
    return T_world_opt.apply(p_opt), np.linalg.norm(p_opt, axis=1)
