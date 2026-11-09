"""깊이 메시지(sensor_msgs/Image, CameraInfo) → DepthFrame. ROS 없이도 쓰고 시험할 수 있는 순수 함수들.

OAK(depthai-ros) 깊이: 보통 encoding 16UC1(mm) 이고 /stereo/camera_info 의 K 는 그 영상 해상도 기준이다. 시뮬 브리지·다른 드라이버는 32FC1(m).
"""
from __future__ import annotations

import math
from typing import Any, Optional, Tuple

import numpy as np

from ..types import DepthFrame
from ..geom3d import RigModel


def decode_depth_image(msg: Any, unit_scale: float = 0.001) -> np.ndarray:
    """sensor_msgs/Image → (H, W) float32 깊이(m). 무효(0, nan, inf)는 0. 지원: 16UC1(× unit_scale), 32FC1(m), 8 비트 아님."""
    h, w = int(msg.height), int(msg.width)
    if h <= 0 or w <= 0:
        raise ValueError("깊이 영상의 폭/높이는 양수여야 한다")
    enc = str(msg.encoding).upper()
    buf = bytes(msg.data) if not isinstance(msg.data, (bytes, bytearray, memoryview)) else msg.data
    big = bool(getattr(msg, "is_bigendian", 0))
    if enc in ("16UC1", "MONO16"):
        dt = np.dtype(">u2" if big else "<u2")
        scale = float(unit_scale)
        if not math.isfinite(scale) or scale <= 0:
            raise ValueError("16UC1 깊이 단위 배율은 유한한 양수여야 한다")
    elif enc == "32FC1":
        dt = np.dtype(">f4" if big else "<f4")
        scale = 1.0
    else:
        raise ValueError(f"깊이 영상 인코딩 {msg.encoding} 은 지원하지 않는다(16UC1·32FC1)")
    step = int(getattr(msg, "step", 0)) or w * dt.itemsize
    if step < w * dt.itemsize:
        raise ValueError("깊이 영상의 step이 한 행의 픽셀보다 짧다")
    if memoryview(buf).nbytes < h * step:
        raise ValueError("깊이 영상 데이터가 height * step보다 짧다")
    raw = np.frombuffer(buf, dtype=np.uint8, count=h * step).reshape(h, step)
    arr = np.frombuffer(np.ascontiguousarray(raw[:, :w * dt.itemsize]).tobytes(), dtype=dt).reshape(h, w)
    d = arr.astype(np.float32) * np.float32(scale)
    d[~np.isfinite(d) | (d <= 0)] = 0.0
    return d


def camera_info_K(info: Any) -> Tuple[Tuple[float, ...], int, int]:
    """sensor_msgs/CameraInfo → (K 9개, 폭, 높이). 정류(rectified) 영상은 P 의 앞 3x3 가 더 정확하므로 P 가 있으면 그것을 쓴다."""
    pv, kv = getattr(info, "p", None), getattr(info, "k", None)
    P = list(pv) if pv is not None else []
    k = list(kv) if kv is not None else []
    w, h = int(info.width), int(info.height)
    if w <= 0 or h <= 0:
        raise ValueError("CameraInfo 폭/높이는 양수여야 한다")
    if P and not np.isfinite(np.asarray(P, dtype=float)).all():
        raise ValueError("CameraInfo P에 유한하지 않은 값이 있다")
    if len(P) == 12 and P[0] > 0 and P[5] > 0:
        K = (float(P[0]), 0.0, float(P[2]), 0.0, float(P[5]), float(P[6]), 0.0, 0.0, 1.0)
    elif len(k) == 9 and k[0] > 0 and k[4] > 0:
        K = tuple(float(x) for x in k)
    else:
        raise ValueError("CameraInfo 에 쓸 수 있는 K/P 가 없다")
    if not np.isfinite(K).all():
        raise ValueError("CameraInfo K에 유한하지 않은 값이 있다")
    return K, w, h


def validate_depth_transform(transform: Any, rig: Any, base_frame: str,
                             tolerance_m: float, tolerance_deg: float) -> Tuple[float, float]:
    """Check ROS base←optical against the fixed-camera RigModel; never adjust calibration.

    ``transform`` has geometry_msgs/Transform's translation and rotation fields.
    RigModel includes base_z above the floor; base_link's transform must omit it.
    Both paths include the optical-axis rotation, not just the depth_link pose.
    """
    if base_frame not in ("base_footprint", "base_link"):
        raise ValueError("depth TF supports only base_footprint or base_link")
    if (not math.isfinite(tolerance_m) or tolerance_m <= 0 or
            not math.isfinite(tolerance_deg) or tolerance_deg <= 0):
        raise ValueError("depth TF tolerances must be finite and positive")
    p, q = transform.translation, transform.rotation
    xyz = np.asarray([p.x, p.y, p.z], dtype=float)
    quat = np.asarray([q.x, q.y, q.z, q.w], dtype=float)
    if not np.isfinite(xyz).all() or not np.isfinite(quat).all():
        raise ValueError("depth TF contains non-finite translation/quaternion")
    norm = float(np.linalg.norm(quat))
    if abs(norm - 1.0) > 1e-3:
        raise ValueError("depth TF quaternion is not unit length")
    x, y, z, w = quat / norm
    actual_R = np.array([[1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
                         [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
                         [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)]])
    expected = RigModel(rig).optical_in_world(0, 0, 0, 0, 0, "depth")
    if base_frame == "base_link":
        expected.t[2] -= float(rig.base_z)
    if not np.isfinite(expected.t).all() or not np.isfinite(expected.R).all():
        raise ValueError("configured depth rig contains non-finite values")
    distance = float(np.linalg.norm(xyz - expected.t))
    angle = math.degrees(math.acos(float(np.clip((np.trace(expected.R.T @ actual_R) - 1) / 2, -1, 1))))
    if distance > tolerance_m or angle > tolerance_deg:
        raise ValueError(f"depth TF differs from calibrated rig: {distance:.4f} m, {angle:.3f} deg "
                         f"(limits {tolerance_m:.4f} m, {tolerance_deg:.3f} deg)")
    return distance, angle


def make_depth_frame(msg: Any, stamp: float, pan: float, tilt: float, unit_scale: float = 0.001,
                     K: Optional[Tuple[float, ...]] = None) -> DepthFrame:
    """stamp 는 반드시 robot.now() 시계로 변환해서 넣을 것(hal/clock.py)."""
    return DepthFrame(depth=decode_depth_image(msg, unit_scale), stamp=stamp, pan=pan, tilt=tilt, K=K)
