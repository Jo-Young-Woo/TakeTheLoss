"""깊이 카메라 자세(extrinsic) 보정 — 바닥 평면 맞춤.

깊이 카메라는 팬/틸트 아래 차체에 고정이라 팬·틸트와 무관하다. 로봇을 평평한 바닥에 두고 깊이 영상을 찍으면, 바닥이 보이는 점들의 평면이 카메라가 차체에 얼마나 기울어(pitch, roll)
달렸고 바닥 위 몇 m 에 있는지(height)를 말해 준다. 이것으로 profile.sensors.depth.rpy(롤·피치)와 xyz 의 z 를 채운다.
- 한계: 바닥 평면은 yaw 와 x, y 를 알려주지 않는다. 그 셋은 줄자(x, y)와 '벽에 정면으로 서서 본 영상의 좌우 대칭'(yaw)으로 재고,
  운영 중에는 깊이↔LiDAR 정합 감시(perception/depth_fusion.py)가 앞뒤 어긋남(x)만 잔차로 드러낸다.
- 여기서 얻는 roll·pitch 는 base_link 대비 설치 각도다(차체가 수평하다는 가정 — 바닥이 평평하고 바퀴가 같은 높이일 때).
"""
from __future__ import annotations

import math
from typing import Any, Dict, List, Optional

import numpy as np

from ..geom3d import R_LINK_OPT, Intrinsics


def floor_points(depth: np.ndarray, intr: Intrinsics, min_range: float = 0.3, max_range: float = 2.5, stride: int = 2) -> np.ndarray:
    """영상 아랫부분의 유효 깊이 점들(optical 프레임 (N,3)). 바닥이 있을 만한 아래쪽 55 % 만 쓴다."""
    h, w = depth.shape
    v0 = int(h * 0.45)
    vv, uu = np.meshgrid(np.arange(v0, h, stride), np.arange(0, w, stride), indexing="ij")
    z = depth[v0::stride, ::stride].astype(float)
    ok = np.isfinite(z) & (z >= min_range) & (z <= max_range)
    z, uu, vv = z[ok], uu[ok].astype(float), vv[ok].astype(float)
    return np.stack([(uu - intr.cx) / intr.fx * z, (vv - intr.cy) / intr.fy * z, z], axis=1)


def fit_plane_ransac(P: np.ndarray, iters: int = 200, tol: float = 0.012, seed: int = 0,
                     n_prior: Optional[np.ndarray] = None, prior_deg: float = 35.0) -> Dict[str, Any]:
    """RANSAC 평면 n·p = d (|n| = 1, d ≥ 0 이 되도록 부호 정함). n_prior 가 있으면 그 방향에서 prior_deg 이상 벗어난 평면은 버린다
    (바닥이 아니라 벽을 잡는 것을 막는다)."""
    rng = np.random.default_rng(seed)
    best = None
    N = len(P)
    if N < 30:
        raise ValueError(f"바닥 후보 점이 {N}개뿐이다 — 바닥이 보이게 찍을 것")
    for _ in range(iters):
        s = P[rng.choice(N, 3, replace=False)]
        n = np.cross(s[1] - s[0], s[2] - s[0])
        nn = np.linalg.norm(n)
        if nn < 1e-9:
            continue
        n = n / nn
        d = float(n @ s[0])
        if d < 0:
            n, d = -n, -d
        if n_prior is not None and math.degrees(math.acos(float(np.clip(n @ n_prior, -1, 1)))) > prior_deg:
            continue
        inl = np.abs(P @ n - d) < tol
        c = int(inl.sum())
        if best is None or c > best[0]:
            best = (c, n, d, inl)
    if best is None:
        raise ValueError("바닥 평면을 찾지 못했다")
    _, n, d, inl = best
    Q = P[inl]                                                   # 인라이어로 최소제곱 재맞춤
    cen = Q.mean(axis=0)
    _, _, vt = np.linalg.svd(Q - cen, full_matrices=False)
    n = vt[-1]
    d = float(n @ cen)
    if d < 0:
        n, d = -n, -d
    res = np.abs(P[inl] @ n - d)
    return {"n": n, "d": d, "inliers": int(inl.sum()), "n_points": N, "rms_m": float(np.sqrt(np.mean(res ** 2)))}


def calibrate_mount_from_floor(depth: np.ndarray, intr: Intrinsics, nominal_rpy=(0.0, 0.0, 0.0), **kw) -> Dict[str, Any]:
    """깊이 영상 한 장(또는 median 한 장) → 깊이 링크의 롤·피치(rad, URDF rpy)와 바닥 위 높이.
    반환 dict: roll, pitch, height, inliers, rms_m (+ 도 단위 *_deg)."""
    P = floor_points(depth, intr, **{k: v for k, v in kw.items() if k in ("min_range", "max_range", "stride")})
    # 광학 → link: n_link = R_LINK_OPT · n_opt. fit_plane_ransac 은 n·p = d (d ≥ 0, 원점에서 평면 쪽으로 향하는 법선)이라 바닥 법선은
    # 카메라에서 '아래'를 향한다 = link 의 (0,0,-1).
    n_prior_opt = R_LINK_OPT.T @ np.array([0.0, 0.0, -1.0])
    fit = fit_plane_ransac(P, n_prior=n_prior_opt, **{k: v for k, v in kw.items() if k in ("iters", "tol", "seed")})
    n_link = R_LINK_OPT @ fit["n"]
    if n_link[2] < 0:                                            # 아래를 향한 법선을 바닥의 위쪽 법선으로 통일
        n_link = -n_link
    # R = Ry(pitch) Rx(roll) 이면 n_link = Rᵀ·(0,0,1) = (-sin p, sin r cos p, cos r cos p)
    pitch = math.asin(float(np.clip(-n_link[0], -1, 1)))
    roll = math.atan2(float(n_link[1]), float(n_link[2]))
    return {"roll": roll, "pitch": pitch, "height": fit["d"], "inliers": fit["inliers"], "n_points": fit["n_points"],
            "rms_m": fit["rms_m"], "roll_deg": math.degrees(roll), "pitch_deg": math.degrees(pitch)}


def median_depth(frames: List[np.ndarray]) -> np.ndarray:
    """여러 장의 깊이 영상을 픽셀별 중앙값으로 합쳐 잡음을 줄인다(무효 0 은 제외)."""
    st = np.stack([np.where(f > 0, f, np.nan) for f in frames])
    with np.errstate(all="ignore"):
        import warnings
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", RuntimeWarning)
            m = np.nanmedian(st, axis=0)
    return np.nan_to_num(m, nan=0.0).astype(np.float32)


def apply_to_profile(profile, res: Dict[str, Any], note: str = "") -> List[str]:
    """프로파일의 sensors.depth.rpy 의 roll, pitch 와 xyz 의 z 를 측정값으로 바꾼다(yaw·x·y 는 그대로)."""
    rpy = list(profile.get("sensors.depth.rpy"))
    xyz = list(profile.get("sensors.depth.xyz"))
    rpy[0], rpy[1] = round(res["roll"], 5), round(res["pitch"], 5)
    base_z = profile.get("model.wheels.radius")
    xyz[2] = round(res["height"] - base_z, 4)                    # 깊이 링크는 base_link 기준(팬/틸트와 무관)
    profile.set("sensors.depth.rpy", rpy, "measured", note)
    profile.set("sensors.depth.xyz", xyz, "measured", note)
    return [f"- 깊이 카메라 설치: roll {res['roll_deg']:+.2f}°, pitch {res['pitch_deg']:+.2f}°, 바닥 위 높이 {res['height']:.3f} m "
            f"(인라이어 {res['inliers']}/{res['n_points']}, RMS {res['rms_m'] * 1000:.1f} mm) → sensors.depth.rpy[0:2], xyz[2]"]
