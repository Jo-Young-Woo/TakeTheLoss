"""OAK Depth 로 상대 UGV 까지의 거리를 재고(스테레오, cm 급), 그 거리로 피아식별 신뢰를 정한다.

2D LiDAR 평면은 UGV 의 궤도 아래를 지나 UGV 를 못 보는 일이 있고, 그러면 거리는 크기 추정(수십 % 오차)에 의존한다. OAK 깊이는 컬러에 정렬돼 있어(config/oak/rgbd.yaml
i_align_depth) 컬러 검출 상자를 그대로 깊이 영상에 옮겨 쓸 수 있다.

  - 상자 가운데 일부(roi_frac)만 본다 — 가장자리에는 배경이 섞인다.
  - 깊이 히스토그램에서 '가장 가까우면서 질량이 충분한 덩어리'를 UGV 로 본다(상자에 들어온 뒤쪽 벽·바닥 제외).
  - 광축 방향 깊이 z → 카메라에서 UGV 까지 수평 거리 r = z / cos(방위). (카메라와 UGV 중심의 높이 차는 무시 — 20 cm 이하라 r 에 1% 미만)
  - 깊이로 잰 상자 폭 = 상자 픽셀 폭 × z / fx. UGV(약 0.2~0.4 m)에서 크게 벗어나면 UGV 가 아니다(그림·반사·상자 면). 영상 가장자리에 걸친 상자는 폭을 믿지 않는다.
  - 피아식별: 안테나 색은 멀면 작아져 오독이 늘어난다. 깊이로 거리를 확인했고 iff_max_range_m 를 넘으면 그 검출의 팀 색은 쓰지 않고, 가까우면 득표 가중을 올린다.
이 모듈은 거리·피아 '증거'만 만든다. 조준·사격에는 쓰지 않는다(OAK 검출은 발사 근거가 아니다).
순수 함수 + 얇은 클래스 — ROS 없이 시험한다.
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import Any, Deque, Optional, Sequence, Tuple

import numpy as np


@dataclass(frozen=True)
class DepthRange:
    r: float                      # 카메라에서 UGV 까지 수평 거리 [m]
    z: float                      # 광축 방향 깊이 [m]
    n: int                        # 덩어리에 든 픽셀 수
    quality: float                # 덩어리 픽셀 / ROI 유효 픽셀
    spread: float                 # 덩어리 깊이의 MAD [m]
    width_m: Optional[float]      # 깊이로 잰 상자 폭 [m]
    bearing: float                # 광축에서 벗어난 수평 각 [rad]


def pick_frame(frames: Sequence[Any], t: float, max_dt: float) -> Optional[Any]:
    """촬영 시각 t 에 가장 가까운 깊이 프레임(|stamp − t| ≤ max_dt). 없으면 None."""
    best, bd = None, max_dt
    for f in frames:
        d = abs(float(f.stamp) - float(t))
        if d <= bd:
            best, bd = f, d
    return best


def depth_range_for_box(depth: np.ndarray, box: Tuple[float, float, float, float], rgb_wh: Tuple[int, int], *,
                        fx: float, cx: float, min_range: float = 0.30, max_range: float = 3.5, roi_frac: float = 0.5,
                        min_pixels: int = 12, fg_mass: float = 0.25, bin_m: float = 0.05) -> Optional[DepthRange]:
    """컬러 영상의 검출 상자(x1,y1,x2,y2 px) → 깊이 영상(정렬됨, 해상도는 달라도 비례)에서 거리. 못 재면 None."""
    if depth is None or getattr(depth, "ndim", 0) != 2 or min(depth.shape) < 2:
        return None
    H, W = depth.shape
    rw, rh = rgb_wh
    if rw <= 0 or rh <= 0 or not all(math.isfinite(v) for v in box) or fx <= 0:
        return None
    sx, sy = W / float(rw), H / float(rh)
    x1, y1, x2, y2 = box
    if x2 <= x1 or y2 <= y1:
        return None
    ucx, ucy = 0.5 * (x1 + x2) * sx, 0.5 * (y1 + y2) * sy
    hw, hh = 0.5 * roi_frac * (x2 - x1) * sx, 0.5 * roi_frac * (y2 - y1) * sy
    u0, u1 = max(0, int(math.floor(ucx - hw))), min(W, int(math.ceil(ucx + hw)))
    v0, v1 = max(0, int(math.floor(ucy - hh))), min(H, int(math.ceil(ucy + hh)))
    if u1 - u0 < 1 or v1 - v0 < 1:
        return None
    patch = np.asarray(depth[v0:v1, u0:u1], dtype=np.float64).ravel()
    vals = patch[np.isfinite(patch) & (patch >= min_range) & (patch <= max_range)]
    if len(vals) < min_pixels:
        return None
    nb = max(3, int(round((max_range - min_range) / bin_m)))
    hist, edges = np.histogram(vals, bins=nb, range=(min_range, max_range))
    pad = np.concatenate([[0], hist, [0]])
    mass = pad[:-2] + pad[1:-1] + pad[2:]                         # 이웃 세 칸 합(한 칸에 걸친 덩어리를 놓치지 않게)
    need = fg_mass * len(vals)
    hit = np.flatnonzero(mass >= need)
    if not len(hit):
        return None
    i = int(hit[0])                                              # 가까운 쪽부터 첫 덩어리
    lo, hi = edges[max(0, i - 1)], edges[min(nb, i + 2)]
    cl = vals[(vals >= lo) & (vals <= hi)]
    if len(cl) < min_pixels:
        return None
    z = float(np.median(cl))
    spread = float(np.median(np.abs(cl - z)))
    bearing = math.atan2(ucx - cx, fx)
    r = z / max(math.cos(bearing), 0.2)
    width = (x2 - x1) * sx * z / fx
    return DepthRange(r=float(r), z=z, n=int(len(cl)), quality=float(len(cl) / len(vals)), spread=spread,
                      width_m=float(width), bearing=float(bearing))


class UgvDepthRanger:
    """설정(OakDetCfg·DepthCfg)을 들고 검출 하나의 깊이 거리를 재는 얇은 래퍼. 통계는 보고용."""

    def __init__(self, cfg: Any) -> None:
        self.oc, self.dc = cfg.oak_det, cfg.depth
        self.stats = {"ok": 0, "none": 0, "size_disagree": 0, "width_reject": 0, "no_frame": 0, "not_aligned": 0}
        self.ring: Deque[Any] = deque(maxlen=6)
        self._last_stamp: Optional[float] = None

    def push(self, frame: Any) -> None:
        """로봇의 최신 깊이 프레임을 보관한다(같은 stamp 는 한 번만)."""
        if frame is None or getattr(frame, "stamp", None) is None or frame.stamp == self._last_stamp:
            return
        self._last_stamp = frame.stamp
        self.ring.append(frame)

    def frame_for(self, t: float) -> Optional[Any]:
        f = pick_frame(self.ring, t, self.oc.depth_max_dt_s)
        if f is None:
            self.stats["no_frame"] += 1
        return f

    def _intrinsics(self, frame: Any):
        H, W = frame.depth.shape
        if getattr(frame, "K", None):
            from ..geom3d import Intrinsics
            base_w, base_h = W, H
            return Intrinsics.from_K(frame.K, base_w, base_h)
        return self.dc.intrinsics(W, H)

    def measure(self, det: Any, frame: Any, rgb_wh: Tuple[int, int], ref_range: Optional[float] = None) -> Tuple[Optional[DepthRange], str]:
        """(DepthRange 또는 None, 사유). 사유: ok | no_frame | not_aligned | none | size_disagree | width_reject.
        ref_range: LiDAR·크기로 이미 얻은 거리 — 크게 어긋나면 깊이를 버린다(상자에 다른 물체·바닥이 들어옴)."""
        oc = self.oc
        if frame is None:
            return None, "no_frame"
        if not oc.depth_aligned_to_rgb:
            self.stats["not_aligned"] += 1
            return None, "not_aligned"
        try:
            intr = self._intrinsics(frame)
            dr = depth_range_for_box(frame.depth, det.box, rgb_wh, fx=intr.fx, cx=intr.cx, min_range=self.dc.min_range,
                                     max_range=self.dc.max_range, roi_frac=oc.depth_roi_frac, min_pixels=oc.depth_min_pixels,
                                     fg_mass=oc.depth_fg_mass, bin_m=oc.depth_bin_m)
        except (ValueError, TypeError, ZeroDivisionError):
            dr = None
        if dr is None:
            self.stats["none"] += 1
            return None, "none"
        lo, hi = oc.ugv_width_m
        x1, x2 = det.box[0], det.box[2]
        touches = x1 <= 2.0 or x2 >= rgb_wh[0] - 2.0
        if dr.width_m is not None and not touches and not (lo <= dr.width_m <= hi):
            self.stats["width_reject"] += 1
            return dr, "width_reject"
        if ref_range is not None and abs(dr.r - ref_range) > oc.depth_agree_tol * max(ref_range, 0.3):
            self.stats["size_disagree"] += 1
            return dr, "size_disagree"
        self.stats["ok"] += 1
        return dr, "ok"
