"""깊이 영상 + 카메라 자세(TF) + 로봇 자세(오도메트리·SLAM) → 3D 지도, 그리고 임무에서 쓰는 질의.

입력
  DepthFrame      깊이 영상(m) + 촬영 시각 + (선택) K  — 깊이 카메라는 팬/틸트 아래 차체에 고정이라 팬/틸트를 쓰지 않는다
  Pose2D          촬영 시각의 차체 자세 — Agent.pose_at(t): SLAM 이 교정한 자세의 이력(오도메트리가 예측, 스캔 정합이 교정)
  RigModel        base_link→깊이 센서(고정) TF, RGB 카메라는 base→팬→틸트→카메라 (config.rig = URDF)
출력
  map             VoxelMap3D (월드 등록된 3D 점유)
  locate_pixel    RGB 영상 한 픽셀이 가리키는 3D 점 — 수광센서의 거리·높이
  beam_blocked    레이저 사선에 장애물이 끼었나
  nav_layer       항법에 더할 2D 장애물 마스크 / forward_clearance 전방 안전 거리
  lidar_check     깊이 점을 LiDAR 와 대조 — 카메라 자세(extrinsic) 오차를 운영 중에 발견
"""
from __future__ import annotations

import math
import warnings
from collections import deque
from dataclasses import dataclass
from typing import Any, Deque, Dict, Optional, Tuple

import numpy as np

from ..config import Config
from ..geom3d import Intrinsics, RigModel, Rt, backproject
from ..types import DepthFrame, Detection, Pose2D, Scan
from .camera import CameraModel
from .voxel_map import VoxelMap3D


@dataclass
class _Shot:
    t: float                 # robot.now() 시계의 촬영 시각
    depth: np.ndarray        # (H, W) float32, m
    intr: Intrinsics
    T: Rt                    # 월드 ← 깊이 광학 프레임
    pose: Pose2D
    pan: float
    tilt: float


class DepthFusion:
    def __init__(self, cfg: Config) -> None:
        self.cfg, self.dc, self.mc = cfg, cfg.depth, cfg.map3d
        self.rig = RigModel(cfg.rig)
        self.map = VoxelMap3D(cfg.arena, cfg.map3d)
        self.rgb = CameraModel(cfg.camera)     # TODO(fisheye): locate_pixel 이 아직 핀홀 광선((u-cx)/f)을 쓴다. 어안 카메라(camera.model=fisheye)로 깊이 지도를 켜려면 self.rgb.angles 로 바꿀 것(지금은 depth 가 꺼져 있다)
        self._recent: Deque[_Shot] = deque(maxlen=8)
        self._last_int = -1e9
        self._res_hist: Deque[float] = deque(maxlen=12)
        self._bad_run = 0
        self.lidar_warned = False
        self.stats: Dict[str, Any] = {"frames": 0, "integrated": 0, "rate_skipped": 0, "bad_frames": 0, "points": 0,
                                      "valid_ratio": None, "lidar_resid_m": None, "lidar_checks": 0, "lidar_bias_alarm": 0,
                                      "locate_ok": 0, "locate_miss": 0}

    # ------------------------------------------------------------------ 입력
    def ingest(self, df: DepthFrame, t_robot: float, pose: Pose2D, scan: Optional[Scan] = None,
               pose_scan: Optional[Pose2D] = None, max_range: Optional[float] = None) -> bool:
        """깊이 프레임 하나. 지도에 쌓았으면 True. (프레임은 항상 최근 목록에 보관 — 픽셀 조회용.)"""
        st = self.stats
        st["frames"] += 1
        d = np.asarray(df.depth, dtype=np.float32)
        if d.ndim != 2 or d.size == 0:
            st["bad_frames"] += 1
            return False
        vr = float(np.mean(np.isfinite(d) & (d > 0)))
        st["valid_ratio"] = round(vr, 3)
        if vr < 0.02:                                   # 거의 다 무효(렌즈 가림·연기·장치 오류) — 지도를 오염시키지 않는다
            st["bad_frames"] += 1
            return False
        intr = Intrinsics.from_K(df.K, d.shape[1], d.shape[0]) if df.K else self.dc.intrinsics(d.shape[1], d.shape[0])
        T = self.rig.optical_in_world(pose.x, pose.y, pose.th, df.pan, df.tilt, "depth")
        self._recent.append(_Shot(t_robot, d, intr, T, pose.copy(), df.pan, df.tilt))
        if t_robot - self._last_int < 1.0 / max(self.mc.integrate_hz, 1e-3):
            st["rate_skipped"] += 1
            return False
        self._last_int = t_robot
        mr = self.dc.max_range if max_range is None else min(self.dc.max_range, max_range)
        pts, _ = backproject(d, intr, T, self.mc.stride, self.dc.min_range, mr, phase=st["integrated"])
        n = self.map.integrate(T.t, pts)
        st["integrated"] += 1
        st["points"] += int(n)
        if self.mc.lidar_check and scan is not None and pose_scan is not None and len(pts):
            self._lidar_check(pts, scan, pose_scan)
        return True

    # ------------------------------------------------------------------ 깊이↔LiDAR 정합 감시
    def _lidar_check(self, pts: np.ndarray, scan: Scan, pose_scan: Pose2D) -> None:
        """LiDAR 평면 높이의 깊이 점을 같은 방위의 LiDAR 거리와 비교한다. 깊이 카메라 설치(위치·방향) 오차는 여기서 잔차로 드러난다."""
        zl = self.rig.base_z + self.rig.lidar_xyz[2]
        band = np.abs(pts[:, 2] - zl) < 0.04
        if band.sum() < 8:
            return
        o = self.rig.lidar_origin(pose_scan.x, pose_scan.y, pose_scan.th)
        q = pts[band]
        dx, dy = q[:, 0] - o[0], q[:, 1] - o[1]
        rd = np.hypot(dx, dy)
        bearing = np.arctan2(dy, dx) - pose_scan.th
        n = len(scan.ranges)
        idx = np.round((bearing - scan.angle_min) / scan.angle_inc).astype(int) % n      # 가장 가까운 빔(1° 간격이면 2 m 에서 ±1.7 cm)
        rl = scan.ranges[idx].astype(float)
        ok = np.isfinite(rl) & (rl > 0.05) & (rl < scan.range_max) & (np.abs(rl - rd) < 0.5)    # 가림·다른 물체로 크게 어긋난 것은 제외
        res = (rd - rl)[ok]
        if len(res) < 6:
            return
        med = float(np.median(res))
        self._res_hist.append(med)
        self.stats["lidar_checks"] += 1
        m = float(np.median(self._res_hist))
        self.stats["lidar_resid_m"] = round(m, 3)
        if abs(m) > self.mc.lidar_check_bias_m and len(self._res_hist) >= 5:
            self._bad_run += 1
            if self._bad_run == 3:
                self.stats["lidar_bias_alarm"] += 1
        else:
            self._bad_run = 0

    @property
    def lidar_misaligned(self) -> bool:
        return self._bad_run >= 3

    # ------------------------------------------------------------------ RGB 픽셀 → 3D 점
    def _nearest(self, t: float) -> Optional[_Shot]:
        best, bd = None, self.mc.sync_max_dt
        for s in self._recent:
            if abs(s.t - t) <= bd:
                best, bd = s, abs(s.t - t)
        return best

    def locate_pixel(self, u: float, v: float, t_rgb: float, pose_rgb: Pose2D, pan: float, tilt: float,
                     max_range: Optional[float] = None) -> Optional[np.ndarray]:
        """RGB 영상 (u, v) 가 가리키는 월드 3D 점. RGB 광선을 깊이 영상에 투영하며 첫 표면을 찾는다(두 카메라의 위치·시각 차이를 함께 처리).
        적절한 시각의 깊이 프레임이 없거나 표면이 안 보이면 None."""
        shot = self._nearest(t_rgb)
        if shot is None:
            self.stats["locate_miss"] += 1
            return None
        T_rgb = self.rig.optical_in_world(pose_rgb.x, pose_rgb.y, pose_rgb.th, pan, tilt, "camera")
        ray = np.array([(u - self.rgb.cx) / self.rgb.f, (v - self.rgb.cy) / self.rgb.fy, 1.0])
        ray /= np.linalg.norm(ray)
        dw = T_rgb.R @ ray
        mr = self.dc.max_range if max_range is None else min(self.dc.max_range, max_range)
        s = np.arange(max(self.dc.min_range - 0.05, 0.1), mr + 0.3, 0.02)
        P = T_rgb.t[None, :] + dw[None, :] * s[:, None]
        Pd = shot.T.inv().apply(P)
        pu, pv, pz = shot.intr.project(Pd)
        H, W = shot.depth.shape
        meas = np.full(len(s), np.nan)
        okp = np.isfinite(pu) & (pu >= 1) & (pu < W - 2) & (pv >= 1) & (pv < H - 2)
        if not okp.any():
            self.stats["locate_miss"] += 1
            return None
        iu, iv = np.round(pu[okp]).astype(int), np.round(pv[okp]).astype(int)
        win = np.stack([shot.depth[iv + dy, iu + dx] for dy in (-1, 0, 1) for dx in (-1, 0, 1)], axis=1).astype(float)
        win[~(np.isfinite(win) & (win > 0))] = np.nan
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", RuntimeWarning)           # 3x3 이 전부 무효인 표본은 nan 으로 둔다
            meas[okp] = np.nanmedian(win, axis=1)
        behind = np.isfinite(meas) & (pz >= meas - 0.02) & (np.abs(pz - meas) < 0.15)
        if not behind.any():
            self.stats["locate_miss"] += 1
            return None
        i = int(np.argmax(behind))
        if i > 0 and np.isfinite(meas[i - 1]):                 # 직전 표본과의 부호 변화로 교차점을 보간
            a, b = pz[i - 1] - meas[i - 1], pz[i] - meas[i]
            k = a / (a - b) if abs(a - b) > 1e-9 else 1.0
            pt = P[i - 1] + (P[i] - P[i - 1]) * float(np.clip(k, 0.0, 1.0))
        else:
            pt = P[i]
        self.stats["locate_ok"] += 1
        return pt

    def det_pixel(self, det: Detection) -> Tuple[float, float]:
        """검출에서 거리를 잴 픽셀: 수광센서가 있으면 그 점, 없으면 박스 중앙."""
        sens = det.extra.get("sensors") if det.extra else None
        if sens:
            return float(sens[0][0]), float(sens[0][1])
        return det.cx, det.cy

    # ------------------------------------------------------------------ 임무용 질의
    def laser_origin(self, pose: Pose2D, pan: float, tilt: float, lateral: float = 0.0, vertical: float = 0.0) -> np.ndarray:
        """레이저 발광부(카메라에서 옆·위로 떨어진 값 포함) 위치."""
        c = self.rig.position(pose.x, pose.y, pose.th, pan, tilt, "camera")
        psi = pose.th + math.radians(pan)
        return c + np.array([-math.sin(psi) * lateral, math.cos(psi) * lateral, vertical])

    def beam_blocked(self, origin: np.ndarray, target: np.ndarray) -> int:
        """레이저 발광부→수광센서 선분 위의 '표면' 복셀 수(이웃 3개 이상 점유 = 면/덩어리). 깊이는 물체의 앞면 한 겹만 보여 주므로
        개수가 아니라 '면에 닿았는가'가 기준이고, 광선 방향으로 늘어선 스테레오 허상(이웃 2개)은 걸러진다."""
        return self.map.segment_blocked(origin, target, 0.05, self.mc.beam_margin_m, thr=self.mc.nav_thr,
                                        min_neighbors=self.mc.nav_min_neighbors)

    def nav_layer(self) -> Optional[np.ndarray]:
        """항법에 더할 2D 장애물 마스크(격자와 같은 모양). 기본은 'LiDAR 가 못 보는 것만'(map3d.nav_only_invisible) —
        LiDAR 가 보는 박스·UGV·벽은 LiDAR 격자(옮겨진 박스 지우기 포함)가 더 믿을 만하고, 깊이 허상이 거기 섞이지 않는다."""
        if not (self.mc.enable and self.mc.use_for_nav):
            return None
        if self.mc.nav_only_invisible:
            # 바닥에서 올라온 낮은 물체만(바닥 바로 위 blind_floor_layers 층에 점유). 구조물이 바뀐 뒤 박스 윗부분만 늦게 지워져 남는
            # '공중에 뜬 잔재'(위쪽 층은 광선이 드물게 지나감)를 장애물로 오인하지 않는다.
            z_hi = self.mc.nav_z_lo + self.mc.blind_floor_layers * self.mc.res
            return self.map.lidar_blind_columns(self.mc.nav_z_lo, z_hi, self.lidar_plane_z, self.mc.lidar_plane_half,
                                                self.mc.nav_thr, self.mc.nav_min_neighbors, self.mc.occ_thr, self.mc.blind_grow_cells)
        return self.map.column_obstacles(self.mc.nav_z_lo, self.mc.nav_z_hi, self.mc.nav_thr, self.mc.nav_min_neighbors)

    @property
    def lidar_plane_z(self) -> float:
        return self.rig.base_z + self.rig.lidar_xyz[2]

    def front_clearance(self, pose: Pose2D, max_d: float = 1.0) -> float:
        """차체 앞 통로의 가장 가까운 (nav_layer 에 해당하는) 장애물까지 거리."""
        col = self.nav_layer()
        return self.map.forward_clearance(pose.x, pose.y, pose.th, self.mc.guard_half_width, self.mc.nav_z_lo, self.mc.nav_z_hi,
                                          max_d, self.mc.nav_thr, self.mc.nav_min_neighbors, col=col)

    def report(self) -> Dict[str, Any]:
        return {**self.stats, **self.map.summary()}
