"""관측 지도(Coverage): '주행할 수 있나'(점유격자)와 별개로 '실제로 봤나'를 기록한다.

점유격자(nav/grid.py)는 미지 영역을 자유로 계획한다(L=0 이 '미지'이면서 '확률 0.5'). 그래서 이 모듈이 두 가지를 따로 가진다.
  SeenMap      LiDAR 가 실제로 훑은 셀(0.10 m). 프런티어(관측한 자유 공간과 못 본 공간의 경계)와 시점의 '새로 볼 면적'을 계산한다.
  BoxModel     점유격자 군집에서 뽑은 박스 가설(40 cm). 완전한 박스뿐 아니라 **한 면만 보인 박스**도 외부 쪽을 자유공간 증거로 정해 중심을 추정한다.
  FaceRec      박스의 면 4개마다 '카메라가 볼 수 있었던 독립 관측 횟수', 가림 횟수, 분류 득표(ObjectMap 에서 동기화), 상태.
면 상태: UNSEEN → VIEWED_NO_DET → (EMPTY | UNCERTAIN | CONFIRMED) / UNREACHABLE / GIVEUP. '방문함'이 아니라 '볼 수 있는 조건에서 봤다'를 기준으로 한다:
  거리 ≤ perception.max_obj_range, 면 법선과 시선 각 ≤ view_max_incidence_deg, 카메라 시야 안, 격자 상 가림 없음.
박스 이동: 박스 군집이 사라지고 그 증거 셀이 자유로 재관측(L<0.3)된 비율이 높으면 '이동함' → 박스·면·ObjectMap 의 옛 면을 지운다.
일시적 가림은 격자에 자유 증거를 만들지 않으므로(광선이 통과해야 L 이 내려간다) 확정 관측을 지우지 않는다.
"""
from __future__ import annotations

import math
from collections import defaultdict
from dataclasses import dataclass, field, replace
from types import SimpleNamespace
from typing import Any, Dict, List, Optional, Set, Tuple

import numpy as np

from ..arena import ArenaGeometry
from ..config import Config
from ..link.base import UavBox
from ..link.schema import server_sector, scoring_sectors
from ..nav.grid import OccupancyGrid
from ..types import Pose2D, Scan
from ..util import wrap
from .camera import CameraModel
from .lidar_faces import FaceMapResult, LidarFaceMap
from .uav_alignment import estimate_uav_translation, transform_uav_boxes

FACE_N = [(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0)]      # E N W S (sim/world.py FACE_N, explore.py FACE_DIRS 와 같다)
UNSEEN, VIEWED_NO_DET, UNCERTAIN, CONFIRMED, EMPTY, UNREACHABLE, GIVEUP = (
    "UNSEEN", "VIEWED_NO_DET", "UNCERTAIN", "CONFIRMED", "EMPTY", "UNREACHABLE", "GIVEUP")
Pt = Tuple[float, float]

PICTURE_HALF_W = 0.18                           # M3 그림(36 cm)의 반폭 m — picture_visible 과 같다
PICTURE_Z = (0.02, 0.38)                        # 그림의 바닥에서의 높이 범위 m — picture_visible 과 같다


def oak_fit_min_dist(cam_z: float, hfov_deg: float, vfov_deg: float, usable: float = 0.92) -> float:
    """OAK(Depth 카메라)가 면의 그림(36 cm)을 '잘리지 않고' 보려면 얼마나 떨어져야 하나(m, 정면에서).
    가로는 반폭 0.18 m, 세로는 그림의 위·아래 끝이 카메라 높이에서 얼마나 떨어졌는지로 잰다. 시야의 usable 비율 안에 들어와야 한다.
    OAK 가 낮게(예 0.10 m) 달려 있으면 세로가 먼저 걸린다 — 가까이 가면 그림 윗부분이 잘린다."""
    h = math.tan(math.radians(float(hfov_deg)) / 2.0 * float(usable))
    v = math.tan(math.radians(float(vfov_deg)) / 2.0 * float(usable))
    up = max(PICTURE_Z[1] - cam_z, cam_z - PICTURE_Z[0], 0.0)
    return max(PICTURE_HALF_W / max(h, 1e-6), up / max(v, 1e-6))


@dataclass
class FaceRec:
    bid: int
    idx: int
    nx: float
    ny: float
    cx: float                                  # 면 중심
    cy: float
    views: int = 0                             # 독립 관측 수(볼 수 있는 조건에서)
    view_poses: List[Tuple[float, float, float]] = field(default_factory=list)
    last_view_t: float = -1e9
    last_cam_yaw: float = 0.0
    occluded_views: int = 0                    # 시야·거리는 맞는데 가려서 못 본 횟수(독립 간격으로)
    last_occl_t: float = -1e9
    arrive_attempts: int = 0                   # 이 면을 보려고 시점에 도착했는데도 못 본 횟수
    det_n: int = 0
    det_votes: Dict[str, float] = field(default_factory=dict)
    det_kind: str = "object"
    exterior_open: bool = True
    suspect: bool = False                      # 박스 증거가 일부 사라졌다(이동 가능성) — 재검증 가치
    id_tries: int = 0                          # OAK 식별 단계(oak_det.identify_fallback)에 이 면을 보러 간 횟수
    reg: Optional[int] = None                  # explore.uav_face_registry: 면 기록부 번호(박스가 사라졌다 다시 생겨도 같은 기록을 이어 받는다)
    recheck: bool = False                      # 팬틸트와 OAK 의 클래스가 엇갈렸다(6장) — 팬틸트로 다시 볼 가치. 새 독립 관측이 생기면 풀린다
    pt_fallback: bool = False                  # oak_det.identify_primary: OAK 가 이 면을 못 봐서 팬틸트로 넘어왔다 — 이때부터 팬틸트 관측·득표를 받는다
    oak_fail: str = ""                         # 넘어온 이유: cropped | no_frames | no_detection | unreached | no_oak_view
    oak_none: int = 0                          # OAK 가 제대로 보고도 검출이 없었던 식별 횟수
    last_id_t: float = -1e9                    # 마지막 OAK 식별을 마친 시각(득표가 장부에 반영될 시간을 두고 폴백 여부를 본다)

    @property
    def key(self) -> Tuple[int, int]:
        return (self.bid, self.idx)

    def top(self) -> Tuple[Optional[str], float, float]:
        """(1위 클래스, 득표 비율 마진, 평균 신뢰도)."""
        if not self.det_votes:
            return None, 0.0, 0.0
        items = sorted(self.det_votes.items(), key=lambda kv: -kv[1])
        tot = sum(v for _, v in items)
        second = items[1][1] if len(items) > 1 else 0.0
        return items[0][0], (items[0][1] - second) / max(tot, 1e-9), items[0][1] / max(self.det_n, 1)


@dataclass
class FaceReg:
    """면 기록부의 한 줄(explore.uav_face_registry). 박스·면 객체(BoxModel/FaceRec)와 따로 살아서, 박스가 한때 사라졌다 같은 자리에 다시 생겨도 '이미 식별한 면'을 알아본다."""
    rid: int
    idx: int                                   # 면 방향 E N W S = 0 1 2 3
    cx: float
    cy: float
    origin: str                                # 처음 기록된 출처: "uav"(UAV 영상의 배치) | "lidar"(지상에서 먼저 본 면)
    t_reg: float
    status: str = "pending"                    # pending(아직 못 풀었다) | identified(분류 확정) | empty(여러 자리에서 봐도 없다) | hidden(이웃·벽에 가려 볼 수 없다)
    cls: Optional[str] = None                  # identified 일 때의 1위 클래스
    t_status: Optional[float] = None
    snap: Dict[str, Any] = field(default_factory=dict)      # 해결된 면의 관측 이력(views·view_poses·시도 횟수…) — 박스가 다시 생길 때 되돌려 준다
    sector: Optional[str] = None               # 서버 섹터 이름(R-S1 … CM). 박스 중심이 속한 섹터 — 기록할 때 정하고 장부를 갱신할 때마다 맞춘다
    census: bool = False                       # explore.uav_census: UAV 초기 영상(첫 배치)에서 뽑은 면이다 — '미탐지'로 올려 두고 식별되면 '탐지'로 바뀐다
    t_detected: Optional[float] = None         # 처음 식별(identified)·빈 면(empty)으로 확정된 시각(미탐지 → 탐지로 바뀐 때)


@dataclass
class BoxModel:
    bid: int
    cx: float
    cy: float
    kind: str                                  # "full" | "face"
    cells: List[Tuple[int, int]]
    t_first: float
    t_last: float
    miss_t: Optional[float] = None
    n_seen: int = 1
    faces: List[FaceRec] = field(default_factory=list)
    sx: float = 0.4
    sy: float = 0.4
    uav_t: Optional[float] = None
    arena_cx: Optional[float] = None
    arena_cy: Optional[float] = None
    arena_sx: Optional[float] = None
    arena_sy: Optional[float] = None
    full_seen: int = 0
    last_full_t: float = -1e9
    free_since: Optional[float] = None
    free_checks: int = 0
    gid: Optional[int] = None                  # explore.lidar_faces: 이 칸이 속한 LiDAR 그룹 번호
    closed: Optional[bool] = None              # 그룹의 닫힌 도형 판정(True 닫힘 | False 열림 | None 보류) — lidar_faces 일 때만
    internal: List[int] = field(default_factory=list)  # 이웃 칸·벽에 가려져 볼 수도 식별할 수도 없는 면 인덱스(E N W S = 0 1 2 3). set 이 아니라 list — 스냅샷·보고서 JSON 이 set 을 못 쓴다
    uav_internal: List[int] = field(default_factory=list)   # explore.uav_outer_faces: UAV 영상의 이웃 상자와 맞닿아 서로 가린 면(LiDAR 가 정한 internal 과 합쳐 쓴다)
    lidar_internal: List[int] = field(default_factory=list)
    uav_frozen: bool = False                   # explore.uav_map_freeze: 첫 UAV 지도로 굳힌 상자 — 영상이 끊겨도 계획에서 빠지지 않는다


# ====================================================================== LiDAR '봤다' 지도
class SeenMap:
    def __init__(self, geo: ArenaGeometry, res: float = 0.10, max_r: float = 4.0) -> None:
        self.geo, self.res, self.max_r = geo, res, max_r
        self.nx, self.ny = int(math.ceil(geo.W / res)), int(math.ceil(geo.H / res))
        self.seen = np.zeros((self.ny, self.nx), dtype=bool)
        xs = (np.arange(self.nx) + 0.5) * res
        ys = (np.arange(self.ny) + 0.5) * res
        X, Y = np.meshgrid(xs, ys)
        self.X, self.Y = X, Y
        self.inside = (X > 0.05) & (X < geo.W - 0.05) & (Y > 0.05) & (Y < geo.H - 0.05)
        self._step = res * 0.8
        self._ts = np.arange(1, int(max_r / self._step) + 1) * self._step
        self.updates = 0

    def update(self, scan: Scan, pose: Pose2D) -> None:
        r = np.asarray(scan.ranges, dtype=float)
        ok = np.isfinite(r) & (r > 0.05)                       # 반사가 없는(무한) 빔은 '봤다'로 치지 않는다 — 장애물 너머를 본 것으로 착각하지 않게
        if not ok.any():
            return
        ang = (scan.angles + pose.th)[ok]
        reach = np.minimum(r[ok], self.max_r)
        m = self._ts[None, :] <= (reach[:, None] + self._step)
        px = pose.x + np.cos(ang)[:, None] * self._ts[None, :]
        py = pose.y + np.sin(ang)[:, None] * self._ts[None, :]
        ix, iy = (px / self.res).astype(int), (py / self.res).astype(int)
        okc = m & (ix >= 0) & (ix < self.nx) & (iy >= 0) & (iy < self.ny)
        self.seen[iy[okc], ix[okc]] = True
        self.updates += 1

    def solid(self, grid: OccupancyGrid) -> np.ndarray:
        """점유(박스·벽) 셀 — 그 안은 볼 수 없으므로 '미관측'에 세지 않는다."""
        occ = grid.occupied()
        out = np.zeros_like(self.seen)
        for dx in (-0.025, 0.025):
            for dy in (-0.025, 0.025):
                iy = ((self.Y + dy - grid.oy) / grid.res).astype(int)
                ix = ((self.X + dx - grid.ox) / grid.res).astype(int)
                okc = (iy >= 0) & (iy < grid.ny) & (ix >= 0) & (ix < grid.nx)
                v = np.zeros_like(out)
                v[okc] = occ[iy[okc], ix[okc]]
                out |= v
        return out

    def unseen(self, solid: np.ndarray) -> np.ndarray:
        return self.inside & ~self.seen & ~solid

    def frontier_mask(self, solid: np.ndarray) -> np.ndarray:
        """관측한 자유 공간 중 4-이웃에 '아직 못 본 공간'이 있는 셀."""
        un = self.unseen(solid)
        free_seen = self.seen & self.inside & ~solid
        nb = np.zeros_like(un)
        nb[1:] |= un[:-1]
        nb[:-1] |= un[1:]
        nb[:, 1:] |= un[:, :-1]
        nb[:, :-1] |= un[:, 1:]
        return free_seen & nb

    def unseen_gain(self, x: float, y: float, solid: np.ndarray, unseen: np.ndarray, rays: int = 36, radius: float = 3.0) -> int:
        """(x, y) 에서 36 방향으로 쏘았을 때(점유에서 멈춤) 지나는 미관측 셀 수 — 시점에서 새로 볼 수 있는 면적의 근사."""
        a = np.linspace(0, 2 * np.pi, rays, endpoint=False)
        ts = np.arange(1, int(radius / self.res) + 1) * self.res
        px = x + np.cos(a)[:, None] * ts[None, :]
        py = y + np.sin(a)[:, None] * ts[None, :]
        ix, iy = (px / self.res).astype(int), (py / self.res).astype(int)
        okc = (ix >= 0) & (ix < self.nx) & (iy >= 0) & (iy < self.ny)
        ixc, iyc = np.clip(ix, 0, self.nx - 1), np.clip(iy, 0, self.ny - 1)
        sol = solid[iyc, ixc] | ~okc
        stop = np.cumsum(sol, axis=1) > 0                         # 첫 점유 이후는 가려진다
        vis = okc & ~stop
        cells = np.unique(iyc[vis] * self.nx + ixc[vis])
        return int(unseen.reshape(-1)[cells].sum())


# ====================================================================== 박스·면 장부
class CoverageMap:
    def __init__(self, cfg: Config, cam: Optional[CameraModel] = None) -> None:
        self.cfg, self.ec, self.geo = cfg, cfg.explore, cfg.arena
        self.cam = cam or CameraModel(cfg.camera)
        self.seen = SeenMap(self.geo, self.ec.seen_res, self.ec.seen_range_m)
        self.boxes: Dict[int, BoxModel] = {}
        self._next = 1
        self.m3_dedupe_last: Dict[str, Any] = {"merged": 0, "detail": []}      # 마지막 M3 집계에서 합친 중복(제출 전 섹터별 중복 제거 결과)
        self._last_refresh = -1e9
        self.stats = {"boxes_created": 0, "boxes_moved": 0, "views": 0, "dropped_faces": 0}
        self.fov_half = self.cam.half_fov_h()                 # 핀홀: atan2(w/2, f) 그대로. usable_hfov_deg 가 있으면 그 절반(7장)
        self._uav_pending: List[Tuple[float, float, float, float, float]] = []
        self._uav_alignment = None
        self.uav_alignment_report: Dict[str, Any] = {}
        self._now = -1e9
        self._layout_stamp: Optional[float] = None
        self._layout_streak = 0
        self._layout_epoch = -1e9
        self.uav_layout_state = "unavailable"
        self.uav_layout_events: List[Dict[str, Any]] = []
        self._static_omap = None  # Last supplied live observations, also used by summary().
        self._lidar: Optional[LidarFaceMap] = LidarFaceMap(cfg) if self.ec.lidar_faces else None
        self._ugv_points: List[Tuple[float, float, float]] = []
        self._cl_units: List[Any] = []
        self.lidar_result: Optional[FaceMapResult] = None
        self.oak_live: bool = False             # manager 가 매 틱 갱신: OAK 를 지금 식별에 쓸 수 있나(드라이버·서비스가 있고 포기하지 않았다)
        self.pt_window: bool = False            # manager 가 갱신: 팬틸트로 폴백 면을 보는 중(위치 닻이 없는 팬틸트 득표를 받는다)
        self.fallback_stats = {"total": 0, "by_reason": {}}
        self.registry: Dict[int, FaceReg] = {}
        self._reg_next = 1
        self.registry_t0: Optional[float] = None
        self.registry_stats = {"registered": 0, "restored": 0, "forgotten": 0, "inner_dropped": 0}
        self._census_t0: Optional[float] = None                  # explore.uav_census: 첫 UAV 배치를 받아들인 시각(이 때부터 uav_census_window_s 안의 면이 초기 영상의 면)
        self.census_stats = {"registered": 0, "converted": 0}
        self._conversions: List[Dict[str, Any]] = []             # 미탐지 → 탐지로 바뀐 사건(매니저가 꺼내 로그에 남긴다)
        self._map_frozen = False                                 # explore.uav_map_freeze: 첫 UAV 배치를 굳혔나
        self._freeze_t0: Optional[float] = None                  # 첫 배치를 받아들인 시각(굳히는 창의 시작)
        self.map_stats = {"ugv_excluded": 0, "inner_faces": 0, "frozen_boxes": 0, "frozen_t": None, "late_boxes_ignored": 0}

    def set_ugv_points(self, points: List[Tuple[float, float, float]]) -> None:
        """UGV 로 알려진 위치 (x, y, 반경). LiDAR 면 지도에서 이 주변을 장애물·면 기록에서 뺀다(항법의 회피와는 별개)."""
        self._ugv_points = [(float(x), float(y), float(r)) for x, y, r in points
                            if math.isfinite(x) and math.isfinite(y) and math.isfinite(r)]

    def _set_layout_state(self, state: str, now: float) -> None:
        if state != self.uav_layout_state:
            self.uav_layout_events.append({"t": round(now, 3), "from": self.uav_layout_state, "to": state})
            del self.uav_layout_events[:-40]
            self.uav_layout_state = state

    def tick_uav_layout(self, now: float) -> str:
        """Watch the geometry image clock, independently of UAV robot detections.

        Missing aerial video is not a mission stop. LiDAR mapping/frontiers and
        ground face observations continue; image-only footprints are suspended.
        Their histories stay available for a later, twice-observed reacquisition.
        """
        self._now = max(self._now, now)
        self._maybe_freeze(self._now)
        if self._layout_stamp is not None and self._now - self._layout_stamp > self.ec.uav_layout_timeout_s:
            self._uav_pending = []
            self._layout_streak = 0
            self._set_layout_state("stale", self._now)
        return self.uav_layout_state

    def _maybe_freeze(self, now: float) -> None:
        """explore.uav_map_freeze: 첫 배치를 받아들인 뒤 창(uav_map_freeze_s)이 지나면 그때까지 모은 UAV 상자를 그 경기의 장애물 지도로 굳힌다.
        상자가 min_boxes 개 모자라면 굳히지 않고 계속 모은다. 영상이 끊긴 채로 창이 지나도(tick 에서 불린다) 굳는다."""
        ec = self.ec
        if not ec.uav_map_freeze or self._map_frozen or self._freeze_t0 is None:
            return
        if now - self._freeze_t0 < ec.uav_map_freeze_s:
            return
        mine = [b for b in self.boxes.values() if b.uav_t is not None]
        if len(mine) < ec.uav_map_freeze_min_boxes:
            return
        if ec.uav_outer_faces:
            self._update_uav_internal()
        for b in mine:
            b.uav_frozen = True
        self._map_frozen = True
        self.map_stats["frozen_boxes"], self.map_stats["frozen_t"] = len(mine), round(float(now), 3)
        self.uav_layout_events.append({"t": round(float(now), 3), "from": self.uav_layout_state, "to": self.uav_layout_state,
                                       "freeze": len(mine)})
        del self.uav_layout_events[:-40]

    def uav_map_ready(self) -> bool:
        """UAV 가 준 장애물 배치를 지금 쓸 수 있나 — 실시간 영상이 살아 있거나, 굳힌 지도가 있다."""
        return self.uav_layout_state == "available" or (self._map_frozen and any(b.uav_frozen for b in self.boxes.values()))

    def _face_occlusions(self, b: BoxModel, idx: int, pool: List[BoxModel]):
        """b 의 idx 번 면(E,N,W,S) 길이 중 이웃 상자가 맞대고 있는 비율(0..1). 이웃은 면 평면에서 ±uav_adjacent_tol_m 안에 가장자리가 있고
        면 바깥쪽에 중심이 있는 상자이며, 반 칸 엇갈린 상자도 겹친 길이만큼 센다."""
        tol = float(self.ec.uav_adjacent_tol_m)
        nx, ny = FACE_N[idx]
        half_n = b.sx / 2 if nx else b.sy / 2
        c_n = b.cx * nx + b.cy * ny
        lo0, hi0 = (b.cy - b.sy / 2, b.cy + b.sy / 2) if nx else (b.cx - b.sx / 2, b.cx + b.sx / 2)
        segs = []
        for v in pool:
            if v is b:
                continue
            vn = (v.sx / 2 if nx else v.sy / 2)
            # 이웃의 안쪽 가장자리(이 면을 향한 쪽)와 이 면 평면의 차이
            near = (v.cx * nx + v.cy * ny) - vn
            plane = c_n + half_n
            gap = near - plane
            if (v.cx - b.cx) * nx + (v.cy - b.cy) * ny <= 0 or abs(gap) >= tol:
                continue
            lo, hi = (v.cy - v.sy / 2, v.cy + v.sy / 2) if nx else (v.cx - v.sx / 2, v.cx + v.sx / 2)
            lo, hi = max(lo, lo0), min(hi, hi0)
            if hi > lo:
                segs.append((lo, hi))
        segs.sort()
        merged, cur = [], None
        for lo, hi in segs:
            if cur is not None and lo <= cur[1]:
                cur[1] = max(cur[1], hi)
            else:
                if cur is not None:
                    merged.append(tuple(cur))
                cur = [lo, hi]
        if cur is not None:
            merged.append(tuple(cur))
        return lo0, hi0, merged

    def _face_cover(self, b: BoxModel, idx: int, pool: List[BoxModel]) -> float:
        lo, hi, intervals = self._face_occlusions(b, idx, pool)
        return sum(y - x for x, y in intervals) / max(hi - lo, 1e-6)

    def _face_on_wall(self, b: BoxModel, idx: int) -> bool:
        """Only physical wall contact excludes a face; reachability is planned separately."""
        nx, ny = FACE_N[idx]
        # Arena boundaries and raw UAV centres share a frame. Locally shifted
        # centres must not turn a real outer face into an alleged wall face.
        cx, cy = self._arena_center(b)
        if not self.ec.exhaustive_scoring_faces:
            return not self.geo.inside(cx + nx * (b.sx / 2 + .30), cy + ny * (b.sy / 2 + .30), .25)
        sx, sy = b.arena_sx or b.sx, b.arena_sy or b.sy
        ex = cx + nx * (sx / 2 + 0.04)
        ey = cy + ny * (sy / 2 + 0.04)
        return not self.geo.inside(ex, ey, 0.0)

    def _update_uav_internal(self) -> None:
        """explore.uav_outer_faces: UAV 상자끼리 맞닿은 면(반 칸 엇갈려 일부만 가린 면 포함, uav_cover_frac 이상)과 벽에 붙은 면을 '안쪽 면'으로 표시한다 — 장부에는 바깥으로 드러난 면만 남는다.
        이웃이 UAV 상자이거나 LiDAR 로 확인한 전체 상자(full)일 때만 센다(일부 면 가설은 중심이 불확실해 이웃으로 치지 않는다)."""
        if not self.ec.uav_outer_faces:
            return
        frac = float(self.ec.uav_cover_frac)
        pool = [b for b in self.boxes.values() if self.active_uav_prior(b) or b.kind == "full"]
        aerial = {}
        if self.ec.exhaustive_scoring_faces:
            # Raw calibrated image geometry determines permanent topology.
            # LiDAR hypotheses remain collision evidence, not proof of contact.
            aerial = {b.bid: replace(b, cx=self._arena_center(b)[0], cy=self._arena_center(b)[1],
                                    sx=b.arena_sx or b.sx, sy=b.arena_sy or b.sy)
                      for b in self.boxes.values() if self.active_uav_prior(b)}
        for b in self.boxes.values():
            if b.uav_t is None:
                continue
            shape, neighbours = (aerial[b.bid], list(aerial.values())) if b.bid in aerial else (b, pool)
            inner = ({i for i in range(4) if self._face_cover(shape, i, neighbours) >= frac or self._face_on_wall(b, i)}
                     if self.box_available(b) else set())
            local_inner = (set(b.internal) - set(b.uav_internal)) | set(b.lidar_internal)
            if sorted(inner) != b.uav_internal:
                self.map_stats["inner_faces"] += len(inner) - len(b.uav_internal)
                b.uav_internal = sorted(inner)
            b.internal = sorted(inner if b.bid in aerial else local_inner | inner)

    def _arena_center(self, b):
        if b.arena_cx is not None and b.arena_cy is not None:
            return b.arena_cx, b.arena_cy
        a = self._uav_alignment
        return b.cx - (a.dx if a else 0.), b.cy - (a.dy if a else 0.)

    def aerial_landmarks(self):
        """Independent calibrated image rectangles for localization (not truth,
        and never SLAM-aligned centres). Require a stable confirmed inventory.
        """
        rows = [(b.arena_cx, b.arena_cy, b.arena_sx or b.sx, b.arena_sy or b.sy) for b in self.boxes.values()
                if self.active_uav_prior(b) and b.arena_cx is not None and b.arena_cy is not None
                and (b.uav_frozen or b.n_seen >= 3)]
        return rows if len(rows) >= 3 else []

    def aerial_face_map(self):
        """Image-derived outer line segments in arena metres. Geometry and
        ground-camera identification are separate; UAV never supplies a class.
        Partial faces retain their exposed segments even if mission policy
        excludes the entire face from scoring/view planning.
        """
        # Form seams in one frame, using only original image geometry. Local
        # LiDAR shifts/shape fits and LiDAR-only boxes cannot alter this map.
        pool = [replace(b, cx=b.arena_cx, cy=b.arena_cy,
                        sx=b.arena_sx or b.sx, sy=b.arena_sy or b.sy)
                for b in self.boxes.values() if self.active_uav_prior(b)
                and b.arena_cx is not None and b.arena_cy is not None]
        rows = []
        for b in pool:
            cx, cy = self._arena_center(b)
            dx, dy = cx - b.cx, cy - b.cy
            for f in b.faces:
                nx, ny = FACE_N[f.idx]
                lo, hi, hidden = self._face_occlusions(b, f.idx, pool)
                exposed, cursor = [], lo
                for a, z in hidden:
                    if a > cursor:
                        exposed.append((cursor, a))
                    cursor = max(cursor, z)
                if cursor < hi:
                    exposed.append((cursor, hi))
                segments = []
                for a, z in exposed:
                    segments.append([[cx + nx * b.sx / 2, a + dy], [cx + nx * b.sx / 2, z + dy]] if nx else
                                    [[a + dx, cy + ny * b.sy / 2], [z + dx, cy + ny * b.sy / 2]])
                rows.append({"box": b.bid, "face": "ENWS"[f.idx], "normal": [nx, ny],
                             "center": [cx, cy], "size": [b.sx, b.sy],
                             "segments": segments, "exposed_fraction": round(sum(z-a for a,z in exposed) / max(hi-lo,1e-6), 3),
                             "wall_inaccessible": self._face_on_wall(b, f.idx),
                             "state": self.face_ledger_state(self.boxes[b.bid], f), "source": "uav_image",
                             "last_image_s": b.uav_t})
        return {"frame": "arena", "units": "m", "faces": rows,
                "assumption": "axis-aligned rectangular footprints; side images require ground camera"}

    def active_uav_prior(self, b: BoxModel) -> bool:
        if b.uav_frozen and b.uav_t is not None:
            return True                                    # 첫 UAV 지도로 굳힌 상자는 영상이 끊겨도 쓴다
        return (self.uav_layout_state == "available" and b.uav_t is not None
                and b.uav_t >= self._layout_epoch
                and 0 <= self._now - b.uav_t <= self.ec.uav_layout_timeout_s)

    def box_available(self, b: BoxModel) -> bool:
        """A local LiDAR observation survives image loss; an aerial guess does not."""
        return b.kind != "uav" or self.active_uav_prior(b)

    def layout_diagnostics(self) -> Dict[str, Any]:
        return {"state": self.uav_layout_state,
                "mode": "uav_assisted" if self.uav_layout_state == "available" else "lidar_camera",
                "age_s": None if self._layout_stamp is None else round(max(0.0, self._now - self._layout_stamp), 3),
                "timeout_s": self.ec.uav_layout_timeout_s,
                "active_priors": sum(self.active_uav_prior(b) for b in self.boxes.values()),
                "suspended_image_only_boxes": sum(not self.box_available(b) for b in self.boxes.values()),
                "local_boxes": sum(b.kind != "uav" for b in self.boxes.values()),
                "map_frozen": self._map_frozen, "frozen_boxes": sum(b.uav_frozen for b in self.boxes.values()),
                "map_stats": dict(self.map_stats),
                "events": list(self.uav_layout_events)}

    # ------------------------------------------------------------------ 입력
    def update_scan(self, scan: Scan, pose: Pose2D) -> None:
        self.seen.update(scan, pose)

    def ingest_uav_boxes(self, hints: Any, now: float, grid: Optional[OccupancyGrid] = None,
                         ugv_xy: Optional[Any] = None) -> None:
        """Accept repeated overhead footprint observations, never side classes or counts.

        Two consistent frames are needed before a new prior is useful. A prior adds
        viewpoints only: it does not mark a face seen, classify it, or alter SLAM.
        """
        if (not math.isfinite(now) or (self._layout_stamp is not None and now <= self._layout_stamp)
                or self._now - now > self.ec.uav_layout_timeout_s):
            return
        self.tick_uav_layout(now)
        if self._layout_streak == 0:
            self._layout_epoch = now
        self._layout_stamp = now
        self._layout_streak += 1
        self._set_layout_state("available" if self._layout_streak >= 2 else "acquiring", now)
        if self.ec.uav_map_freeze and self._freeze_t0 is None and self._layout_streak >= 2:
            self._freeze_t0 = now                                # 굳히는 창의 시작: 일관된 두 번째 프레임을 받아들인 때
        raw = []
        for hint in hints:
            try:
                raw.append(UavBox(*(float(getattr(hint, name)) for name in ("cx", "cy", "sx", "sy")),
                                  float(getattr(hint, "confidence", 1.0))))
            except (TypeError, ValueError, AttributeError):
                continue
        if self.ec.uav_exclude_ugv and ugv_xy:
            # UAV 영상에서 UGV 로 검출된 자리 근처의 상자 후보는 UGV 윗면일 수 있다 — 지도·정렬·면 장부에 넣지 않는다.
            rad = float(self.ec.uav_ugv_exclude_m)
            pts = [(float(p[0]), float(p[1])) for p in ugv_xy if len(p) >= 2 and math.isfinite(p[0]) and math.isfinite(p[1])]
            kept = [h for h in raw if all(math.hypot(h.cx - x, h.cy - y) >= rad for x, y in pts)]
            self.map_stats["ugv_excluded"] += len(raw) - len(kept)
            raw = kept
        if grid is not None:
            local = [SimpleNamespace(kind=kind, cx=cx, cy=cy, sx=sx, sy=sy)
                     for kind, cx, cy, cells, sx, sy in self._clusters(grid) if kind == "full"]
            alignment = estimate_uav_translation(raw, local)
            if alignment is not None:
                old_x, old_y = ((self._uav_alignment.dx, self._uav_alignment.dy)
                                if self._uav_alignment is not None else (0.0, 0.0))
                dx, dy = alignment.dx - old_x, alignment.dy - old_y
                # Only aerial-only geometry moves with this registration;
                # independently observed LiDAR centres are already local.
                for box in self.boxes.values():
                    if box.uav_t is not None and box.kind != "full":
                        box.cx, box.cy = box.cx + dx, box.cy + dy
                        self._position_faces(box)
                self._uav_pending = [(x + dx, y + dy, sx, sy, t) for x, y, sx, sy, t in self._uav_pending]
                self._uav_alignment = alignment
                self.uav_alignment_report = {"dx": round(alignment.dx, 3), "dy": round(alignment.dy, 3),
                                             "matches": alignment.matches, "rms": round(alignment.rms, 3), "t": now}
        hints = transform_uav_boxes(raw, self._uav_alignment)
        if self._map_frozen:
            # 굳힌 지도: 정렬(위 블록)만 갱신하고 상자를 더하거나 옮기지 않는다. 지도에 없던 자리의 검출 수는 진단용으로만 센다.
            for h in hints:
                if not any(math.hypot(b.cx - h.cx, b.cy - h.cy) <= 0.25 for b in self.boxes.values()):
                    self.map_stats["late_boxes_ignored"] += 1
            self._uav_pending = []
            return
        previous = self._uav_pending
        pending = []
        for hint_index, hint in enumerate(hints):
            try:
                cx, cy, sx, sy = (float(getattr(hint, name)) for name in ("cx", "cy", "sx", "sy"))
                confidence = float(getattr(hint, "confidence", 1.0))
            except (TypeError, ValueError, AttributeError):
                continue
            if not all(math.isfinite(v) for v in (cx, cy, sx, sy, confidence)) or confidence < 0.6:
                continue
            if not (0.25 <= sx <= 1.0 and 0.25 <= sy <= 1.0):
                continue
            if not (self.geo.inside(cx - sx / 2, cy - sy / 2, 0.10)
                    and self.geo.inside(cx + sx / 2, cy + sy / 2, 0.10)):
                continue
            pending.append((cx, cy, sx, sy, now))
            consistent = any(0 < now - t <= self.ec.uav_layout_timeout_s and math.hypot(cx - x, cy - y) < 0.16
                             and abs(sx - dx) < 0.16 and abs(sy - dy) < 0.16
                             for x, y, dx, dy, t in previous)
            if not consistent:
                continue
            best = min(self.boxes.values(), key=lambda b: math.hypot(b.cx - cx, b.cy - cy), default=None)
            if best is None or math.hypot(best.cx - cx, best.cy - cy) > 0.25:
                best = BoxModel(self._next, cx, cy, "uav", [], now, now, sx=sx, sy=sy, uav_t=now)
                self._next += 1
                self._make_faces(best)
                self._restore_box(best)                      # 같은 자리에 이미 식별한 면의 기록이 있으면 이어 받는다(explore.uav_face_registry)
                self.boxes[best.bid] = best
                self.stats["boxes_created"] += 1
                self.stats["uav_boxes_created"] = self.stats.get("uav_boxes_created", 0) + 1
            else:
                # A locally observed full footprint is already in the SLAM
                # frame. Aerial world coordinates must not pull it away.
                if best.kind != "full":
                    best.cx = 0.7 * best.cx + 0.3 * cx
                    best.cy = 0.7 * best.cy + 0.3 * cy
                    best.sx, best.sy = 0.7 * best.sx + 0.3 * sx, 0.7 * best.sy + 0.3 * sy
                best.uav_t, best.t_last, best.miss_t = now, now, None
                best.free_since, best.free_checks = None, 0
                self._position_faces(best)
            # Submission sectors remain in the calibrated arena frame even
            # when navigation priors are registered into a drifting SLAM frame.
            best.arena_cx, best.arena_cy = raw[hint_index].cx, raw[hint_index].cy
            best.arena_sx, best.arena_sy = raw[hint_index].sx, raw[hint_index].sy
            self._register_box(best, "uav", now)             # UAV 영상으로 받아들인 박스의 면은 모두 기록부에 올린다
            if grid is not None:
                # Perimeter samples let later LiDAR free-space evidence invalidate
                # a removed/misdetected footprint even after UAV video stops.
                best.cells = list({grid.to_cell(best.cx + nx * best.sx / 2 + (-ny) * d,
                                                best.cy + ny * best.sy / 2 + nx * d)
                                   for nx, ny in FACE_N
                                   for d in np.linspace(-0.12, 0.12, 5)
                                   if grid.in_bounds(*grid.to_cell(best.cx + nx * best.sx / 2 + (-ny) * d,
                                                                  best.cy + ny * best.sy / 2 + nx * d))})
        self._uav_pending = pending
        self._update_uav_internal()

    def visibility_occupancy(self, grid: OccupancyGrid) -> np.ndarray:
        """Overhead priors can occlude side views without overwriting the SLAM map."""
        occ = grid.occupied().copy()
        prior = np.zeros_like(occ)
        for b in self.boxes.values():
            if b.uav_t is None or not (self.active_uav_prior(b) or b.kind == "full"):
                continue
            iy0, ix0 = grid.to_cell(b.cx - b.sx / 2, b.cy - b.sy / 2)
            iy1, ix1 = grid.to_cell(b.cx + b.sx / 2, b.cy + b.sy / 2)
            prior[max(0, iy0):min(grid.ny, iy1 + 1), max(0, ix0):min(grid.nx, ix1 + 1)] = True
        # The aerial calibration/SLAM registration is approximate. Strong local
        # free-space evidence wins over a coarse footprint prior; otherwise a
        # small offset duplicates obstacle edges and closes real corridors.
        free_evidence = grid.L <= 3 * grid.L_FREE
        self.stats["uav_prior_free_overrides"] = int(np.count_nonzero(prior & free_evidence & ~occ))
        return occ | (prior & ~free_evidence)

    def planning_cost_maps(self, grid: OccupancyGrid) -> Tuple[np.ndarray, np.ndarray]:
        """Inflate a temporary union of SLAM obstacles and confirmed UAV priors.

        Navigation waypoints use these masks, but sensor evidence in grid.L is
        untouched and can subsequently confirm or contradict the prior.
        """
        nav = self.cfg.nav
        dist = grid.distance_field(self.visibility_occupancy(grid), nav.inflate_soft)
        blocked = dist <= nav.inflate_hard
        if grid.keepout is not None:
            blocked |= grid.keepout  # Planning policy must not become a perceived cube.
        cost = 1.0 + nav.soft_weight * np.clip((nav.inflate_soft - dist) /
                                               max(nav.inflate_soft - nav.inflate_hard, 1e-6), 0, 1) ** 2
        return blocked, cost.astype(np.float32)

    def confirmed_static_footprints(self, now: float, omap: Any = None) -> List[BoxModel]:
        """Solid geometry supported by LiDAR plus an independent image source.

        Used only to reject impossible LiDAR-only motion hypotheses *inside*
        a solid obstacle. The existing UAV branch remains unchanged. In the
        opt-in PDF mission, recent high-confidence ground object/target faces
        can instead corroborate repeated, recently full LiDAR geometry. A
        remembered class alone, a partial edge, or an actual UGV cannot do so.

        Retain the last supplied live ObjectMap for summary calls; its original
        observation timestamps are checked every time, never refreshed here.
        """
        if omap is not None:
            self._static_omap = omap
        ground_faces = getattr(self._static_omap, "faces", ())
        classes = {"object": set(self.cfg.perception.object_classes),
                   "target": set(self.cfg.perception.target_classes)}
        result = []
        for b in self.boxes.values():
            if b.kind != "full" or b.n_seen < 3 or b.full_seen < 3:
                continue
            if b.uav_t is not None and (b.uav_frozen or 0 <= now - b.uav_t <= 12.0):
                result.append(b)
                continue
            if not self.ec.complete_m3_faces:
                continue
            if (not 0 <= now - b.last_full_t <= max(2.0, 2 * self.ec.box_refresh_s)
                    or b.miss_t is not None or b.free_checks or b.free_since is not None
                    or any(f.suspect for f in b.faces)):
                continue
            corroborated = False
            for of in ground_faces:
                if (of.kind not in classes or of.n < self.cfg.perception.min_votes
                        or not 0 <= now - of.last_t <= 5.0 or not of.votes):
                    continue
                votes = sorted(of.votes.items(), key=lambda item: -item[1])
                cls, top = votes[0]
                margin = (top - (votes[1][1] if len(votes) > 1 else 0.0)) / max(sum(of.votes.values()), 1e-9)
                if (cls not in classes[of.kind] or not math.isfinite(top) or not math.isfinite(margin)
                        or top / of.n < .70 or margin < .50):
                    continue
                norm = math.hypot(of.vx, of.vy)
                if not math.isfinite(norm) or norm < .5:
                    continue
                for f in b.faces:
                    if (f.views >= 2 and 0 <= now - f.last_view_t <= 5.0
                            and math.hypot(of.x - f.cx, of.y - f.cy) <= .12
                            and (of.vx * f.nx + of.vy * f.ny) / norm >= math.cos(math.pi / 6)):
                        corroborated = True
                        break
                if corroborated:
                    break
            if corroborated:
                result.append(b)
        return result

    # ------------------------------------------------------------------ 박스 추출
    def _lidar_clusters(self, grid: OccupancyGrid) -> List[Tuple[str, float, float, List[Tuple[int, int]], float, float]]:
        """explore.lidar_faces: 점유격자에서 면(직선 ≥ 40 cm)을 먼저 찾고 40 cm 칸 단위 박스 가설로 돌려준다. 칸 i 의 상세는 self._cl_units[i]."""
        assert self._lidar is not None
        res = self._lidar.update(grid, self._ugv_points, self._now)
        self.lidar_result = res
        by_gid = {g.gid: g.cells for g in res.groups}
        size, out, units = self.geo.box_size, [], []
        for u in res.units:
            half = size / 2 + 0.06
            cells = [c for c in by_gid.get(u.gid, ())
                     if abs(grid.to_world(*c)[0] - u.cx) <= half and abs(grid.to_world(*c)[1] - u.cy) <= half]
            out.append((u.kind, u.cx, u.cy, cells, size, size))
            units.append(u)
        self._cl_units = units
        return out

    def _clusters(self, grid: OccupancyGrid) -> List[Tuple[str, float, float, List[Tuple[int, int]], float, float]]:
        if self._lidar is not None:
            return self._lidar_clusters(grid)
        self._cl_units = []
        occ = (grid.L > grid.L_THR) & ~grid.static
        ys, xs = np.nonzero(occ)
        wm = 0.14
        cells = set()
        for iy, ix in zip(ys.tolist(), xs.tolist()):
            x, y = grid.to_world(iy, ix)
            if wm < x < self.geo.W - wm and wm < y < self.geo.H - wm:
                cells.add((iy, ix))
        out, seen = [], set()
        res = grid.res
        for c in cells:
            if c in seen:
                continue
            comp, st = [], [c]
            seen.add(c)
            while st:
                cy, cx = st.pop()
                comp.append((cy, cx))
                for dy in (-1, 0, 1):
                    for dx in (-1, 0, 1):
                        n = (cy + dy, cx + dx)
                        if n in cells and n not in seen:
                            seen.add(n)
                            st.append(n)
            if len(comp) < 5:
                continue
            wx = [grid.to_world(*p) for p in comp]
            xsw, ysw = [p[0] for p in wx], [p[1] for p in wx]
            ex, ey = max(xsw) - min(xsw) + res, max(ysw) - min(ysw) + res
            size, tol = self.geo.box_size, 0.12
            sx = min((size, 2 * size), key=lambda v: abs(ex - v))
            sy = min((size, 2 * size), key=lambda v: abs(ey - v))
            if abs(ex - sx) <= tol and abs(ey - sy) <= tol and min(sx, sy) == size:
                out.append(("full", 0.5 * (max(xsw) + min(xsw)), 0.5 * (max(ysw) + min(ysw)), comp, sx, sy))
                continue
            for a_ext, b_ext, along_x in ((ex, ey, True), (ey, ex, False)):
                length = min((size, 2 * size), key=lambda v: abs(a_ext - v))
                if abs(a_ext - length) <= tol and b_ext <= 0.22:
                    pc = self._face_cluster_center(grid, xsw, ysw, along_x)
                    if pc is not None:
                        out.append(("face", pc[0], pc[1], comp, length if along_x else size, size if along_x else length))
                    break
        return out

    def _face_cluster_center(self, grid: OccupancyGrid, xs: List[float], ys: List[float], along_x: bool) -> Optional[Pt]:
        """한 면만 보인 군집: 외부(광선이 지나간 쪽 = L 이 더 낮은 쪽)를 정해 면에서 0.2 m 안쪽을 중심으로."""
        a = np.array(xs if along_x else ys)
        b = np.array(ys if along_x else xs)
        b0 = float(b.mean())
        amin, amax = float(a.min()), float(a.max())

        def side_L(sgn: float) -> float:
            vals = []
            for aa in np.linspace(amin, amax, 7):
                px, py = (aa, b0 + sgn * 0.17) if along_x else (b0 + sgn * 0.17, aa)
                iy, ix = grid.to_cell(px, py)
                if grid.in_bounds(iy, ix):
                    vals.append(float(grid.L[iy, ix]))
            return float(np.mean(vals)) if vals else 0.0
        lp, lm = side_L(+1.0), side_L(-1.0)
        if abs(lp - lm) < 0.5:
            return None                                                # 어느 쪽이 바깥인지 모르면 만들지 않는다(더 관측되면 만들어진다)
        interior = -1.0 if lp < lm else +1.0                          # 바깥 = 더 낮은 쪽, 안쪽 = 반대
        ac, bc = 0.5 * (amin + amax), b0 + interior * 0.2
        return (ac, bc) if along_x else (bc, ac)

    def _apply_unit(self, bx: BoxModel, unit: Any) -> None:
        """LiDAR 면 지도의 칸 정보(그룹 번호·닫힘 판정·안쪽 면)를 박스에 붙인다. lidar_faces 가 꺼져 있으면 unit=None 이라 아무 일도 없다."""
        if unit is None:
            return
        bx.gid, bx.closed = unit.gid, (unit.closed if unit.gid is not None else None)
        bx.lidar_internal = sorted(set(unit.internal))
        bx.internal = sorted(set(bx.uav_internal) if self.ec.exhaustive_scoring_faces and self.active_uav_prior(bx)
                             else set(bx.lidar_internal) | set(bx.uav_internal))

    def _make_faces(self, bx: BoxModel) -> None:
        bx.faces = [FaceRec(bx.bid, i, nx, ny, bx.cx + nx * bx.sx / 2, bx.cy + ny * bx.sy / 2)
                    for i, (nx, ny) in enumerate(FACE_N)]

    @staticmethod
    def _position_faces(bx: BoxModel) -> None:
        for f in bx.faces:
            f.cx, f.cy = bx.cx + f.nx * bx.sx / 2, bx.cy + f.ny * bx.sy / 2

    # ------------------------------------------------------------------ 주기 갱신(박스·이동·분류 동기화)
    def refresh(self, grid: OccupancyGrid, omap: Any, now: float, force: bool = False) -> None:
        self.tick_uav_layout(now)
        if not force and now - self._last_refresh < self.ec.box_refresh_s:
            return
        self._last_refresh = now
        cl = self._clusters(grid)
        units = list(self._cl_units)
        matched: Set[int] = set()
        for ci, (kind, cx, cy, cells, sx, sy) in enumerate(cl):
            unit = units[ci] if ci < len(units) else None
            best, bd = None, 0.25
            for b in self.boxes.values():
                d = math.hypot(b.cx - cx, b.cy - cy)
                if d < bd:
                    best, bd = b, d
            if best is None:
                bx = BoxModel(self._next, cx, cy, kind, cells, now, now, sx=sx, sy=sy,
                              full_seen=1 if kind == "full" else 0,
                              last_full_t=now if kind == "full" else -1e9)
                self._next += 1
                self._make_faces(bx)
                self._apply_unit(bx, unit)
                self._restore_box(bx)
                self.boxes[bx.bid] = bx
                self.stats["boxes_created"] += 1
                matched.add(bx.bid)
            else:
                k = (1.0 if best.kind == "face" and kind == "full" else 0.3)
                if kind == "face" and self.active_uav_prior(best):
                    k = 0.0
                best.cx += (cx - best.cx) * k
                best.cy += (cy - best.cy) * k
                if k > 0 and (kind == "full" or best.kind == "face"):
                    best.sx, best.sy = sx, sy
                if self.ec.exhaustive_scoring_faces and self.active_uav_prior(best):
                    # A merged/partial scan must not expand a photographed cube
                    # into its wall passage. Apply the globally registered
                    # image geometry rather than a noisy individual cluster.
                    ax, ay = self._arena_center(best)
                    a = self._uav_alignment
                    ax += a.dx if a else 0.
                    ay += a.dy if a else 0.
                    best.cx, best.cy = ax, ay
                    best.sx, best.sy = best.arena_sx or best.sx, best.arena_sy or best.sy
                best.kind = "full" if (best.kind == "full" or kind == "full") else "face"
                best.cells, best.t_last, best.miss_t = cells, now, None
                best.free_since, best.free_checks = None, 0
                best.n_seen += 1
                if kind == "full":
                    best.full_seen += 1
                    best.last_full_t = now
                self._position_faces(best)
                self._apply_unit(best, unit)
                for f in best.faces:
                    f.suspect = False
                matched.add(best.bid)
        for bid in list(self.boxes):
            if bid in matched:
                continue
            b = self.boxes[bid]
            if self.active_uav_prior(b) and not b.uav_frozen:
                b.free_since, b.free_checks = None, 0
                continue
            if b.miss_t is None:
                b.miss_t = now
            fr = self._free_frac(grid, b)
            interior_free, nearby_occupied = self._interior_evidence(grid, b)
            contradicted = (fr >= self.ec.contradict_free_frac
                            and interior_free >= self.ec.contradict_free_frac and not nearby_occupied)
            if contradicted:
                if b.free_since is None:
                    b.free_since = now
                b.free_checks += 1
                if b.free_checks >= 3 and now - b.free_since >= 2.0:
                    self._drop_box(b, omap)
            else:
                b.free_since, b.free_checks = None, 0
            if fr > 0.25 and bid in self.boxes:
                for f in b.faces:
                    f.suspect = True                                   # 일부 증거가 사라졌다 — 재검증 가치(삭제는 하지 않는다)
        occ = self.visibility_occupancy(grid)
        for b in self.boxes.values():
            for f in b.faces:
                ex, ey = f.cx + f.nx * 0.30, f.cy + f.ny * 0.30
                iy, ix = grid.to_cell(ex, ey)
                f.exterior_open = (self.geo.inside(ex, ey, 0.25) and grid.in_bounds(iy, ix) and not occ[iy, ix]
                                   and f.idx not in b.internal)
                if self.ec.exhaustive_scoring_faces and self.active_uav_prior(b):
                    f.exterior_open = f.idx not in b.internal
        if omap is not None:
            self._sync_detections(omap)
        self._update_registry(now)
        self.review_fallbacks(now)

    def _free_frac(self, grid: OccupancyGrid, b: BoxModel) -> float:
        if not b.cells:
            return 0.0
        n = sum(1 for iy, ix in b.cells if grid.L[iy, ix] < -0.3)
        return n / len(b.cells)

    @staticmethod
    def _interior_evidence(grid: OccupancyGrid, b: BoxModel) -> Tuple[float, bool]:
        """A cleared thin edge alone can be SLAM jitter, not a moved box.

        Require rays through the former solid interior and no nearby occupied
        fragment. Unknown interior and a fragmented/shifted edge keep the record.
        Navigation still uses the live grid, never these remembered hypotheses.
        """
        xs = np.linspace(b.cx - b.sx / 2 + .07, b.cx + b.sx / 2 - .07, 5)
        ys = np.linspace(b.cy - b.sy / 2 + .07, b.cy + b.sy / 2 - .07, 5)
        cells = {grid.to_cell(x, y) for x in xs for y in ys}
        vals = [grid.L[iy, ix] for iy, ix in cells if grid.in_bounds(iy, ix)]
        fraction = sum(v < -.3 for v in vals) / max(len(cells), 1)
        iy0, ix0 = grid.to_cell(b.cx - b.sx / 2 - .10, b.cy - b.sy / 2 - .10)
        iy1, ix1 = grid.to_cell(b.cx + b.sx / 2 + .10, b.cy + b.sy / 2 + .10)
        region = grid.L[max(0, iy0):min(grid.ny, iy1 + 1), max(0, ix0):min(grid.nx, ix1 + 1)]
        return fraction, bool(np.any(region > grid.L_THR))

    def _drop_box(self, b: BoxModel, omap: Any) -> None:
        self.boxes.pop(b.bid, None)
        self.stats["boxes_moved"] += 1
        # A disappearing partial hypothesis cannot invalidate independent
        # camera classifications. Only a repeatedly resolved full footprint,
        # now repeatedly cleared through its interior, proves an object moved.
        if b.kind == "full" and b.full_seen >= 3 and omap is not None and hasattr(omap, "drop_box_faces"):
            self.stats["dropped_faces"] += omap.drop_box_faces(b.faces)
            self._registry_forget(b)                           # 움직였다는 것이 입증된 박스의 기록은 더 이상 믿지 않는다

    def _sync_detections(self, omap: Any) -> None:
        """ObjectMap 의 원시 면 득표를 장부의 면에 붙인다(위치 0.25 m 안 + 법선 내적 > 0.5)."""
        for b in self.boxes.values():
            for f in b.faces:
                f.det_n, f.det_votes = 0, {}
        assignments = []
        exact_votes = defaultdict(int)
        submittable = getattr(omap, "submittable", None)
        for of in omap.faces:
            if submittable is not None and not submittable(of):
                continue                                   # OAK 로만 본 면은 팬틸트로 확인될 때까지 장부에 올리지 않는다(6-4)
            nv = math.hypot(of.vx, of.vy) or 1.0
            best, bd = None, 0.25
            geometry_key = getattr(of, "geometry_key", None)
            if geometry_key is not None:
                anchored = self.get_face(geometry_key)
                if (anchored is not None and math.hypot(anchored.cx-of.x, anchored.cy-of.y) < .25
                        and (not self.ec.uav_outer_faces or anchored.idx not in self.boxes[anchored.bid].internal)
                        and (anchored.nx*of.vx+anchored.ny*of.vy)/nv > .5):
                    best = anchored
            for b in (() if geometry_key is not None else self.boxes.values()):
                if abs(b.cx - of.cx) > 0.5 or abs(b.cy - of.cy) > 0.5:
                    continue
                for f in b.faces:
                    if self.ec.uav_outer_faces and f.idx in b.internal:
                        continue                       # 안쪽 면에는 득표를 배정하지 않는다
                    d = math.hypot(f.cx - of.x, f.cy - of.y)
                    if d < bd and (f.nx * of.vx + f.ny * of.vy) / nv > 0.5:
                        best, bd = f, d
            if best is not None:
                assignments.append((of, best, geometry_key is not None))
                if geometry_key is not None:
                    exact_votes[best.key] += of.n
        for of, best, exact in assignments:
            # A sufficiently observed, explicitly matched plane owns its vote
            # pool. Nearby legacy points may have a viewing direction mistaken
            # for a surface normal; blending them can flip an adjacent photo's
            # class. Keep those observations in ObjectMap, but do not mix their
            # weaker spatial association into this measured plane's evidence.
            if not exact and exact_votes[best.key] >= self.cfg.perception.min_votes:
                continue
            best.det_n += of.n
            best.det_kind = of.kind
            for c, v in of.votes.items():
                best.det_votes[c] = best.det_votes.get(c, 0.0) + v

    # ------------------------------------------------------------------ 가시성(기록·계획 공용)
    def face_visible(self, vx: float, vy: float, vth: Optional[float], f: FaceRec, occ: np.ndarray, grid: OccupancyGrid,
                     rmax: Optional[float] = None, fov_extra: float = 0.0) -> Tuple[bool, str]:
        """(x, y) 의 카메라가 면 f 를 '분류 가능하게' 볼 수 있나. vth=None 이면 방위 검사 생략(제자리 회전). 이유: ok|range|incidence|fov|occluded."""
        rmax = self.cfg.perception.max_obj_range if rmax is None else rmax
        dx, dy = f.cx - vx, f.cy - vy
        d = math.hypot(dx, dy)
        if d < 0.4 or d > rmax:
            return False, "range"
        cosi = -(dx * f.nx + dy * f.ny) / d
        if cosi < math.cos(math.radians(self.ec.view_max_incidence_deg)):
            return False, "incidence"
        if vth is not None and abs(wrap(math.atan2(dy, dx) - vth)) > self.fov_half * 0.92 + fov_extra:
            return False, "fov"
        ex, ey = f.cx + f.nx * 0.06, f.cy + f.ny * 0.06
        n = max(int(d / (grid.res * 0.5)), 2)
        for i in range(1, n):
            t = i / n
            px, py = vx + (ex - vx) * t, vy + (ey - vy) * t
            if math.hypot(px - ex, py - ey) < 0.12:
                break
            iy, ix = grid.to_cell(px, py)
            if grid.in_bounds(iy, ix) and occ[iy, ix]:
                return False, "occluded"
        return True, "ok"

    def picture_visible(self, vx: float, vy: float, vth: Optional[float], f: FaceRec,
                        occ: np.ndarray, grid: OccupancyGrid) -> Tuple[bool, str]:
        """A complete 36 cm M3 picture fits a neutral-tilt camera view.

        Full-image inspection applies to exhaustive scoring coverage and the
        optional four-photo protocol. Geometry never supplies a class label.
        ``vth=None`` asks whether aiming at this face during a spin can work.
        """
        heading = math.atan2(f.cy-vy, f.cx-vx) if vth is None else vth
        ok, why = self.face_visible(vx, vy, heading, f, occ, grid, rmax=1.4)
        if not ok:
            return ok, why
        distance = math.hypot(f.cx-vx, f.cy-vy)
        incidence = self.ec.view_max_incidence_deg if self.exhaustive_face(f) else 55.0
        if ((vx-f.cx)*f.nx + (vy-f.cy)*f.ny) / distance < math.cos(math.radians(incidence)):
            return False, "incidence"
        c, s = math.cos(heading), math.sin(heading)
        for offset in (-.18, 0., .18):
            px, py = f.cx + f.nx*.002 - f.ny*offset, f.cy + f.ny*.002 + f.nx*offset
            dx, dy = px-vx, py-vy
            if self.exhaustive_face(f) and self._picture_edge_occluded(vx, vy, px, py, f):
                return False, "occluded"
            depth, left = dx*c + dy*s, -dx*s + dy*c
            if depth <= .08:
                return False, "fov"
            for z in (.02, .38):
                uv = self.cam.project_point(depth, left, z-self.cfg.robot.cam_height)      # 핀홀: cx - f·left/depth 그대로. 어안은 방향 투영, 영역 밖이면 None
                # ObjectMap rejects detections within six pixels of an edge.
                if uv is None or not (8 < uv[0] < self.cam.w-8 and 8 < uv[1] < self.cam.h-8):
                    return False, "fov"
            # A visible centre ray does not establish that both picture edges
            # are clear. Stop just short of the observed face's own grid cells.
            length = math.hypot(dx, dy)
            for along in np.arange(grid.res*.5, max(0., length-grid.res*1.5), grid.res*.5):
                iy, ix = grid.to_cell(vx+dx*along/length, vy+dy*along/length)
                if grid.in_bounds(iy, ix) and occ[iy, ix]:
                    if self.exhaustive_face(f):
                        # A grazing ray intersects the target's rasterized skin
                        # before its measured plane. Suppress only those cells;
                        # other observed boxes are checked continuously above.
                        b = self.boxes[f.bid]
                        gx, gy = grid.to_world(iy, ix)
                        own_skin = (abs(gx-b.cx) <= b.sx/2 + grid.res/2
                                    and abs(gy-b.cy) <= b.sy/2 + grid.res/2
                                    and abs((gx-f.cx)*f.nx + (gy-f.cy)*f.ny) <= grid.res)
                        if own_skin:
                            continue
                    return False, "occluded"
        return True, "ok"

    def _picture_edge_occluded(self, vx, vy, px, py, face) -> bool:
        """Continuous intersection with other observed boxes, never world truth."""
        for b in self.boxes.values():
            if b.bid == face.bid or not self.box_available(b) or not (b.kind == "full" or self.active_uav_prior(b)):
                continue
            lo, hi = 0., 1.
            for origin, delta, center, size in ((vx, px-vx, b.cx, b.sx), (vy, py-vy, b.cy, b.sy)):
                if abs(delta) < 1e-9:
                    if not center-size/2 <= origin <= center+size/2:
                        hi = -1.
                        break
                else:
                    a, z = sorted(((center-size/2-origin)/delta, (center+size/2-origin)/delta))
                    lo, hi = max(lo, a), min(hi, z)
            if max(lo, 1e-6) < min(hi, 1.-1e-6):
                return True
        return False

    def record_view(self, pose: Pose2D, pan_deg: float, grid: OccupancyGrid, now: float) -> int:
        """카메라 프레임 하나 처리: 지금 실제로 볼 수 있는 면에 독립 관측을 쌓는다. 반환: 새로 센 관측 수."""
        occ = self.visibility_occupancy(grid)
        th_cam = pose.th + math.radians(pan_deg)
        n_new = 0
        outer = self.ec.uav_outer_faces
        for b in self.boxes.values():
            if not self.box_available(b):
                continue
            if abs(b.cx - pose.x) > 3.0 or abs(b.cy - pose.y) > 3.0:
                continue
            for f in b.faces:
                if outer and f.idx in b.internal:
                    continue                           # 맞닿은·벽에 붙은 면은 볼 수 없다 — 이웃 상자 옆면을 '봤는데 없었다'로 쌓아 빈 면으로 확정하지 않는다(uav_outer_faces)
                if not self.pt_allowed(f):             # identify_primary: OAK 가 맡은 면은 팬틸트가 본 것을 관측으로 세지 않는다(검출이 버려지는데 '봤는데 없었다'만 쌓이면 빈 면으로 잘못 확정된다)
                    continue
                ok, why = (self.picture_visible(pose.x, pose.y, th_cam, f, occ, grid)
                           if self.requires_full_view(f) else self.face_visible(pose.x, pose.y, th_cam, f, occ, grid))
                if not ok and why != "occluded":
                    continue
                if not self._independent(f.view_poses[-1] if f.view_poses else None, pose, th_cam, f.last_cam_yaw,
                                         f.last_view_t if ok else f.last_occl_t, now):
                    continue
                if ok:
                    f.views += 1
                    f.view_poses.append((pose.x, pose.y, pose.th))
                    del f.view_poses[:-6]
                    f.last_view_t, f.last_cam_yaw = now, th_cam
                    f.recheck = False                  # 팬틸트로 다시 봤다(6장: 카메라 간 불일치 재확인)
                    n_new += 1
                else:
                    f.occluded_views += 1
                    f.last_occl_t = now
        self.stats["views"] += n_new
        return n_new

    def _independent(self, last: Optional[Tuple[float, float, float]], pose: Pose2D, th_cam: float, last_cam: float,
                     last_t: float, now: float) -> bool:
        """독립 관측: 이전 관측과 시간(≥0.5 s) 간격이 있고, 로봇이 움직였거나(≥0.15 m) **카메라 방향이 바뀌었을 때(≥12°, 팬 스윕 포함)**."""
        ec = self.ec
        if last is None or now - last_t >= 3.0:
            return True
        if now - last_t < ec.view_min_gap_s:
            return False
        moved = math.hypot(pose.x - last[0], pose.y - last[1])
        turned = abs(math.degrees(wrap(th_cam - last_cam)))
        return moved >= ec.view_min_gap_m or turned >= ec.view_min_gap_deg

    # ------------------------------------------------------------------ 상태·정보 가치
    def photo_box(self, b: BoxModel) -> bool:
        """Recognise a four-picture cube only from confident ground detections.

        Aerial geometry, model names and referee object counts are never used.
        The PDF four-photo protocol is opt-in for conforming competition scenes.
        """
        return (self.ec.complete_m3_faces and self.m3_candidate_box(b) and any(
            f.det_kind == "object" and f.det_n >= self.cfg.perception.min_votes
            and f.top()[1] >= self.ec.confirm_min_margin and f.top()[2] >= .65
            for f in b.faces))

    def picture_pending(self, f: FaceRec) -> bool:
        b = self.boxes.get(f.bid)
        if b is None or not self.photo_box(b) or f.idx in b.internal:
            return False
        return not (f.det_kind == "object" and f.det_n >= self.cfg.perception.min_votes
                    and f.top()[1] >= self.ec.confirm_min_margin and f.top()[2] >= .65)

    def exhaustive_face(self, f: FaceRec) -> bool:
        b = self.boxes.get(f.bid)
        return bool(self.ec.exhaustive_scoring_faces and b is not None and self.m3_candidate_box(b)
                    and f.idx not in b.internal
                    and (self.cfg.team is None or server_sector(self.box_sector(b) or "") in scoring_sectors(self.cfg.team)))

    def requires_full_view(self, f: FaceRec) -> bool:
        return self.exhaustive_face(f) or self.picture_pending(f)

    def face_state(self, f: FaceRec) -> str:
        ec = self.ec
        cls, margin, mconf = f.top()
        if self.picture_pending(f):
            # Every vertical face of an observed photo cube has a picture.
            # Failed viewpoints are retried elsewhere; they cannot prove EMPTY.
            return UNCERTAIN if f.det_n else UNSEEN
        if f.det_kind == "target" and f.det_n >= 2:
            return CONFIRMED                                          # 지정표적 면은 M2 담당 — M3 에서 다시 볼 필요 없음
        if f.det_n >= self.cfg.perception.min_votes and mconf >= 0.45 and margin >= ec.confirm_min_margin:
            return CONFIRMED
        if f.arrive_attempts >= ec.attempts_giveup and not self.exhaustive_face(f):
            return GIVEUP                                             # 세 번 가서 봤는데도 확정이 안 된다 — 더는 시간을 쓰지 않는다
        if f.det_n > 0:
            return UNCERTAIN
        if not f.exterior_open and f.views == 0:
            return UNREACHABLE
        if f.views >= ec.empty_min_views and len({(round(p[0], 1), round(p[1], 1)) for p in f.view_poses}) >= ec.empty_min_poses:
            return EMPTY
        return VIEWED_NO_DET if f.views > 0 else UNSEEN

    def face_weight(self, f: FaceRec) -> float:
        """이 면을 (더) 보는 것의 정보 가치 0~1."""
        st = self.face_state(f)
        if self.picture_pending(f):
            return self.ec.m3_missing_face_weight / (1.0 + .2 * min(f.arrive_attempts, 5))
        resolved = st in (CONFIRMED, EMPTY) or self._reg_resolved(f)       # 기록부가 '식별함·빈 면'으로 적어 둔 면은 지금 득표가 흔들려도 해결로 본다
        if not resolved and self.exhaustive_face(f):
            # Failed approaches do not silently remove a required face.
            # Viewpoint diversity and navigation tabu bound repeated attempts.
            return max(0.65, 1.0 / (1.0 + .12 * f.arrive_attempts))
        if resolved or st in (UNREACHABLE, GIVEUP):
            if not resolved:
                return 0.0
            if not self.ec.redetect_resolved:
                return 0.4 if f.recheck else 0.0                   # 식별한 면은 다시 보지 않는다 — 센서 간 불일치(recheck)는 '확정'을 의심하는 것이라 예외
            return 0.4 if (f.suspect or f.recheck) else 0.0         # 이동 의심(또는 카메라 간 불일치)이면 재검증 가치만 남긴다
        w = {UNSEEN: 1.0, VIEWED_NO_DET: 0.55, UNCERTAIN: 0.85}[st]
        if st == UNSEEN and f.occluded_views >= 2:
            w *= 0.5                                                  # 가려서 못 본 면은 같은 자리에서 또 보려 하지 않는다
        return w * (0.5 ** f.arrive_attempts)

    def m3_candidate_box(self, b: BoxModel) -> bool:
        """M3 photos belong to 0.4 m cubes, not known long arena obstacles.

        A lone LiDAR line may be a partial/merged observation, so keep its
        hypotheses until a full cluster or repeated UAV footprint resolves it.
        Geometry remains in the ledger for visibility and M2 normal matching.
        """
        known_footprint = b.kind == "full" or b.uav_t is not None
        if (self.ec.exhaustive_scoring_faces and self.uav_map_ready()
                and any(self.active_uav_prior(other) for other in self.boxes.values())
                and b.uav_t is None):
            # With the confirmed aerial inventory, unassociated LiDAR clusters
            # remain navigation/scout evidence, not additional photo boxes.
            # Without that inventory the LiDAR-only coverage path is retained.
            return False
        sx, sy = ((b.arena_sx or b.sx, b.arena_sy or b.sy)
                  if self.ec.exhaustive_scoring_faces and self.active_uav_prior(b) else (b.sx, b.sy))
        return not (known_footprint and max(sx, sy) > 1.5 * self.geo.box_size)

    def box_sector(self, b: BoxModel) -> Optional[str]:
        """Territory belongs to the box centre, not its viewing position or face."""
        return self.geo.sector_of(b.cx if b.arena_cx is None else b.arena_cx,
                                  b.cy if b.arena_cy is None else b.arena_cy)

    def pending_faces(self, sectors: Optional[Set[str]] = None) -> List[FaceRec]:
        allowed = None if sectors is None else {name for sector in sectors if (name := server_sector(sector)) is not None}
        return [f for b in self.boxes.values() if self.m3_candidate_box(b) and self.box_available(b)
                and (allowed is None or server_sector(self.box_sector(b)) in allowed)
                for f in b.faces if self.face_weight(f) > 0.0 and self.pt_allowed(f)
                and (f.exterior_open or self.requires_full_view(f))
                and (not self.ec.exhaustive_scoring_faces or f.idx not in b.internal)]

    def identify_faces(self, sectors: Optional[Set[str]] = None) -> List[FaceRec]:
        """팬틸트로 풀지 못해 OAK RGB + YOLO 로 다시 식별해 볼 면(oak_det.identify_fallback).
        조건: 열린 바깥 면, 아직 확정·빈 면이 아니고, 팬틸트가 이 면에 identify_min_attempts 번 이상 도착했는데도 못 풀었고, 식별 시도 횟수가 남았다.
        (GIVEUP 면은 팬틸트 쪽 가치가 0 이라 pending_faces 에서 빠지지만 여기서는 대상이다.)
        oak_det.identify_primary 이면 팬틸트가 먼저 가 볼 필요가 없다(identify_min_attempts 를 보지 않는다). 대신 팬틸트로 넘어간(pt_fallback) 면은 뺀다."""
        oc = self.cfg.oak_det
        allowed = None if sectors is None else {name for sector in sectors if (name := server_sector(sector)) is not None}
        out = []
        for b in self.boxes.values():
            if not (self.m3_candidate_box(b) and self.box_available(b)):
                continue
            if allowed is not None and server_sector(self.box_sector(b)) not in allowed:
                continue
            for f in b.faces:
                if not f.exterior_open or f.idx in b.internal or f.id_tries >= oc.identify_max_attempts:
                    continue
                if self.face_state(f) not in (UNSEEN, VIEWED_NO_DET, UNCERTAIN, GIVEUP) or self._reg_resolved(f):
                    continue
                if oc.identify_primary:
                    if f.pt_fallback:
                        continue
                elif f.arrive_attempts < oc.identify_min_attempts:
                    continue
                out.append(f)
        return out

    def mark_identify(self, keys: List[Tuple[int, int]], outcome: Optional[str] = None, pose: Optional[Pose2D] = None,
                      now: float = 0.0) -> None:
        """식별 단계 한 번을 마쳤다(성공 여부와 무관) — 같은 면을 끝없이 다시 보러 가지 않게 횟수를 올린다.

        outcome (oak_det.identify_primary 일 때만 쓴다):
          cropped / no_frames  OAK 가 이 면을 못 봤다(가까워 잘림 / 프레임 없음) → 바로 팬틸트로 넘긴다
          none                 OAK 가 제대로 보고도 검출이 없었다 → 독립 관측 1회로 세고, identify_max_attempts 번 그러면 팬틸트로 넘긴다
          seen                 검출이 있었다(득표가 쌓인다) → 독립 관측 1회. 확정이 안 나면 review_fallbacks 가 나중에 넘긴다
          unreached            그 면 앞 자리에 닿지 못했다(길 막힘·시간 초과) → 횟수만 올리고, identify_max_attempts 번 그러면 팬틸트로 넘긴다
          None                 그 밖(시간 제한 등) → 횟수만 올린다"""
        oc = self.cfg.oak_det
        for key in keys:
            f = self.get_face(key)
            if f is None:
                continue
            f.id_tries += 1
            f.last_id_t = float(now)
            if not oc.identify_primary:
                continue
            if outcome in ("cropped", "no_frames"):
                self.pt_fallback_face(f, outcome, now)
            elif outcome == "unreached":
                if f.id_tries >= max(int(oc.identify_max_attempts), 1):
                    self.pt_fallback_face(f, "unreached", now)
            elif outcome in ("none", "seen"):
                self.oak_view(f, pose, now)
                if outcome == "none":
                    f.oak_none += 1
                    if f.oak_none >= max(int(oc.identify_max_attempts), 1):
                        self.pt_fallback_face(f, "no_detection", now)

    # ------------------------------------------------------------------ OAK 우선 식별(oak_det.identify_primary) — 팬틸트는 폴백
    def primary_oak(self) -> bool:
        """지금 면 식별을 OAK 가 맡고 있나: 설정이 켜져 있고 OAK 를 실제로 쓸 수 있다. OAK 를 못 쓰면(드라이버 없음·포기) False → 모든 면을 팬틸트가 본다."""
        oc = self.cfg.oak_det
        return bool(oc.identify_primary and oc.enable and self.oak_live)

    def pt_allowed(self, f: FaceRec) -> bool:
        """팬틸트가 이 면을 보고(관측 기록·득표) 방문 후보로 삼아도 되나."""
        return (not self.primary_oak()) or f.pt_fallback

    def pt_gate(self, key: Optional[Tuple[int, int]]) -> bool:
        """팬틸트 M3 물체 득표를 받을지(ObjectMap.observe 의 pt_gate). key=None 은 위치 닻이 없는 득표 — 폴백 면을 보는 중일 때만 받는다."""
        if not self.primary_oak():
            return True
        if key is None:
            return self.pt_window
        f = self.get_face(key)
        return f is not None and f.pt_fallback

    def pt_fallback_face(self, f: FaceRec, reason: str, now: float = 0.0) -> None:
        """OAK 가 못 보는 면을 팬틸트로 넘긴다(되돌리지 않는다)."""
        if f.pt_fallback:
            return
        f.pt_fallback, f.oak_fail = True, str(reason)
        self.fallback_stats["total"] += 1
        by = self.fallback_stats["by_reason"]
        by[reason] = by.get(reason, 0) + 1
        ev = self.fallback_stats.setdefault("events", [])
        ev.append({"t": round(float(now), 2), "face": list(f.key), "reason": str(reason)})
        del ev[:-40]

    def oak_view(self, f: FaceRec, pose: Optional[Pose2D], now: float) -> None:
        """OAK 로 제대로 본 식별 한 번을 그 면의 독립 관측 1회로 센다(빈 면 확정은 서로 다른 자리 2곳 이상에서 3회 본 것이 기준이다)."""
        f.views += 1
        if pose is not None:
            f.view_poses.append((pose.x, pose.y, pose.th))
            del f.view_poses[:-6]
        f.last_view_t = float(now)
        self.stats["views"] += 1

    def identify_gain_weight(self, f: FaceRec) -> float:
        """OAK 식별 후보의 정보 가치 배율(0~1): 한 번도 못 본 면이 가장 크고, 보고도 못 풀었거나 이미 시도한 면은 줄인다."""
        st = self.face_state(f)
        w = {UNSEEN: 1.0, UNCERTAIN: 0.85, VIEWED_NO_DET: 0.7}.get(st, 0.6)
        return w * (0.7 ** max(f.id_tries, 0))

    def review_fallbacks(self, now: float) -> None:
        """식별을 identify_max_attempts 번 해도 확정·빈 면이 안 된 면은 팬틸트로 넘긴다. 마지막 식별의 득표가 장부에 반영될 시간(1.5 s)을 둔다."""
        oc = self.cfg.oak_det
        if not oc.identify_primary:
            return
        need = max(int(oc.identify_max_attempts), 1)
        for b in self.boxes.values():
            for f in b.faces:
                if f.pt_fallback or f.id_tries < need or f.last_id_t < 0 or now - f.last_id_t < 1.5:
                    continue
                if self.face_state(f) in (CONFIRMED, EMPTY) or self._reg_resolved(f):
                    continue
                self.pt_fallback_face(f, "unresolved", now)

    def on_anchor_change(self, pivot: Pt, trim: Tuple[float, float, float], now: float) -> Dict[str, int]:
        """SLAM→경기장 앵커가 바뀌었다(PoseGuard 앵커 재설정·원복). 변환 p' = R(dth)(p−pivot)+pivot+d 를 SLAM 지도에 묶인 기록에만 적용한다.
          · LiDAR 로 본 박스·면(중심·관측 자세·기록부 사본): 그대로 옮긴다 — 격자도 같은 변환으로 옮겨져서 다음 군집이 같은 박스에 맞는다(식별한 면이 새 박스로 둔갑하지 않게).
          · UAV 영상으로만 아는 박스: UAV 원좌표로 되돌리고(정렬 dx,dy 가 옛 앵커 오차를 품고 있었다) 정렬을 지운다 — 다음 UAV 프레임에서 새 앵커 기준으로 다시 잰다.
          · UAV 원좌표(arena_cx/cy), 서버 보고, 팀·적 UGV 트랙은 UAV 틀이라 건드리지 않는다.
        '봤다' 지도는 같은 변환으로 옮긴다. 면 장부 자체(상태·득표)는 지우지 않는다. 반환: 옮긴 수."""
        from ..nav.grid import rigid_resample
        dx, dy, dth = trim
        c, s_ = math.cos(dth), math.sin(dth)

        def fn(x: float, y: float, th: float = 0.0) -> Tuple[float, float, float]:
            ax, ay = x - pivot[0], y - pivot[1]
            return (pivot[0] + c * ax - s_ * ay + dx, pivot[1] + s_ * ax + c * ay + dy, wrap(th + dth))

        al = self._uav_alignment
        odx, ody = (al.dx, al.dy) if al is not None else (0.0, 0.0)
        moved = reset = 0
        for b in self.boxes.values():
            if b.uav_t is not None and b.kind != "full":
                b.cx, b.cy = b.cx - odx, b.cy - ody
                reset += 1
            else:
                b.cx, b.cy, _ = fn(b.cx, b.cy)
                moved += 1
            self._position_faces(b)
            for f in b.faces:
                f.view_poses = [fn(*p) for p in f.view_poses]
        for rec in self.registry.values():
            if rec.snap.get("view_poses"):
                rec.snap["view_poses"] = [list(fn(*p)) for p in rec.snap["view_poses"]]
        linked = {f.reg for b in self.boxes.values() for f in b.faces if f.reg is not None}
        for rec in self.registry.values():                    # 박스에 묶이지 않은 기록(박스가 사라진 뒤 남은 것)은 출처의 틀을 따른다
            if rec.rid in linked:
                continue
            if rec.origin == "lidar":
                rec.cx, rec.cy, _ = fn(rec.cx, rec.cy)
            else:
                rec.cx, rec.cy = rec.cx - odx, rec.cy - ody
        if self._lidar is not None and getattr(self._lidar, "_prev", None):
            self._lidar._prev = [(gid, *fn(px, py)[:2]) for gid, px, py in self._lidar._prev]   # 그룹 번호를 이어 가려면 이전 중심도 같이 옮긴다
        if al is not None:
            self._uav_pending = [(x - odx, y - ody, sx, sy, t) for x, y, sx, sy, t in self._uav_pending]
            self._uav_alignment = None
            self.uav_alignment_report = {}
        self.seen.seen = rigid_resample(self.seen.seen, 0.0, 0.0, self.seen.res, pivot, trim, False)
        self.stats["anchor_changes"] = self.stats.get("anchor_changes", 0) + 1
        return {"moved": moved, "uav_reset": reset}

    def flag_recheck(self, x: float, y: float, radius: float = 0.45) -> Optional[Tuple[int, int]]:
        """(x, y) 근처 면을 '팬틸트로 다시 볼 후보'로 표시한다(6-4: 팬틸트와 OAK 의 결과가 엇갈림). 표시한 면의 key, 없으면 None.
        face_weight 가 0.4 의 재검증 가치를 주므로 기존 M3 관측 시점 선택(pending_faces)에 그대로 올라간다. 새 독립 관측이 생기면 풀린다."""
        best, bd = None, radius
        for b in self.boxes.values():
            for f in b.faces:
                d = math.hypot(f.cx - x, f.cy - y)
                if d < bd:
                    best, bd = f, d
        if best is None:
            return None
        best.recheck = True
        return best.key

    def mark_arrival(self, keys: List[Tuple[int, int]]) -> None:
        """시점에 도착해 관측을 마쳤는데도 아직 해결되지 않은(확정·빈 면이 아닌) 면은 시도 횟수를 올린다 — 같은 곳을 무한히 반복하지 않게."""
        for b in self.boxes.values():
            for f in b.faces:
                if f.key in keys and self.face_state(f) in (UNSEEN, VIEWED_NO_DET, UNCERTAIN):
                    f.arrive_attempts += 1

    # ------------------------------------------------------------------ 면 기록부(explore.uav_face_registry)
    def _reg_resolved(self, f: FaceRec) -> bool:
        if not self.ec.uav_face_registry or f.reg is None:
            return False
        rec = self.registry.get(f.reg)
        return rec is not None and rec.status in ("identified", "empty")

    def _register_box(self, b: BoxModel, origin: str, now: float) -> None:
        """박스의 면 4개를 모두 기록부에 올린다(이미 기록된 면은 그대로 잇는다). UAV 영상으로 박스를 받아들일 때 부른다."""
        if not self.ec.uav_face_registry:
            return
        linked = {f.reg for bb in self.boxes.values() for f in bb.faces if f.reg is not None}
        outer = self.ec.uav_outer_faces
        if outer and not self.m3_candidate_box(b):
            return                                                # 긴 장애물은 M3 사진 대상이 아니다 — 기록부에 올리지 않는다(장부·pending_faces 와 같은 기준)
        for f in b.faces:
            if f.reg is not None and f.reg in self.registry:
                continue
            if outer and f.idx in b.internal:
                continue                                          # 이웃 상자·벽에 가린 면은 볼 수 없다 — 기록부에 올리지 않는다(uav_outer_faces)
            rec = self._reg_near(f, linked)
            if rec is None:
                rec = FaceReg(self._reg_next, f.idx, f.cx, f.cy, origin, float(now))
                self._reg_next += 1
                self.registry[rec.rid] = rec
                self.registry_stats["registered"] += 1
                if self.registry_t0 is None:
                    self.registry_t0 = float(now)
            rec.sector = server_sector(self.box_sector(b) or "")
            f.reg = rec.rid
            linked.add(rec.rid)
            if self.ec.uav_census and origin == "uav" and not rec.census and rec.status == "pending":
                if self._census_t0 is None:
                    self._census_t0 = float(now)
                if now - self._census_t0 <= self.ec.uav_census_window_s:
                    rec.census = True
                    self.census_stats["registered"] += 1

    def _reg_near(self, f: FaceRec, linked: Set[Optional[int]], resolved_only: bool = False) -> Optional[FaceReg]:
        best, bd = None, 0.25
        for rec in self.registry.values():
            if rec.idx != f.idx or rec.rid in linked or (resolved_only and rec.status not in ("identified", "empty")):
                continue
            d = math.hypot(rec.cx - f.cx, rec.cy - f.cy)
            if d < bd:
                best, bd = rec, d
        return best

    def _restore_box(self, b: BoxModel) -> None:
        """새로 생긴 박스의 면이 이미 해결된 기록과 같은 자리·같은 방향이면 관측 이력을 되돌려 준다 — 박스가 잠깐 사라졌다 다시 생겨도 같은 면을 처음부터 다시 보지 않게."""
        if not self.ec.uav_face_registry or not self.registry:
            return
        linked = {f.reg for bb in self.boxes.values() for f in bb.faces if f.reg is not None}
        for f in b.faces:
            rec = self._reg_near(f, linked, resolved_only=True)
            if rec is None:
                continue
            f.reg = rec.rid
            linked.add(rec.rid)
            sn = rec.snap
            f.views, f.view_poses = int(sn.get("views", 0)), [tuple(p) for p in sn.get("view_poses", [])]
            f.last_view_t, f.last_cam_yaw = float(sn.get("last_view_t", -1e9)), float(sn.get("last_cam_yaw", 0.0))
            f.occluded_views, f.last_occl_t = int(sn.get("occluded_views", 0)), float(sn.get("last_occl_t", -1e9))
            f.arrive_attempts, f.id_tries = int(sn.get("arrive_attempts", 0)), int(sn.get("id_tries", 0))
            self.registry_stats["restored"] += 1

    def _update_registry(self, now: float) -> None:
        """장부(면 상태)를 기록부에 반영한다(refresh 끝에서, 1 s 주기). 해결된 면(확정·빈 면)은 기록부에 '식별함'으로 남아 다시 보러 가지 않는다."""
        if not self.ec.uav_face_registry:
            return
        linked = {f.reg for b in self.boxes.values() for f in b.faces if f.reg is not None}
        outer = self.ec.uav_outer_faces
        for b in self.boxes.values():
            long_obstacle = outer and not self.m3_candidate_box(b)
            for f in b.faces:
                if outer and (f.idx in b.internal or long_obstacle):
                    # 나중에 이웃 상자가 보이거나 LiDAR 가 맞닿음을 알아낸 면: 아직 해결 전이면 기록부에서 뺀다(식별·빈 면 기록은 지우지 않는다)
                    rec_i = self.registry.get(f.reg) if f.reg is not None else None
                    if rec_i is not None and rec_i.status not in ("identified", "empty"):
                        self.registry.pop(f.reg, None)
                        self.registry_stats["inner_dropped"] += 1
                        f.reg = None
                    continue
                st = self.face_state(f)
                rec = self.registry.get(f.reg) if f.reg is not None else None
                if rec is None:
                    if st not in (CONFIRMED, EMPTY):
                        continue                                  # 해결된 면만 새로 올린다(UAV 박스의 면은 받아들일 때 이미 올렸다)
                    rec = self._reg_near(f, linked)
                    if rec is None:
                        rec = FaceReg(self._reg_next, f.idx, f.cx, f.cy, "lidar" if b.uav_t is None else "uav", float(now))
                        self._reg_next += 1
                        self.registry[rec.rid] = rec
                        self.registry_stats["registered"] += 1
                        if self.registry_t0 is None:
                            self.registry_t0 = float(now)
                    f.reg = rec.rid
                    linked.add(rec.rid)
                rec.cx, rec.cy = f.cx, f.cy
                rec.sector = server_sector(self.box_sector(b) or "")
                if st in (CONFIRMED, EMPTY):
                    new = "identified" if st == CONFIRMED else "empty"
                    if rec.status != new or (new == "identified" and rec.cls != f.top()[0]):
                        rec.status, rec.t_status = new, float(now)
                    rec.cls = f.top()[0] if new == "identified" else None
                    if rec.t_detected is None:
                        rec.t_detected = float(now)
                        if rec.census:                                # 초기 영상의 '미탐지' 면이 탐지로 바뀌었다
                            self.census_stats["converted"] += 1
                            self._conversions.append({"rid": rec.rid, "sector": rec.sector or "none", "idx": rec.idx, "x": round(rec.cx, 2),
                                                      "y": round(rec.cy, 2), "status": new, "cls": rec.cls, "t": float(now),
                                                      "waited_s": None if self._census_t0 is None else round(float(now) - self._census_t0, 1)})
                    rec.snap = {"views": f.views, "view_poses": [list(p) for p in f.view_poses[-12:]], "last_view_t": f.last_view_t,
                                "last_cam_yaw": f.last_cam_yaw, "occluded_views": f.occluded_views, "last_occl_t": f.last_occl_t,
                                "arrive_attempts": f.arrive_attempts, "id_tries": f.id_tries}
                elif rec.status not in ("identified", "empty"):
                    rec.status = "hidden" if f.idx in b.internal else "pending"      # 해결된 기록은 득표가 잠시 흔들려도 지우지 않는다

    def _registry_forget(self, b: BoxModel) -> None:
        for f in b.faces:
            if f.reg is not None and self.registry.pop(f.reg, None) is not None:
                self.registry_stats["forgotten"] += 1

    def registry_counts(self) -> Dict[str, int]:
        c = {"registered": len(self.registry), "identified": 0, "empty": 0, "pending": 0, "hidden": 0, "uav": 0, "restored": self.registry_stats["restored"]}
        for r in self.registry.values():
            c[r.status] += 1
            c["uav"] += r.origin == "uav"
        return c

    def registry_by_sector(self) -> Dict[str, Dict[str, int]]:
        """면 기록부를 섹터별로 센다(상자가 사라진 면의 기록도 포함). 키는 서버 섹터 이름, 섹터를 모르면 'none'."""
        out: Dict[str, Dict[str, int]] = {}
        for r in self.registry.values():
            row = out.setdefault(r.sector or "none", {"registered": 0, "identified": 0, "empty": 0, "pending": 0, "hidden": 0})
            row["registered"] += 1
            row[r.status] += 1
        return out

    def pop_conversions(self) -> List[Dict[str, Any]]:
        """'미탐지 → 탐지' 로 바뀐 초기 영상의 면(explore.uav_census)을 꺼내고 비운다. 꺼져 있으면 빈 목록."""
        out, self._conversions = self._conversions, []
        return out

    def uav_map_report(self, sectors: Optional[Any] = None, limit: int = 40) -> Dict[str, Any]:
        """UAV 지도의 면 장부(초기 영상 기준): 섹터별로 처음 올린 면 수, 지금까지 탐지로 바뀐 수, 아직 미탐지, 가려진 면, 초기 영상 뒤에 새로 생긴 면.
        sectors 를 주면 그 섹터(서버 이름)만. undetected 는 아직 미탐지인 초기 영상 면의 목록(최대 limit 개: 섹터, x, y, 방향)."""
        wanted = None if sectors is None else {n for n in (server_sector(x) for x in sectors) if n is not None}
        rows: Dict[str, Dict[str, int]] = {}
        todo: List[List[Any]] = []
        for r in self.registry.values():
            sec = r.sector or "none"
            if wanted is not None and sec not in wanted:
                continue
            row = rows.setdefault(sec, {"census": 0, "detected": 0, "undetected": 0, "hidden": 0, "new_faces": 0, "new_detected": 0})
            done = r.status in ("identified", "empty")
            if r.census:
                row["census"] += 1
                if done:
                    row["detected"] += 1
                elif r.status == "hidden":
                    row["hidden"] += 1
                else:
                    row["undetected"] += 1
                    if len(todo) < limit:
                        todo.append([sec, round(r.cx, 2), round(r.cy, 2), "ENWS"[r.idx % 4]])
            else:
                row["new_faces"] += 1
                row["new_detected"] += int(done)
        return {"enabled": bool(self.ec.uav_census), "t0": self._census_t0, "window_s": self.ec.uav_census_window_s,
                "registered": self.census_stats["registered"], "converted": self.census_stats["converted"],
                "undetected": sum(v["undetected"] for v in rows.values()), "sectors": rows, "todo": todo}

    # ------------------------------------------------------------------ 섹터별 면 장부(UAV 로 받은 상자의 면을 섹터로 나눠 진행을 본다)
    LEDGER_STATES = ("identified", "empty", "hidden", "unreachable", "uncertain", "viewed", "pending")

    def face_ledger_state(self, b: BoxModel, f: FaceRec) -> str:
        """면 하나의 섹터 장부 상태. identified(분류 확정) | empty(여러 자리에서 봐도 없다) | hidden(이웃 상자·벽에 가려 볼 수 없다) |
        unreachable(시도했지만 못 풀었다: 포기·도달 불가) | uncertain(득표가 모자란다) | viewed(봤지만 검출이 없다) | pending(아직 못 봤다) |
        excluded(uav_outer_faces 켜짐: 맞닿은·벽에 붙은 면 — 장부 집계에 넣지 않는다)."""
        if self.ec.uav_outer_faces and f.idx in b.internal:
            return "excluded"                                  # 맞닿은·벽에 붙은 면 — 식별·빈 면으로도 세지 않는다
        rec = self.registry.get(f.reg) if f.reg is not None else None
        st = self.face_state(f)
        if st == CONFIRMED or (rec is not None and rec.status == "identified"):
            return "identified"
        if st == EMPTY or (rec is not None and rec.status == "empty"):
            return "empty"
        if f.idx in b.internal or not f.exterior_open:
            return "hidden"
        if st in (UNREACHABLE, GIVEUP):
            return "unreachable"
        if st == UNCERTAIN:
            return "uncertain"
        if st == VIEWED_NO_DET:
            return "viewed"
        return "pending"

    def sector_ledger(self, sectors: Optional[Any] = None) -> Dict[str, Dict[str, Any]]:
        """현재 장부의 상자·면을 박스 중심 섹터별로 센다(키는 서버 이름 R-S1…CM, 섹터 밖은 'none'). sectors 를 주면 그 섹터만.
        todo 는 아직 해야 할 면(pending·viewed·uncertain)의 key 목록, complete 는 상자가 있고 todo 가 비었다는 뜻이다.
        M3 후보가 아닌 긴 장애물·영상이 끊긴 UAV 가설 상자는 세지 않는다(pending_faces 와 같은 기준)."""
        wanted = None if sectors is None else {n for n in (server_sector(x) for x in sectors) if n is not None}
        names = [server_sector(n) for n in self.geo.sector_names()]
        out: Dict[str, Dict[str, Any]] = {}
        for n in names + ["none"]:
            if wanted is None or n in wanted:
                out[n] = {"boxes": 0, "faces": 0, **{k: 0 for k in self.LEDGER_STATES}, "excluded": 0, "todo": [], "complete": False}
        for b in self.boxes.values():
            if not (self.m3_candidate_box(b) and self.box_available(b)):
                continue
            sec = server_sector(self.box_sector(b) or "") or "none"
            row = out.get(sec)
            if row is None:
                continue
            row["boxes"] += 1
            for f in b.faces:
                if self.ec.uav_outer_faces and f.idx in b.internal:
                    row["excluded"] += 1
                    continue                                       # 맞닿은·벽에 붙은 면은 장부에 올리지 않는다(uav_outer_faces)
                state = self.face_ledger_state(b, f)
                row["faces"] += 1
                row[state] += 1
                if state in ("pending", "viewed", "uncertain") or (
                        self.exhaustive_face(f) and state in ("hidden", "unreachable")):
                    row["todo"].append(list(f.key))
        for row in out.values():
            row["complete"] = row["boxes"] > 0 and not row["todo"]
        return out

    def sector_progress(self, sectors: Any) -> Dict[str, Any]:
        """지정한 섹터들의 진행 요약: 섹터별 장부 + 합계(done = 식별+빈 면, todo, hidden, unreachable) + 전부 끝났는지."""
        ledger = self.sector_ledger(sectors)
        tot = {"boxes": 0, "faces": 0, "done": 0, "todo": 0, "hidden": 0, "unreachable": 0}
        for row in ledger.values():
            tot["boxes"] += row["boxes"]
            tot["faces"] += row["faces"]
            tot["done"] += row["identified"] + row["empty"]
            tot["todo"] += len(row["todo"])
            tot["hidden"] += row["hidden"]
            tot["unreachable"] += row["unreachable"]
        return {"sectors": ledger, **tot, "complete": bool(ledger) and all(r["complete"] for r in ledger.values())}

    def get_face(self, key: Tuple[int, int]) -> Optional[FaceRec]:
        b = self.boxes.get(key[0])
        return b.faces[key[1]] if b is not None else None

    def face_weight_at(self, f: FaceRec, vx: float, vy: float) -> float:
        """시점 (vx, vy) 에서 이 면을 보는 가치. 이미 본(검출 없음) 면은 **다른 자리**에서 봐야 '빈 면'이 확정되므로 같은 자리는 가치를 낮춘다."""
        w = self.face_weight(f)
        if w > 0 and f.views > 0 and f.det_n == 0 and any(math.hypot(p[0] - vx, p[1] - vy) < 0.4 for p in f.view_poses):
            w *= 0.15
        return w

    def m3_entries(self, omap: Any) -> List[Dict[str, Any]]:
        """M3 집계에 들어가는 **확정 객체 면 목록**(섹터별 객체 리스트의 원자료). 박스 면 하나 = 항목 하나.
        장부(상자 면)에서 먼저 모으고, 장부에 없는(군집이 안 된) 관측은 ObjectMap 확정 면으로 보충한다.
        항목: sec(서버 섹터 이름) · cls · x,y(면 중심) · nx,ny(면 법선) · conf(평균 신뢰도) · n(표 수) · src(ledger|omap) · key(장부 면 키 또는 None) · status('confirmed')."""
        out: List[Dict[str, Any]] = []
        covered: List[Tuple[float, float, float, float]] = []
        covered_keys = set()
        full_cubes = [b for b in self.boxes.values() if b.kind == "full" and self.m3_candidate_box(b)]
        envelope_pad = min(.05, max(0., self.cfg.nav.res))
        for b in self.boxes.values():
            sec = self.geo.sector_of(b.cx if b.arena_cx is None else b.arena_cx,
                                     b.cy if b.arena_cy is None else b.arena_cy)
            for f in b.faces:
                # Anchored detections belong to this ledger even if unresolved,
                # excluded, or not a cube. Fallback must not reintroduce them.
                covered_keys.add(f.key)
                if (not self.m3_candidate_box(b)
                        or (self.ec.uav_outer_faces and f.idx in b.internal)):
                    continue
                if sec is None or f.det_kind != "object":
                    continue
                cls, margin, mconf = f.top()
                st = self.face_state(f)
                if (cls is None or st != CONFIRMED or f.det_n < self.cfg.perception.min_votes
                        or mconf < .65 or margin < self.ec.confirm_min_margin):
                    continue
                covered.append((f.cx, f.cy, f.nx, f.ny))
                covered_keys.add(f.key)
                out.append({"sec": sec, "cls": cls, "x": f.cx, "y": f.cy, "nx": f.nx, "ny": f.ny, "conf": float(mconf),
                            "n": int(f.det_n), "src": "ledger", "key": tuple(f.key), "status": "confirmed"})
        if omap is not None:
            for of in omap.confirmed_faces():
                if getattr(of, "geometry_key", None) is None and any(
                        abs(of.x - b.cx) <= b.sx / 2 + envelope_pad
                        and abs(of.y - b.cy) <= b.sy / 2 + envelope_pad for b in full_cubes):
                    # Old unanchored records may have a wrong surface normal
                    # and centre estimate. A point on a resolved cube is not
                    # an extra object beside its four ledger faces. Keep the
                    # source evidence for reassociation, but defer fallback
                    # counting until it has an unambiguous observed face.
                    continue
                # ObjectMap's generic confirmed query uses a looser confidence
                # threshold. Do not reintroduce weak evidence through fallback.
                votes = sorted(of.votes.values(), reverse=True)
                margin = ((votes[0] - (votes[1] if len(votes) > 1 else 0.)) / max(sum(votes), 1e-9)
                          if votes else 0.)
                if (of.n < self.cfg.perception.min_votes or not votes
                        or votes[0] / max(of.n, 1) < .65 or margin < self.ec.confirm_min_margin):
                    continue
                if of.kind != "object" or getattr(of, "geometry_key", None) in covered_keys or any(math.hypot(of.x - x, of.y - y) < 0.35
                                             and (of.vx * nx + of.vy * ny) / (math.hypot(of.vx, of.vy) or 1.0) > .5
                                             for x, y, nx, ny in covered):
                    continue
                sec = self.geo.sector_of(of.cx, of.cy)
                if sec is not None:
                    vn = math.hypot(of.vx, of.vy) or 1.0
                    out.append({"sec": sec, "cls": of.cls, "x": of.x, "y": of.y, "nx": of.vx / vn, "ny": of.vy / vn,
                                "conf": float(votes[0] / max(of.n, 1)), "n": int(of.n), "src": "omap", "key": None, "status": "confirmed"})
        return out

    @staticmethod
    def dedupe_entries(entries: List[Dict[str, Any]], dist: float = 0.25) -> Tuple[List[Dict[str, Any]], List[Dict[str, Any]]]:
        """섹터별로 같은 면의 중복 항목을 합친다. 같은 섹터에서 면 중심이 dist 안이고 법선이 비슷하면(내적 > 0.5) 같은 면이다
        (이웃 상자의 같은 방향 면은 0.4 m 떨어져 있어 합쳐지지 않는다). 표 수×신뢰도가 큰 항목을 남기고 클래스가 달라도 한 면은 한 번만 센다.
        반환: (남긴 항목, 합쳐 없앤 항목 — 'kept_cls' 로 어느 클래스에 흡수됐는지 표시)."""
        kept: List[Dict[str, Any]] = []
        merged: List[Dict[str, Any]] = []
        for e in sorted(entries, key=lambda e: (-(e["n"] * e["conf"]), e["src"] != "ledger")):
            dup = None
            for k in kept:
                if k["sec"] != e["sec"] or math.hypot(k["x"] - e["x"], k["y"] - e["y"]) >= dist:
                    continue
                dot = k["nx"] * e["nx"] + k["ny"] * e["ny"]
                if dot > 0.5:
                    dup = k
                    break
            if dup is None:
                kept.append(e)
            else:
                merged.append({**e, "kept_cls": dup["cls"]})
        return kept, merged

    def m3_counts(self, omap: Any, dedupe: bool = False, dedupe_dist: float = 0.25) -> Dict[str, Dict[str, int]]:
        """장부 기준 M3 집계: **박스 면 하나 = 객체 하나**. 같은 면을 여러 시점에서 봐도 한 번만 센다(위치 기반 합치기의 중복 계수 방지)
        섹터는 면 위치 추정이 아니라 LiDAR 군집에서 구한 박스 중심으로 정한다. 장부에 없는(군집이 안 된) 관측은 기존 ObjectMap 집계로 보충한다.
        확정(CONFIRMED) 객체 면만 센다. 사진 확정과 같은 min_votes(기본 4), 평균 신뢰도 0.65,
        confirm_min_margin을 적용하며, 보충 관측도 같은 기준을 통과해야 한다.
        dedupe=True 면 센 항목에서 섹터별 같은 면의 중복을 한 번 더 합친다(dedupe_entries).
        매번 현재 증거로 전체 스냅샷을 다시 만들므로 오분류 정정 시 계수가 줄어들 수 있다."""
        entries = self.m3_entries(omap)
        self.m3_dedupe_last = {"merged": 0, "detail": []}
        if dedupe:
            entries, merged = self.dedupe_entries(entries, dedupe_dist)
            self.m3_dedupe_last = {"merged": len(merged), "detail": [
                {"sec": m["sec"], "cls": m["cls"], "kept": m["kept_cls"], "x": round(m["x"], 2), "y": round(m["y"], 2)} for m in merged[:12]]}
        out: Dict[str, Dict[str, int]] = {}
        for e in entries:
            out.setdefault(e["sec"], {})
            out[e["sec"]][e["cls"]] = out[e["sec"]].get(e["cls"], 0) + 1
        return out

    def sector_object_list(self, omap: Any = None, sectors: Optional[Any] = None, dedupe: bool = False,
                           dedupe_dist: float = 0.25) -> Dict[str, List[Dict[str, Any]]]:
        """섹터별 객체 리스트: 확정(status='confirmed' — 분류 완료, M3 에 센다) + 후보(status='pending' — 객체로 검출됐지만 득표·신뢰도가
        모자라 아직 확정 전, 세지 않는다). sectors 를 주면(서버 이름) 그 섹터만. 항목 순서는 섹터 안에서 확정 → 후보."""
        wanted = None if sectors is None else {n for n in (server_sector(x) for x in sectors) if n is not None}
        entries = self.m3_entries(omap)
        if dedupe:
            entries, _ = self.dedupe_entries(entries, dedupe_dist)
        out: Dict[str, List[Dict[str, Any]]] = {}

        def put(e: Dict[str, Any]) -> None:
            sec = server_sector(e["sec"]) or e["sec"]
            if wanted is None or sec in wanted:
                out.setdefault(sec, []).append({k: (round(v, 3) if isinstance(v, float) else v) for k, v in e.items() if k != "sec"})
        for e in entries:
            put(e)
        confirmed_keys = {e["key"] for e in entries if e["key"] is not None}
        for b in self.boxes.values():
            sec = self.geo.sector_of(b.cx if b.arena_cx is None else b.arena_cx, b.cy if b.arena_cy is None else b.arena_cy)
            if sec is None:
                continue
            for f in b.faces:
                if f.det_kind != "object" or tuple(f.key) in confirmed_keys:
                    continue
                cls, margin, mconf = f.top()
                if cls is None:
                    continue
                put({"sec": sec, "cls": cls, "x": f.cx, "y": f.cy, "nx": f.nx, "ny": f.ny, "conf": float(mconf), "n": int(f.det_n),
                     "src": "ledger", "key": tuple(f.key), "status": "pending"})
        return out

    def summary(self) -> Dict[str, Any]:
        cnt: Dict[str, int] = defaultdict(int)
        for b in self.boxes.values():
            for f in b.faces:
                cnt[self.face_state(f)] += 1
        self.seen.unseen(np.zeros_like(self.seen.seen))
        lidar = None
        if self.lidar_result is not None:
            groups = self.lidar_result.groups
            lidar = {"groups": len(groups), "closed": sum(g.closed is True for g in groups),
                     "open": sum(g.closed is False for g in groups), "deferred": sum(g.closed is None and g.faces != [] for g in groups),
                     "units": len(self.lidar_result.units), "faces": len(self.lidar_result.faces), **self.lidar_result.stats}
        return {"lidar_faces": lidar, "face_registry": self.registry_counts() if self.ec.uav_face_registry else None,
                "sector_ledger": self.sector_ledger() if self.ec.uav_face_registry else None, "boxes": len(self.boxes), "m3_candidate_boxes": sum(self.m3_candidate_box(b) for b in self.boxes.values()),
                "uav_layout": self.layout_diagnostics(),
                "photo_cubes": sum(self.photo_box(b) for b in self.boxes.values()),
                "photo_faces_pending": sum(self.picture_pending(f) for b in self.boxes.values() for f in b.faces),
                "cube_candidates": [{"bid": b.bid, "cx": round(b.cx, 3), "cy": round(b.cy, 3),
                                      "sx": round(b.sx, 3), "sy": round(b.sy, 3), "source": b.kind,
                                      "available": self.box_available(b),
                                      "faces": [self.face_state(f) for f in b.faces]}
                                     for b in self.boxes.values() if self.m3_candidate_box(b)],
                "confirmed_static_boxes": len(self.confirmed_static_footprints(self._last_refresh)),
                "uav_alignment": dict(self.uav_alignment_report),
                "uav_arena_centres": [{"bid": b.bid, "cx": round(b.arena_cx, 3), "cy": round(b.arena_cy, 3)}
                                      for b in self.boxes.values() if b.arena_cx is not None and b.arena_cy is not None],
                "oak_identify": {"primary": self.primary_oak(), "oak_live": bool(self.oak_live), **{k: v for k, v in self.fallback_stats.items()}},
                "faces": dict(cnt), "seen_frac": round(float((self.seen.seen & self.seen.inside).sum() / max(self.seen.inside.sum(), 1)), 3),
                **self.stats}
