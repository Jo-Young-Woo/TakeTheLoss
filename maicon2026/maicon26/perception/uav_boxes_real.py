"""실기 UAV(주최측 고정 상공 카메라) 상자 윗면 검출기.

입력  /api/report/drone 의 BGR 프레임 1장 + 연습 기간에 만든 보정(GridCalibration · FourCornerCalibration).
출력  아레나 좌표(항법 격자와 같은 비음수 좌표)의 상자 윗면 칸 목록 [UavBox(cx, cy, 0.4, 0.4, 신뢰도)].
      윗면 한 칸이 곧 상자 하나다(0.4 m 정육면체). 붙어 있는 L·T·줄 모양 덩어리도 칸마다 따로 낸다.

하는 일과 하지 않는 일
  - 위에서 본 '모양'만 쓴다. 옆면 그림·분류·개수는 만들지 않는다(그건 지상 카메라가 본다). M3 점수는 이 모듈과 무관하다.
  - 시뮬 검출기(uav_layout.FixedUavLayoutDetector)와 같은 기하(상자 윗면 평면 0.4 m 로 편평화 → 0.4 m 칸 맞춤)를 쓰되,
    Gazebo 가정이던 '검정 = 밝기 18 이하' 같은 고정 문턱을 바닥 밝기에 맞춰 스스로 정한다. 조명·노출이 달라도 따라간다.
  - 칸마다 기각 이유를 남긴다(작다·크다·채움 부족·가장자리·무시 구역). 연습 때 scripts/uav_boxes_tool.py 가 겹쳐 그려 보여 준다.
  - 실영상으로 검증한 것이 아니다. 문턱·색은 연습 시간에 같은 도구로 맞춘다(config uav_boxes.*, field_overrides.json).

윗면 평면 편평화에서 옆면은 카메라 쪽(영상 중심 쪽)으로 번져 보인다. 그래서 어두운 덩어리의 '카메라에서 먼 가장자리'부터 0.4 m 칸을 깔고,
칸 안이 충분히(fill_min) 채워진 것만 윗면으로 인정한다. 안쪽 구멍(표적 수광부·LED)은 윤곽을 채워서 무시한다.
"""
from __future__ import annotations

import logging
import math
import time
from dataclasses import dataclass, field
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

from ..link.base import UavBox
from .uav_calibration import FourCornerCalibration, GridCalibration, load_calibration

log = logging.getLogger("maicon26.uav_boxes")
CELL_M = 0.4                                     # 상자 한 변 [규정 0.4 m 정육면체]
Calibration = (FourCornerCalibration, GridCalibration)


@dataclass
class UavCell:
    """후보 칸 하나. status: accepted(상자로 냄) | ignored(무시 구역) | rejected(기각)."""
    cx: float
    cy: float
    fill: float
    conf: float
    status: str
    reason: str = ""
    comp: int = 0

    def to_dict(self) -> Dict[str, Any]:
        return {"cx": round(self.cx, 3), "cy": round(self.cy, 3), "fill": round(self.fill, 3), "conf": round(self.conf, 3),
                "status": self.status, "reason": self.reason, "comp": self.comp}


@dataclass
class UavBoxReport:
    ok: bool = False
    reason: str = ""                             # 프레임 전체를 못 쓴 이유: bad_frame size_mismatch dark_frame bright_frame low_contrast no_cv2
    boxes: List[UavBox] = field(default_factory=list)
    cells: List[UavCell] = field(default_factory=list)
    floor_v: float = 0.0                         # 아레나 바닥 밝기(중앙값, 0~255)
    thr_v: float = 0.0                           # '어두운 윗면' 문턱(검정 모드) — floor_v 와 설정에서 정해진다
    fg_fraction: float = 0.0                     # 아레나 안에서 윗면 색으로 분류된 화소 비율
    components: int = 0
    rejects: Dict[str, int] = field(default_factory=dict)
    note: str = ""                               # 프레임은 썼지만 이상한 점: no_dark_region(윗면색 화소가 거의 없다 — 빈 경기장이거나 노출·문턱이 틀렸다)
    elapsed_ms: float = 0.0
    debug: Dict[str, Any] = field(default_factory=dict)   # 도구용 배열(편평화 영상·마스크) — 요약에는 넣지 않는다

    def summary(self) -> Dict[str, Any]:
        """JSON 으로 쓸 수 있는 작은 요약(통계·로그·보고용)."""
        return {"ok": self.ok, "reason": self.reason, "boxes": len(self.boxes), "floor_v": round(self.floor_v, 1),
                "thr_v": round(self.thr_v, 1), "fg": round(self.fg_fraction, 4), "components": self.components,
                "rejects": dict(self.rejects), "note": self.note, "ms": round(self.elapsed_ms, 1)}


def _inside_rect(x: float, y: float, rect: Sequence[float], margin: float) -> bool:
    x0, y0, x1, y1 = rect
    return x0 - margin <= x <= x1 + margin and y0 - margin <= y <= y1 + margin


def rects_from_flat(flat: Iterable[float]) -> List[Tuple[float, float, float, float]]:
    """설정의 평평한 수 목록 (x0, y0, x1, y1, x0, y0, …) 을 사각형 목록으로. 개수가 4 의 배수가 아니면 ValueError."""
    vals = [float(v) for v in flat]
    if len(vals) % 4:
        raise ValueError("무시 구역은 x0 y0 x1 y1 네 수씩 묶어서 적는다")
    rects = []
    for i in range(0, len(vals), 4):
        x0, y0, x1, y1 = vals[i:i + 4]
        if not all(math.isfinite(v) for v in (x0, y0, x1, y1)) or x1 <= x0 or y1 <= y0:
            raise ValueError("무시 구역 사각형이 올바르지 않다")
        rects.append((x0, y0, x1, y1))
    return rects


class RealUavBoxDetector:
    """보정된 고정 상공 영상에서 상자 윗면 칸을 찾는다. 호출 가능 객체라 HttpServerLink(uav_layout_detector=…)에 그대로 넣는다."""

    def __init__(self, calibration, cfg: Any = None, ignore_rects: Optional[Sequence[Sequence[float]]] = None,
                 camera: Optional[str] = None) -> None:
        if not isinstance(calibration, Calibration):
            raise TypeError("UAV 보정(GridCalibration · FourCornerCalibration)이 필요하다")
        if cfg is None:
            from ..config import UavBoxesCfg
            cfg = UavBoxesCfg()
        self.cal, self.cfg = calibration, cfg
        self.camera = camera if camera is not None else getattr(cfg, "camera", None)
        self.ignore_rects = [tuple(float(v) for v in r) for r in (ignore_rects if ignore_rects is not None
                                                                   else rects_from_flat(getattr(cfg, "ignore_rects", ())))]
        if not math.isfinite(float(cfg.ppm)) or not 48 <= float(cfg.ppm) <= 256:
            raise ValueError("uav_boxes.ppm 은 48..256 이어야 한다")
        if abs(calibration.object_plane_z - CELL_M) > 0.02:
            raise ValueError("UAV 보정의 상자 윗면 높이가 0.4 m 가 아니다")
        self.stats: Dict[str, Any] = {"frames": 0, "frames_ok": 0, "boxes_last": 0, "boxes_total": 0, "unusable": 0,
                                      "zero_streak": 0, "ms_last": 0.0, "ms_max": 0.0, "last": None}
        self.last: Optional[UavBoxReport] = None
        self._setup_geometry()

    # ------------------------------------------------------------------ 보정에서 정해지는 고정 값
    def _setup_geometry(self) -> None:
        import cv2
        cal, s = self.cal, float(self.cfg.ppm)
        self.scale = s
        to_px = np.array([[s, 0.0, 0.0], [0.0, -s, cal.arena_height * s], [0.0, 0.0, 1.0]])
        self._plane_h = cal.plane_homography(cal.object_plane_z)          # 영상 화소 → 윗면 평면의 아레나 XY
        self._rect_m = to_px @ self._plane_h                              # 영상 화소 → 편평화 화소
        self.size = (int(math.ceil(cal.arena_width * s)) + 1, int(math.ceil(cal.arena_height * s)) + 1)   # (폭, 높이) 화소
        w, h = cal.image_size
        self._valid = cv2.warpPerspective(np.full((h, w), 255, np.uint8), self._rect_m, self.size, flags=cv2.INTER_NEAREST)
        cam_x, cam_y, cam_z = cal.camera_center
        self.cam_px = (cam_x * s, (cal.arena_height - cam_y) * s)        # 카메라 바로 아래 점의 편평화 화소
        # 벽 안쪽 띠: 윗면 평면으로 펴면 벽 바닥(z=ref)이 d·(zt−zr)/(Hc−zr) 만큼 안쪽으로 밀린다(d 는 카메라 연직선까지 거리).
        zr, zt = cal.reference_plane_z, cal.object_plane_z
        k = (zt - zr) / max(cam_z - zr, 1e-6)
        extra = float(self.cfg.wall_band_extra_m)
        bands = {"x0": cam_x * k + extra, "x1": (cal.arena_width - cam_x) * k + extra,
                 "y0": cam_y * k + extra, "y1": (cal.arena_height - cam_y) * k + extra}
        band = np.zeros((self.size[1], self.size[0]), np.uint8)
        if self.cfg.wall_band:
            bx0, bx1 = int(round(bands["x0"] * s)), int(round(bands["x1"] * s))
            by0, by1 = int(round(bands["y0"] * s)), int(round(bands["y1"] * s))
            band[:, :max(bx0, 0)] = 1                                     # x = 0 쪽(편평화 영상의 왼쪽)
            band[:, self.size[0] - max(bx1, 0):] = 1
            band[self.size[1] - max(by0, 0):, :] = 1                      # y = 0 은 영상 아래쪽
            band[:max(by1, 0), :] = 1
        self._band = band.astype(bool)
        self.wall_band_m = bands
        interior = np.zeros_like(band)
        m = int(round(0.1 * s))
        interior[m:self.size[1] - m, m:self.size[0] - m] = 1
        self._interior = (interior.astype(bool) & ~self._band & (self._valid > 0))

    # ------------------------------------------------------------------ 외부 인터페이스
    def __call__(self, bgr: np.ndarray) -> List[UavBox]:
        return self.detect(bgr)

    def detect(self, bgr: np.ndarray) -> List[UavBox]:
        rep = self.analyze(bgr)
        self.last = rep
        st = self.stats
        st["frames"] += 1
        st["frames_ok"] += int(rep.ok)
        st["unusable"] += int(not rep.ok)
        st["boxes_last"] = len(rep.boxes)
        st["boxes_total"] += len(rep.boxes)
        st["ms_last"] = round(rep.elapsed_ms, 1)
        st["ms_max"] = max(st["ms_max"], round(rep.elapsed_ms, 1))
        st["last"] = rep.summary()
        st["zero_streak"] = st["zero_streak"] + 1 if (rep.ok and not rep.boxes) else 0
        if st["zero_streak"] in (5, 65, 185):                  # 경기 중 상자가 하나도 안 잡히는 프레임이 이어지면 사람이 보게 한다(조용히 멈춰 있지 않게)
            log.warning("[UAV 상자] 프레임 %d장 연속으로 상자 0개 — 바닥 밝기 %.0f, 윗면 문턱 %.0f, 윗면색 화소 %.2f%%%s. "
                        "빈 경기장이 아니면 노출·uav_boxes.* 를 scripts/uav_boxes_tool.py 로 확인한다",
                        st["zero_streak"], rep.floor_v, rep.thr_v, 100.0 * rep.fg_fraction, f" ({rep.note})" if rep.note else "")
        if not rep.ok:
            raise ValueError("UAV 프레임을 쓸 수 없다: " + rep.reason)      # HttpServerLink 가 drone_layout_rejected 로 센다
        return list(rep.boxes)

    # ------------------------------------------------------------------ 분석
    def analyze(self, bgr: np.ndarray, debug: bool = False) -> UavBoxReport:
        t0 = time.perf_counter()
        rep = UavBoxReport()
        try:
            self._analyze(bgr, rep, debug)
        finally:
            rep.elapsed_ms = (time.perf_counter() - t0) * 1000.0
        return rep

    def _analyze(self, bgr: np.ndarray, rep: UavBoxReport, debug: bool) -> None:
        try:
            import cv2
        except ImportError:
            rep.reason = "no_cv2"
            return
        cfg, cal = self.cfg, self.cal
        if not isinstance(bgr, np.ndarray) or bgr.dtype != np.uint8 or bgr.ndim != 3 or bgr.shape[2] != 3:
            rep.reason = "bad_frame"
            return
        if (bgr.shape[1], bgr.shape[0]) != tuple(cal.image_size):
            rep.reason = "size_mismatch"
            return
        top = cv2.warpPerspective(bgr, self._rect_m, self.size, flags=cv2.INTER_LINEAR)
        hsv = cv2.cvtColor(top, cv2.COLOR_BGR2HSV)
        v = cv2.medianBlur(hsv[:, :, 2], 3)
        sat = hsv[:, :, 1]
        floor_v = float(np.median(v[self._interior])) if self._interior.any() else 0.0
        rep.floor_v = floor_v
        mean_all = float(v[self._valid > 0].mean()) if (self._valid > 0).any() else 0.0
        if mean_all < cfg.min_mean_v:
            rep.reason = "dark_frame"
            return
        if mean_all > cfg.max_mean_v:
            rep.reason = "bright_frame"
            return
        # ---- 윗면 색 마스크(검정·회색·둘 다)
        mode = cfg.top_color
        mask = np.zeros(v.shape, bool)
        dark_full = np.zeros(v.shape, bool)
        thr = 0.0
        if mode in ("black", "both"):
            thr = min(float(cfg.v_black_max), max(float(cfg.v_black_rel) * floor_v, float(cfg.v_black_min)))
            rep.thr_v = thr
            if floor_v < cfg.min_floor_contrast * max(thr, 1.0) and mode == "black":
                rep.reason = "low_contrast"              # 바닥이 검정 문턱과 구분되지 않는다 — 노출·색 설정을 의심
                return
            dark_full = v <= thr
            mask |= dark_full
        if mode in ("grey", "both"):
            grey = (sat <= cfg.grey_s_max) & (v >= cfg.grey_v_min) & (v <= cfg.grey_v_max)
            mask |= grey
            dark_full = dark_full | grey if mode == "both" else grey
        valid = self._valid > 0
        mask &= valid
        rep.fg_fraction = float(mask[self._interior | self._band].mean()) if (self._interior | self._band).any() else 0.0
        nowall = mask & ~self._band
        m8 = nowall.astype(np.uint8) * 255
        m8 = cv2.morphologyEx(m8, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        full8 = (dark_full & valid).astype(np.uint8)
        if debug:
            rep.debug = {"top": top, "mask": m8, "band": self._band}
        contours, _ = cv2.findContours(m8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        rep.components = len(contours)
        s = self.scale
        side = CELL_M * s
        min_area = (cfg.min_top_m * s) ** 2
        seen_centres: List[Tuple[float, float]] = []

        def reject(reason: str) -> None:
            rep.rejects[reason] = rep.rejects.get(reason, 0) + 1

        for ci, contour in enumerate(contours):
            area = float(cv2.contourArea(contour))
            x, y, w, h = cv2.boundingRect(contour)
            if area < min_area:
                reject("small")
                if area >= (0.5 * cfg.min_top_m * s) ** 2:       # 사람이 볼 만한 크기만 도구 표시용으로 남긴다
                    cx, cy = (x + w / 2) / s, cal.arena_height - (y + h / 2) / s
                    rep.cells.append(UavCell(cx, cy, 0.0, 0.0, "rejected", "small", ci))
                continue
            if max(w, h) > cfg.max_extent_m * s:
                reject("huge")
                cx, cy = (x + w / 2) / s, cal.arena_height - (y + h / 2) / s
                rep.cells.append(UavCell(cx, cy, 0.0, 0.0, "rejected", "huge", ci))
                continue
            comp = np.zeros(m8.shape, np.uint8)
            cv2.drawContours(comp, [contour], -1, 1, thickness=cv2.FILLED)      # 안쪽 구멍(수광부·LED)은 메운다
            # 벽 띠에 걸친 칸은 띠 안의 어두운 화소도 윗면 후보로 센다(띠가 칸을 잘라 먹지 않게).
            comp_fill = comp | (self._band.astype(np.uint8) & full8)
            from_right = x + w / 2 > self.cam_px[0]
            from_bottom = y + h / 2 > self.cam_px[1]
            nx_cells, ny_cells = max(1, math.ceil(w / side - cfg.tile_slack)), max(1, math.ceil(h / side - cfg.tile_slack))
            tiles: List[UavCell] = []
            covered = 0.0
            for ix in range(nx_cells):
                for iy in range(ny_cells):
                    left = x + w - (ix + 1) * side if from_right else x + ix * side
                    topy = y + h - (iy + 1) * side if from_bottom else y + iy * side
                    x0, x1, y0, y1 = int(round(left)), int(round(left + side)), int(round(topy)), int(round(topy + side))
                    cx, cy = (left + side / 2) / s, cal.arena_height - (topy + side / 2) / s
                    if x0 < 0 or y0 < 0 or x1 > m8.shape[1] or y1 > m8.shape[0]:
                        tiles.append(UavCell(cx, cy, 0.0, 0.0, "rejected", "outside", ci))
                        reject("outside")
                        continue
                    patch = comp_fill[y0:y1, x0:x1]
                    fill = float(patch.mean())
                    vfrac = float((self._valid[y0:y1, x0:x1] > 0).mean())
                    if vfrac < 0.98:
                        tiles.append(UavCell(cx, cy, fill, 0.0, "rejected", "outside", ci))
                        reject("outside")
                        continue
                    if fill < cfg.fill_min:
                        if fill >= 0.35:                     # 거의 빈 칸(옆면 번짐·가림)은 표시용으로도 남기지 않는다
                            tiles.append(UavCell(cx, cy, fill, 0.0, "rejected", "fill", ci))
                        reject("fill")
                        continue
                    if not (cfg.edge_margin_m <= cx <= cal.arena_width - cfg.edge_margin_m
                            and cfg.edge_margin_m <= cy <= cal.arena_height - cfg.edge_margin_m):
                        tiles.append(UavCell(cx, cy, fill, 0.0, "rejected", "edge", ci))
                        reject("edge")
                        continue
                    covered += side * side
                    tiles.append(UavCell(cx, cy, fill, 0.0, "accepted", "", ci))
            extra = max(0.0, 1.0 - covered / max(area, 1.0)) if covered > 0 else 0.0
            for t in tiles:
                if t.status != "accepted":
                    rep.cells.append(t)
                    continue
                conf = 0.6 + 0.38 * min(max((t.fill - cfg.fill_min) / max(1.0 - cfg.fill_min, 1e-6), 0.0), 1.0)
                conf -= 0.5 * max(0.0, extra - cfg.extra_free)
                conf = min(max(conf, 0.05), 0.98)
                t.conf = conf
                if conf < cfg.min_conf:
                    t.status, t.reason = "rejected", "extra"
                    reject("extra")
                elif any(_inside_rect(t.cx, t.cy, r, cfg.ignore_margin_m) for r in self.ignore_rects):
                    t.status, t.reason = "ignored", "ignore_zone"
                    reject("ignored")
                elif any(math.hypot(t.cx - ox, t.cy - oy) < 0.15 for ox, oy in seen_centres):
                    t.status, t.reason = "rejected", "duplicate"
                    reject("duplicate")
                else:
                    seen_centres.append((t.cx, t.cy))
                    rep.boxes.append(UavBox(t.cx, t.cy, CELL_M, CELL_M, t.conf))
                rep.cells.append(t)
        if not rep.boxes and rep.fg_fraction < 0.002:
            rep.note = "no_dark_region"
        if len(rep.boxes) > cfg.max_boxes:                       # 이상한 프레임(바닥 전체가 어두움 등) — 통째로 버린다
            rep.reason = "too_many"
            rep.boxes = []
            return
        rep.boxes.sort(key=lambda b: (round(b.cx, 2), round(b.cy, 2)))
        rep.ok = True

    # ------------------------------------------------------------------ 도구용
    def arena_to_pixels(self, points: Sequence[Sequence[float]], plane_z: Optional[float] = None) -> np.ndarray:
        """아레나 XY(주어진 높이 평면 위) → 원본 영상 화소. 겹쳐 그리기와 색 표본 위치에 쓴다."""
        z = self.cal.object_plane_z if plane_z is None else plane_z
        inv = np.linalg.inv(self.cal.plane_homography(z))
        pts = np.column_stack([np.asarray(points, dtype=float).reshape(-1, 2), np.ones(len(points))]) @ inv.T
        return pts[:, :2] / pts[:, 2:]

    def draw(self, bgr: np.ndarray, rep: UavBoxReport) -> np.ndarray:
        """보고서를 원본 영상 위에 겹친다. 초록=상자, 하늘=무시 구역, 빨강=기각(이유·채움 비율)."""
        import cv2
        out = bgr.copy()
        palette = {"accepted": (60, 220, 60), "ignored": (230, 200, 40), "rejected": (50, 60, 240)}
        half = CELL_M / 2
        for c in rep.cells:
            corners = [(c.cx - half, c.cy - half), (c.cx + half, c.cy - half), (c.cx + half, c.cy + half), (c.cx - half, c.cy + half)]
            px = np.round(self.arena_to_pixels(corners)).astype(np.int32)
            cv2.polylines(out, [px.reshape(-1, 1, 2)], True, palette.get(c.status, (255, 255, 255)), 2)
            label = f"{c.conf:.2f}" if c.status == "accepted" else f"{c.reason}{'' if c.status == 'ignored' else f' {c.fill:.2f}'}"
            cv2.putText(out, label, (int(px[:, 0].min()), int(px[:, 1].min()) - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                        palette.get(c.status, (255, 255, 255)), 1, cv2.LINE_AA)
        for r in self.ignore_rects:
            corners = [(r[0], r[1]), (r[2], r[1]), (r[2], r[3]), (r[0], r[3])]
            px = np.round(self.arena_to_pixels(corners)).astype(np.int32)
            cv2.polylines(out, [px.reshape(-1, 1, 2)], True, palette["ignored"], 1)
        text = f"boxes={len(rep.boxes)} floorV={rep.floor_v:.0f} thr={rep.thr_v:.0f} {rep.reason or 'ok'}"
        cv2.putText(out, text, (12, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
        return out

    def rectified(self, bgr: np.ndarray, plane_z: Optional[float] = None) -> np.ndarray:
        """윗면(기본) 또는 임의 높이 평면으로 편평화한 영상 — 사람이 위에서 본 모양을 확인할 때."""
        import cv2
        z = self.cal.object_plane_z if plane_z is None else plane_z
        s = self.scale
        to_px = np.array([[s, 0.0, 0.0], [0.0, -s, self.cal.arena_height * s], [0.0, 0.0, 1.0]])
        return cv2.warpPerspective(bgr, to_px @ self.cal.plane_homography(z), self.size, flags=cv2.INTER_LINEAR)


# ====================================================================== 색 표본(연습 때 문턱 정하기)
def probe_points(detector: RealUavBoxDetector, bgr: np.ndarray, points: Sequence[Sequence[float]],
                 plane_z: Optional[float] = None, half_m: float = 0.10) -> List[Dict[str, Any]]:
    """아레나 좌표 점마다 주변(±half_m)의 HSV 밝기·채도 분포. 상자 윗면 중심점은 plane_z=None(윗면), 바닥 점은 plane_z=0."""
    import cv2
    img = detector.rectified(bgr, plane_z)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    s, H = detector.scale, detector.cal.arena_height
    out = []
    for x, y in points:
        px, py = int(round(x * s)), int(round((H - y) * s))
        r = max(2, int(round(half_m * s)))
        patch = hsv[max(py - r, 0):py + r + 1, max(px - r, 0):px + r + 1].reshape(-1, 3)
        if len(patch) == 0:
            out.append({"x": x, "y": y, "n": 0})
            continue
        vv, ss = patch[:, 2].astype(float), patch[:, 1].astype(float)
        out.append({"x": float(x), "y": float(y), "n": int(len(patch)),
                    "v_p05": float(np.percentile(vv, 5)), "v_p20": float(np.percentile(vv, 20)), "v_p50": float(np.percentile(vv, 50)),
                    "v_p80": float(np.percentile(vv, 80)), "v_p95": float(np.percentile(vv, 95)),
                    "v_max": float(vv.max()), "s_p50": float(np.percentile(ss, 50)), "s_p95": float(np.percentile(ss, 95))})
    return out


def suggest_thresholds(box_probes: Sequence[Dict[str, Any]], floor_probes: Sequence[Dict[str, Any]],
                       min_floor_contrast: float = 1.6) -> Dict[str, Any]:
    """상자 윗면 표본과 바닥 표본에서 uav_boxes.* 값을 제안한다. 근거 숫자와 함께 돌려주고, 못 정하면 ok=False + 이유.
    윗면 한가운데의 LED·수광부 구멍이 표본에 섞여도 되도록 윗면은 80% 값, 바닥은 20% 값(그림자·표식 한두 곳은 무시)을 본다."""
    boxes = [p for p in box_probes if p.get("n", 0) > 0]
    floors = [p for p in floor_probes if p.get("n", 0) > 0]
    if not boxes or not floors:
        return {"ok": False, "why": "상자 윗면 점과 바닥 점이 각각 1개 이상 필요하다"}
    top_hi = max(p["v_p80"] for p in boxes)
    top_lo = min(p["v_p20"] for p in boxes)
    top_s = max(p["s_p50"] for p in boxes)
    floor_lo = float(np.median([p["v_p20"] for p in floors]))
    floor_med = float(np.median([p["v_p50"] for p in floors]))
    out: Dict[str, Any] = {"top_v_p80_max": top_hi, "top_v_p20_min": top_lo, "top_s_p50_max": top_s,
                           "floor_v_p20_median": floor_lo, "floor_v_p50_median": floor_med}
    if top_hi + 6 < floor_lo:                       # 상자 윗면이 바닥보다 어둡다 — 검정 모드
        mid = 0.5 * (top_hi + floor_lo)             # 윗면과 바닥의 한가운데 — 조명이 조금 변해도 양쪽에 여유가 있다
        cap = floor_med / (min_floor_contrast * 1.08)   # 검출기는 바닥 밝기가 문턱의 min_floor_contrast 배 미만이면 프레임을 버린다 — 그 안쪽으로만 제안
        thr = min(mid, cap)
        if thr < top_hi + 2:
            out.update(ok=False, why="바닥이 충분히 밝지 않아 검정 모드 문턱을 둘 수 없다(윗면 상한 %.0f, 허용 문턱 %.0f) — 노출을 올리거나 min_floor_contrast 를 낮춘다" % (top_hi, cap))
            return out
        out.update(ok=True, top_color="black", v_black_max=int(min(255, math.floor(thr))),
                   v_black_rel=round(float(min(0.9, max(0.1, thr / max(floor_med, 1.0)))), 2),
                   why="윗면 밝기 80% 값이 바닥 밝기 20% 값보다 충분히 낮다(문턱은 둘의 한가운데, 바닥 대비 한도 안)")
    elif top_lo > floor_med + 20 and top_s < 70:    # 윗면이 바닥보다 밝은 회색
        out.update(ok=True, top_color="grey", grey_v_min=int(max(0, math.floor(top_lo - 6))), grey_v_max=int(min(255, math.ceil(top_hi + 6))),
                   grey_s_max=int(min(255, math.ceil(top_s + 10))), why="윗면이 바닥보다 밝고 채도가 낮다")
    else:
        out.update(ok=False, why="윗면과 바닥의 밝기가 겹친다 — 표본 점이 상자 윗면 한가운데·빈 바닥인지 확인하거나 조명·노출을 바꾼다")
    return out


# ====================================================================== 실행 진입점에서 쓰는 로더
def load_real_uav_detector(path: str, arena: Any, cfg: Any, camera: Optional[str] = None) -> RealUavBoxDetector:
    """보정 JSON 을 읽어 검출기를 만든다. 경기장 치수가 다르거나 보정이 쓸 수 없으면 ValueError/OSError."""
    calibration = load_calibration(path)
    for name, mine, theirs in (("폭", calibration.arena_width, arena.W), ("높이", calibration.arena_height, arena.H)):
        if not math.isclose(mine, theirs, abs_tol=0.005, rel_tol=0.0):
            raise ValueError(f"UAV 보정의 경기장 {name} {mine} m 가 설정의 {theirs} m 와 다르다 — --arena-spec 과 보정을 맞춘다")
    if not math.isclose(arena.box_size, CELL_M, abs_tol=1e-6, rel_tol=0.0):
        raise ValueError("UAV 상자 검출기는 0.4 m 정육면체만 지원한다")
    return RealUavBoxDetector(calibration, cfg, camera=camera)
