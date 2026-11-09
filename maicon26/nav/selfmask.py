"""LiDAR 자기 가림 처리.

2D LiDAR 평면에 로봇 자신의 구조물(팬틸트 기둥, 안테나, 카메라 브래킷)이 들어오면 그 방향의 반사가 항상 '아주 가까운 장애물'로
읽힌다. 안전 제한기는 전방 17 cm 안의 반사를 정지로 보므로, 전방 쪽이면 로봇이 영영 못 움직이고 막힘 복구만 반복한다.

  apply    설정한 각도 구간(self_mask_sectors) 안에서 self_mask_range 보다 가까운 반사, 그리고 모든 방향에서 min_range 보다
           가까운 반사를 무효 처리한다(NaN; 뒤쪽 공간은 미관측). 그보다 먼 반사는 실제 장애물이라 남긴다.
  observe  설정 없이도 문제를 알아채게 한다: **같은 방향·같은 거리의 아주 가까운 반사가 로봇이 제자리에서 돌아도(world 의 물체라면
           방향이 밀려난다) 계속 남으면** 차체 일부로 의심하고 마스크 각도 후보를 돌려준다. 벽이 곁에 붙은 경우는 로봇이 돌면 그 반사의
           방향이 바뀌어 걸리지 않는다. 자동으로 지우지는 않는다(진짜 장애물을 지울 수 있다) — 사람이 확인하고 설정에 넣는다.
"""
from __future__ import annotations

import math
from typing import Dict, List, Tuple

import numpy as np

from ..config import LidarCfg
from ..types import Scan
from ..util import wrap


class SelfMask:
    BIN = math.radians(5.0)

    def __init__(self, cfg: LidarCfg) -> None:
        self.cfg = cfg
        self.sectors: List[Tuple[float, float]] = [(math.radians(c), math.radians(h)) for c, h in cfg.self_mask_sectors]
        self._track: Dict[int, dict] = {}
        self.suspects: Dict[int, Tuple[float, float]] = {}       # bin → (각도 deg, 거리 m)

    @property
    def active(self) -> bool:
        return bool(self.sectors) or self.cfg.min_range > 0.0

    # ------------------------------------------------------------------ 적용
    def apply(self, scan: Scan) -> Scan:
        if not self.active:
            return scan
        r = scan.ranges.astype(float).copy()
        ang = scan.angles
        near = np.zeros(len(r), dtype=bool)
        if self.cfg.min_range > 0.0:
            near |= r < self.cfg.min_range
        for c, h in self.sectors:
            in_sector = np.abs(np.angle(np.exp(1j * (ang - c)))) <= h
            near |= in_sector & (r < self.cfg.self_mask_range)
        # A body occlusion is unknown space beyond the mask, not a clear ray.
        r[near & np.isfinite(r)] = np.nan
        return Scan(scan.angle_min, scan.angle_inc, r, scan.stamp, scan.range_max)

    # ------------------------------------------------------------------ 의심 구간 감시
    def observe(self, scan: Scan, heading: float, now: float) -> List[str]:
        """원본(마스크 전) 스캔을 본다. 새로 의심하게 된 구간의 경고 문구 목록."""
        c = self.cfg
        r = scan.ranges
        ang = np.angle(np.exp(1j * scan.angles))
        ok = np.isfinite(r) & (r > 0.05) & (r < c.suspect_range)
        bins = np.floor(ang / self.BIN).astype(int)
        cur: Dict[int, float] = {}
        for b, rr in zip(bins[ok], r[ok]):
            if b not in cur or rr < cur[b]:
                cur[b] = float(rr)
        for b in list(self._track):                                # 사라진 반사는 추적을 끝낸다
            if b not in cur:
                del self._track[b]
        new_bins: List[Tuple[int, float, dict]] = []
        for b, rr in cur.items():
            tr = self._track.get(b)
            if tr is not None and abs(rr - tr["r"]) <= 0.015:
                tr["rot"] += abs(wrap(heading - tr["th"]))
                tr["th"] = heading
            else:
                self._track[b] = tr = {"r": rr, "th": heading, "rot": 0.0, "t0": now}
            if b not in self.suspects and tr["rot"] >= c.suspect_rot and now - tr["t0"] >= c.suspect_s \
                    and not self._masked(b):
                self.suspects[b] = ((b + 0.5) * math.degrees(self.BIN), rr)
                new_bins.append((b, rr, tr))
        # 이웃한 칸(같은 구조물이 여러 칸에 걸친 것)은 한 건으로 묶어 경고한다
        out: List[str] = []
        new_bins.sort(key=lambda x: x[0])
        group: List[Tuple[int, float, dict]] = []
        for item in new_bins + [None]:                           # type: ignore[list-item]
            if item is not None and group and item[0] - group[-1][0] == 1 and abs(item[1] - group[-1][1]) <= 0.05:
                group.append(item)
                continue
            if group:
                lo, hi = group[0][0], group[-1][0]
                center = 0.5 * (lo + hi + 1) * math.degrees(self.BIN)
                half = 0.5 * (hi - lo + 1) * math.degrees(self.BIN) + 2.0
                rr = min(g[1] for g in group)
                tr = group[0][2]
                out.append(f"LiDAR 자기 가림 의심: 차체 {center:+.0f}°(±{half:.0f}°) 방향 {rr:.2f} m 반사가 {now - tr['t0']:.0f}s 동안, "
                           f"로봇이 {math.degrees(tr['rot']):.0f}° 도는 사이에도 그대로다 → lidar.self_mask_sectors 에 "
                           f"(({center:.0f}, {half:.0f}),) 를 넣을지 확인")
            group = [item] if item is not None else []
        return out

    def _masked(self, b: int) -> bool:
        a = (b + 0.5) * self.BIN
        return any(abs(wrap(a - c)) <= h for c, h in self.sectors)
