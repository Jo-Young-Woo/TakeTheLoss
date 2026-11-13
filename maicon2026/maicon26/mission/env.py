"""환경·보조 신호 감시기 (상태전이표 6장 '이벤트 요소'). 로봇·시뮬 없이 단위 시험이 된다.

SmogMonitor  스모그 발생장치(포그머신)가 만든 연기를 LiDAR '고립 반사'(연기 입자)와 영상 대비 저하로 감지한다. 서버 이벤트가 오면 그것을 따른다.
speckle_filter  스모그 중 고립된 가까운 반사를 지운다 — 안 지우면 전방 17 cm 정지 규칙이 허상에 걸려 로봇이 멈추고 지도에 허상이 쌓인다.
UavMonitor   드론 영상 주기 변화(1 s → 2 s → 5 s → 중단)를 '새 프레임 도착 간격'으로 측정한다.
TowerMonitor 3색 타워 LED 상태(준비·진행·완료)를 연속 프레임으로 확인한다.
"""
from __future__ import annotations

import statistics
from dataclasses import replace
from collections import deque
from typing import Deque, List, Optional, Tuple

import numpy as np

from ..config import EventCfg
from ..types import Scan


# ====================================================================== E3 스모그
def isolated_near_mask(scan: Scan, near_m: float, nbr_m: float) -> np.ndarray:
    """가까운(near_m 미만) 유효 반사 중 '고립'된 것(이웃 빔 ±2 어디와도 거리가 nbr_m 안으로 맞지 않는 것)의 마스크.
    같은 면의 점들은 이웃과 거리가 비슷하지만 연기 입자의 반사는 제각각이다."""
    r0 = scan.ranges.astype(float)
    n = len(r0)
    valid = np.isfinite(r0) & (r0 > 0.05)
    r = np.where(valid, r0, 0.0)                 # 무효(inf/nan)는 0 으로 두고 valid 마스크로 거른다(inf−inf 경고 방지)
    near = valid & (r < near_m)
    support = np.zeros(n, dtype=bool)
    for k in (-2, -1, 1, 2):
        rk, vk = np.roll(r, k), np.roll(valid, k)
        support |= vk & (np.abs(r - rk) <= nbr_m)
    return near & ~support


def speckle_filter(scan: Scan, near_m: float, nbr_m: float) -> Scan:
    iso = isolated_near_mask(scan, near_m, nbr_m)
    if not iso.any():
        return scan
    r = scan.ranges.astype(float).copy()
    # Removing a suspected particle does not reveal anything behind it.
    r[iso] = np.nan
    return replace(scan, ranges=r)


def image_contrast(image: Optional[np.ndarray]) -> Optional[float]:
    """영상 대비 지표(그레이 표준편차). 연기가 끼면 낮아진다. 영상이 없으면(시뮬) None."""
    if image is None:
        return None
    g = image if image.ndim == 2 else image.mean(axis=2)
    return float(np.std(g[::4, ::4]))


class SmogMonitor:
    def __init__(self, cfg: EventCfg) -> None:
        self.cfg = cfg
        self.active = False
        self.index = 0.0                        # 최근 고립 반사 비율
        self.forced: Optional[bool] = None      # 서버 이벤트가 알려준 상태(있으면 그것을 따른다)
        self.changes = 0
        self._cand_since: Optional[float] = None
        self._clear_since: Optional[float] = None
        self._contrast: Deque[float] = deque(maxlen=120)
        self._baseline: Optional[float] = None
        self.contrast_low = False

    def force(self, on: Optional[bool]) -> Optional[bool]:
        """서버 이벤트(smog on/off). None 이면 센서 판단으로 되돌린다. 상태가 바뀌면 새 상태를 돌려준다."""
        self.forced = on
        if on is not None and on != self.active:
            self.active, self._cand_since, self._clear_since = on, None, None
            self.changes += 1
            return on
        return None

    def update(self, scan: Scan, now: float, contrast: Optional[float] = None) -> Optional[bool]:
        """새 스캔마다 호출. 상태가 바뀌었으면 새 상태(True/False), 아니면 None."""
        c = self.cfg
        if not c.smog_enable:
            return None
        # 고립된 가까운 반사 수 / 전체 빔 수. 분모를 '가까운 반사 수'로 잡으면 벽·박스가 가까운 구역(시작 거점 등)에서 비율이 희석돼
        # 연기를 놓친다(실측: 가까운 반사 180~227개 중 가짜 24~38개 → 0.12~0.19). 정상 장면의 고립 반사는 0~2개다.
        iso = int(isolated_near_mask(scan, c.smog_near_m, c.speckle_nbr_m).sum())
        self.index = iso / max(len(scan.ranges), 1)
        if contrast is not None:
            self._contrast.append(contrast)
            if self._baseline is None and len(self._contrast) >= 40:
                self._baseline = float(statistics.median(self._contrast))     # 시작 직후(연기 전) 값을 기준으로
            if self._baseline:
                self.contrast_low = float(statistics.median(list(self._contrast)[-10:])) < c.smog_contrast_drop * self._baseline
        if self.forced is not None:
            return None
        suspect = self.index >= c.smog_on_ratio or (self.contrast_low and self.index >= 0.5 * c.smog_on_ratio)
        clear = self.index <= c.smog_off_ratio and not self.contrast_low
        if not self.active:
            self._cand_since = (self._cand_since if self._cand_since is not None else now) if suspect else None
            if self._cand_since is not None and now - self._cand_since >= c.smog_on_s:
                self.active, self._cand_since = True, None
                self.changes += 1
                return True
        else:
            self._clear_since = (self._clear_since if self._clear_since is not None else now) if clear else None
            if self._clear_since is not None and now - self._clear_since >= c.smog_off_s:
                self.active, self._clear_since = False, None
                self.changes += 1
                return False
        return None


# ====================================================================== E4 드론 영상 주기
class UavMonitor:
    """새 프레임이 '도착'하는 간격으로 주기를 잰다(stamp 는 서버 시계라 도착 시각을 쓴다).
    state: unknown(아직 못 봄) | normal(≤1.5 s) | slow2(≤3.5 s) | slow5(≤8 s) | off(새 프레임 없음 uav_off_s 이상)."""

    def __init__(self, cfg: EventCfg) -> None:
        self.cfg = cfg
        self.state = "unknown"
        self.period = 1.0
        self._last_arrival: Optional[float] = None
        self._intervals: Deque[float] = deque(maxlen=5)
        self.changes: List[Tuple[float, str]] = []

    def observe(self, now: float) -> None:
        """새 stamp 의 프레임이 도착했을 때 호출."""
        if self._last_arrival is not None:
            self._intervals.append(now - self._last_arrival)
        self._last_arrival = now
        if len(self._intervals) >= 2:
            self.period = float(statistics.median(self._intervals))
        self._set(self._classify(), now)

    def tick(self, now: float) -> None:
        """매 틱 호출: 한참 새 프레임이 없으면 중단으로 본다."""
        if self._last_arrival is not None and now - self._last_arrival >= max(self.cfg.uav_off_s, 0.0):
            self._set("off", now)

    def _classify(self) -> str:
        p = self.period
        return "normal" if p <= 1.5 else "slow2" if p <= 3.5 else "slow5"

    def _set(self, state: str, now: float) -> None:
        if state != self.state:
            self.changes.append((now, state))
            self.state = state

    def track_max_age(self, base: float = 3.0) -> float:
        """UGV 트랙을 '오래됨'으로 보지 않는 최대 나이. 영상이 2 s·5 s 에 한 장이면 그 사이에 트랙이 사라지면 안 된다."""
        return max(base, 1.6 * min(self.period, 6.0)) if self.state in ("slow2", "slow5") else base


# ====================================================================== E2 3색 타워 LED
class TowerMonitor:
    def __init__(self, cfg: EventCfg) -> None:
        self.cfg = cfg
        self._obs: Deque[Tuple[float, str]] = deque(maxlen=200)

    def state_of(self, color: Optional[str]) -> Optional[str]:
        c = self.cfg
        return {c.tower_ready_color: "ready", c.tower_running_color: "running", c.tower_finished_color: "finished"}.get(color or "")

    def observe(self, color: Optional[str], now: float) -> Optional[str]:
        st = self.state_of(color)
        if st is not None:
            self._obs.append((now, st))
        return st

    def confirmed(self, state: str, now: float) -> bool:
        """최근 tower_confirm_s 동안의 관측이 충분히 많고(≥8), 80 % 이상이 그 상태이며, 그 구간을 대부분 덮는다."""
        w = self.cfg.tower_confirm_s
        recent = [(t, s) for t, s in self._obs if now - t <= w]
        if len(recent) < 8:
            return False
        same = sum(1 for _, s in recent if s == state)
        return same / len(recent) >= 0.8 and (recent[-1][0] - recent[0][0]) >= 0.8 * w
