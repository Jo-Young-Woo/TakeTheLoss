"""전략 비교용 시뮬 벤치: 같은 시드·같은 배치에서 설정만 바꿔 M3 정밀도·재현율, 이동 거리, 중복 방문, M5, 피격·충돌을 모은다.

!! 시뮬 수치이며 실기 성능이 아니다(시뮬의 검출 누락·잡음·상대 행동 가정에 의존).
"""
from __future__ import annotations

import math
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

from ..config import Config
from .dynamic import shuffle_structures
from .run import build_match, run, score


def make_cfg(strategy: str = "online", depth: bool = False, backend: str = "builtin", mut: Optional[Callable[[Config], None]] = None) -> Config:
    cfg = Config()
    cfg.nav.backend = backend
    cfg.depth.enable = depth
    cfg.mission.spin_w, cfg.mission.tour_nx, cfg.mission.tour_ny = 1.8, 3, 3        # 시험 공통(기존 시험·벤치와 같은 값)
    cfg.explore.strategy = strategy
    if mut:
        mut(cfg)
    return cfg


class Probe:
    """매 스텝 호출: 이동 거리(235 s 시점)·중복 방문·전이 횟수·적과의 최소 거리·노출 시간·피격 시각을 모은다."""

    def __init__(self, uid: str, t_mark: float = 235.0, cell: float = 0.3, revisit_gap_s: float = 15.0) -> None:
        self.uid, self.t_mark, self.cell, self.gap = uid, t_mark, cell, revisit_gap_s
        self.dist_mark: Optional[float] = None
        self._cells: Dict[Tuple[int, int], float] = {}
        self.samples = self.revisits = 0
        self._last_sample = -1.0
        self.modes: List[Tuple[float, str]] = []
        self.min_dist = math.inf
        self.exposure_s = 0.0
        self.hits_t: List[float] = []
        self._hits = 0

    def __call__(self, m, hostiles: Sequence[Any] = ()) -> None:
        w = m.world
        u = next(x for x in w.ugvs if x.id == self.uid)
        if self.dist_mark is None and w.t >= self.t_mark:
            self.dist_mark = u.distance
        if w.t - self._last_sample >= 0.2:
            self._last_sample = w.t
            k = (int(u.pose.x / self.cell), int(u.pose.y / self.cell))
            t0 = self._cells.get(k)
            self.samples += 1
            if t0 is not None and w.t - t0 > self.gap:
                self.revisits += 1
            if t0 is None or w.t - t0 > self.gap:
                self._cells[k] = w.t
        ag = next((a for a in m.agents if a.name == self.uid), None)
        if ag is not None and (not self.modes or self.modes[-1][1] != ag.mode.value):
            self.modes.append((w.t, ag.mode.value))
        if u.hits_taken > self._hits:
            self.hits_t += [w.t] * (u.hits_taken - self._hits)
            self._hits = u.hits_taken
        for h in hostiles:
            if not getattr(h, "spawned", True):
                continue  # A future encounter has no distance/exposure yet.
            d = math.hypot(h.u.pose.x - u.pose.x, h.u.pose.y - u.pose.y) - 0.26
            self.min_dist = min(self.min_dist, d)
            if d < w.cfg.laser.max_range and h.can_hit(u):
                self.exposure_s += 0.05


def chatter(modes: List[Tuple[float, str]], within_s: float = 2.0) -> int:
    """모드가 A→B→A 로 within_s 안에 되돌아온 횟수(상태 전이 진동)."""
    n = 0
    for i in range(2, len(modes)):
        if modes[i][1] == modes[i - 2][1] and modes[i][0] - modes[i - 2][0] < within_s:
            n += 1
    return n


def run_trial(seed: int, strategy: str = "online", shuffle: Sequence[float] = (), uid: str = "B-UGV-2", team: str = "B2",
              cfg: Optional[Config] = None, setup: Optional[Callable[[Any], Sequence[Any]]] = None, max_t: float = 340.0,
              events: Optional[List[Tuple[float, Callable]]] = None, layout_seed: Optional[int] = None) -> Dict[str, Any]:
    """setup(match) -> hostiles(리스트; 각각 .u, .can_hit(u), .step(match))  — 적대 상대를 만든다."""
    cfg = cfg or make_cfg(strategy)
    m = build_match(seed=seed, cfg=cfg, teams=[team], layout_seed=layout_seed)
    hostiles = list(setup(m)) if setup else []
    probe = Probe(uid)
    evs = [(t, shuffle_structures) for t in shuffle] + list(events or [])

    def tick(mm) -> None:
        for h in hostiles:
            h.step(mm)
        probe(mm, hostiles)
    run(m, max_t=max_t, events=evs, probe=tick)
    sc = score(m)["agents"][uid]
    a = m.agents[0]
    u = next(x for x in m.world.ugvs if x.id == uid)
    m3 = sc["m3"] or {"recall": 0.0, "precision": 0.0, "n_sub": 0, "n_truth": 0}
    return {"seed": seed, "recall": m3["recall"], "precision": m3["precision"], "n_sub": m3["n_sub"], "n_truth": m3["n_truth"],
            "m5": sc["m5_reached"], "m2_ok": sc["m2_ok"], "dist_235": probe.dist_mark if probe.dist_mark is not None else u.distance,
            "dist": sc["distance"], "collisions": sc["collisions"], "hits_taken": u.hits_taken,
            "revisit": probe.revisits / max(probe.samples, 1), "chatter": chatter(probe.modes),
            "min_dist": None if math.isinf(probe.min_dist) else probe.min_dist, "exposure_s": probe.exposure_s,
            "first_hit_t": probe.hits_t[0] if probe.hits_t else None, "stats": a.stats, "agent": a}


def summarize(rows: List[Dict[str, Any]]) -> Dict[str, Any]:
    n = max(len(rows), 1)
    avg = lambda k: sum(r[k] for r in rows if r[k] is not None) / max(sum(1 for r in rows if r[k] is not None), 1)
    m5 = [r["m5"] for r in rows if r["m5"] is not None]
    return {"n": len(rows), "recall": avg("recall"), "precision": avg("precision"), "dist_235": avg("dist_235"), "revisit": avg("revisit"),
            "m5_rate": len(m5) / n, "m5_t": (sum(m5) / len(m5)) if m5 else None, "collisions": avg("collisions"), "hits": avg("hits_taken"),
            "m2_rate": sum(1 for r in rows if r["m2_ok"]) / n, "chatter": avg("chatter"),
            "min_dist": min((r["min_dist"] for r in rows if r["min_dist"] is not None), default=None),
            "exposure_s": avg("exposure_s")}
