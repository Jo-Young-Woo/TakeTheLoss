"""사전 서브게임 시뮬: 뒤돌아 시작, 약 3 m 앞 레이더 시설. 2회 시도 → 빠른 기록."""
from __future__ import annotations

import math
import random
from typing import Dict, List, Optional

from ..config import Config
from ..mission.subgame import SubgameRunner
from ..types import Pose2D
from .robot import SimRobot, SimServer
from .world import Cube, Item, World


def build_subgame(seed: int = 0, cfg: Optional[Config] = None, dist: float = 3.0, lateral: float = 0.0,
                  face: int = 2):
    """face: 시설에서 로봇을 향하는 면(0=E,1=N,2=W,3=S)."""
    cfg = cfg or Config()
    rng = random.Random(seed)
    w = World(cfg, seed=seed)
    w.cubes.clear()
    u = w.add_ugv("BLUE", 2)
    sx, sy = 1.3 + rng.uniform(-0.1, 0.1), 2.5 + rng.uniform(-0.3, 0.3)
    u.pose = Pose2D(sx, sy, math.pi + rng.uniform(-0.25, 0.25))   # 표적 반대 방향(180°)을 보고 시작
    c = Cube(sx + dist, sy + lateral)
    c.items[face] = Item("target", "RADAR_FACILITY")
    w.cubes.append(c)
    r = SimRobot(w, u)
    sv = SimServer(w, u)
    run = SubgameRunner(r, sv, cfg=cfg)
    return w, run


def run_try(seed: int = 0, cfg: Optional[Config] = None, dt: float = 0.05, **kw) -> Dict:
    w, run = build_subgame(seed, cfg, **kw)
    while w.t < 61.0 and run.phase.value not in ("DONE", "TIMEOUT"):
        run.tick()
        w.step(dt)
    return {"ok": run.res.success, "t": run.res.t_hit, "shots": run.res.shots, "phases": run.res.phase_log}


def best_of_two(seed: int = 0, cfg: Optional[Config] = None, **kw) -> Dict:
    tries: List[Dict] = [run_try(seed * 2 + i, cfg, **kw) for i in range(2)]
    oks = [t["t"] for t in tries if t["ok"]]
    return {"best": min(oks) if oks else None, "tries": tries}
