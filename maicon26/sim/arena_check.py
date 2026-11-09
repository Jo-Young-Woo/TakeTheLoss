"""경기장 정적 유효성 검사 — 무작위로 만든 배치가 *실제 차체로* 경기를 할 수 있는지, Agent 를 돌리기 전에 확인한다.

검사 항목(하나라도 실패하면 그 배치는 경기와 점수 집계에서 제외하고 다시 만든다)
  placement      박스를 못 놓은 구역이 없다(조용히 건너뛰지 않는다) / 박스 사이 이격 ≥ arena.min_gap [PDF p24]
  content        양 진영에 필수 표적 종류(config.perception.target_classes)가 각각 1개씩, 객체 박스가 3개씩 있다
  start          6개 출발 거점 자세에서 직사각형 차체(+계획 여유)가 박스·벽과 안 겹친다
  goal           각 출발 거점에서 상대 진영 3개 목표거점 각각의 중심까지, **실제 직사각형 + 오차 여유**를 적용한 주행 경로가 있다
  target         각 출발 거점에서 지정될 수 있는 표적 3종 각각에 대해, 접근·조준 자세(차체가 면을 향하고 사선이 안 가려짐)까지 주행 가능한 표적 면이 있다
로봇 중심점만 지나는 A* 경로는 통과 판정으로 쓰지 않는다: `nav/fpplan.plan_footprint` 가 진행 방향 직사각형과 꼭짓점 회전 공간을 검증한다
(내장 항법이 쓰는 함수와 같다 → 검사와 실제 주행의 기준이 어긋나지 않는다).

!! 정적 검사다. 경기 중 구조물이 바뀌는 시나리오(shuffle_structures)는 별도이며 이 검사를 다시 하지 않는다.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

from ..config import Config
from ..footprint import rect_cells_free
from ..nav.fpplan import plan_footprint
from ..nav.grid import OccupancyGrid


class ArenaGenerationError(RuntimeError):
    """재시도 상한까지 유효한 배치를 만들지 못했다. .meta 에 시도별 제외 사유가 있다."""

    def __init__(self, msg: str, meta: Dict[str, Any]) -> None:
        super().__init__(msg)
        self.meta = meta


@dataclass
class ArenaReport:
    ok: bool
    reasons: List[str] = field(default_factory=list)
    stats: Dict[str, Any] = field(default_factory=dict)


FACE_N = [(1, 0), (0, 1), (-1, 0), (0, -1)]


def static_grid(world, cfg: Config) -> OccupancyGrid:
    """박스를 칸 *중심*이 사각형 안에 있는 칸으로 래스터화한다(OccupancyGrid.add_static_box 는 경계가 걸친 칸까지 포함해 0.05 m 두꺼워질 수 있어
    정확히 40 cm 인 통로를 잘못 막는다). LiDAR 끝점이 표면 칸에 찍히는 실제 점유와 같은 크기 오차(±반 칸) 안이다."""
    g = OccupancyGrid(cfg.arena, cfg.nav.res)
    xs = g.ox + (np.arange(g.nx) + 0.5) * g.res
    ys = g.oy + (np.arange(g.ny) + 0.5) * g.res
    for c in world.cubes:
        x0, y0, x1, y1 = c.rect()
        mx = (xs >= x0 - 1e-9) & (xs <= x1 + 1e-9)
        my = (ys >= y0 - 1e-9) & (ys <= y1 + 1e-9)
        g.static[np.ix_(my, mx)] = True
    return g


def _start_pose(cfg: Config, team: str, slot: int) -> Tuple[float, float, float]:
    cx, cy = cfg.arena.base_center(f"{'B' if team == 'BLUE' else 'R'}-{slot}")
    return cx + (0.15 if team == "BLUE" else -0.15), cy, (0.0 if team == "BLUE" else math.pi)


def check_arena(world, cfg: Optional[Config] = None, full: bool = True) -> ArenaReport:
    cfg = cfg or world.cfg
    geo = cfg.arena
    bud = cfg.budget()
    reasons: List[str] = []
    stats: Dict[str, Any] = {"n_cubes": len(world.cubes)}
    # ---- placement: 못 놓은 박스, 이격
    for s in getattr(world, "layout_failures", []):
        reasons.append(f"placement:{s}")
    cubes = world.cubes
    for i in range(len(cubes)):
        for j in range(i + 1, len(cubes)):
            gap = max(abs(cubes[i].cx - cubes[j].cx), abs(cubes[i].cy - cubes[j].cy)) - 2 * cubes[i].half
            if gap < geo.min_gap - 1e-6:
                reasons.append(f"spacing:{gap:.2f}m<{geo.min_gap:.2f}m")
    # ---- content: 양 진영 필수 표적/객체
    need = set(cfg.perception.target_classes)
    for side, (lo, hi) in (("B", (0.0, geo.W / 2)), ("R", (geo.W / 2, geo.W))):
        tg, ob = [], 0
        for c in cubes:
            if not (lo <= c.cx < hi):
                continue
            kinds = {it.kind for it in c.items.values()}
            if "target" in kinds:
                tg += [it.cls for it in c.items.values() if it.kind == "target"]
            elif "object" in kinds:
                ob += 1
        if sorted(tg) != sorted(need):
            reasons.append(f"content:{side}-targets={sorted(tg)} need={sorted(need)}")
        if ob < 3:
            reasons.append(f"content:{side}-object-boxes={ob}<3")
    if reasons and not full:
        return ArenaReport(False, reasons, stats)
    # ---- start: 직사각형이 안 겹친다
    grid = static_grid(world, cfg)
    occ = grid.occupied()
    hl, hw = bud.fp.half_l + bud.plan_margin, bud.fp.half_w + bud.plan_margin
    starts = {}
    for team in ("BLUE", "RED"):
        for slot in (1, 2, 3):
            x, y, th = _start_pose(cfg, team, slot)
            starts[(team, slot)] = (x, y, th)
            if not rect_cells_free(occ, grid.res, grid.ox, grid.oy, x, y, th, hl, hw):
                reasons.append(f"start:{team[0]}-{slot}")
    if reasons and not full:
        return ArenaReport(False, reasons, stats)
    bad_starts = {r.split(":")[1] for r in reasons if r.startswith("start:")}
    # ---- goal: 각 출발 → 상대 3개 거점 중심
    n_plans = 0
    for (team, slot), (x, y, th) in starts.items():
        if f"{team[0]}-{slot}" in bad_starts:
            continue
        opp = "RED" if team == "BLUE" else "BLUE"
        for b in geo.bases(opp):
            gx, gy = geo.base_center(b)
            n_plans += 1
            if plan_footprint(grid, cfg.nav.inflate_hard, cfg.nav.inflate_soft, cfg.nav.soft_weight, bud, (x, y), (gx, gy), th) is None:
                reasons.append(f"goal:{team[0]}-{slot}->{b}")
    # ---- target: 각 출발 × 표적 3종 → 접근·조준 자세
    faces_by_cls: Dict[str, List[Tuple[float, float, int]]] = {}
    for c in cubes:
        for fi, it in c.items.items():
            if it.kind == "target":
                faces_by_cls.setdefault(it.cls, []).append((c.cx, c.cy, fi))
    blocked_ref, _ = grid.cost_maps(cfg.nav.inflate_hard, cfg.nav.inflate_soft, cfg.nav.soft_weight)
    for (team, slot), (x, y, th) in starts.items():
        if f"{team[0]}-{slot}" in bad_starts:
            continue
        for cls in need:
            ok = False
            for (cx, cy, fi) in faces_by_cls.get(cls, []):
                if ok:
                    break
                nx, ny = FACE_N[fi]
                fcx, fcy = cx + 0.2 * nx, cy + 0.2 * ny
                for d in (cfg.mission.strike_range, 0.85, 0.65):
                    if ok:
                        break
                    for lat in (0.0, 0.15, -0.15):
                        px, py = fcx + nx * d - ny * lat, fcy + ny * d + nx * lat
                        pth = math.atan2(fcy - py, fcx - px)
                        if not rect_cells_free(occ, grid.res, grid.ox, grid.oy, px, py, pth, hl, hw):
                            continue
                        if world._blocked(px, py, fcx + nx * 0.02, fcy + ny * 0.02):
                            continue                                          # 사선이 다른 박스에 가려짐
                        n_plans += 1
                        if plan_footprint(grid, cfg.nav.inflate_hard, cfg.nav.inflate_soft, cfg.nav.soft_weight, bud, (x, y), (px, py), th) is not None:
                            ok = True
                            break
            if not ok:
                reasons.append(f"target:{team[0]}-{slot}:{cls}")
    stats["n_plans"] = n_plans
    return ArenaReport(not reasons, reasons, stats)

