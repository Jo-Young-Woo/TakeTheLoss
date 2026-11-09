"""M3 온라인 탐색: 다음 관측 시점을 '정보 가치 / (이동 + 관측 시간)'으로 고른다. 경로는 기존 Nav2/A* 가 그 지점까지 계산한다.

후보 두 종류를 한 목록에 넣고 같은 효용으로 비교한다.
  프런티어   LiDAR 로 관측한 자유 공간과 못 본 공간의 경계. 정보 = 그 점에서 새로 볼 수 있는 미관측 면적(w_frontier·셀 수).
  NBV(면)    박스의 아직 못 본/불확실한 면 앞 시점. 정보 = 그 시점에서 **볼 수 있는**(거리·입사각·시야·가림 검사 통과) 면들의 가치 합 —
             한 시점에서 여러 면(여러 박스)을 한 번에 볼 수 있으면 한 번만 가서 모두 센다. 박스가 한 개만 발견돼도 바로 후보가 된다
             (예전처럼 완전한 박스 4개 이상을 기다리지 않는다).
관측 방식: 가치가 한 방향에 모이면 'look'(그 방향 1.2 s), 흩어지면 'spin'(제자리 360°).
효용   = 정보 × (1 − 위협 위험) / (이동시간 + 관측시간 + 오버헤드). 이동시간 = 위험 인지 A*(nav/risk_astar.py) 길이 ÷ 계획속도 × 보정(가정).
시간   : t + 이동 + 관측 ≤ 사전 이동 시작(prestage_at) − 여유, 그리고 그 뒤 스테이징 지점까지의 예상 시간을 더해도 t_base_cmd 전에 도착해야 한다.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Set, Tuple

import numpy as np

from ..config import Config
from ..link.schema import scoring_sectors, server_sector
from ..nav import risk_astar
from ..nav.grid import OccupancyGrid, line_free, nearest_free
from ..perception.coverage import CoverageMap, FaceRec
from ..types import Pose2D

Pt = Tuple[float, float]


@dataclass
class Cand:
    kind: str                                  # "face" | "frontier"
    vp: Pt
    heading: float
    mode: str                                  # "look" | "spin"
    faces: List[Tuple[int, int]] = field(default_factory=list)      # 이 시점에서 볼 수 있는 면 키
    gain_f: float = 0.0
    gain_u: float = 0.0
    t_move: float = 0.0
    t_obs: float = 0.0
    risk_s: float = 0.0
    utility: float = 0.0
    path: Optional[List[Pt]] = None
    t_stage: float = 0.0
    route_vp: Optional[Pt] = None
    incumbent: bool = False                     # Explicitly refreshed current goal; outside sampling quotas.
    primary_face: Optional[Tuple[int, int]] = None  # Face that defined the planned camera direction.

    @property
    def gain(self) -> float:
        return self.gain_f + self.gain_u


class OnlineExplorer:
    def __init__(self, cfg: Config, cov: CoverageMap) -> None:
        self.cfg, self.ec, self.cov, self.geo = cfg, cfg.explore, cov, cfg.arena
        self.tabu: List[Tuple[float, float, float]] = []
        self.stats = {"selected": 0, "frontier": 0, "face": 0, "rejected_time": 0, "no_candidate": 0}
        self._last_vp: Optional[Pt] = None
        self._repeat = 0
        self._now = 0.0
        self._started: Optional[float] = None
        self._sector_observed: dict = {}
        self._surveyed: Set[str] = set()
        self.diagnostics: dict = {"last_build": {}, "last_evaluate": {}, "observed_sectors": {}, "events": []}

    def note_observation(self, cand: Cand, now: float) -> None:
        """A completed ground-camera stop services a sector; UAV alone never does."""
        sector = self.geo.sector_of(*cand.vp)
        if sector is not None:
            self._sector_observed[sector] = now
            self._surveyed.add(sector)
            counts = self.diagnostics["observed_sectors"]
            counts[sector] = counts.get(sector, 0) + 1
        events = self.diagnostics["events"]
        events.append({"t": round(now, 2), "kind": "observed", "sector": sector, "vp": cand.vp,
                       "faces": len(cand.faces)})
        del events[:-80]

    # ------------------------------------------------------------------ 후보 생성
    def mark_bad(self, vp: Pt, now: float) -> None:
        self.tabu.append((vp[0], vp[1], now + self.ec.tabu_s))

    def route_goal(self, cand: Cand, pose: Pose2D, grid: OccupancyGrid) -> Tuple[Pt, bool]:
        """Send bounded waypoints along the evaluated UAV-aware path to Nav2.

        Nav2 has not yet scanned distant UAV boxes, so sending the final view
        alone lets its independent planner take a shortcut through them.
        """
        if not cand.path or (not self.ec.complete_m3_faces and not any(
                self.cov.active_uav_prior(b) or (b.uav_t is not None and b.kind == "full")
                for b in self.cov.boxes.values())):
            return cand.vp, False
        blocked, _ = self.cov.planning_cost_maps(grid)
        here = (pose.x, pose.y)
        check_start = here
        if blocked[grid.to_cell(*here)]:
            # The path evaluator snaps an inflated start to a nearby free cell.
            # Use that same start for line checks instead of rejecting every
            # otherwise valid route merely because its first cell is inflated.
            start = cand.path[0]
            if (math.hypot(start[0] - pose.x, start[1] - pose.y) <= 0.25
                    and line_free(grid, grid.occupied(), here, start)):
                check_start = start
            else:
                return here, True
        if (cand.route_vp is not None and math.hypot(cand.route_vp[0] - pose.x, cand.route_vp[1] - pose.y) > 0.18
                and line_free(grid, blocked, check_start, cand.route_vp)):
            return cand.route_vp, True
        if math.hypot(cand.vp[0] - pose.x, cand.vp[1] - pose.y) < 0.7 and line_free(grid, blocked, check_start, cand.vp):
            return cand.vp, False
        nearest = min(range(len(cand.path)), key=lambda i: math.hypot(cand.path[i][0] - pose.x, cand.path[i][1] - pose.y))
        target = None
        for point in cand.path[nearest + 1:]:
            d = math.hypot(point[0] - pose.x, point[1] - pose.y)
            if d > 0.75 or not line_free(grid, blocked, check_start, point):
                break
            if d >= 0.25:
                target = point
        if target is None:
            # Replan instead of making a potentially obstructed direct jump.
            return here, True
        intermediate = math.hypot(target[0] - cand.vp[0], target[1] - cand.vp[1]) > 0.18
        cand.route_vp = target if intermediate else None
        return target, intermediate

    def _is_tabu(self, vp: Pt, now: float) -> bool:
        self.tabu = [b for b in self.tabu if b[2] > now]
        return any(math.hypot(vp[0] - b[0], vp[1] - b[1]) < 0.35 for b in self.tabu)

    def _free(self, grid: OccupancyGrid, blocked: np.ndarray, p: Pt) -> bool:
        iy, ix = grid.to_cell(*p)
        return grid.in_bounds(iy, ix) and not blocked[iy, ix] and self.geo.inside(p[0], p[1], 0.30)

    def _visible_set(self, vp: Pt, heading: Optional[float], pend: List[FaceRec], occ: np.ndarray, grid: OccupancyGrid,
                     extra: float, weights) -> Tuple[List[FaceRec], float]:
        got, g = [], 0.0
        for f in pend:
            if abs(f.cx - vp[0]) > 3.0 or abs(f.cy - vp[1]) > 3.0:
                continue
            if self.cov.picture_pending(f):
                ok, _ = self.cov.picture_visible(vp[0], vp[1], heading, f, occ, grid)
            else:
                ok, _ = self.cov.face_visible(vp[0], vp[1], heading, f, occ, grid, fov_extra=extra)
            if ok:
                w = weights(f, vp)
                if w > 0:
                    got.append(f)
                    g += w
        return got, g

    def build(self, pose: Pose2D, grid: OccupancyGrid, blocked: np.ndarray, now: float,
              prefer: Optional[Set[str]] = None, current: Optional[Cand] = None,
              team: Optional[str] = None) -> List[Cand]:
        ec, cov = self.ec, self.cov
        cov.tick_uav_layout(now)
        self._now = now
        if self._started is None:
            self._started = now
        allowed = None if team is None else set(scoring_sectors(team))
        # The M1 target-search sectors do not define M3 scoring territory.
        # Actual Agents always supply team; unscoped tooling remains available.
        if team is not None:
            prefer = None
        pend = cov.pending_faces(allowed)
        occ = cov.visibility_occupancy(grid)
        sweep = math.radians(25.0)                                  # 이동·관측 중 팬 스윕 진폭(_sweep_pan 와 같다)

        def wgt(f: FaceRec, vp: Pt) -> float:
            w = cov.face_weight_at(f, vp[0], vp[1])
            if prefer and self.geo.sector_of(f.cx, f.cy) in prefer:
                w *= 1.0 + ec.sector_bonus
            sector = cov.box_sector(cov.boxes[f.bid])
            # A bounded service debt prevents the nearest upper-row faces from
            # consuming the whole match while lower sectors remain unvisited.
            last = self._sector_observed.get(sector, self._started)
            w *= 1.0 + min(ec.coverage_debt_max, max(0.0, now - last) / ec.coverage_debt_s)
            return w
        out: List[Cand] = []
        seen_vp: Set[Tuple[int, int]] = set()
        # ---- (a) 미확인 면 NBV
        raw: List[Tuple[Pt, FaceRec]] = []
        for f in pend:
            tx, ty = -f.ny, f.nx
            for d in ec.nbv_dists:
                for lat in ec.nbv_lateral:
                    vp = (f.cx + f.nx * d + tx * lat, f.cy + f.ny * d + ty * lat)
                    if not self._free(grid, blocked, vp) or occ[grid.to_cell(*vp)] or self._is_tabu(vp, now):
                        continue
                    if not line_free(grid, blocked, vp, (f.cx + f.nx * 0.28, f.cy + f.ny * 0.28)):
                        continue
                    heading = math.atan2(f.cy-vp[1], f.cx-vp[0])
                    visible = (cov.picture_visible(vp[0], vp[1], heading, f, occ, grid)
                               if cov.picture_pending(f)
                               else cov.face_visible(vp[0], vp[1], heading, f, occ, grid))
                    if not visible[0]:
                        # Do not let three close but clipped/grazing samples
                        # consume a face's budget before a valid sample is tried.
                        continue
                    key = (int(round(vp[0] / 0.15)), int(round(vp[1] / 0.15)))
                    if key in seen_vp:
                        continue
                    seen_vp.add(key)
                    raw.append((vp, f))
        raw.sort(key=lambda r: math.hypot(r[0][0] - pose.x, r[0][1] - pose.y))
        per_face: dict = {}
        for vp, f in raw:                                            # 면마다 가까운 시점 3개까지(먼 면의 후보가 가까운 면에 밀려 사라지지 않게)
            choices = per_face.setdefault(f.key, [])
            if len(choices) < 3:
                choices.append((vp, f))
        # Round-robin over faces before taking alternate viewpoints. The old
        # distance-sorted [:60] silently discarded distant boxes altogether.
        picked = [choices[i] for i in range(3) for choices in per_face.values() if len(choices) > i]
        for vp, f in picked[:max(60, min(96, len(pend)))]:
            heading = math.atan2(f.cy - vp[1], f.cx - vp[0])
            got_l, g_l = self._visible_set(vp, heading, pend, occ, grid, sweep, wgt)
            if any(cov.picture_pending(x) for x in got_l):
                # The controller holds pan at zero while classifying photos.
                # Its predicted gain must use that same camera orientation.
                got_l, g_l = self._visible_set(vp, heading, pend, occ, grid, 0.0, wgt)
            got_s, g_s = self._visible_set(vp, None, pend, occ, grid, 0.0, wgt)
            if g_s <= 0:
                continue
            look = cov.picture_pending(f) or g_l >= 0.75 * g_s
            got, g = (got_l, g_l) if look else (got_s, g_s)
            photo = any(cov.picture_pending(x) for x in got)
            # Stable frontal views provide enough classification votes; simply
            # passing through the camera cone is not completion of an M3 face.
            out.append(Cand("face", vp, heading, "look" if look else "spin", [x.key for x in got], gain_f=g,
                            t_obs=(ec.m3_observe_s if photo else ec.observe_look_s) if look else self._spin_s(),
                            primary_face=f.key))
        # ---- (b) 프런티어
        solid = cov.seen.solid(grid)
        un = cov.seen.unseen(solid)
        fm = cov.seen.frontier_mask(solid)
        if allowed is not None:
            territory = np.zeros_like(un)
            for name in self.geo.sector_names():
                if server_sector(name) in allowed:
                    x0, y0, x1, y1 = self.geo.sector_rect(name)
                    territory |= ((cov.seen.X >= x0) & (cov.seen.X < x1) &
                                  (cov.seen.Y >= y0) & (cov.seen.Y < y1))
            # Only information value is scoped. The navigation occupancy grid
            # and all known/unknown obstacles remain untouched during transit.
            un &= territory
            fm &= territory
        for cx, cy, n in self._frontier_clusters(fm, cov.seen.res):
            vp = (cx, cy)
            if not self._free(grid, blocked, vp):
                nf = nearest_free(blocked, *grid.to_cell(cx, cy), max_r=8)
                if nf is None:
                    continue
                vp = grid.to_world(*nf)
                if not self._free(grid, blocked, vp):
                    continue
            if self._is_tabu(vp, now):
                continue
            if allowed is not None and server_sector(self.geo.sector_of(*vp)) not in allowed:
                continue
            gu = cov.seen.unseen_gain(vp[0], vp[1], solid, un, ec.frontier_rays, ec.frontier_radius) * ec.w_frontier
            if cov.uav_layout_state != "available":
                # Without overhead geometry, discover neglected sectors using
                # actual LiDAR frontiers rather than repeatedly servicing the
                # nearest visible boxes. No hidden arena-object map is used.
                last = self._sector_observed.get(self.geo.sector_of(*vp), self._started)
                gu *= 1.0 + min(ec.coverage_debt_max, max(0.0, now - last) / ec.coverage_debt_s)
            got_s, g_s = self._visible_set(vp, None, pend, occ, grid, 0.0, wgt)
            if gu + g_s <= 0:
                continue
            out.append(Cand("frontier", vp, 0.0, "spin", [x.key for x in got_s], gain_f=g_s, gain_u=gu, t_obs=self._spin_s()))
        # If LiDAR frontiers have vanished, search each unsurveyed sector once
        # rather than parking at the M5 staging point for the rest of M3.
        if not any(c.kind == "frontier" and c.gain >= ec.min_gain for c in out):
            for sector in self.geo.sector_names():
                if allowed is not None and server_sector(sector) not in allowed:
                    continue
                if sector in self._surveyed:
                    continue
                sx, sy = self.geo.sector_center(sector)
                options = [(sx + dx, sy + dy) for dx in (0.0, -0.45, 0.45) for dy in (0.0, -0.45, 0.45)]
                options = [p for p in options if self._free(grid, blocked, p)
                           and not occ[grid.to_cell(*p)] and not self._is_tabu(p, now)
                           and self.geo.sector_of(*p) == sector]
                if not options:
                    continue
                vp = min(options, key=lambda p: math.hypot(p[0] - pose.x, p[1] - pose.y))
                got, gf = self._visible_set(vp, None, pend, occ, grid, 0.0, wgt)
                out.append(Cand("survey", vp, 0.0, "spin", [f.key for f in got], gain_f=gf,
                                gain_u=1.2, t_obs=self._spin_s()))
        # Sampling closest viewpoints/frontier centroids must not silently
        # discard a still-useful goal that the robot is already approaching.
        # Recompute its information from current sensor evidence at the EXACT
        # destination; this candidate still has to pass a new path/deadline test.
        incumbent = self._refresh_incumbent(current, pend, occ, grid, blocked, solid, un, now, wgt, allowed)
        if incumbent is not None:
            out = [c for c in out if c.vp != incumbent.vp]
            out.append(incumbent)
        self.diagnostics["last_build"] = {"t": round(now, 2), "pending_faces": len(pend), "raw_viewpoints": len(raw),
                                          "sensor_mode": cov.layout_diagnostics()["mode"],
                                          "m3_scoring_sectors": None if allowed is None else sorted(allowed),
                                          "incumbent_requested": current is not None,
                                          "incumbent_rebuilt": incumbent is not None,
                                          "face": sum(c.kind == "face" for c in out),
                                          "frontier": sum(c.kind == "frontier" for c in out),
                                          "survey": sum(c.kind == "survey" for c in out)}
        return out

    def _refresh_incumbent(self, current, pending, occ, grid, blocked, solid, unseen, now, weights, allowed=None):
        if (current is None or not self._free(grid, blocked, current.vp)
                or occ[grid.to_cell(*current.vp)] or self._is_tabu(current.vp, now)):
            return None
        if (allowed is not None and current.kind in ("frontier", "survey")
                and server_sector(self.geo.sector_of(*current.vp)) not in allowed):
            return None
        vp, heading, mode = current.vp, current.heading, current.mode
        primary_face = current.primary_face
        # A small measured box-centre refinement may change the look heading,
        # but never moves the committed destination to another sampled point.
        if mode == "look":
            anchors = [f for f in pending if f.key in current.faces]
            if anchors:
                def turn(f):
                    delta = math.atan2(f.cy-vp[1], f.cx-vp[0]) - heading
                    return abs(math.atan2(math.sin(delta), math.cos(delta)))
                anchor = next((f for f in anchors if f.key == primary_face), None)
                if anchor is None:
                    anchor = min(anchors, key=turn)
                primary_face = anchor.key
                heading = math.atan2(anchor.cy-vp[1], anchor.cx-vp[0])
        got, gain = self._visible_set(vp, heading if mode == "look" else None, pending, occ, grid,
                                      math.radians(25.) if mode == "look" else 0., weights)
        photo = any(self.cov.picture_pending(f) for f in got)
        if mode == "look" and photo:
            got, gain = self._visible_set(vp, heading, pending, occ, grid, 0., weights)
            photo = any(self.cov.picture_pending(f) for f in got)
        gain_u = 0.
        if current.kind == "frontier":
            gain_u = self.cov.seen.unseen_gain(vp[0], vp[1], solid, unseen,
                                               self.ec.frontier_rays, self.ec.frontier_radius) * self.ec.w_frontier
            if self.cov.uav_layout_state != "available":
                last = self._sector_observed.get(self.geo.sector_of(*vp), self._started)
                gain_u *= 1. + min(self.ec.coverage_debt_max, max(0., now-last) / self.ec.coverage_debt_s)
        elif current.kind == "survey" and self.geo.sector_of(*vp) not in self._surveyed:
            gain_u = 1.2
        return Cand(current.kind, vp, heading, mode, [f.key for f in got], gain_f=gain, gain_u=gain_u,
                    t_obs=(self.ec.m3_observe_s if photo else self.ec.observe_look_s) if mode == "look" else self._spin_s(),
                    incumbent=True, primary_face=primary_face if any(f.key == primary_face for f in got) else None)

    def _spin_s(self) -> float:
        w = self.cfg.mission.spin_w
        return (2 * math.pi / w) if w > 0 else self.ec.observe_look_s

    @staticmethod
    def _frontier_clusters(mask: np.ndarray, res: float, min_cells: int = 6) -> List[Tuple[float, float, int]]:
        ys, xs = np.nonzero(mask)
        cells = set(zip(ys.tolist(), xs.tolist()))
        seen, out = set(), []
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
            if len(comp) < min_cells:
                continue
            my = sum(p[0] for p in comp) / len(comp)
            mx = sum(p[1] for p in comp) / len(comp)
            best = min(comp, key=lambda p: (p[0] - my) ** 2 + (p[1] - mx) ** 2)       # 군집 중심에 가장 가까운 실제 프런티어 셀
            out.append(((best[1] + 0.5) * res, (best[0] + 0.5) * res, len(comp)))
        return out

    # ------------------------------------------------------------------ 평가
    def evaluate(self, cands: List[Cand], pose: Pose2D, grid: OccupancyGrid, blocked: np.ndarray, cost: np.ndarray, t: float,
                 stage_pt: Pt, field=None, w_risk: float = 3.0, w_close: float = 1.5) -> List[Cand]:
        ec, mc = self.ec, self.cfg.mission
        v = max(self.cfg.nav.v_cruise * ec.plan_speed_scale, 0.05)
        deadline = mc.prestage_at - ec.deadline_margin_s
        stage_deadline = mc.t_base_cmd - ec.stage_margin_s
        good = []
        rejected = {"low_gain": 0, "deadline": 0, "no_path": 0, "path_deadline": 0}
        for c in cands:
            if c.gain < ec.min_gain:
                rejected["low_gain"] += 1
                continue
            d_eu = math.hypot(c.vp[0] - pose.x, c.vp[1] - pose.y)
            t_lb = d_eu / v                                          # 직선거리 하한: 이것만으로 마감을 넘으면 A* 를 돌릴 필요가 없다
            if t + t_lb + c.t_obs > deadline:
                self.stats["rejected_time"] += 1
                rejected["deadline"] += 1
                continue
            c.utility = c.gain / (t_lb + c.t_obs + ec.time_overhead_s)           # 직선거리 사전 순위
            good.append(c)
        good.sort(key=lambda c: -c.utility)
        incumbents = [c for c in good if c.incumbent]
        good = [c for c in good if not c.incumbent]
        # Give every sector a path check before filling remaining slots by
        # utility. Obstacles around nearby candidates must not hide a reachable
        # lower-row candidate behind the A* evaluation budget.
        representatives, sectors = [], set()
        for c in good:
            sector = self.geo.sector_of(*c.vp)
            if sector not in sectors:
                representatives.append(c)
                sectors.add(sector)
        rep_ids = {id(c) for c in representatives}
        good = representatives + [c for c in good if id(c) not in rep_ids]
        res: List[Cand] = []
        tried = incumbent_tried = accepted_new = 0
        for c in incumbents + good:
            if not c.incumbent and (accepted_new >= ec.max_eval or tried >= 3 * ec.max_eval):
                break
            if c.incumbent:
                incumbent_tried += 1
            else:
                tried += 1
            pr = risk_astar.plan(grid, blocked, cost, (pose.x, pose.y), c.vp, v, field, w_risk, w_close)
            if pr is None:
                rejected["no_path"] += 1
                continue
            c.path = pr.path
            c.t_move = pr.length * ec.travel_factor / v
            c.t_stage = math.hypot(stage_pt[0] - c.vp[0], stage_pt[1] - c.vp[1]) * 1.3 / v
            fin = t + c.t_move + c.t_obs
            if fin > deadline or fin + c.t_stage > stage_deadline:
                self.stats["rejected_time"] += 1
                rejected["path_deadline"] += 1
                continue
            c.risk_s = pr.risk_int
            tot = c.t_move + c.t_obs + ec.time_overhead_s
            c.utility = c.gain * (1.0 - min(0.9, ec.w_threat * c.risk_s)) / tot
            res.append(c)
            accepted_new += not c.incumbent
        res.sort(key=lambda c: -c.utility)
        start_cell = grid.to_cell(pose.x, pose.y)
        snapped_start = nearest_free(blocked, *start_cell)
        field_start_blocked = (bool(field.body[0][snapped_start])
                               if field is not None and snapped_start is not None else False)
        self.diagnostics["last_evaluate"] = {"t": round(t, 2), "candidates": len(cands), "path_checks": tried + incumbent_tried,
                                             "incumbent_path_checks": incumbent_tried,
                                             "incumbent_accepted": any(c.incumbent for c in res),
                                             "new_candidate_path_checks": tried,
                                             "accepted": len(res), "rejected": rejected,
                                             "start_blocked": bool(blocked[grid.to_cell(pose.x, pose.y)]),
                                             "start_observed_occupied": bool(grid.occupied()[grid.to_cell(pose.x, pose.y)]),
                                             "snapped_start": None if snapped_start is None else grid.to_world(*snapped_start),
                                             "field_blocked_start": field_start_blocked,
                                             "prior_free_overrides": self.cov.stats.get("uav_prior_free_overrides", 0)}
        self.diagnostics["stats"] = dict(self.stats)
        return res

    def choose(self, ranked: List[Cand], cur: Optional[Cand]) -> Optional[Cand]:
        if not ranked:
            self.stats["no_candidate"] += 1
            return None
        best = ranked[0]
        if cur is not None:
            same = [c for c in ranked if c.incumbent and c.vp == cur.vp]
            if not same:
                same = [c for c in ranked if math.hypot(c.vp[0] - cur.vp[0], c.vp[1] - cur.vp[1]) < 0.3]
            if same and same[0].utility * self.ec.switch_ratio >= best.utility:
                n = same[0]                                          # 히스테리시스: 조금 나은 후보로 갈아타지 않는다 — 같은 객체를 갱신해 돌려준다
                if n.vp != cur.vp:
                    cur.route_vp = None
                # A legacy nearby substitute must use the destination and
                # heading actually checked by A*, rather than its old endpoint.
                cur.vp, cur.heading, cur.mode, cur.t_obs, cur.t_stage = n.vp, n.heading, n.mode, n.t_obs, n.t_stage
                cur.primary_face = n.primary_face
                cur.gain_f, cur.gain_u, cur.faces, cur.t_move, cur.utility, cur.path, cur.risk_s = (
                    n.gain_f, n.gain_u, n.faces, n.t_move, n.utility, n.path, n.risk_s)
                return cur
        # 반복 방지: 같은 시점을 연속 3번째 고르면 쉬게 하고(tabu) 다음 후보로 간다
        if self._last_vp is not None and math.hypot(best.vp[0] - self._last_vp[0], best.vp[1] - self._last_vp[1]) < 0.3:
            self._repeat += 1
        else:
            self._repeat = 0
        self._last_vp = best.vp
        if self._repeat >= 2:
            self.mark_bad(best.vp, self._now)
            self.stats["loop_breaks"] = self.stats.get("loop_breaks", 0) + 1
            self._repeat, self._last_vp = 0, None
            rest = [c for c in ranked if not self._is_tabu(c.vp, self._now)]
            if not rest:
                return None
            best = rest[0]
        self.stats["selected"] += 1
        self.stats[best.kind] = self.stats.get(best.kind, 0) + 1
        events = self.diagnostics["events"]
        events.append({"t": round(self._now, 2), "kind": "selected", "candidate": best.kind,
                       "vp": best.vp, "gain": round(best.gain, 3), "utility": round(best.utility, 3)})
        del events[:-80]
        return best
