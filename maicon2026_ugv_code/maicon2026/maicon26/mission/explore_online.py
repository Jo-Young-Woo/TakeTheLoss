"""M3 온라인 탐색: 다음 관측 시점을 '정보 가치 / (이동 + 관측 시간)'으로 고른다. 경로는 기존 Nav2/A* 가 그 지점까지 계산한다.

후보 두 종류를 한 목록에 넣고 같은 효용으로 비교한다.
  프런티어   LiDAR 로 관측한 자유 공간과 못 본 공간의 경계. 정보 = 그 점에서 새로 볼 수 있는 미관측 면적(w_frontier·셀 수).
  NBV(면)    박스의 아직 못 본/불확실한 면 앞 시점. 정보 = 그 시점에서 **볼 수 있는**(거리·입사각·시야·가림 검사 통과) 면들의 가치 합 —
             한 시점에서 여러 면(여러 박스)을 한 번에 볼 수 있으면 한 번만 가서 모두 센다. 박스가 한 개만 발견돼도 바로 후보가 된다
             (예전처럼 완전한 박스 4개 이상을 기다리지 않는다).
  정찰(scout) explore.lidar_faces 일 때만. LiDAR 면 지도(perception/lidar_faces.py)에서 '닫힌 도형'이 안 나온 장애물 그룹의 가려진 쪽(아직 광선이
             지나가지 않은 칸) 가까이 가서 뒷면을 LiDAR 로 훑는다. 정보 = lidar_open_bonus + 그 자리에서 새로 볼 수 있는 면. 같은 쪽을 lidar_scout_max_attempts 번
             가 보고도 안 닫히면 포기한다(UGV 가 가려서 판정 보류인 그룹은 정찰하지 않는다).
  식별(identify) oak_det.identify_fallback 일 때만. 팬틸트가 그 면에 identify_min_attempts 번 가 보고도 못 푼 면(미식별)을 OAK RGB + YOLO 로 다시 식별하러
             면 앞 identify_dist_m 에 가서 몸을 면 쪽으로 정렬한 채 identify_dwell_s 동안 머문다(그동안만 OAK 검출을 켠다 — manager._oak_wanted).
             같은 면을 identify_max_attempts 번 가 보고도 못 풀면 포기한다.
             oak_det.identify_primary 이면 팬틸트가 먼저 가 보지 않는다 — LiDAR 로 찾은 미식별 면은 처음부터 OAK 로 식별한다. 면 그림이 시야에 온전히 들어오는
             최소 거리(coverage.oak_fit_min_dist) 밖에서 보고, 못 보는 면(잘림·프레임 없음·보일 자리 없음·검출 없음)은 팬틸트 폴백으로 넘긴다.
관측 방식: 가치가 한 방향에 모이면 'look'(그 방향 1.2 s), 흩어지면 'spin'(제자리 360°).
효용   = 정보 × (1 − 위협 위험) / (이동시간 + 관측시간 + 오버헤드). 이동시간 = 위험 인지 A*(nav/risk_astar.py) 길이 ÷ 계획속도 × 보정(가정).
시간   : t + 이동 + 관측 ≤ 사전 이동 시작(prestage_at) − 여유, 그리고 그 뒤 스테이징 지점까지의 예상 시간을 더해도 t_base_cmd 전에 도착해야 한다.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Set, Tuple

import numpy as np

from ..config import Config
from ..link.schema import scoring_sectors, server_sector
from ..nav import risk_astar
from ..nav.grid import OccupancyGrid, line_free, nearest_free
from ..perception.coverage import CoverageMap, FaceRec, oak_fit_min_dist
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
    scout_key: Optional[Tuple[int, int]] = None    # kind == "scout": (LiDAR 그룹 번호, 가려진 쪽 0~3=E N W S) — 시도 횟수 기준

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
        self._scout_tries: Dict[Tuple[int, int], int] = {}
        self._open_scouts: Dict[Tuple[int, int], Pt] = {}
        self._scout_vps: Dict[Tuple[int, int], Tuple[int, int]] = {}
        self._identify_vps: Dict[Tuple[int, int], Tuple[int, int]] = {}
        self.identify_ready = True                                  # OAK 프레임이 실제로 들어오고 있나(manager 가 매번 갱신). 아니면 식별 후보를 만들지 않는다
        self.oak_fov: Optional[Tuple[float, float]] = None          # (가로, 세로) 화각[deg] — manager 가 CameraInfo 의 K 로 갱신. None 이면 oak_det.fit_*_deg
        self._noview_since: Dict[Tuple[int, int], float] = {}       # identify_primary: OAK 로 볼 자리가 없던 면 → 그 상태가 시작된 시각
        self.m1_tier: Dict[str, Any] = {}                           # M1 지정 구역 우선순위 상태(보고용): enemy·cm 섹터, 적 섹터 완료·CM 보류 여부
        self._m1_key: Optional[frozenset] = None
        self._m1_t0: Optional[float] = None
        self._m1_cm_released = False
        self.m1_sectors: Set[str] = set()                           # M1 이 지정한 탐색 섹터(서버 이름) — manager 가 갱신. explore.m1_sector_sweep 가 켜져 있을 때만 쓴다
        self.diagnostics: dict = {"last_build": {}, "last_evaluate": {}, "observed_sectors": {}, "events": []}

    def note_observation(self, cand: Cand, now: float, outcome: Optional[str] = None, pose: Optional[Pose2D] = None) -> None:
        """A completed ground-camera stop services a sector; UAV alone never does.
        outcome / pose: OAK 식별 단계의 결과(cropped·no_frames·none·seen)와 그때 자세 — coverage.mark_identify 가 폴백·독립 관측 계산에 쓴다."""
        sector = self.geo.sector_of(*cand.vp)
        if sector is not None:
            self._sector_observed[sector] = now
            self._surveyed.add(sector)
            counts = self.diagnostics["observed_sectors"]
            counts[sector] = counts.get(sector, 0) + 1
        if cand.kind == "scout" and cand.scout_key is not None:
            self._scout_tries[cand.scout_key] = self._scout_tries.get(cand.scout_key, 0) + 1
        if cand.kind == "identify":
            self.cov.mark_identify(list(cand.faces), outcome=outcome, pose=pose, now=now)
        events = self.diagnostics["events"]
        events.append({"t": round(now, 2), "kind": "observed", "sector": sector, "vp": cand.vp,
                       "faces": len(cand.faces)})
        del events[:-80]

    # ------------------------------------------------------------------ 후보 생성
    def mark_bad(self, vp: Pt, now: float) -> None:
        self.tabu.append((vp[0], vp[1], now + self.ec.tabu_s))
        key = self._scout_vps.get((int(round(vp[0] / 0.05)), int(round(vp[1] / 0.05))))
        if key is not None:                                          # 정찰 지점에 못 닿거나 관측을 못 끝낸 것도 한 번의 시도다 — 끝없이 되풀이하지 않게
            self._scout_tries[key] = self._scout_tries.get(key, 0) + 1
        fkey = self._identify_vps.pop((int(round(vp[0] / 0.05)), int(round(vp[1] / 0.05))), None)
        if fkey is not None:                                         # 식별 지점도 마찬가지
            self.cov.mark_identify([fkey], outcome="unreached", now=now)

    def oak_fit_dist(self) -> float:
        """OAK RGB 로 면 그림(0.36 m 정사각)이 시야에 온전히 들어오는 '카메라 ↔ 면' 최소 거리[m]. 화각은 CameraInfo 의 K 에서(manager), 없으면 oak_det.fit_*_deg."""
        oc = self.cfg.oak_det
        hfov, vfov = self.oak_fov if self.oak_fov is not None else (oc.fit_hfov_deg, oc.fit_vfov_deg)
        return oak_fit_min_dist(float(self.cfg.rig.oak_rgb_xyz[2]), hfov, vfov, oc.fit_usable)

    def identify_stand_off(self) -> float:
        """식별 시점에서 '몸 기준점 ↔ 면' 거리[m]. 기본은 identify_dist_m. identify_primary 이면 면 그림이 온전히 보이는 최소 거리 + 카메라가 몸 앞에 있는 만큼 + 여유."""
        oc = self.cfg.oak_det
        if not oc.identify_primary:
            return float(oc.identify_dist_m)
        return max(float(oc.identify_dist_m), self.oak_fit_dist() + max(float(self.cfg.rig.oak_rgb_xyz[0]), 0.0) + 0.05)

    def _identify_candidates(self, pose: Pose2D, grid: OccupancyGrid, blocked: np.ndarray, occ: np.ndarray, now: float,
                             allowed: Optional[Set[str]]) -> List[Cand]:
        """미식별 면 앞의 시점. 시선 방위는 검사하지 않는다(도착해서 몸을 면 쪽으로 돌려 OAK 를 맞춘다).
        기본(identify_fallback): 팬틸트로 못 푼 면 앞 identify_dist_m.
        identify_primary: LiDAR 로 찾은 미식별 면 앞 identify_stand_off() — 면 그림이 OAK 시야에 온전히 들어오는 거리. 그 거리에서 닿을 수 있는 자리가
        identify_noview_s 동안 하나도 없으면 그 면은 팬틸트로 넘긴다(no_oak_view)."""
        oc, cov = self.cfg.oak_det, self.cov
        primary = bool(oc.identify_primary)
        if not (oc.enable and (oc.identify_fallback or primary) and self.identify_ready):
            return []
        out: List[Cand] = []
        if len(self._identify_vps) > 400:
            self._identify_vps.clear()
        d_stand = self.identify_stand_off()
        rmax = max(float(oc.identify_max_range_m), d_stand + 0.15) if primary else float(oc.identify_max_range_m)
        for f in cov.identify_faces(allowed):
            tx, ty = -f.ny, f.nx
            vps: List[Pt] = []
            lats: Dict[Pt, float] = {}
            # primary: 옆 ±0.75 m 까지. 면 맞은편에 벽·다른 구조물이 있는 좁은 통로에서는 정면 자리가 막혀, 통로를 따라 비켜 선 비스듬한 자리만 남는다
            # (지도 검토: 옆 3곳 18/44면 → 옆 7곳 34/44면). 입사각은 아래 face_visible 이 검사한다
            for lat in ((0.0, -0.25, 0.25, -0.5, 0.5, -0.75, 0.75) if primary else (0.0, -0.25, 0.25)):
                vp = (f.cx + f.nx * d_stand + tx * lat, f.cy + f.ny * d_stand + ty * lat)       # 옆으로 비켜도 법선 방향 거리는 줄이지 않는다(카메라 ↔ 면 거리가 최소 거리 밑으로 내려가지 않게)
                if (not self._free(grid, blocked, vp) or occ[grid.to_cell(*vp)] or self._is_tabu(vp, now)
                        or not line_free(grid, blocked, vp, (f.cx + f.nx * 0.28, f.cy + f.ny * 0.28))):
                    continue
                if not cov.face_visible(vp[0], vp[1], None, f, occ, grid, rmax=rmax)[0]:
                    continue
                vps.append(vp)
                lats[vp] = lat
            if primary:
                if not vps:
                    t0 = self._noview_since.setdefault(f.key, now)
                    if now - t0 >= float(oc.identify_noview_s):
                        cov.pt_fallback_face(f, "no_oak_view", now)
                        self._noview_since.pop(f.key, None)
                    continue
                self._noview_since.pop(f.key, None)
            vps.sort(key=lambda v: math.hypot(v[0] - pose.x, v[1] - pose.y) + (0.6 * abs(lats.get(v, 0.0)) if primary else 0.0))   # primary: 정면에 가까운 자리를 선호(분류가 더 안정)
            gain = float(oc.identify_gain) * (cov.identify_gain_weight(f) if primary else 1.0)
            for vp in vps[:2]:
                self._identify_vps[(int(round(vp[0] / 0.05)), int(round(vp[1] / 0.05)))] = f.key
                out.append(Cand("identify", vp, math.atan2(f.cy - vp[1], f.cx - vp[0]), "look", [f.key], gain_f=gain,
                                t_obs=oc.identify_dwell_s, primary_face=f.key))
        return out

    @staticmethod
    def _side(dx: float, dy: float) -> int:
        return (0 if dx > 0 else 2) if abs(dx) >= abs(dy) else (1 if dy > 0 else 3)

    def _scout_candidates(self, pose: Pose2D, grid: OccupancyGrid, blocked: np.ndarray, occ: np.ndarray, now: float,
                          pend: List[FaceRec], allowed: Optional[Set[str]], wgt, sweep: float) -> List[Cand]:
        """닫히지 않은 LiDAR 그룹의 가려진 쪽을 보러 가는 후보. 뒷면 정찰은 '그 칸 근처 + 가려진 방향'에서 LiDAR 가 광선을 통과시키게 하는 것이 목적이다."""
        ec, cov = self.ec, self.cov
        self._open_scouts = {}
        res = getattr(cov, "lidar_result", None)
        if not ec.lidar_faces or res is None:
            return []
        out: List[Cand] = []
        for g in res.groups:
            if g.closed is not False or g.ugv_near or not g.scout_points:
                continue
            if allowed is not None and server_sector(self.geo.sector_of(g.cx, g.cy)) not in allowed:
                continue
            for px, py, dx, dy in g.scout_points:
                key = (g.gid, self._side(dx, dy))
                if self._scout_tries.get(key, 0) >= ec.lidar_scout_max_attempts:
                    continue
                self._open_scouts[key] = (px, py)
                tx, ty = -dy, dx
                vps: List[Pt] = []
                for d in (ec.lidar_scout_standoff_m, ec.lidar_scout_standoff_m + 0.3):
                    for lat in (0.0, -0.4, 0.4):
                        vp = (px + dx * d + tx * lat, py + dy * d + ty * lat)
                        if (not self._free(grid, blocked, vp) or occ[grid.to_cell(*vp)] or self._is_tabu(vp, now)
                                or not line_free(grid, grid.occupied(), vp, (px, py))):
                            continue
                        vps.append(vp)
                vps.sort(key=lambda v: math.hypot(v[0] - pose.x, v[1] - pose.y))
                for vp in vps[:2]:
                    heading = math.atan2(py - vp[1], px - vp[0])
                    got, gf = self._visible_set(vp, heading, pend, occ, grid, sweep, wgt)
                    self._scout_vps[(int(round(vp[0] / 0.05)), int(round(vp[1] / 0.05)))] = key
                    out.append(Cand("scout", vp, heading, "look", [f.key for f in got], gain_f=gf, gain_u=ec.lidar_open_bonus,
                                    t_obs=ec.observe_look_s, scout_key=key))
        return out

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

    def _m1_tier_weights(self, m1: Set[str], team: Optional[str], now: float) -> Dict[str, float]:
        """M1 지정 구역의 섹터별 면 가치 배수. explore.m1_priority_enemy_first 가 꺼져 있으면 {} (모두 m1_sector_weight 하나 — 기존 동작).
        켜져 있으면 1순위 = 적 섹터(m1_enemy_weight), 2순위 = CM: 적 섹터에 아직 할 면이 남아 있고 M1 수신 뒤 m1_enemy_budget_s 안이면
        m1_cm_weight × m1_cm_defer 로 깎아 적 섹터 면이 먼저 선택되게 하고, 적 섹터가 끝났거나 시간 상한이 지나면 m1_cm_weight 로 풀어 준다.
        M1 이 섹터를 새로 지정하면(집합이 바뀌면) 시간 기준을 다시 잡는다. 적 섹터 면이 아직 하나도 안 보여도(영상 없음) 상한 시간이 지나면 풀린다."""
        ec = self.ec
        if not ec.m1_priority_enemy_first:
            self.m1_tier = {}
            return {}
        key = frozenset(m1)
        if key != self._m1_key:
            self._m1_key, self._m1_t0, self._m1_cm_released = key, now, False
        enemy = {n for n in m1 if n != "CM" and n in set(scoring_sectors(team or ""))} if team else set()
        out: Dict[str, float] = {n: ec.m1_enemy_weight for n in enemy}
        enemy_done, deferred = True, False
        if enemy:
            prog = self.cov.sector_progress(sorted(enemy))
            enemy_done = bool(prog["complete"])
            elapsed = now - (self._m1_t0 if self._m1_t0 is not None else now)
            over = ec.m1_enemy_budget_s > 0 and elapsed >= ec.m1_enemy_budget_s
            deferred = (not enemy_done) and (not over) and not self._m1_cm_released
            if not deferred:
                self._m1_cm_released = True
        if "CM" in m1:
            out["CM"] = ec.m1_cm_weight * (ec.m1_cm_defer if deferred else 1.0)
        for n in m1:
            out.setdefault(n, ec.m1_sector_weight)                      # 적 섹터도 CM 도 아닌 지정 구역은 예전 가중
        self.m1_tier = {"enemy": sorted(enemy), "cm": "CM" in m1, "enemy_done": enemy_done, "cm_deferred": deferred,
                        "elapsed_s": None if self._m1_t0 is None else round(now - self._m1_t0, 1),
                        "weights": {k: round(v, 2) for k, v in sorted(out.items())}}
        return out

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
        # explore.m1_sector_sweep: M1 이 지정한 섹터의 면도 모두 확인·분류한다(채점 구역 밖이어도 장부·면 기록부에 식별로 남는다).
        # 채점 구역(M3 제출)은 그대로이고, 이 섹터들은 탐색 범위와 면 가치만 넓힌다.
        m1 = {n for n in self.m1_sectors if server_sector(n) is not None} if (allowed is not None and ec.m1_sector_sweep) else set()
        if m1:
            allowed = allowed | m1
        pend = cov.pending_faces(allowed)
        tier = self._m1_tier_weights(m1, team, now) if m1 else {}
        occ = cov.visibility_occupancy(grid)
        sweep = math.radians(25.0)                                  # 이동·관측 중 팬 스윕 진폭(_sweep_pan 와 같다)

        def wgt(f: FaceRec, vp: Pt) -> float:
            w = cov.face_weight_at(f, vp[0], vp[1])
            if prefer and self.geo.sector_of(f.cx, f.cy) in prefer:
                w *= 1.0 + ec.sector_bonus
            sector = cov.box_sector(cov.boxes[f.bid])
            if m1 and server_sector(sector or "") in m1:
                w *= tier.get(server_sector(sector or ""), ec.m1_sector_weight)
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
        # ---- (c) 닫히지 않은 장애물 그룹의 가려진 쪽 정찰(explore.lidar_faces)
        out.extend(self._scout_candidates(pose, grid, blocked, occ, now, pend, allowed, wgt, sweep))
        # ---- (d) 팬틸트로 못 푼 면의 OAK 식별(oak_det.identify_fallback)
        out.extend(self._identify_candidates(pose, grid, blocked, occ, now, allowed))
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
                                          "m1_sweep_sectors": sorted(m1),
                                          "incumbent_requested": current is not None,
                                          "incumbent_rebuilt": incumbent is not None,
                                          "face": sum(c.kind == "face" for c in out),
                                          "frontier": sum(c.kind == "frontier" for c in out),
                                          "survey": sum(c.kind == "survey" for c in out),
                                          "scout": sum(c.kind == "scout" for c in out),
                                          "identify": sum(c.kind == "identify" for c in out)}
        return out

    def _refresh_incumbent(self, current, pending, occ, grid, blocked, solid, unseen, now, weights, allowed=None):
        if (current is None or not self._free(grid, blocked, current.vp)
                or occ[grid.to_cell(*current.vp)] or self._is_tabu(current.vp, now)):
            return None
        if (allowed is not None and current.kind in ("frontier", "survey")
                and server_sector(self.geo.sector_of(*current.vp)) not in allowed):
            return None
        if current.kind == "identify":                              # 팬틸트 대기 면이 아니라 '식별 대상' 면이므로 pending 으로 가치를 다시 재면 0 이 된다
            keep = {f.key for f in self.cov.identify_faces(allowed)}
            if not current.faces or current.faces[0] not in keep:
                return None
            gain = float(self.cfg.oak_det.identify_gain)
            if self.cfg.oak_det.identify_primary:
                fobj = self.cov.get_face(current.faces[0])
                gain *= self.cov.identify_gain_weight(fobj) if fobj is not None else 1.0
            return Cand("identify", current.vp, current.heading, "look", list(current.faces), gain_f=gain,
                        t_obs=self.cfg.oak_det.identify_dwell_s, incumbent=True, primary_face=current.primary_face)
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
        elif current.kind == "scout" and current.scout_key in self._open_scouts:
            gain_u = self.ec.lidar_open_bonus                          # 그룹이 아직 닫히지 않았다 — 닫히면 0 이 되어 후보에서 빠진다
        return Cand(current.kind, vp, heading, mode, [f.key for f in got], gain_f=gain, gain_u=gain_u,
                    t_obs=(self.ec.m3_observe_s if photo else self.ec.observe_look_s) if mode == "look" else self._spin_s(),
                    incumbent=True, primary_face=primary_face if any(f.key == primary_face for f in got) else None,
                    scout_key=current.scout_key)

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
