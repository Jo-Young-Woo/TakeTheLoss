"""시뮬레이션 실행기: N대의 Agent를 같은 World에서 돌리고 지표를 낸다."""
from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Optional, Tuple

from ..config import Config
from ..game_rules import M3_COMMON_POINTS, M3_SECTOR_POINTS, alliance_bonus, danger_score, m4_points, m6_points
from ..link.schema import OBJECT_CODES, scoring_sectors, server_sector
from ..mission.manager import Agent
from .robot import SimRobot, SimServer
from .world import World


@dataclass
class Match:
    world: World
    agents: List[Agent]
    servers: List[SimServer]
    robots: List[SimRobot]
    wall_s: float = 0.0


def build_match(seed: int = 0, cfg: Optional[Config] = None, teams: Optional[List[str]] = None,
                m5_ends_match: bool = True, announce_identity: bool = True, style: str = "nato",
                idle_ids: Optional[List[str]] = None, layout_seed: Optional[int] = None,
                llm=None) -> Match:
    cfg = cfg or Config()
    w = World(cfg, seed=seed, m5_ends_match=m5_ends_match, layout_seed=layout_seed)
    spec = teams or ["B1", "B2", "B3", "R1", "R2", "R3"]
    agents, servers, robots = [], [], []
    for s in spec:
        team = "BLUE" if s[0] == "B" else "RED"
        slot = int(s[1])
        u = w.add_ugv(team, slot)
        r = SimRobot(w, u)
        sv = SimServer(w, u, announce_identity=announce_identity, sector_style=style)
        a = Agent(r, sv, cfg=cfg, seed=seed * 10 + len(agents), name=u.id, llm=llm)
        agents.append(a)
        servers.append(sv)
        robots.append(r)
    idle = set(idle_ids or [])
    m = Match(w, [a for a in agents if a.name not in idle], servers, robots)
    m.idle = idle  # type: ignore[attr-defined]
    return m


def run(m: Match, dt: float = 0.05, max_t: float = 400.0, verbose: bool = False,
        events: Optional[List[Tuple[float, Callable[[World], None]]]] = None,
        probe: Optional[Callable[["Match"], None]] = None) -> Match:
    """events: [(시각 s, fn(world))] — 경기 중 구조물 변경 등. probe: 매 스텝 호출(지표 수집용)."""
    t0 = time.time()
    w = m.world
    pending = sorted(events or [], key=lambda e: e[0])
    while not w.finished and w.t < max_t:
        while pending and w.t >= pending[0][0]:
            pending.pop(0)[1](w)
        for a in m.agents:
            a.tick()
        if probe is not None:
            probe(m)
        w.step(dt)
        if verbose and int(w.t / dt) % 200 == 0:
            print(f"t={w.t:6.1f}", " ".join(f"{a.name}:{a.mode.value}" for a in m.agents))
        if all(a.mode.value == "DONE" for a in m.agents):
            break
    m.wall_s = time.time() - t0
    return m


# ====================================================================== 채점
def score_m3(truth, payload, team):
    """Official points plus legacy count recall/precision diagnostics.

    A sector needs an explicit report, including an explicitly empty object
    list, to earn its points. ``exact`` retains the existing count metric;
    ``official_exact`` means all four scored sectors were reported correctly.
    """
    allowed = set(scoring_sectors(team))
    expected, submitted = {}, {}
    for source, target in ((truth, expected), (payload.get("sectors", {}), submitted)):
        for sector, counts in source.items():
            canonical = server_sector(sector)
            if canonical not in allowed:
                continue
            bucket = target.setdefault(canonical, {})
            for kind, count in counts.items():
                # Wrong reported kinds never match a legitimate picture.
                if isinstance(count, int) and not isinstance(count, bool) and count >= 0:
                    bucket[kind] = bucket.get(kind, 0) + count
    total_truth = sum(n for counts in expected.values() for kind, n in counts.items() if kind in OBJECT_CODES)
    total_sub = sum(sum(counts.values()) for counts in submitted.values())
    correct = sum(min(n, expected.get(sector, {}).get(kind, 0))
                  for sector, counts in submitted.items() for kind, n in counts.items()
                  if kind in OBJECT_CODES)
    exact = correct == total_truth == total_sub
    sector_results = {}
    for sector in scoring_sectors(team):
        wanted = {kind: n for kind, n in expected.get(sector, {}).items() if n > 0}
        actual = {kind: n for kind, n in submitted.get(sector, {}).items() if n > 0}
        reported = sector in submitted
        matched = reported and actual == wanted
        maximum = M3_COMMON_POINTS if sector == "CM" else M3_SECTOR_POINTS
        sector_results[sector] = {"reported": reported, "exact": matched, "points": maximum if matched else 0,
                                  "max_points": maximum}
    return {"recall": round(correct/max(total_truth, 1), 2),
            "precision": round(correct/max(total_sub, 1), 2), "n_correct": correct,
            "n_sub": total_sub, "n_truth": total_truth, "exact": exact,
            "scored_sectors": scoring_sectors(team), "sector_results": sector_results,
            "points": sum(result["points"] for result in sector_results.values()),
            "official_exact": all(result["exact"] for result in sector_results.values())}


def score(m: Match) -> Dict[str, Any]:
    w = m.world
    truth_obj = w.objects_truth()
    res: Dict[str, Any] = {"t_end": w.end_time, "wall_s": round(m.wall_s, 1), "agents": {}}

    def destroyed_target(team, target):
        # Referee-only evaluation. Attribution is the target's assigned team,
        # regardless of the shooter, including friendly fire (S6.3).
        return any((cube.cx > w.geo.W / 2) == (team == "BLUE") and
                   any(item.kind == "target" and item.cls == target and item.led_on for item in cube.items.values())
                   for cube in w.cubes)

    m3_points_by_id = {u.id: score_m3(truth_obj, w.submissions.get(u.id, {}).get("m3", {}).get("payload", {"sectors": {}}), u.team)["points"]
                      for u in w.ugvs}
    for a, sv in zip(m.agents, m.servers):
        u = sv.u
        sub = w.submissions.get(u.id, {})
        tr = sv.truth
        d: Dict[str, Any] = {}
        # M1
        p1 = sub.get("m1", {}).get("payload")
        if p1:
            d["m1"] = {"target": p1.get("target") == tr["target"],
                       "sectors": p1.get("sectors") is not None and sorted(p1["sectors"]) == sorted(tr["sectors"]),
                       "goal_base": p1.get("goal_base") == tr["goal_base"]}
        else:
            d["m1"] = None
        d["m2_targets_hit"] = list(u.targets_hit)
        d["m2_designated"] = tr["target"]
        d["m2_ok"] = destroyed_target(u.team, tr["target"])
        alliance = [x for x in w.ugvs if x.team == u.team and x.id in w.assign]
        all_designated = len(alliance) == 3 and all(destroyed_target(x.team, w.assign[x.id]["target"]) for x in alliance)
        d["m2_points"] = (10 if d["m2_ok"] else 0) + (5 if all_designated else 0)
        # M3
        p3 = sub.get("m3", {}).get("payload")
        if p3:
            d["m3"] = score_m3(truth_obj, p3, u.team)
        else:
            d["m3"] = None
        # M4/M5
        d["m4_hits_dealt"] = len(u.hits_dealt)
        d["friendly_hits"] = u.friendly_hits
        d["hits_taken"] = u.hits_taken
        d["m4_points"] = m4_points(u.hits_taken)
        d["shots"] = u.shots
        d["wasted"] = a.laser.wasted
        d["m5_reached"] = None if u.reached_base is None else round(u.reached_base, 1)
        d["m5_points"] = 10 if getattr(w, "m5_winner", None) == u.team else 0
        d["collisions"] = u.collisions
        d["distance"] = round(u.distance, 1)
        d["modes"] = {k: round(v) for k, v in a.stats["mode_time"].items()}
        # Referee-only M6 truth: actual accepted M3 points, never travel/coverage.
        p6 = sub.get("m6", {}).get("payload")
        opp = [x for x in w.ugvs if x.team != u.team]
        if p6 and opp:
            true_scores = {x.id: danger_score(4 - x.hits_taken, x.ammo, m3_points_by_id[x.id]) for x in opp}
            true_rank = sorted(true_scores, key=lambda k: (-true_scores[k], k))
            d["m6"] = {"submitted": p6["ranking"], "true": true_rank,
                       "top1": bool(p6["ranking"]) and p6["ranking"][0] == true_rank[0],
                       "points": m6_points(p6["ranking"], true_rank), "true_scores": true_scores,
                       "tie_policy": "stable_id_order; official tie equivalence is provisional"}
        else:
            d["m6"] = None
        res["agents"][a.name] = d
        p1_score = sum(points for key, points in (("target", 3), ("sectors", 3), ("goal_base", 4)) if d["m1"] and d["m1"][key])
        d["points_before_alliance_bonus"] = p1_score + d["m2_points"] + m3_points_by_id[u.id] + d["m4_points"] + d["m5_points"] + (d["m6"]["points"] if d["m6"] else 0)
    # A solo/partial rehearsal cannot determine the two complete alliance totals.
    if all(sum(a.team == camp for a in m.agents) == 3 for camp in ("BLUE", "RED")):
        totals = {camp: sum(res["agents"][a.name]["points_before_alliance_bonus"] for a in m.agents if a.team == camp)
                  for camp in ("BLUE", "RED")}
        bonuses = alliance_bonus(totals["BLUE"], totals["RED"])
        res["alliance_totals_before_bonus"] = totals
        for a in m.agents:
            result = res["agents"][a.name]
            result["alliance_bonus"] = bonuses[a.team]
            result["points"] = result["points_before_alliance_bonus"] + bonuses[a.team]
    res["events"] = len(w.events)
    return res
