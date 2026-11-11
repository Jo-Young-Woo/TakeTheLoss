"""M3 searches the scored territories while retaining the full navigation map."""
from types import SimpleNamespace

import numpy as np
import pytest

from maicon26.config import Config
from maicon26.link.schema import scoring_sectors, server_sector
from maicon26.mission.explore_online import Cand, OnlineExplorer
from maicon26.mission.manager import Agent
from maicon26.nav.grid import OccupancyGrid
from maicon26.perception.coverage import BoxModel, CoverageMap
from maicon26.types import Pose2D


def layout():
    cfg = Config()
    cov, grid = CoverageMap(cfg), OccupancyGrid(cfg.arena)
    names = {}
    for bid, sector in enumerate(cfg.arena.sector_names(), 1):
        cx, cy = cfg.arena.sector_center(sector)
        box = BoxModel(bid, cx, cy, "full", [], 1., 1., full_seen=3, last_full_t=1.)
        cov._make_faces(box)
        cov.boxes[bid] = box
        names[bid] = sector
        for iy in range(grid.ny):
            for ix in range(grid.nx):
                x, y = grid.to_world(iy, ix)
                if abs(x-cx) < .2 and abs(y-cy) < .2:
                    grid.L[iy, ix] = grid.L_MAX
    return cfg, cov, grid, names


@pytest.mark.parametrize("team", ["BLUE", "RED"])
def test_all_geometry_only_cubes_in_scored_territory_are_eligible(team):
    cfg, cov, grid, names = layout()
    allowed = set(scoring_sectors(team))
    pending = cov.pending_faces(allowed)
    expected = {bid for bid, name in names.items() if server_sector(name) in allowed}
    assert {f.bid for f in pending} == expected
    assert len(pending) == 16, "All four sides of every unknown 0.4m cube remain candidates"
    assert all(f.det_n == 0 for f in pending), "No photo class or referee label is needed"
    assert len(cov.boxes) == 7
    for box in cov.boxes.values():
        assert grid.occupied()[grid.to_cell(box.cx, box.cy)], "Unscored structures remain physical obstacles"
    # The internal and server names for the common area are equivalent.
    assert {f.bid for f in cov.pending_faces({"CM"})} == {f.bid for f in cov.pending_faces({"COMMON"})}


@pytest.mark.parametrize("team", ["BLUE", "RED"])
def test_frontiers_surveys_and_photo_quests_respect_score_not_m1_subset(team):
    cfg, cov, grid, names = layout()
    explorer = OnlineExplorer(cfg, cov)
    blocked, _ = cov.planning_cost_maps(grid)
    before = grid.L.copy()
    allowed = set(scoring_sectors(team))
    # Deliberately pass a narrow M1 preference. Actual team scope takes priority.
    candidates = explorer.build(Pose2D(.4 if team == "BLUE" else 5.7, 2.5, 0.), grid, blocked, 10.,
                                prefer={scoring_sectors(team)[1]}, team=team)
    photo_boxes = {bid for c in candidates for bid, _ in c.faces}
    expected = {bid for bid, name in names.items() if server_sector(name) in allowed}
    assert photo_boxes == expected
    surveys = [c for c in candidates if c.kind == "survey"]
    assert {server_sector(cfg.arena.sector_of(*c.vp)) for c in surveys} == allowed
    assert np.array_equal(grid.L, before)
    assert explorer.diagnostics["last_build"]["m3_scoring_sectors"] == sorted(allowed)


def test_home_territory_is_usable_for_transit_to_scored_cube():
    cfg, cov, grid, _ = layout()
    explorer = OnlineExplorer(cfg, cov)
    blocked, cost = cov.planning_cost_maps(grid)
    before = blocked.copy()
    current = Cand("face", (3.4, 1.6), 0., "look", gain_f=3., t_obs=1.2)
    ranked = explorer.evaluate([current], Pose2D(1., .7, 0.), grid, blocked, cost, 10., (3.6, 3.0))
    assert ranked and current.path
    assert any(cfg.arena.sector_of(*p) == "B-S3" and p[0] > 1.15 for p in current.path)
    assert np.array_equal(blocked, before), "Score eligibility must never become a no-travel mask"


@pytest.mark.parametrize("team", ["BLUE", "RED"])
def test_frontier_observation_stops_are_in_scoring_territory(team):
    cfg = Config()
    cov, grid = CoverageMap(cfg), OccupancyGrid(cfg.arena)
    cov.seen.seen[:] = cov.seen.Y < 2.5
    explorer = OnlineExplorer(cfg, cov)
    blocked, _ = cov.planning_cost_maps(grid)
    candidates = explorer.build(Pose2D(.4 if team == "BLUE" else 5.7, 2., 0.), grid, blocked, 10., team=team)
    frontiers = [c for c in candidates if c.kind == "frontier"]
    assert frontiers
    assert all(server_sector(cfg.arena.sector_of(*c.vp)) in scoring_sectors(team) for c in frontiers)


def test_common_cube_can_be_observed_from_home_and_uses_box_centre_territory():
    cfg = Config()
    cov, grid = CoverageMap(cfg), OccupancyGrid(cfg.arena)
    box = BoxModel(1, cfg.arena.W/2, 2.5, "full", [], 1., 1.)
    cov._make_faces(box)
    cov.boxes[1] = box
    explorer = OnlineExplorer(cfg, cov)
    current = Cand("face", (2.1, 2.5), 0., "look", [(1, 2)], gain_f=1.)
    assert cfg.arena.sector_of(*current.vp) == "B-S2"
    blocked, _ = cov.planning_cost_maps(grid)
    incumbent, = [c for c in explorer.build(Pose2D(1., 2., 0.), grid, blocked, 10., current=current, team="BLUE") if c.incumbent]
    assert incumbent.vp == current.vp and (1, 2) in incumbent.faces and incumbent.gain > 0
    # A translated navigation estimate cannot change the measured arena sector.
    box.cx, box.arena_cx, box.arena_cy = 2.3, cfg.arena.W/2, 2.5
    assert cov.pending_faces(set(scoring_sectors("BLUE")))


def test_manager_does_not_use_m1_search_sectors_to_narrow_m3():
    cfg = Config()
    cfg.explore.sector_priority = True  # Even the legacy preference switch cannot reintroduce M1 filtering.
    grid = OccupancyGrid(cfg.arena)
    captured = {}

    class Captured(Exception):
        pass

    def build(*args, **kwargs):
        captured.update(kwargs)
        assert len(args) == 4, "M1 preferences must not be passed positionally either"
        raise Captured()

    agent = Agent.__new__(Agent)
    agent.cfg, agent.geo, agent.team = cfg, cfg.arena, "BLUE"
    agent.cmd = SimpleNamespace(sectors=["R-S2"])
    agent.stack = SimpleNamespace(pose=Pose2D(.4, 2.5, 0.), grid=grid)
    agent.coverage = CoverageMap(cfg)
    agent.explorer = SimpleNamespace(build=build)
    agent._ex_phase, agent._ex_cand, agent._ex_sel_t, agent._ex_goal_t0 = "travel", None, -100., 0.
    with pytest.raises(Captured):
        agent._mode_explore_online(10., .1, 10.)
    assert captured["team"] == "BLUE" and "prefer" not in captured


def test_old_unscored_survey_is_not_kept_as_an_incumbent():
    cfg = Config()
    cov, grid = CoverageMap(cfg), OccupancyGrid(cfg.arena)
    explorer = OnlineExplorer(cfg, cov)
    blocked, _ = cov.planning_cost_maps(grid)
    current = Cand("survey", (1.5, 2.5), 0., "spin", gain_u=99.)
    candidates = explorer.build(Pose2D(1., 2.5, 0.), grid, blocked, 10., current=current, team="BLUE")
    assert not any(c.incumbent for c in candidates)
