"""An active observation goal survives candidate sampling, not new obstacles."""
import math
from types import SimpleNamespace

import numpy as np

from maicon26.config import Config
from maicon26.mission.explore_online import Cand, OnlineExplorer
from maicon26.nav.grid import OccupancyGrid
from maicon26.perception.coverage import BoxModel, CoverageMap
from maicon26.types import Pose2D


def observed_box_scene():
    cfg = Config()
    cov, grid = CoverageMap(cfg), OccupancyGrid(cfg.arena)
    box = BoxModel(1, 2., 2., "full", [], 1., 1., full_seen=3, last_full_t=1.)
    cov._make_faces(box)
    cov.boxes[1] = box
    for iy in range(grid.ny):
        for ix in range(grid.nx):
            x, y = grid.to_world(iy, ix)
            if 1.8 < x < 2.2 and 1.8 < y < 2.2:
                grid.L[iy, ix] = grid.L_MAX
    return cfg, cov, grid, box, OnlineExplorer(cfg, cov)


def test_exact_active_viewpoint_is_refreshed_even_when_not_in_sampled_grid():
    _, cov, grid, _, explorer = observed_box_scene()
    current = Cand("face", (3.23, 2.04), math.pi, "look", [(1, 0)], gain_f=99., path=[(1., 1.), (3.23, 2.04)],
                   primary_face=(1, 0))
    blocked, _ = cov.planning_cost_maps(grid)
    pose = Pose2D(1., 1., 0.)
    fresh = explorer.build(pose, grid, blocked, 10.)
    assert all(c.primary_face in c.faces for c in fresh if c.kind == "face")
    assert not any(c.vp == current.vp for c in fresh)
    candidates = explorer.build(pose, grid, blocked, 10., current=current)
    incumbent, = [c for c in candidates if c.incumbent]
    assert incumbent.vp == current.vp
    assert (1, 0) in incumbent.faces
    assert incumbent.primary_face == current.primary_face
    assert 0 < incumbent.gain_f < 99., "Do not retain obsolete information gain"
    assert incumbent.path is None, "Replanning must validate the path again"
    assert current.gain_f == 99., "Do not mutate the running goal before validation"


def test_resolved_faces_and_new_occluders_remove_obsolete_active_gain():
    _, cov, grid, box, explorer = observed_box_scene()
    current = Cand("face", (3.23, 2.), math.pi, "look", [(1, 0)], gain_f=99.)
    # Hide the target face behind a newly observed obstacle, while leaving the
    # exact observation destination itself free and geometrically reachable.
    for iy in range(grid.ny):
        for ix in range(grid.nx):
            x, y = grid.to_world(iy, ix)
            if 2.55 < x < 2.65 and 1.4 < y < 2.6:
                grid.L[iy, ix] = grid.L_MAX
    blocked, cost = cov.planning_cost_maps(grid)
    incumbent, = [c for c in explorer.build(Pose2D(4., 2., 0), grid, blocked, 10., current=current) if c.incumbent]
    assert incumbent.gain == 0.
    assert explorer.evaluate([incumbent], Pose2D(4., 2., 0), grid, blocked, cost, 10., (3.6, 3.0)) == []
    for f in box.faces:
        f.det_n, f.det_votes = 4, {"AMMO_BOX": 4.}
    rebuilt, = [c for c in explorer.build(Pose2D(4., 2., 0), grid, blocked, 11., current=current) if c.incumbent]
    assert rebuilt.faces == [] and rebuilt.gain == 0.


def test_active_goal_has_an_extra_path_check_outside_new_candidate_quota():
    cfg = Config()
    cfg.explore.max_eval = 1
    cov, grid = CoverageMap(cfg), OccupancyGrid(cfg.arena)
    explorer = OnlineExplorer(cfg, cov)
    blocked, cost = cov.planning_cost_maps(grid)
    incumbent = Cand("face", (4., 2.), 0., "look", gain_f=1., t_obs=1.2, incumbent=True)
    new = [Cand("face", (1.4 + .1*i, 1.), 0., "look", gain_f=20.-i, t_obs=1.2) for i in range(5)]
    ranked = explorer.evaluate(new + [incumbent], Pose2D(1., 1., 0.), grid, blocked, cost, 10., (3.6, 3.0))
    assert any(c is incumbent for c in ranked)
    assert sum(not c.incumbent for c in ranked) == 1
    diagnostic = explorer.diagnostics["last_evaluate"]
    assert diagnostic["incumbent_path_checks"] == diagnostic["new_candidate_path_checks"] == 1
    assert incumbent.t_move > 0 and incumbent.path


def test_incumbent_still_fails_new_dynamic_barrier_and_mission_deadline():
    cfg = Config()
    cov, grid = CoverageMap(cfg), OccupancyGrid(cfg.arena)
    explorer = OnlineExplorer(cfg, cov)
    blocked, cost = cov.planning_cost_maps(grid)
    incumbent = Cand("face", (4., 2.), 0., "look", gain_f=10., t_obs=1.2, incumbent=True)
    other = Cand("face", (1.5, 2.), 0., "look", gain_f=1., t_obs=1.2)
    body = np.zeros((1, grid.ny, grid.nx), dtype=bool)
    body[:, :, grid.to_cell(3., 2.)[1]] = True
    zeros = np.zeros_like(body, dtype=np.float32)
    field = SimpleNamespace(K=1, dtau=1., body=body, risk=zeros, close=zeros)
    ranked = explorer.evaluate([incumbent, other], Pose2D(1., 2., 0.), grid, blocked, cost, 10., (3.6, 3.0), field)
    assert all(c is not incumbent for c in ranked) and any(c is other for c in ranked)
    assert explorer.diagnostics["last_evaluate"]["rejected"]["no_path"] == 1
    late = cfg.mission.prestage_at - cfg.explore.deadline_margin_s - .1
    assert explorer.evaluate([incumbent], Pose2D(1., 2., 0.), grid, blocked, cost, late, (3.6, 3.0)) == []


def test_hysteresis_keeps_validated_goal_identity_and_new_path_without_new_selection():
    explorer = OnlineExplorer(Config(), CoverageMap(Config()))
    current = Cand("face", (3.23, 2.04), math.pi, "look", gain_f=99., utility=99.)
    refreshed = Cand("face", current.vp, 2.9, "look", [(2, 1)], gain_f=4., t_obs=2.,
                     utility=1., t_move=7., path=[(2., 2.), current.vp], incumbent=True, primary_face=(2, 1))
    alternative = Cand("frontier", (4., 4.), 0., "spin", gain_u=5., utility=1.2)
    assert explorer.choose([alternative, refreshed], current) is current
    assert current.path == refreshed.path and current.faces == [(2, 1)]
    assert current.heading == 2.9 and current.t_obs == 2.
    assert current.primary_face == (2, 1)
    assert explorer.stats["selected"] == 0
    alternative.utility = 2.
    assert explorer.choose([alternative, refreshed], current) is alternative, "A substantially better valid option may still preempt"


def test_nearby_legacy_substitute_cannot_keep_an_unchecked_old_endpoint():
    explorer = OnlineExplorer(Config(), CoverageMap(Config()))
    current = Cand("face", (2., 2.), 0., "look", route_vp=(1.8, 2.))
    checked = Cand("face", (2.2, 2.), 1., "spin", gain_f=1., utility=1., path=[(1., 2.), (2.2, 2.)])
    assert explorer.choose([checked], current) is current
    assert current.vp == checked.vp == current.path[-1]
    assert current.route_vp is None and current.heading == 1. and current.mode == "spin"
