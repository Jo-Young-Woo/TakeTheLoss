"""Ground observation, rectangular geometry and UAV-assisted exploration contracts."""
from types import SimpleNamespace

import numpy as np

from maicon26.config import Config
from maicon26.mission.explore_online import Cand, OnlineExplorer
from maicon26.nav import risk_astar
from maicon26.nav.grid import OccupancyGrid
from maicon26.perception.coverage import CoverageMap
from maicon26.types import Pose2D


def _footprint(x=2.0, y=1.0, sx=0.4, sy=0.4, confidence=0.95):
    return SimpleNamespace(cx=x, cy=y, sx=sx, sy=sy, confidence=confidence)


def test_uav_requires_two_frames_and_never_claims_seen_faces_or_classes():
    cfg = Config()
    cov, grid = CoverageMap(cfg), OccupancyGrid(cfg.arena)
    original = grid.L.copy()
    hint = _footprint(sx=0.8)
    cov.ingest_uav_boxes([hint], 1.0, grid)
    assert not cov.boxes
    cov.ingest_uav_boxes([hint], 2.0, grid)
    box = next(iter(cov.boxes.values()))
    assert box.sx == 0.8 and box.sy == 0.4
    assert box.faces[0].cx == 2.4 and box.faces[2].cx == 1.6
    assert all(face.views == 0 and face.det_n == 0 and not face.det_votes for face in box.faces)
    assert cov.m3_counts(None) == {}
    assert np.array_equal(original, grid.L), "A footprint prior must not overwrite SLAM"
    cov.refresh(grid, None, 3.0, force=True)
    assert len(cov.boxes) == 1, "Unobserved grid cells are not evidence that a box vanished"


def test_invalid_or_single_noisy_uav_boxes_are_not_promoted():
    cfg = Config()
    cov = CoverageMap(cfg)
    hints = [_footprint(confidence=0.2), _footprint(x=float("nan")), _footprint(sx=3.0)]
    cov.ingest_uav_boxes(hints, 1.0)
    cov.ingest_uav_boxes(hints, 2.0)
    assert not cov.boxes
    cov.ingest_uav_boxes([_footprint()], 3.0)
    cov.ingest_uav_boxes([_footprint(x=3.0)], 4.0)
    assert not cov.boxes


def test_lidar_rectangular_cluster_keeps_its_long_face_position():
    cfg = Config()
    grid, cov = OccupancyGrid(cfg.arena), CoverageMap(cfg)
    for iy in range(grid.ny):
        for ix in range(grid.nx):
            x, y = grid.to_world(iy, ix)
            if 1.6 < x < 2.4 and 1.8 < y < 2.2:
                grid.L[iy, ix] = grid.L_MAX
    cov.refresh(grid, None, 1.0, force=True)
    box = next(iter(cov.boxes.values()))
    assert box.sx == 0.8 and box.sy == 0.4
    assert abs(box.faces[0].cx - 2.4) < 0.03
    assert abs(box.faces[1].cy - 2.2) < 0.03
    assert cov.pending_faces() == [], "Known rectangular arena obstacles are not M3 photo boxes"


def test_m3_skips_known_rectangles_but_keeps_uncertain_face_hypotheses():
    cfg = Config()
    cov, grid = CoverageMap(cfg), OccupancyGrid(cfg.arena)
    hints = [_footprint(1.8, 1.0), _footprint(4.3, 1.0, sx=0.8)]
    cov.ingest_uav_boxes(hints, 1.0, grid)
    cov.ingest_uav_boxes(hints, 2.0, grid)
    cube, rectangle = sorted(cov.boxes.values(), key=lambda b: b.cx)
    assert {f.bid for f in cov.pending_faces()} == {cube.bid}
    assert len(rectangle.faces) == 4, "Rectangle geometry is retained for occlusion/M2"
    assert cov.visibility_occupancy(grid)[grid.to_cell(rectangle.cx, rectangle.cy)]
    rectangle.uav_t, rectangle.kind = None, "face"
    assert {f.bid for f in cov.pending_faces()} == {cube.bid, rectangle.bid}
    rectangle.kind = "full"
    assert {f.bid for f in cov.pending_faces()} == {cube.bid}


def test_face_candidates_include_distant_lower_and_upper_boxes():
    cfg = Config()
    cov, grid = CoverageMap(cfg), OccupancyGrid(cfg.arena)
    hints = [_footprint(1.8, 1.0), _footprint(4.3, 1.0), _footprint(1.8, 4.0), _footprint(4.3, 4.0)]
    cov.ingest_uav_boxes(hints, 1.0, grid)
    cov.ingest_uav_boxes(hints, 2.0, grid)
    explorer = OnlineExplorer(cfg, cov)
    blocked, cost = grid.cost_maps(cfg.nav.inflate_hard, cfg.nav.inflate_soft, cfg.nav.soft_weight)
    cands = explorer.build(Pose2D(1.0, 4.0, 0.0), grid, blocked, 5.0)
    seen_boxes = {key[0] for cand in cands for key in cand.faces}
    assert seen_boxes == set(cov.boxes)
    assert any(c.vp[1] < 1.7 for c in cands if c.kind == "face")
    ranked = explorer.evaluate(cands, Pose2D(1.0, 4.0, 0.0), grid, blocked, cost, 5.0, (3.6, 3.0))
    assert ranked
    assert explorer.diagnostics["last_evaluate"]["accepted"] == len(ranked)


def test_no_frontier_still_surveys_unsurveyed_sectors_before_staging():
    cfg = Config()
    cov, grid = CoverageMap(cfg), OccupancyGrid(cfg.arena)
    explorer = OnlineExplorer(cfg, cov)
    blocked, _ = grid.cost_maps(cfg.nav.inflate_hard, cfg.nav.inflate_soft, cfg.nav.soft_weight)
    cands = explorer.build(Pose2D(1.0, 4.0, 0.0), grid, blocked, 5.0)
    assert {cfg.arena.sector_of(*c.vp) for c in cands} == set(cfg.arena.sector_names())
    assert all(c.kind == "survey" for c in cands)
    observed = cands[0]
    explorer.note_observation(observed, 10.0)
    next_cands = explorer.build(Pose2D(1.0, 4.0, 0.0), grid, blocked, 11.0)
    assert cfg.arena.sector_of(*observed.vp) not in {cfg.arena.sector_of(*c.vp) for c in next_cands}


def test_prior_plan_routes_around_unscanned_box_without_changing_slam():
    cfg = Config()
    cov, grid = CoverageMap(cfg), OccupancyGrid(cfg.arena)
    cov.ingest_uav_boxes([_footprint(3.0, 2.5)], 1.0, grid)
    cov.ingest_uav_boxes([_footprint(3.0, 2.5)], 2.0, grid)
    before = grid.L.copy()
    blocked, cost = cov.planning_cost_maps(grid)
    assert blocked[grid.to_cell(3.0, 2.5)]
    plan = risk_astar.plan(grid, blocked, cost, (1.0, 2.5), (5.0, 2.5))
    assert plan is not None
    assert all(not blocked[grid.to_cell(*point)] for point in plan.path)
    assert np.array_equal(grid.L, before)
    explorer = OnlineExplorer(cfg, cov)
    cand = Cand("face", (5.0, 2.5), 0.0, "look", path=plan.path)
    waypoint, intermediate = explorer.route_goal(cand, Pose2D(1.0, 2.5, 0.0), grid)
    assert intermediate and waypoint != cand.vp
    assert explorer.route_goal(cand, Pose2D(1.05, 2.5, 0.0), grid)[0] == waypoint, "Hold a waypoint until arrival"


def test_uav_priors_register_to_full_lidar_footprints_without_pulling_local_boxes():
    cfg = Config()
    cov, grid = CoverageMap(cfg), OccupancyGrid(cfg.arena)
    hints = [_footprint(1.8, 1.0), _footprint(4.3, 1.0, sx=0.8), _footprint(3.0, 3.8)]
    for hint in hints:
        for iy in range(grid.ny):
            for ix in range(grid.nx):
                x, y = grid.to_world(iy, ix)
                if abs(x - hint.cx - 0.10) < hint.sx / 2 and abs(y - hint.cy - 0.20) < hint.sy / 2:
                    grid.L[iy, ix] = grid.L_MAX
    cov.refresh(grid, None, 0.0, force=True)
    before = {(b.bid, round(b.cx, 3), round(b.cy, 3)) for b in cov.boxes.values()}
    cov.ingest_uav_boxes(hints, 1.0, grid)
    cov.ingest_uav_boxes(hints, 2.0, grid)
    assert cov.uav_alignment_report["matches"] == 3
    assert abs(cov.uav_alignment_report["dx"] - 0.10) < 0.03
    assert abs(cov.uav_alignment_report["dy"] - 0.20) < 0.03
    assert {(b.bid, round(b.cx, 3), round(b.cy, 3)) for b in cov.boxes.values()} == before
    assert all(face.det_n == 0 for box in cov.boxes.values() for face in box.faces)


def test_m3_submission_uses_raw_arena_sector_after_navigation_alignment():
    from maicon26.perception.uav_alignment import UavAlignment
    cfg = Config()
    cov = CoverageMap(cfg)
    cov._uav_alignment = UavAlignment(0.0, 0.25, 3, 0.01)
    hint = _footprint(1.8, 3.20)
    cov.ingest_uav_boxes([hint], 1.0)
    cov.ingest_uav_boxes([hint], 2.0)
    box = next(iter(cov.boxes.values()))
    assert cfg.arena.sector_of(box.cx, box.cy) == "B-S1"
    assert cfg.arena.sector_of(box.arena_cx, box.arena_cy) == "B-S2"
    face = box.faces[0]
    face.det_n, face.det_votes = 4, {"AMMO_BOX": 4.0}
    assert cov.m3_counts(None) == {"B-S2": {"AMMO_BOX": 1}}
    cov.ingest_uav_boxes([], 20.0)
    assert cov.m3_counts(None) == {"B-S2": {"AMMO_BOX": 1}}


def test_local_free_evidence_overrides_aerial_prior_but_not_observed_obstacles():
    cfg = Config()
    cov, grid = CoverageMap(cfg), OccupancyGrid(cfg.arena)
    cov.ingest_uav_boxes([_footprint(3.0, 2.5)], 1.0, grid)
    cov.ingest_uav_boxes([_footprint(3.0, 2.5)], 2.0, grid)
    centre = grid.to_cell(3.0, 2.5)
    unknown = grid.to_cell(3.1, 2.5)
    occupied = grid.to_cell(2.9, 2.5)
    grid.L[centre], grid.L[occupied] = grid.L_MIN, grid.L_MAX
    occ = cov.visibility_occupancy(grid)
    assert not occ[centre] and occ[unknown] and occ[occupied]
    assert grid.L[centre] == grid.L_MIN and grid.L[occupied] == grid.L_MAX
    assert cov.stats["uav_prior_free_overrides"] >= 1


def test_route_uses_planners_nearby_free_start_without_discarding_goal():
    cfg = Config()
    cov, grid = CoverageMap(cfg), OccupancyGrid(cfg.arena)
    cov.ingest_uav_boxes([_footprint(3.0, 2.5)], 1.0, grid)
    cov.ingest_uav_boxes([_footprint(3.0, 2.5)], 2.0, grid)
    blocked, cost = cov.planning_cost_maps(grid)
    pose = Pose2D(3.35, 2.5, 0.0)
    assert blocked[grid.to_cell(pose.x, pose.y)]
    goal = (4.5, 2.5)
    plan = risk_astar.plan(grid, blocked, cost, (pose.x, pose.y), goal)
    assert plan is not None
    cand = Cand("face", goal, 0.0, "look", path=plan.path)
    route_goal, intermediate = OnlineExplorer(cfg, cov).route_goal(cand, pose, grid)
    assert intermediate and route_goal[0] > pose.x + 0.25


def test_static_motion_rejection_requires_recent_aerial_and_repeated_full_lidar_support():
    cfg = Config()
    cov, grid = CoverageMap(cfg), OccupancyGrid(cfg.arena)
    hint = _footprint(2.0, 2.0)
    cov.ingest_uav_boxes([hint], 1.0, grid)
    cov.ingest_uav_boxes([hint], 2.0, grid)
    assert cov.confirmed_static_footprints(2.0) == [], "Aerial priors alone cannot suppress threats"
    for iy in range(grid.ny):
        for ix in range(grid.nx):
            x, y = grid.to_world(iy, ix)
            if 1.8 < x < 2.2 and 1.8 < y < 2.2:
                grid.L[iy, ix] = grid.L_MAX
    cov.refresh(grid, None, 3.0, force=True)
    cov.refresh(grid, None, 4.0, force=True)
    assert cov.confirmed_static_footprints(4.0) == [], "Require three full footprint observations"
    cov.refresh(grid, None, 5.0, force=True)
    boxes = cov.confirmed_static_footprints(5.0)
    assert len(boxes) == 1 and boxes[0].full_seen == 3
    assert cov.confirmed_static_footprints(20.0) == [], "Stale aerial support cannot suppress threats"
    boxes[0].uav_t = None
    assert cov.confirmed_static_footprints(5.0) == [], "LiDAR-only geometry is insufficient"
