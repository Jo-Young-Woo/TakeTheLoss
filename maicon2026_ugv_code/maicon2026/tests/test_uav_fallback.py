"""Aerial image outages must not stop local discovery or invent fresh geometry."""
import numpy as np

from maicon26.config import Config
from maicon26.link.base import UavBox, UavFrame
from maicon26.mission.explore_online import OnlineExplorer
from maicon26.nav.grid import OccupancyGrid
from maicon26.perception.coverage import CoverageMap
from maicon26.types import Pose2D


def setup():
    cfg = Config()
    return cfg, CoverageMap(cfg), OccupancyGrid(cfg.arena)


def fill_box(grid, x=2.0, y=2.0):
    for iy in range(grid.ny):
        for ix in range(grid.nx):
            wx, wy = grid.to_world(iy, ix)
            if abs(wx - x) < 0.2 and abs(wy - y) < 0.2:
                grid.L[iy, ix] = grid.L_MAX


def test_cold_start_discovers_cube_faces_and_plans_without_uav():
    cfg, cov, grid = setup()
    fill_box(grid)
    cov.refresh(grid, None, 1.0, force=True)
    assert len(cov.boxes) == 1 and len(cov.pending_faces()) == 4
    assert cov.layout_diagnostics()["mode"] == "lidar_camera"
    explorer = OnlineExplorer(cfg, cov)
    blocked, cost = cov.planning_cost_maps(grid)
    pose = Pose2D(1.0, 2.0, 0.0)
    cands = explorer.build(pose, grid, blocked, 2.0)
    assert any(c.kind == "face" and c.faces for c in cands)
    assert explorer.evaluate(cands, pose, grid, blocked, cost, 2.0, (3.6, 3.0))
    assert cov.m3_counts(None) == {}, "Geometry alone never supplies M3 classes"


def test_cold_start_unknown_map_surveys_sectors_without_hidden_boxes():
    cfg, cov, grid = setup()
    explorer = OnlineExplorer(cfg, cov)
    blocked, _ = cov.planning_cost_maps(grid)
    cands = explorer.build(Pose2D(1.0, 2.5, 0.0), grid, blocked, 1.0)
    assert not cov.boxes
    assert {cfg.arena.sector_of(*c.vp) for c in cands} == set(cfg.arena.sector_names())
    assert all(c.kind == "survey" for c in cands)
    assert explorer.diagnostics["last_build"]["sensor_mode"] == "lidar_camera"


def test_outage_suspends_image_only_targets_and_obstacles_without_erasing_history():
    _, cov, grid = setup()
    hint = UavBox(2.0, 2.0, .4, .4)
    cov.ingest_uav_boxes([hint], 1.0, grid)
    cov.ingest_uav_boxes([hint], 2.0, grid)
    box = next(iter(cov.boxes.values()))
    box.faces[0].views = 1
    before = grid.L.copy()
    assert cov.visibility_occupancy(grid)[grid.to_cell(2.0, 2.0)]
    cov.refresh(grid, None, 15.0, force=True)
    assert cov.uav_layout_state == "stale"
    assert cov.pending_faces() == []
    assert not cov.visibility_occupancy(grid)[grid.to_cell(2.0, 2.0)]
    assert box.bid in cov.boxes and box.faces[0].views == 1
    assert np.array_equal(grid.L, before)
    assert cov.summary()["uav_layout"]["suspended_image_only_boxes"] == 1


def test_local_lidar_box_survives_aerial_outage_with_all_ground_evidence():
    _, cov, grid = setup()
    hint = UavBox(2.0, 2.0, .4, .4)
    cov.ingest_uav_boxes([hint], 1.0, grid)
    cov.ingest_uav_boxes([hint], 2.0, grid)
    fill_box(grid)
    cov.refresh(grid, None, 3.0, force=True)
    box = next(iter(cov.boxes.values()))
    box.faces[0].det_n, box.faces[0].det_votes = 4, {"AMMO_BOX": 4.0}
    counts, before = cov.m3_counts(None), grid.L.copy()
    cov.refresh(grid, None, 20.0, force=True)
    assert cov.uav_layout_state == "stale" and box.kind == "full"
    assert cov.box_available(box) and cov.pending_faces()
    assert cov.visibility_occupancy(grid)[grid.to_cell(2.0, 2.0)]
    assert cov.m3_counts(None) == counts and counts
    assert np.array_equal(grid.L, before)


def test_recovery_requires_two_new_matching_frames_and_preserves_face_history():
    _, cov, grid = setup()
    hint = UavBox(2.0, 2.0, .4, .4)
    cov.ingest_uav_boxes([hint], 1.0, grid)
    cov.ingest_uav_boxes([hint], 2.0, grid)
    box = next(iter(cov.boxes.values()))
    box.faces[0].views = 1
    cov.tick_uav_layout(15.0)
    cov.ingest_uav_boxes([hint], 16.0, grid)
    assert cov.uav_layout_state == "acquiring" and cov.pending_faces() == []
    cov.ingest_uav_boxes([hint], 16.0, grid)  # duplicate is not another camera observation
    assert cov.uav_layout_state == "acquiring"
    cov.ingest_uav_boxes([hint], 17.0, grid)
    assert cov.uav_layout_state == "available" and len(cov.boxes) == 1
    assert cov.pending_faces() and box.faces[0].views == 1
    assert [e["to"] for e in cov.uav_layout_events] == ["acquiring", "available", "stale", "acquiring", "available"]


def test_recovery_does_not_reactivate_old_footprints_missing_from_new_images():
    _, cov, grid = setup()
    hint = UavBox(2.0, 2.0, .4, .4)
    cov.ingest_uav_boxes([hint], 1.0, grid)
    cov.ingest_uav_boxes([hint], 2.0, grid)
    cov.tick_uav_layout(15.0)
    cov.ingest_uav_boxes([], 16.0, grid)
    cov.ingest_uav_boxes([], 17.0, grid)
    assert cov.uav_layout_state == "available"  # received valid images of an empty area
    assert not cov.pending_faces()
    assert not cov.visibility_occupancy(grid)[grid.to_cell(2.0, 2.0)]


def test_old_geometry_timestamp_cannot_mask_image_outage():
    _, cov, grid = setup()
    hint = UavBox(2.0, 2.0, .4, .4)
    cov.ingest_uav_boxes([hint], 1.0, grid)
    cov.ingest_uav_boxes([hint], 2.0, grid)
    for now in (4.0, 8.0, 12.0, 16.0):
        cov.tick_uav_layout(now)
        cov.ingest_uav_boxes([hint], 2.0, grid)  # robot positions may arrive, geometry capture has not advanced
    assert cov.uav_layout_state == "stale"
    assert cov.layout_diagnostics()["age_s"] == 14.0
    assert cov.pending_faces() == []


def test_agent_detects_image_outage_while_other_uav_detections_keep_arriving():
    from maicon26.sim.run import build_match, run

    match = build_match(seed=2, teams=["B2"])
    world, agent, server = match.world, match.agents[0], match.servers[0]
    server.uav_layout_enabled = True
    server.uav_layout_provider = lambda: (
        UavFrame(world.t, boxes=[UavBox(2.0, 2.0, .4, .4)], layout_source="test_camera")
        if world.t < 4.0 else None)
    run(match, dt=.1, max_t=20.0)
    assert agent.uavmon.state == "normal", "UAV robot positions still arrive"
    assert agent.stats["uav_layout_state"] == "stale", "Geometry images have their own freshness clock"
    assert agent.coverage.layout_diagnostics()["mode"] == "lidar_camera"
    assert agent.mode.value != "SAFE_STOP"
    assert any("UAV 배치 영상 available → stale" in text for _, text in agent.log)
