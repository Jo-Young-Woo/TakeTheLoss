"""SLAM edge changes must not erase independently observed object pictures."""
from collections import defaultdict

import numpy as np

from maicon26.config import Config
from maicon26.hal.ros_mil import scan_in_base
from maicon26.mission.explore_online import OnlineExplorer
from maicon26.nav.grid import OccupancyGrid
from maicon26.perception.coverage import CoverageMap
from maicon26.perception.object_map import Face, ObjectMap
from maicon26.types import Pose2D, Scan


def scene():
    cfg = Config()
    cov, grid, omap = CoverageMap(cfg), OccupancyGrid(cfg.arena), ObjectMap(cfg)
    # A closed LiDAR outline has a known full footprint, but unknown interior.
    for iy in range(grid.ny):
        for ix in range(grid.nx):
            x, y = grid.to_world(iy, ix)
            if (1.8 < x < 2.2 and 1.8 < y < 2.2
                    and (x < 1.86 or x > 2.14 or y < 1.86 or y > 2.14)):
                grid.L[iy, ix] = grid.L_MAX
    for now in (0.0, 1.0, 2.0):
        cov.refresh(grid, omap, now, force=True)
    box = next(iter(cov.boxes.values()))
    assert box.kind == "full" and box.full_seen == 3
    omap.faces = [Face(2.2, 2.0, 2.0, 2.0, defaultdict(float, AMMO_BOX=4.0), 4, vx=1.0)]
    return cfg, cov, grid, omap, box


def clear_interior(grid):
    for iy in range(grid.ny):
        for ix in range(grid.nx):
            x, y = grid.to_world(iy, ix)
            if 1.6 < x < 2.4 and 1.6 < y < 2.4:
                grid.L[iy, ix] = grid.L_MIN


def test_thin_outline_clearing_without_interior_rays_keeps_box_and_ground_votes():
    _, cov, grid, omap, box = scene()
    for cell in box.cells:
        grid.L[cell] = grid.L_MIN
    for now in (3.0, 4.0, 5.0, 6.0):
        cov.refresh(grid, omap, now, force=True)
    assert box.bid in cov.boxes
    assert len(omap.confirmed_faces()) == 1 and cov.stats["dropped_faces"] == 0


def test_turning_160_degree_lidar_away_keeps_box_and_confirmed_picture():
    _, cov, grid, omap, box = scene()
    # The real HAL rebins a forward-only scan to 360 degrees. The rear bins are
    # NaN, which previously became fictitious rays through the entire old box.
    amin, inc, ranges, rmax = scan_in_base(-np.deg2rad(80), np.deg2rad(1),
                                         np.full(161, 2.5), .05, 8., .039815, 0., 0.)
    scan = Scan(amin, inc, ranges, 1., rmax)
    for now in range(3, 25):
        grid.update(scan, Pose2D(3., 2., 0.))
        cov.refresh(grid, omap, float(now), force=True)
    assert box.bid in cov.boxes and box.free_checks == 0
    assert len(omap.confirmed_faces()) == 1
    assert cov.stats["boxes_moved"] == 0 and cov.stats["dropped_faces"] == 0


def test_nearby_fragment_prevents_false_movement_during_slam_edge_shift():
    _, cov, grid, omap, box = scene()
    clear_interior(grid)
    # Fewer than five cells cannot form a new cluster, but still refute removal.
    grid.L[grid.to_cell(2.24, 2.0)] = grid.L_MAX
    for now in (3.0, 4.0, 5.0, 6.0):
        cov.refresh(grid, omap, now, force=True)
    assert box.bid in cov.boxes and len(omap.faces) == 1


def test_verified_full_box_is_removed_after_repeated_free_interior_observations():
    _, cov, grid, omap, box = scene()
    clear_interior(grid)
    cov.refresh(grid, omap, 3.0, force=True)
    cov.refresh(grid, omap, 4.0, force=True)
    assert box.bid in cov.boxes and len(omap.faces) == 1
    cov.refresh(grid, omap, 5.0, force=True)
    assert box.bid not in cov.boxes and omap.faces == []
    assert cov.stats["dropped_faces"] == 1


def test_partial_hypothesis_disappearance_cannot_erase_camera_classifications():
    _, cov, grid, omap, box = scene()
    box.kind, box.full_seen = "face", 0
    clear_interior(grid)
    for now in (3.0, 4.0, 5.0):
        cov.refresh(grid, omap, now, force=True)
    assert box.bid not in cov.boxes
    assert len(omap.confirmed_faces()) == 1 and cov.stats["dropped_faces"] == 0
    assert sum(sum(v.values()) for v in cov.m3_counts(omap).values()) == 1


def test_reobserved_outline_resets_the_consecutive_clearance_requirement():
    _, cov, grid, omap, box = scene()
    original = grid.L.copy()
    clear_interior(grid)
    for now in (3.0, 4.0):
        cov.refresh(grid, omap, now, force=True)
    assert box.free_checks == 2
    grid.L[:] = original
    cov.refresh(grid, omap, 5.0, force=True)
    assert box.free_checks == 0 and box.free_since is None
    clear_interior(grid)
    for now in (6.0, 7.0):
        cov.refresh(grid, omap, now, force=True)
    assert box.bid in cov.boxes and len(omap.faces) == 1
    cov.refresh(grid, omap, 8.0, force=True)
    assert box.bid not in cov.boxes and omap.faces == []


def test_moved_box_face_deletion_preserves_touching_neighbour_opposite_face():
    _, _, _, omap, box = scene()
    neighbour = Face(2.2, 2.0, 2.4, 2.0, defaultdict(float, DRUM=4.0), 4, vx=-1.0)
    omap.faces.append(neighbour)
    assert omap.drop_box_faces(box.faces) == 1
    assert omap.faces == [neighbour]


def test_unconfirmed_geometry_does_not_mask_independent_confirmed_picture():
    _, cov, _, omap, _ = scene()
    omap.faces[0].x, omap.faces[0].cx = 2.50, 2.30
    # .30 m is outside face association (.25), but inside the old blanket .35 suppression.
    assert sum(sum(v.values()) for v in cov.m3_counts(omap).values()) == 1


def test_counted_east_face_does_not_mask_independent_north_face():
    _, cov, _, omap, box = scene()
    east = box.faces[0]
    east.det_n, east.det_votes = 4, {"AMMO_BOX": 4.0}
    north = Face(2.0, 2.2, 2.0, 2.0, defaultdict(float, DRUM=4.0), 4, vy=1.0)
    omap.faces.append(north)
    cov._sync_detections(omap)  # Known full-cube photos must resolve to a ledger face before counting.
    counts = cov.m3_counts(omap)
    assert sum(sum(v.values()) for v in counts.values()) == 2


def test_confirmed_face_copy_preserves_normal_timestamp_and_led_votes():
    omap = ObjectMap(Config())
    face = Face(2.0, 2.2, 2.0, 2.0, defaultdict(float, DRUM=4.0), 4,
                led_on=True, led_votes=3, last_t=12.5, vx=.1, vy=.995)
    omap.faces.append(face)
    copy = omap.confirmed_faces()[0]
    assert (copy.vx, copy.vy, copy.last_t, copy.led_votes, copy.led_on) == (.1, .995, 12.5, 3, True)
    assert copy.votes is not face.votes


def test_tiny_low_gain_frontier_does_not_prevent_unsurveyed_sector_search():
    cfg = Config()
    cov, grid = CoverageMap(cfg), OccupancyGrid(cfg.arena)
    explorer = OnlineExplorer(cfg, cov)
    cov.seen.seen[:] = True
    cov.seen.seen[20:22, 20:22] = False
    blocked, _ = cov.planning_cost_maps(grid)
    cands = explorer.build(Pose2D(1.0, 2.0, 0.0), grid, blocked, 1.0)
    tiny = [c for c in cands if c.kind == "frontier"]
    assert tiny and all(c.gain < cfg.explore.min_gain for c in tiny)
    assert any(c.kind == "survey" and c.gain >= cfg.explore.min_gain for c in cands)
