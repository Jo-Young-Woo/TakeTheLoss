"""Ground-observed geometry and camera votes must suffice for four-face NBV.

No World, aerial feed, model names, hidden cube list or expected class labels
are passed to the explorer. The fixture is a resolved LiDAR occupancy cluster
and successive localized ground-camera observations.
"""
import math

import numpy as np

from maicon26.config import Config
from maicon26.mission.explore_online import OnlineExplorer
from maicon26.nav.grid import OccupancyGrid
from maicon26.perception.coverage import CoverageMap, FaceRec
from maicon26.perception.object_map import ObjectMap
from maicon26.types import Detection, Pose2D


def ground_scene():
    cfg = Config()
    cfg.explore.complete_m3_faces = True
    grid = OccupancyGrid(cfg.arena)
    iy0, ix0 = grid.to_cell(1.8, 1.8)
    iy1, ix1 = grid.to_cell(2.2, 2.2)
    grid.L[iy0:iy1, ix0:ix1] = grid.L_MAX
    cov, omap = CoverageMap(cfg), ObjectMap(cfg)
    cov.refresh(grid, omap, 1., force=True)
    assert len(cov.boxes) == 1
    box = next(iter(cov.boxes.values()))
    return cfg, grid, cov, omap, box


def observe_face(omap, cov, face, pose, now):
    # Feed the existing camera-localization association with repeated, already
    # localized ground observations; no class is available to the route builder.
    for i in range(5):
        detection = Detection("DRUM", .95, (100., 100., 200., 200.), now+i*.1)
        omap._add(detection, face.cx, face.cy,
                  math.atan2(face.cy-pose.y, face.cx-pose.x), now+i*.1, pose,
                  nrm=(face.nx, face.ny))
    cov._sync_detections(omap)


def test_ground_photo_drives_three_remaining_face_visits_without_aerial_geometry():
    cfg, grid, cov, omap, box = ground_scene()
    pose = Pose2D(box.cx+.8, box.cy, math.pi)
    observe_face(omap, cov, box.faces[0], pose, 2.)
    explorer = OnlineExplorer(cfg, cov)
    assert cov.photo_box(box)
    assert box.uav_t is None
    pending = {f.key for f in cov.pending_faces()}
    assert len(pending) == 3
    for step in range(3):
        blocked, cost = cov.planning_cost_maps(grid)
        candidates = [c for c in explorer.build(pose, grid, blocked, 3.+step*5)
                      if c.kind == "face" and any(key in pending for key in c.faces)]
        # Every still-unclassified face retains a usable candidate; closest
        # invalid samples cannot exhaust its three-sample budget.
        assert pending <= {key for c in candidates for key in c.faces}
        ranked = explorer.evaluate(candidates, pose, grid, blocked, cost,
                                   3.+step*5, (3.6, 3.))
        assert ranked and ranked[0].path
        chosen = ranked[0]
        assert chosen.mode == "look"
        pose = Pose2D(*chosen.vp, chosen.heading)
        for key in chosen.faces:
            if key in pending:
                face = cov.get_face(key)
                assert cov.picture_visible(pose.x, pose.y, pose.th, face,
                                           cov.visibility_occupancy(grid), grid)[0]
                observe_face(omap, cov, face, pose, 4.+step*5)
        pending = {f.key for f in cov.pending_faces()}
        if not pending:
            break
    assert not pending
    assert sum(sum(counts.values()) for counts in cov.m3_counts(omap).values()) == 4


def test_picture_center_in_view_is_not_enough_when_an_edge_is_clipped():
    cfg = Config()
    cov, grid = CoverageMap(cfg), OccupancyGrid(cfg.arena)
    face = FaceRec(1, 0, 1., 0., 2., 2.)
    occ = np.zeros_like(grid.static)
    heading = math.pi+math.radians(45.)
    assert cov.face_visible(2.6, 2., heading, face, occ, grid)[0]
    assert cov.picture_visible(2.6, 2., heading, face, occ, grid) == (False, "fov")


def test_picture_edge_occluder_is_rejected_even_when_center_ray_is_clear():
    cfg = Config()
    cov, grid = CoverageMap(cfg), OccupancyGrid(cfg.arena)
    face = FaceRec(1, 0, 1., 0., 2.2, 2.)
    occ = np.zeros_like(grid.static)
    occ[grid.to_cell(2.5, 2.08)] = True
    assert cov.face_visible(2.8, 2., math.pi, face, occ, grid)[0]
    assert cov.picture_visible(2.8, 2., math.pi, face, occ, grid) == (False, "occluded")


def test_strict_picture_view_still_allows_accessible_oblique_close_view():
    cfg = Config()
    cov, grid = CoverageMap(cfg), OccupancyGrid(cfg.arena)
    face = FaceRec(1, 0, 1., 0., 2., 2.)
    occ = np.zeros_like(grid.static)
    heading = math.atan2(-.3, -.3)
    assert cov.picture_visible(2.3, 2.3, heading, face, occ, grid)[0]
    # Ordinary unclassified-cube visibility remains less restrictive.
    assert cov.face_visible(3.6, 2., math.pi, face, occ, grid)[0]
    assert not cov.picture_visible(3.6, 2., math.pi, face, occ, grid)[0]
