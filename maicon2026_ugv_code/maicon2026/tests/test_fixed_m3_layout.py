"""PDF M3 additions must keep the fixed field navigable and reproducible."""
from dataclasses import replace
import math
from types import SimpleNamespace

import numpy as np
import pytest

from maicon26.config import Config
from maicon26.sim.fixed_maicon import (
    STRUCTURES, M3_CLASSES, generate_m3_structures, structure_gap, _reachable_views,
    FixedStructure, audit_m3_accessibility, M3_TURN_RADIUS,
    M3_VIEW_NORMAL_DISTANCES,
    _mirrored_x_lattice,
)
from maicon26.footprint import aabb_corners, ray_rect
from maicon26.sim.world import World
from maicon26.types import Pose2D


def geometry():
    return replace(Config().arena, base_depth=.4, base_height=1.2)


@pytest.mark.parametrize("seed", [0, 1, 2, 3, 4, 8, 19, 42, 99])
def test_m3_seeds_preserve_clearance_and_reachable_four_sides(seed):
    geo = geometry()
    cubes = generate_m3_structures(seed, geo)
    assert len(cubes) == 4
    assert _reachable_views((*STRUCTURES, *cubes), cubes, geo)
    sectors = {geo.sector_of(cube.x, cube.y) for cube in cubes}
    assert any(sector and sector.startswith("B-S") for sector in sectors)
    assert any(sector and sector.startswith("R-S") for sector in sectors)
    assert min(cube.y for cube in cubes) < geo.H/2 < max(cube.y for cube in cubes)
    for i, cube in enumerate(cubes):
        assert (cube.sx, cube.sy) == (.4, .4)
        assert cube.x-.2 >= .4-1e-8 and geo.W-cube.x-.2 >= .4-1e-8
        assert cube.y-.2 >= .4-1e-8 and geo.H-cube.y-.2 >= .4-1e-8
        assert {face for face, _, _ in cube.faces} == {0, 1, 2, 3}
        assert all(kind == "object" and cls in M3_CLASSES for _, kind, cls in cube.faces)
        assert all(structure_gap(cube, other) >= .4-1e-8 for other in (*STRUCTURES, *cubes[:i]))


def test_m3_seed_is_reproducible_and_changes_layout_and_labels():
    geo = geometry()
    assert generate_m3_structures(2, geo) == generate_m3_structures(2, geo)
    assert generate_m3_structures(2, geo) != generate_m3_structures(3, geo)
    assert generate_m3_structures(2, geo, count=0) == ()
    with pytest.raises(ValueError, match="fixed_m3_count"):
        generate_m3_structures(2, geo, count=-1)


def test_fixed_width_lattice_contains_both_10cm_phases_and_is_mirror_closed():
    points = _mirrored_x_lattice(6.12)
    assert .80 in points and 5.32 in points
    assert all(round(6.12-x, 2) in points for x in points)
    assert all(round(x, 2) == x and .60 <= x <= 5.52 for x in points)


def test_fixed_referee_only_counts_dedicated_m3_cube_faces():
    cfg = Config()
    cfg.arena = geometry()
    world = World(cfg, seed=2, scenario="maicon_fixed")
    assert all(item.kind != "object" for cube in world.cubes[:len(STRUCTURES)] for item in cube.items.values())
    assert sum(item.kind == "object" for cube in world.cubes for item in cube.items.values()) == 16
    assert len([item for cube in world.cubes for item in cube.items.values() if item.kind == "target"]) == 6
    assert len(World(cfg, seed=2, scenario="maicon_fixed", fixed_m3_count=0).cubes) == len(STRUCTURES)


def test_every_photo_face_has_two_reachable_complete_picture_witnesses():
    geo = geometry()
    assert set(M3_VIEW_NORMAL_DISTANCES) <= set(Config().explore.nbv_dists), "Explorer must propose the audited pose family"
    cubes = generate_m3_structures(2, geo)
    structures = (*STRUCTURES, *cubes)
    report = audit_m3_accessibility(structures, cubes, geo)
    assert report["valid"] and len(report["faces"]) == 16
    assert report["camera"] == {"width": 640, "height": 480, "height_m": .25,
                                "forward_offset_m": 0., "pan_deg": 0., "tilt_deg": 0.}
    cfg = Config()
    cfg.arena = geo
    world = World(cfg, seed=2, scenario="maicon_fixed")
    robot = world.add_ugv("BLUE", 2)
    # Test the actual synthetic ground-camera projection without random dropout
    # or classification noise; visibility and cropping still use production code.
    world.rng = SimpleNamespace(random=lambda: .99, gauss=lambda mean, std: mean,
                                choice=lambda seq: seq[0])
    by_name = {cube.name: cube for cube in cubes}
    for face in report["faces"]:
        cube = by_name[face["cube"]]
        nx, ny = ((1, 0), (0, 1), (-1, 0), (0, -1))[face["face"]]
        fx, fy = cube.x + .2*nx, cube.y + .2*ny
        a, b = face["poses"]
        assert math.hypot(a["x"]-b["x"], a["y"]-b["y"]) >= .20-1e-8
        for pose in face["poses"]:
            assert pose["incidence_deg"] <= 55 and .40 <= pose["distance_m"] <= 1.4
            assert .30-1e-9 <= pose["x"] <= geo.W-.30+1e-9
            assert .30-1e-9 <= pose["y"] <= geo.H-.30+1e-9
            u0, v0, u1, v1 = pose["picture_bbox_px"]
            assert 2 <= u0 < u1 <= 638 and 2 <= v0 < v1 <= 478
            robot.pose = Pose2D(pose["x"], pose["y"], pose["yaw"])
            detections = world.camera_detections(robot)
            assert any(det.extra.get("_gt") == (cube.x, cube.y, face["face"])
                       for det in detections), "Production ground camera must see the audited photo face"
            route = np.array(pose["route"])
            assert np.allclose(route[0], (.35, geo.base_cy[1]))
            assert np.allclose(route[-1], (pose["x"], pose["y"]))
            assert np.max(np.linalg.norm(np.diff(route, axis=0), axis=1)) < .072
            # Check centres and segment midpoints independently of the audit's
            # occupancy mask. The full circumscribed turning disk must fit.
            points = np.concatenate((route, (route[:-1]+route[1:])/2))
            assert np.min(points[:, 0]) >= M3_TURN_RADIUS
            assert np.max(points[:, 0]) <= geo.W-M3_TURN_RADIUS
            assert np.min(points[:, 1]) >= M3_TURN_RADIUS
            assert np.max(points[:, 1]) <= geo.H-M3_TURN_RADIUS
            for obstacle in structures:
                x0, x1 = obstacle.x-obstacle.sx/2, obstacle.x+obstacle.sx/2
                y0, y1 = obstacle.y-obstacle.sy/2, obstacle.y+obstacle.sy/2
                dx = np.maximum(np.maximum(x0-points[:, 0], points[:, 0]-x1), 0.)
                dy = np.maximum(np.maximum(y0-points[:, 1], points[:, 1]-y1), 0.)
                assert np.min(np.hypot(dx, dy)) >= M3_TURN_RADIUS-1e-8
                # Independent ray/polygon check for the entire horizontal
                # photo extent; checking only its centre used to miss this.
                for lateral in (-.18, 0., .18):
                    tx, ty = fx + nx*.002 - ny*lateral, fy + ny*.002 + nx*lateral
                    dx, dy = tx-pose["x"], ty-pose["y"]
                    length = math.hypot(dx, dy)
                    assert ray_rect(pose["x"], pose["y"], dx/length, dy/length,
                                    aabb_corners(x0, y0, x1, y1), length) is None


def test_legal_40cm_wall_gap_is_rejected_when_picture_views_are_impossible():
    geo = geometry()
    cube = FixedStructure("wall_cube", .6, 1.4, .4, .4,
                          tuple((face, "object", "DRUM") for face in range(4)))
    assert cube.x-cube.sx/2 == pytest.approx(.4)
    assert all(structure_gap(cube, old) >= .4-1e-8 for old in STRUCTURES)
    report = audit_m3_accessibility((*STRUCTURES, cube), (cube,), geo)
    assert not report["valid"]
    assert "reachable views" in report["reason"]
