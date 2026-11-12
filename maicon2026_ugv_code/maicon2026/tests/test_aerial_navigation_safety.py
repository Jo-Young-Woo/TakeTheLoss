"""Cross-frame navigation regressions; no Gazebo/referee state enters agents."""
import math
from types import SimpleNamespace

import numpy as np

from maicon26.config import Config
from maicon26.nav.grid import OccupancyGrid
from maicon26.nav.localize import PoseEstimator
from maicon26.nav.navigator import Navigator
from maicon26.nav.safety import SafetyLimiter
from maicon26.perception.coverage import CoverageMap
from maicon26.types import Pose2D, Scan


def points_scan(points):
    ranges = np.full(720, np.inf)
    for x, y in points:
        i = round((math.atan2(y, x) + math.pi) / (2 * math.pi) * len(ranges)) % len(ranges)
        ranges[i] = math.hypot(x, y)
    return Scan(-math.pi, 2 * math.pi / len(ranges), ranges)


def test_side_corner_blocks_pure_rotation_but_allows_straight_escape():
    guard = SafetyLimiter(Config())
    scan = points_scan([(.11, .14)])  # Outside frontal +/-35deg, inside rotating corner.
    assert guard.limit_swept(scan, 0., 1.) == (0., 0.)
    assert guard.limit_swept(scan, -.1, 0.) == (-.1, 0.)


def test_swept_forward_reverse_and_no_return_are_distinct():
    guard = SafetyLimiter(Config())
    assert guard.limit_swept(points_scan([(.21, 0)]), .4, 0.) == (0., 0.)
    assert guard.limit_swept(points_scan([(-.21, 0)]), -.4, 0.) == (0., 0.)
    assert guard.limit_swept(points_scan([]), .3, .5) == (.3, .5)
    scan = points_scan([])
    scan.ranges[:] = np.nan
    assert guard.limit_swept(scan, .3, .5) == (0., 0.)


def make_coverage():
    cfg = Config()
    cfg.explore.uav_outer_faces = True
    cov, grid = CoverageMap(cfg), OccupancyGrid(cfg.arena)
    return cfg, cov, grid


def feed(cov, rows):
    for t in (1., 2., 3.):
        cov.ingest_uav_boxes([SimpleNamespace(cx=x, cy=y, sx=.4, sy=.4, confidence=.95) for x, y in rows], t)


def test_uav_internal_is_reversible_without_erasing_lidar_internal():
    _, cov, grid = make_coverage()
    feed(cov, [(2., 2.), (2.4, 2.)])
    cov.refresh(grid, None, 4., force=True)
    a, b = sorted(cov.boxes.values(), key=lambda b: b.cx)
    assert 0 in a.internal
    a.lidar_internal = [1]
    b.cx = b.arena_cx = 3.5
    cov._update_uav_internal()
    assert a.internal == [1]


def test_wall_decision_uses_arena_not_shifted_slam_frame():
    _, cov, _ = make_coverage()
    feed(cov, [(2., 3.8)])
    b = next(iter(cov.boxes.values()))
    before = cov._face_on_wall(b, 1)
    b.cy += .7
    assert cov._face_on_wall(b, 1) == before


def test_outer_segments_preserve_partial_exposure_and_remove_seams():
    _, cov, grid = make_coverage()
    feed(cov, [(2., 2.), (2.4, 2.2)])
    cov.refresh(grid, None, 4., force=True)
    a = min(cov.boxes.values(), key=lambda b: b.cx)
    east = next(f for f in cov.aerial_face_map()['faces'] if f['box'] == a.bid and f['face'] == 'E')
    assert east['exposed_fraction'] == .5
    assert np.allclose(east['segments'], [[[2.2, 1.8], [2.2, 2.0]]])
    assert east['state'] != 'identified'


def test_priors_block_actual_path_without_becoming_lidar_measurements():
    cfg, cov, grid = make_coverage()
    feed(cov, [(3., 2.4)])
    before = grid.L.copy()
    prior = cov.visibility_occupancy(grid) & ~grid.occupied()
    grid.set_planning_prior(prior)
    cell = grid.to_cell(3., 2.4)
    assert grid.planning_occupied()[cell] and not grid.occupied()[cell]
    nav = Navigator(cfg)
    nav.grid = grid
    nav.set_goal((4., 2.4))
    assert nav._plan(Pose2D(2., 2.4, 0.), 4.)
    # Actual path must detour around the image box, despite unknown LiDAR interior.
    assert any(abs(y - 2.4) > .35 for _, y in nav.path)
    assert np.array_equal(grid.L, before)
    grid.L[cell] = grid.L_FREE
    assert cov.visibility_occupancy(grid)[cell]  # One ray cannot erase a confirmed image footprint.
    grid.L[cell] = -2.
    assert not cov.visibility_occupancy(grid)[cell]


def test_independent_aerial_geometry_resists_a_wrong_local_likelihood():
    cfg = Config()
    pe = PoseEstimator(cfg.arena, n=200, seed=1)
    pe.init(Pose2D(1., 2., 0.), Pose2D(), sd_xy=0., sd_th=0.)
    pe.p[100:, 0] += .3
    pe.p[100:, 2] += .1
    # A warped local map declares *both* hypotheses perfect. Independent
    # calibrated rectangles must still favour the actual observed surfaces.
    field = SimpleNamespace(lookup=lambda ex, ey: np.zeros_like(ex))
    pts = [(1., y) for y in np.linspace(-.2, .2, 20)]
    scan = points_scan(pts)
    pe.correct(scan, field=field, aerial_landmarks=[(2.2, 2., .4, .4), (4., 3., .4, .4), (3., 1., .4, .4)])
    assert pe.pose.x < 1.06 and abs(pe.pose.th) < .025


def test_landmarks_stay_in_image_frame_and_expire_without_freeze():
    _, cov, grid = make_coverage()
    feed(cov, [(2., 2.), (4., 3.), (3., 1.)])
    for b in cov.boxes.values():
        b.uav_frozen = True
        b.cx += .3
    assert sorted(r[0] for r in cov.aerial_landmarks()) == [2., 3., 4.]
    for b in cov.boxes.values():
        b.uav_frozen = False
    cov.refresh(grid, None, 40., force=True)
    assert cov.aerial_landmarks() == []
