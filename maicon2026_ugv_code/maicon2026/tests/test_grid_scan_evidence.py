"""Missing LiDAR directions must never fabricate free-space evidence."""
import math

import numpy as np
import pytest

from maicon26.config import Config, LidarCfg
from maicon26.hal.ros_mil import scan_in_base
from maicon26.nav.grid import OccupancyGrid
from maicon26.nav.selfmask import SelfMask
from maicon26.mission.env import speckle_filter
from maicon26.sim.world import World
from maicon26.types import Pose2D, Scan


@pytest.mark.parametrize("invalid", [np.nan, -np.inf, -1., 0., .1, .12])
def test_invalid_or_too_close_ranges_change_neither_free_nor_occupied_cells(invalid):
    grid = OccupancyGrid(Config().arena)
    grid.L[grid.to_cell(2., 2.)] = grid.L_MAX
    before = grid.L.copy()
    scan = Scan(-math.pi, math.pi/180, np.full(360, invalid), 1., 8.)
    grid.update(scan, Pose2D(3., 2.))
    np.testing.assert_array_equal(grid.L, before)


def test_rebinned_160_degree_scan_cannot_erase_a_box_behind_the_robot():
    grid = OccupancyGrid(Config().arena)
    pose = Pose2D(3., 2., 0.)
    behind = grid.to_cell(2., 2.)
    grid.L[behind] = grid.L_MAX
    amin, inc, ranges, rmax = scan_in_base(-math.radians(80), math.radians(1),
                                         np.full(161, 2.0), .05, 8., .039815, 0., 0.)
    scan = Scan(amin, inc, ranges, 1., rmax)
    assert np.isnan(ranges).any()
    for _ in range(20):
        grid.update(scan, pose)
    assert grid.L[behind] == grid.L_MAX
    assert grid.L[grid.to_cell(4., 2.)] < 0.


def test_a_masked_hole_between_valid_rays_is_not_an_observed_clear_ray():
    grid = OccupancyGrid(Config().arena)
    masked = grid.to_cell(2., 2.)
    grid.L[masked] = grid.L_MAX
    scan = Scan(-math.pi/3, math.pi/3, np.array([2., np.nan, 2.]), 1., 8.)
    for _ in range(20):
        grid.update(scan, Pose2D(1., 2., 0.))
    assert grid.L[masked] == grid.L_MAX
    assert (grid.L < 0.).any()  # Neighboring measured rays still clear normally.


@pytest.mark.parametrize("no_return", [np.inf, 2., 20.])
def test_explicit_no_return_clears_only_to_sensor_range_without_endpoint(no_return):
    grid = OccupancyGrid(Config().arena)
    near, beyond = grid.to_cell(2., 2.), grid.to_cell(3.5, 2.)
    grid.L[near] = grid.L[beyond] = grid.L_MAX
    scan = Scan(0., 1., np.array([no_return]), 1., 2.)
    for _ in range(20):
        grid.update(scan, Pose2D(1., 2., 0.), free_range=4.)
    assert grid.L[near] == grid.L_MIN
    assert grid.L[beyond] == grid.L_MAX
    assert grid.L[grid.to_cell(3., 2.)] == 0.  # No phantom max-range obstacle.


def test_finite_hit_stops_clearing_at_its_occupied_endpoint():
    grid = OccupancyGrid(Config().arena)
    scan = Scan(0., 1., np.array([1.5]), 1., 8.)
    grid.update(scan, Pose2D(1., 2., 0.))
    assert grid.L[grid.to_cell(2., 2.)] < 0.
    assert grid.L[grid.to_cell(2.5, 2.)] == pytest.approx(grid.L_OCC)
    assert grid.L[grid.to_cell(3., 2.)] == 0.


@pytest.mark.parametrize("filter_name", ["body_mask", "smog_speckle"])
def test_filtered_occlusion_does_not_erase_an_obstacle_behind_it(filter_name):
    grid = OccupancyGrid(Config().arena)
    obstacle = grid.to_cell(2., 2.)
    grid.L[obstacle] = grid.L_MAX
    ranges = np.full(360, np.nan)
    ranges[180] = .2
    scan = Scan(-math.pi, math.pi/180, ranges, 1., 8.)
    if filter_name == "body_mask":
        scan = SelfMask(LidarCfg(self_mask_sectors=((0., 15.),), self_mask_range=.35)).apply(scan)
    else:
        scan = speckle_filter(scan, 1.2, .25)
    assert np.isnan(scan.ranges[180])
    for _ in range(20):
        grid.update(scan, Pose2D(1., 2., 0.))
    assert grid.L[obstacle] == grid.L_MAX


@pytest.mark.parametrize("rmax", [.2, 20.])
def test_simulated_sensor_dropouts_are_unknown_not_clear_no_return_rays(rmax):
    cfg = Config()
    cfg.lidar_err.sim_dropout_p = 1.
    cfg.lidar_err.sim_outlier_p = 0.
    world = World(cfg, seed=2)
    robot = world.add_ugv("BLUE", 2)
    scan = world.lidar(robot, rmax=rmax)
    assert np.isnan(scan.ranges).all()
    grid = OccupancyGrid(cfg.arena)
    grid.update(scan, robot.pose)
    assert not np.any(grid.L)
