"""Long obstacles remain solid away from their centre; no planner is involved."""
import math

import numpy as np
import pytest

from maicon26.config import Config
from maicon26.footprint import aabb_corners, sat_push
from maicon26.sim.world import World
from maicon26.types import Pose2D


def _empty_world(bounds):
    cfg = Config()
    cfg.arena.validate_layout = False
    world = World(cfg, seed=2)
    world.cubes = []
    world.add_low_obstacle(*bounds, .12)
    return world, world.add_ugv("BLUE", 2)


@pytest.mark.parametrize("yaw", [0., math.pi/4, math.pi/2])
@pytest.mark.parametrize("horizontal", [False, True])
def test_long_obstacle_resolves_rotated_footprint_contact_far_from_its_center(yaw, horizontal):
    bounds = (.55, 1.55, 5.55, 1.75) if horizontal else (1.55, .55, 1.75, 4.45)
    world, ugv = _empty_world(bounds)
    half_x = .125*abs(math.cos(yaw)) + .115*abs(math.sin(yaw))
    half_y = .125*abs(math.sin(yaw)) + .115*abs(math.cos(yaw))
    ugv.pose = (Pose2D(4.6, 1.55-half_y+.02, yaw) if horizontal
                else Pose2D(1.55-half_x+.02, 3.3, yaw))
    before = np.array([ugv.pose.x, ugv.pose.y])
    assert sat_push(world.ugv_corners(ugv), aabb_corners(*bounds)) is not None
    world.resolve_collisions(ugv)
    residual = sat_push(world.ugv_corners(ugv), aabb_corners(*bounds))
    assert residual is None or np.linalg.norm(residual) < 1e-8
    assert np.linalg.norm(np.array([ugv.pose.x, ugv.pose.y])-before) >= .019
    assert ugv.collisions == 1


def test_simulated_forward_drive_cannot_pass_through_far_end_of_long_barrier():
    world, ugv = _empty_world((1.55, .55, 1.75, 4.45))
    ugv.pose = Pose2D(1.25, 3.3, 0.)
    ugv.cmd_v = .5
    for _ in range(100):
        world.step(.05)
        assert world.ugv_corners(ugv)[:, 0].max() <= 1.55 + 1e-8
    assert ugv.collisions > 0
    assert ugv.pose.x == pytest.approx(1.425, abs=.002)


def test_long_barrier_clear_gap_does_not_push_a_rotated_robot():
    world, ugv = _empty_world((1.55, .55, 1.75, 4.45))
    ugv.pose = Pose2D(1.65, 4.75, math.pi/4)
    before = ugv.pose.copy()
    world.resolve_collisions(ugv)
    assert ugv.pose == before
    assert ugv.collisions == 0
