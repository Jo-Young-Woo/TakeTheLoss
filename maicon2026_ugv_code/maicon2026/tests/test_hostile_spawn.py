"""A delayed encounter is absent, rather than a visible parked opponent."""
import math
from types import SimpleNamespace

import numpy as np
import pytest

from maicon26.config import Config
from maicon26.sim.bench import Probe
from maicon26.sim.hostile import Blocker
from maicon26.sim.robot import SimServer
from maicon26.sim.world import World
from maicon26.types import Pose2D


def empty_world():
    cfg = Config()
    cfg.lidar_err.sim_range_sigma_m = cfg.lidar_err.sim_range_sigma_per_m = 0.
    cfg.lidar_err.sim_range_bias_m = cfg.lidar_err.sim_dropout_p = 0.
    world = World(cfg, seed=3, m5_ends_match=False)
    world.cubes, world.low_obstacles = [], []
    world.uav_latency = world.uav_sigma = 0.
    world.ugv_miss_base = world.ugv_miss_slope = world.team_flip = 0.
    observer = world.add_ugv("BLUE", 2)
    observer.pose = Pose2D(1., 2.5, 0.)
    return world, observer, SimpleNamespace(world=world, agents=[])


def delayed(world, observer, **kwargs):
    # Deliberate overlap makes even a collision leak observable before spawn.
    return Blocker(world, "RED", 1, observer.pose.copy(), observer.id,
                   spawn_rel=(2., 0.), start_t=kwargs.pop("start_t", 10.), **kwargs)


def test_before_spawn_no_lidar_camera_collision_uav_or_exposure_ghost():
    world, observer, match = empty_world()
    server = SimServer(world, observer)
    baseline = world.lidar(observer).ranges.copy()
    hostile = delayed(world, observer)
    assert world.ugvs == [observer] and not hostile.spawned
    np.testing.assert_equal(world.lidar(observer).ranges, baseline)
    assert not any(d.cls == "UGV" for d in world.camera_detections(observer))
    assert [d.team for d in server.get_uav().detections] == ["BLUE"]
    pose = observer.pose.copy()
    world.resolve_collisions(observer)
    assert observer.pose == pose and observer.collisions == 0
    assert not hostile.can_hit(observer)
    probe = Probe(observer.id)
    probe(match, [hostile])
    assert math.isinf(probe.min_dist) and probe.exposure_s == 0.
    world.t = 9.999
    hostile.step(match)
    assert world.ugvs == [observer] and not hostile.spawned
    assert [d.team for d in server.get_uav().detections] == ["BLUE"]


def test_successful_appearance_enters_all_existing_sensor_and_physics_paths_once():
    world, observer, match = empty_world()
    server = SimServer(world, observer)
    hostile = delayed(world, observer)
    world.t = 10.
    assert hostile.active(match)
    assert hostile.spawned and world.ugvs.count(hostile.u) == 1
    assert hostile.spawn_xy == pytest.approx((3., 2.5))
    scan = world.lidar(observer)
    forward = int(round(-scan.angle_min/scan.angle_inc))
    assert scan.ranges[forward] == pytest.approx(2.-world.cfg.robot.length/2, abs=.01)
    assert any(d.cls == "UGV" and d.team == "RED" for d in world.camera_detections(observer))
    assert sorted(d.team for d in server.get_uav().detections) == ["BLUE", "RED"]
    for _ in range(4):
        hostile.step(match)
    assert world.ugvs.count(hostile.u) == 1
    # It now participates in ordinary collisions, with no custom ghost layer.
    hostile.u.pose = observer.pose.copy()
    pose = observer.pose.copy()
    world.resolve_collisions(observer)
    assert observer.pose != pose and observer.collisions > 0


def test_blocked_placement_stays_absent_and_retry_registers_only_on_success(monkeypatch):
    world, observer, match = empty_world()
    hostile = delayed(world, observer)
    world.t = 10.
    original = world._blocked
    monkeypatch.setattr(world, "_blocked", lambda *args, **kwargs: True)
    for _ in range(3):
        assert not hostile.active(match)
        world.t += .05
    assert world.ugvs == [observer] and hostile.spawn_xy is None
    assert not hostile.can_hit(observer)
    monkeypatch.setattr(world, "_blocked", original)
    assert hostile.active(match)
    for _ in range(3):
        assert hostile.active(match)
    assert world.ugvs == [observer, hostile.u]


def test_trigger_relative_delay_has_no_parked_body_before_trigger_or_during_wait():
    world, observer, match = empty_world()
    armed = {"value": False}
    hostile = delayed(world, observer, start_t=3., trigger=lambda match: armed["value"])
    world.t = 50.
    assert not hostile.active(match) and hostile.u not in world.ugvs
    armed["value"] = True
    assert not hostile.active(match) and hostile.start_t == 53.
    world.t = 52.999
    assert not hostile.active(match) and hostile.u not in world.ugvs
    world.t = 53.
    assert hostile.active(match) and world.ugvs.count(hostile.u) == 1


def test_preexisting_opponent_with_delayed_action_remains_a_physical_robot():
    world, observer, match = empty_world()
    hostile = Blocker(world, "RED", 1, Pose2D(3., 2.5, math.pi), observer.id,
                      start_t=10., spawn_rel=None)
    assert hostile.spawned and hostile.u in world.ugvs
    assert not hostile.active(match)
    assert any(d.cls == "UGV" for d in world.camera_detections(observer))
    world.t = 10.
    assert hostile.active(match) and world.ugvs.count(hostile.u) == 1
