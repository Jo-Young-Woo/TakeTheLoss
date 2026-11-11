"""An image-only Gazebo UAV feed must not leak hidden robot coordinates."""
from types import SimpleNamespace

import numpy as np
import pytest

from maicon26.link.base import UavBox, UavDet, UavFrame
from maicon26.sim.robot import SimServer
from maicon26.types import Pose2D


def setup_server():
    robot = SimpleNamespace(id="B-UGV-2", team="BLUE", slot=2, pose=Pose2D(.35, 2.5))
    enemy = SimpleNamespace(id="R-UGV-1", team="RED", slot=1, pose=Pose2D(5.5, 4.))
    world = SimpleNamespace(scenario="maicon_fixed", assign={}, goal_base_types={}, ugvs=[robot, enemy],
                            t=0., uav_sigma=0., uav_latency=.4,
                            uav_frame_interval=lambda team: 1.)
    server = SimServer(world, robot)
    server.uav_stamp_offset = 50.
    server.uav_image_only = True
    return world, server


def deliver(world, server, t):
    world.t = t
    assert server.get_uav() is None
    world.t = t+.41
    return server.get_uav()


def test_real_image_without_robot_detector_does_not_invent_hidden_robot_detections():
    world, server = setup_server()
    observed = UavFrame(50., boxes=[UavBox(1., 1., .4, .4)], layout_source="gazebo_uav_image")
    server.uav_layout_provider = lambda: observed
    frame = deliver(world, server, 0.)
    assert frame.detections == []
    assert frame.boxes == observed.boxes and frame.layout_stamp == 50.
    world.ugvs[0].pose = Pose2D(2., 3.)
    world.ugvs[1].pose = Pose2D(4., 1.)
    observed.stamp = 51.
    assert deliver(world, server, 1.).detections == []


def test_actual_provider_detections_and_capture_stamp_are_forwarded_with_latency():
    world, server = setup_server()
    observed = UavFrame(49.8, image=np.zeros((2, 2, 3), np.uint8),
                        detections=[UavDet(.7, .8, "RED", "visible-id")],
                        boxes=[UavBox(1., 1., .4, .4)], layout_source="gazebo_uav_image")
    server.uav_layout_provider = lambda: observed
    frame = deliver(world, server, 0.)
    assert frame.detections == observed.detections
    assert frame.detections is not observed.detections
    assert frame.image is observed.image
    assert frame.stamp == 50. and frame.layout_stamp == 49.8
    assert frame.age_s == pytest.approx(.61)
    assert world.t+server.uav_stamp_offset-frame.age_s == pytest.approx(49.8)
    assert frame.interval_s == 1.
    assert server.uav_frames_generated == server.uav_frames_delivered == 1
    # A repeated image cannot create fresh robot evidence at the next cadence.
    duplicate = deliver(world, server, 1.)
    assert duplicate.detections == duplicate.boxes == []
    observed.stamp = 60.  # future / wrong clock
    assert deliver(world, server, 2.).detections == []
    observed.stamp = 49.  # stale / out of order
    assert deliver(world, server, 3.).detections == []
    # The delayed new image is older than the preceding empty packet, but is
    # newer than the last actual image. Its packet sequence must still advance.
    observed.stamp = 52.8
    resumed = deliver(world, server, 4.)
    assert resumed.stamp == 54. and resumed.detections == observed.detections
    assert resumed.layout_stamp == 52.8
    assert world.t+server.uav_stamp_offset-resumed.age_s == pytest.approx(52.8)


def test_missing_provider_and_disabled_layout_never_fall_back_to_world_in_image_mode():
    world, server = setup_server()
    # The fixture has no world.cubes attribute: touching truth geometry fails.
    frame = deliver(world, server, 0.)
    assert frame.detections == frame.boxes == [] and frame.layout_source is None
    server.uav_layout_enabled = False

    def forbidden():
        raise AssertionError("Disabled UAV image source was read")

    server.uav_layout_provider = forbidden
    frame = deliver(world, server, 1.)
    assert frame.detections == frame.boxes == []
    world.uav_frame_interval = lambda team: None
    world.t = 2.
    assert server.get_uav() is None


def test_legacy_default_keeps_synthetic_robot_sensor_contract():
    world, image_server = setup_server()
    server = SimServer(world, image_server.u)
    assert server.uav_image_only is False
    server.uav_layout_provider = lambda: None
    frame = deliver(world, server, 0.)
    assert [(d.x, d.y, d.team) for d in frame.detections] == [(.35, 2.5, "BLUE"), (5.5, 4., "RED")]
    assert frame.boxes == []
