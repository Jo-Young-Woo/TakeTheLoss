"""UAV may guide ground inspection, but must not supply hidden M3 labels."""
from dataclasses import fields

from maicon26.link.base import UavBox, UavFrame
from maicon26.sim.robot import SimServer
from maicon26.sim.world import World


def setup_server():
    world = World(seed=2, scenario="maicon_fixed")
    server = SimServer(world, world.add_ugv("BLUE", 2))
    server.uav_stamp_offset = 50.0
    return world, server


def deliver(world, server, t):
    world.t = t
    server.get_uav()
    world.t = t + .41
    return server.get_uav()


def test_synthetic_hints_are_geometry_only_and_not_object_truth():
    world, server = setup_server()
    frame = deliver(world, server, 0.0)
    assert frame.layout_source == "synthetic_overhead_footprints"
    assert len(frame.boxes) == len(world.cubes)
    assert {f.name for f in fields(UavBox)} == {"cx", "cy", "sx", "sy", "confidence"}
    assert all(.39 <= b.sx <= .81 and .39 <= b.sy <= .81 for b in frame.boxes)
    assert world.submissions == {}


def test_gazebo_provider_has_no_hidden_geometry_fallback():
    world, server = setup_server()
    server.uav_layout_provider = lambda: None
    frame = deliver(world, server, 0.0)
    assert frame.boxes == [] and frame.layout_source is None
    assert server.uav_layout_frames == 0


def test_image_hints_are_new_recent_and_gated_by_uav_schedule():
    world, server = setup_server()
    observed = UavFrame(50.0, boxes=[UavBox(1.0, 1.0, .4, .4)],
                        layout_source="gazebo_uav_image")
    server.uav_layout_provider = lambda: observed
    frame = deliver(world, server, 0.0)
    assert frame.boxes == observed.boxes
    assert frame.layout_source == "gazebo_uav_image"
    assert frame.layout_stamp == observed.stamp
    assert deliver(world, server, 1.0).boxes == []  # duplicate camera frame
    observed.stamp = 48.0
    assert deliver(world, server, 2.0).boxes == []  # stale camera frame
    observed.stamp = 60.0
    assert deliver(world, server, 3.0).boxes == []  # wrong clock / future frame
    observed.stamp = 54.0
    assert deliver(world, server, 4.0).boxes
    world.set_uav_period(float("inf"))
    observed.stamp = 55.0
    world.t = 5.0
    assert server.get_uav() is None
    assert server.uav_layout_frames == 2


def test_lidar_only_control_does_not_read_layout_provider():
    world, server = setup_server()
    server.uav_layout_enabled = False

    def unexpected():
        raise AssertionError("Disabled image layout must not be consumed")

    server.uav_layout_provider = unexpected
    frame = deliver(world, server, 0.0)
    assert not frame.boxes


def test_generic_random_scenarios_keep_their_existing_sensor_contract():
    world = World(seed=2)
    server = SimServer(world, world.add_ugv("BLUE", 2))
    assert not server.uav_layout_enabled
    assert deliver(world, server, 0.0).boxes == []
    server.uav_layout_enabled = True
    frame = deliver(world, server, 1.0)
    assert frame.boxes and frame.layout_source == "synthetic_overhead_footprints"
