"""The fixed Gazebo field and mission referee must describe the same arena."""

from dataclasses import replace
from types import SimpleNamespace

from maicon26.config import Config
from maicon26.mission.manager import Agent
from maicon26.link.base import UavDet, UavFrame
from maicon26.perception.tracks import UgvTracker
from scripts.generate_gazebo_world import generate
from maicon26.sim.fixed_maicon import FIXED_STAGE_POINT, STRUCTURES
from maicon26.sim.robot import SimServer
from maicon26.sim.world import World
from scripts.run_gazebo_match import parse_uav_schedule
from maicon26.types import Pose2D


def test_fixed_field_has_one_robot_and_shared_mission_layout():
    sdf, world = generate(seed=2, robot="B2", layout="fixed")
    scene = sdf.find("world")
    assert scene is not None

    models = {model.get("name"): model for model in scene.findall("model")}
    assert "wingbot_ugv" in models
    uav = models["fixed_uav"]
    assert uav.findtext("pose").split()[2] == "2.5000"
    camera = uav.find("link/sensor[@name='arena_camera']")
    assert camera.get("type") == "camera"
    assert camera.findtext("topic") == "/uav/image"
    assert camera.findtext("update_rate") == "1"
    assert (camera.findtext("camera/image/width"), camera.findtext("camera/image/height")) == ("1280", "720")
    assert not any(name.endswith("_ugv") and name != "wingbot_ugv" for name in models)
    assert models["wingbot_ugv"].findtext("link/sensor[@name='laser']/ray/scan/horizontal/samples") == "360"
    drive = models["wingbot_ugv"].find("plugin[@name='ignition::gazebo::systems::DiffDrive']")
    assert {joint.text for joint in drive.findall("left_joint")} == {
        "wheel_front_left_joint", "wheel_rear_left_joint"}
    assert {joint.text for joint in drive.findall("right_joint")} == {
        "wheel_front_right_joint", "wheel_rear_right_joint"}
    assert len(world.cubes) == len(STRUCTURES) + 4
    assert world.layout_meta["m3_face_count"] == 16
    sx, sy = FIXED_STAGE_POINT
    assert all(not (x1 - .18 <= sx <= x2 + .18 and y1 - .18 <= sy <= y2 + .18)
               for x1, y1, x2, y2 in (cube.rect() for cube in world.cubes))
    for index, structure in enumerate(STRUCTURES):
        cube = world.cubes[index]
        assert (cube.cx, cube.cy, cube.sx, cube.sy) == (
            structure.x, structure.y, structure.sx, structure.sy)
        model = models[f"{('target' if any(i.kind == 'target' for i in cube.items.values()) else 'object')}_{index:02d}"]
        assert tuple(map(float, model.findtext("pose").split()[:2])) == (cube.cx, cube.cy)
        assert tuple(map(float, model.findtext("link/collision/geometry/box/size").split()[:2])) == (cube.sx, cube.sy)

    floor = models["floor"]
    assert floor.findtext("link/visual[@name='field_markings']/geometry/mesh/uri").endswith("field_markings.obj")
    assert len(scene.findall(".//visual")) < 200
    for index, cube in enumerate(world.cubes[len(STRUCTURES):], len(STRUCTURES)):
        model = models[f"object_{index:02d}"]
        assert tuple(map(float, model.findtext("link/collision/geometry/box/size").split())) == (.4, .4, .4)
        for face, item in cube.items.items():
            mesh = model.findtext(f"link/visual[@name='object_{face}']/geometry/mesh/uri")
            assert mesh.endswith(f"m3_photos/{item.cls}.obj")

    b2 = world.add_ugv("BLUE", 2)
    mission = SimServer(world, b2).truth
    assert mission["target"] == "COMM_FACILITY"
    assert mission["m5_base"] == "COMMAND_POST"
    assert world.base_types["R-1"] == mission["m5_base"]


def test_fixed_goal_entry_creep_is_limited_and_scanned():
    cfg = Config()
    cfg.arena = replace(cfg.arena, base_depth=.4, base_height=1.2)
    cfg.mission.goal_entry_creep_s = 3.0
    calls = []
    agent = Agent.__new__(Agent)
    agent.cfg = cfg
    agent.geo = cfg.arena
    agent.team = "BLUE"
    agent.m5_base = "R-1"
    agent.m5_entered_local = None
    agent.stack = SimpleNamespace(pose=Pose2D(5.82, 4.13, 0.0))
    agent.laser = SimpleNamespace(firing=lambda: False)
    agent._log = lambda *_: None
    agent._apply = lambda *args, **kwargs: calls.append((args, kwargs))

    Agent._mode_goal(agent, 10.0, .05, 45.0)
    assert calls[-1][0][0] == .08 and calls[-1][1]["limit"]
    Agent._mode_goal(agent, 13.1, .05, 48.1)
    assert calls[-1][0][0] == 0.0


def test_uav_feed_delays_frames_and_slows_on_opposing_target_hits():
    world = World(seed=2, scenario="maicon_fixed")
    b2 = world.add_ugv("BLUE", 2)
    red = world.add_ugv("RED", 1)
    red.pose = Pose2D(5.25, world.geo.base_center("R-1")[1], 3.14159)
    server = SimServer(world, b2)
    server.uav_stamp_offset = 50.0
    assert server.get_uav() is None
    world.t = .4
    frame = server.get_uav()
    assert frame is not None and frame.stamp == 50.0 and frame.interval_s == 1.0
    tracker = UgvTracker(world.cfg, "BLUE")
    tracker.update_uav(frame, Pose2D(.35, 2.5, 0))
    assert any(t.ident == "R-UGV-1" for t in tracker.tracks)
    assert server.get_uav() is None
    world.events.append({"type": "target_hit", "by": "B2"})
    assert world.uav_frame_interval("BLUE") == 1.0
    world.events.append({"type": "target_hit", "by": "R1"})
    assert world.uav_frame_interval("BLUE") == 2.0
    world.t = 2.1
    assert server.get_uav() is None
    world.t = 2.51
    assert server.get_uav().interval_s == 2.0
    world.events.append({"type": "target_hit", "by": "R2"})
    assert world.uav_frame_interval("BLUE") == 5.0
    world.events.append({"type": "target_hit", "by": "R3"})
    assert world.uav_frame_interval("BLUE") is None
    assert server.get_uav() is None
    assert parse_uav_schedule("90:2,160:5,220:off") == [(90.0, 2.0), (160.0, 5.0), (220.0, float("inf"))]


def test_uav_start_identification_uses_elapsed_time_not_absolute_clock():
    cfg = Config()
    tracker = UgvTracker(cfg, "BLUE")
    x, y = cfg.arena.base_center("R-1")
    tracker.update_uav(UavFrame(stamp=50.0, detections=[UavDet(x, y, "RED")]),
                       Pose2D(.35, 2.5, 0))
    assert tracker.tracks[0].ident == "R-UGV-1"
    assert not tracker._ident_done
    tracker.update_uav(UavFrame(stamp=53.1), Pose2D(.35, 2.5, 0))
    assert tracker._ident_done
