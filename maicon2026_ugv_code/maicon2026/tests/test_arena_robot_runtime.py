"""Explicit legacy primitive model contracts; no Gazebo or ROS required."""
from pathlib import Path
import importlib.util
import math
import xml.etree.ElementTree as ET

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[1]/"ros2/maicon_arena_gz"


def module(name):
    spec = importlib.util.spec_from_file_location(name, ROOT/"scripts"/(name+".py"))
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


def inputs(profile="standard"):
    return tuple(yaml.safe_load(path.read_text(encoding="utf-8-sig")) for path in (
        ROOT/"config/arena_spec.yaml", ROOT/"config/robot_assumptions.yaml", ROOT/"config/profiles"/(profile+".yaml")))


@pytest.mark.parametrize("profile,wheels", [("standard", 6), ("lite", 2), ("high", 6)])
def test_one_ego_has_real_sensors_and_five_dummies_have_one_box_only(profile, wheels):
    spec, _, config = inputs(profile)
    world = ET.Element("world", name="maicon_arena")
    def add_system(parent, key, simulator):
        return ET.SubElement(parent, "plugin", name=key)
    result = module("robot_models").add_ugvs(world, spec, config, 6, "B-2", system_adder=add_system, robot_model="legacy")
    assert result["sensors"] == {"camera": 1, "gpu_lidar": 1, "imu": 1}
    assert result["ego_model"] == "ugv_B2" and len(result["robot_models"]) == 6
    for model in world.findall("model"):
        slot = next(slot for slot, name in result["slots"].items() if name == model.get("name"))
        pose = list(map(float, model.findtext("pose").split()))
        assert pose[:2] == spec["bases"]["start_pose"][slot][:2] and pose[2] == 0
        assert sum(float(node.text) for node in model.findall("link/inertial/mass")) == pytest.approx(spec["ugv"]["mass"])
        for link in model.findall("link"):
            names = [child.get("name") for child in link if child.tag in ("collision", "visual", "sensor")]
            assert len(names) == len(set(names)), "SDFormat named link children must be unique"
        if slot != "B-2":
            assert not model.findall(".//sensor") and not model.findall("joint")
            assert len(model.findall(".//collision")) == 1
            assert model.find("plugin[@name='VelocityControl']") is not None
        else:
            drive = model.find("plugin[@name='DiffDrive']")
            assert len(drive.findall("left_joint"))+len(drive.findall("right_joint")) == wheels
            assert model.find(".//sensor[@type='gpu_lidar']/lidar/scan/horizontal/samples").text == str(config["ugv"].get("lidar_samples", spec["ugv"]["lidar"]["sim_samples_assumed"]))
            assert model.find("plugin[@name='JointStatePublisher']") is not None
            assert model.find("frame[@name='laser_frame_emitter']") is not None


def core():
    spec, settings, _ = inputs()
    result = module("mil_emulator").EmulatorCore(spec, settings, "B-2")
    result.receive_state({"phase": "running", "paused": False, "ugvs": {"B-2": {
        "hp": 4, "ammo": 8, "lock_until": 0., "drive_lock_until": 0., "reload_until": 0.}}}, 10.)
    return result


def test_velocity_limits_watchdog_phase_and_stale_referee_all_fail_closed():
    c = core()
    c.velocity(100., -100., 10.)
    assert c.output_velocity(10.) == (.6, -3.)
    c.receive_state(c.state, 10.6)
    assert c.output_velocity(10.6) == (0., 0.), "Fresh referee must not hide expired command"
    c.velocity(.2, .1, 11.)
    assert c.output_velocity(11.2) == (0., 0.), "Fresh command must not hide stale referee"
    c.receive_state({**c.state, "paused": True}, 11.2)
    assert c.output_velocity(11.2) == (0., 0.)
    c.receive_state({**c.state, "paused": False, "phase": "reporting"}, 11.2)
    assert c.output_velocity(11.2) == (0., 0.)
    assert c.match_command() == "report"


def test_gimbal_api_right_positive_and_tilt_up_positive_with_speed_and_hit_lock():
    c = core()
    assert c.gimbal(100., 100., 10., 20., 10.) == "ok"
    assert c.target == pytest.approx((-math.radians(50), math.radians(15)))
    assert c.output_gimbal(10.1, .1) == pytest.approx((-math.radians(1), math.radians(1)))
    c.state["ugvs"]["B-2"]["lock_until"] = 20.
    assert c.gimbal(0., 0., 10., 20., 10.1) == "locked"
    assert c.output_gimbal(10.1, .1) == (c.pan, c.tilt)


def test_initial_locked_gimbal_keeps_zero_command_despite_gravity_sag():
    spec, settings, _ = inputs()
    c = module("mil_emulator").EmulatorCore(spec, settings, "B-2")
    c.receive_state({"phase": "ready", "ugvs": {"B-2": {"lock_until": 0.}}}, 10.)
    for now, measured_tilt in ((10., -.01), (10.1, -.04), (10.2, -.10)):
        c.pan, c.tilt, c.joint_t = .002, measured_tilt, now
        assert c.output_gimbal(now, .1) == (0., 0.)
        assert c.target == (0., 0.), "READY feedback must not ratchet the zero hold downward"


@pytest.mark.parametrize("lock_reason", ["hit", "pause", "reporting", "stale_referee"])
def test_gimbal_freezes_measured_pose_once_per_unlocked_to_locked_transition(lock_reason):
    c = core()
    assert c.gimbal(40., 40., 100., 20., 10.) == "ok"
    assert c.output_gimbal(10., .1) != (0., 0.)
    c.pan, c.tilt = -.08, .04
    locked_at = 10.1
    if lock_reason == "hit":
        c.state["ugvs"]["B-2"]["lock_until"] = 30.
    elif lock_reason == "pause":
        c.state["paused"] = True
    elif lock_reason == "reporting":
        c.state["phase"] = "reporting"
    else:
        locked_at = 10. + c.settings["emulator"]["stale_referee_s"] + .1
    assert c.output_gimbal(locked_at, .1) == (-.08, .04)
    assert c.target == (-.08, .04), "Capture actual feedback, not the previous motion target"
    for index in range(1, 4):
        c.pan, c.tilt = -.08 + .002 * index, .04 - .01 * index
        assert c.output_gimbal(locked_at + .01 * index, .01) == (-.08, .04)
        assert c.target == (-.08, .04)
    # After a genuine unlock, a later lock must capture a new measured pose.
    c.receive_state({"phase": "running", "paused": False,
                     "ugvs": {"B-2": {"lock_until": 0.}}}, 40.)
    assert c.gimbal(-20., 20., 100., 20., 40.) == "ok"
    c.output_gimbal(40., .1)
    c.pan, c.tilt = .03, .02
    c.state["paused"] = True
    assert c.output_gimbal(40.1, .1) == (.03, .02)
    c.pan, c.tilt = .03, -.02
    assert c.output_gimbal(40.2, .1) == (.03, .02)


def test_fire_requires_actual_joint_feedback_and_authoritative_ammo_reload():
    c = core()
    assert c.fire(10.) == ("locked", 0)
    c.joint_t = 10.
    c.state["ugvs"]["B-2"]["reload_until"] = 12.
    assert c.fire(10.) == ("reloading", 2000)
    c.state["ugvs"]["B-2"]["reload_until"] = 0.
    assert c.fire(10.) == ("fired", 10000)
    assert c.fire(10.1)[0] == "reloading", "Prevent two requests before referee reply"
    assert c.own()["ammo"] == 8, "Only the referee consumes ammo"
    c.state["ugvs"]["B-2"]["ammo"] = 0
    assert c.fire(10.1) == ("empty", 0)


def test_invalid_velocity_and_gimbal_inputs_do_not_create_nonfinite_commands():
    c = core()
    c.velocity(math.nan, .1, 10.)
    assert c.output_velocity(10.) == (0., 0.)
    assert c.gimbal(math.nan, 0., 20., 20., 10.) == "locked"


def test_actual_ros_transform_accepts_integer_yaml_zero_coordinates():
    messages = pytest.importorskip("geometry_msgs.msg")
    builtin = pytest.importorskip("builtin_interfaces.msg")
    transform = module("mil_emulator").measured_transform("base_link", "pan_link", [0, 0, .1],
                                                          [0, 0, 0], builtin.Time(sec=2))
    assert isinstance(transform, messages.TransformStamped)
    assert transform.transform.translation.y == 0. and transform.transform.rotation.w == 1.
    assert transform.header.stamp.sec == 2
