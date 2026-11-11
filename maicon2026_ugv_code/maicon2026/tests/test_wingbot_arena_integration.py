"""Model authority → emulator/referee wiring, independent of mission strategy."""
import copy
import ast
import inspect
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
ARENA = ROOT/"ros2/maicon_arena_gz"
WINGBOT = ROOT/"ros2/wingbot_description"
sys.path.insert(0, str(ARENA/"scripts"))
sys.path.insert(0, str(WINGBOT/"scripts"))
from arena_referee import ArenaReferee, ray_hemisphere  # noqa: E402
from mil_emulator import EmulatorCore  # noqa: E402
from wingbot_runtime import WingbotFrames, crop_scan  # noqa: E402
import robot_models  # noqa: E402


@pytest.fixture
def frames():
    return WingbotFrames(WINGBOT/"config/frames.yaml")


def load(path):
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def test_laser_camera_offset_remains_a_rigid_vector_under_both_actual_joints(frames):
    mods = load(WINGBOT/"config/wingbot_mods.yaml")
    expected = mods["laser_emitter"]["boresight_offset_from_camera"]
    for pan, tilt in ((0., 0.), (.3, .2), (-.5, -.15)):
        transform = frames.transforms(pan, tilt)
        laser, camera = (transform[frames.semantic[k]] for k in ("laser", "camera"))
        assert camera[:3, :3].T @ (laser[:3, 3]-camera[:3, 3]) == pytest.approx(expected, abs=1e-8)
        origin, direction = frames.laser(pan, tilt)
        assert origin == pytest.approx(laser[:3, 3])
        assert direction == pytest.approx(laser[:3, 0])
    _, right_up = frames.laser(.3, .2)
    assert right_up[1] < 0 and right_up[2] > 0


def test_installed_root_frames_export_uses_same_source_and_kinematics(frames):
    root_export = WingbotFrames(WINGBOT/"frames.yaml")
    assert root_export.laser(.3,.2) == frames.laser(.3,.2)


def test_api_positive_pan_commands_positive_right_rotating_urdf_joint(frames):
    spec = load(ARENA/"config/arena_spec.yaml")
    core = EmulatorCore(spec, load(ARENA/"config/wingbot_engineering.yaml"), "B-2", frames)
    core.receive_state({"phase": "running", "ugvs": {"B-2": {
        "lock_until": 0., "drive_lock_until": 0., "ammo": 8}}}, 10.)
    assert core.gimbal(100., 100., 270., 20., 10.) == "ok"
    assert core.target == pytest.approx((math.radians(50), math.radians(15)))
    assert frames.laser(*core.target)[1][1] < 0
    assert core.fire(10.) == ("locked", 0), "Commands never impersonate measured joint feedback"
    core.pan, core.tilt, core.joint_t = .3, .1, 10.
    assert core.fire(10.)[0] == "fired"


def test_lidar_clockwise_crop_and_preserved_description_yaw_remove_rear_only(frames):
    values, masked = crop_scan([2.]*360, 0., math.pi/180, frames.semantic["lidar"], frames.data)
    assert masked == list(range(45, 136))
    assert all(math.isnan(values[index]) for index in masked)
    assert all(values[index] == 2. for index in set(range(360))-set(masked))
    lidar = frames.zero[frames.semantic["lidar"]]
    for index in (45, 90, 135):
        angle = math.radians(index)
        direction = lidar[:3, :3] @ [math.cos(angle), math.sin(angle), 0.]
        assert direction[0] < -.70, "The masked wedge must face the rear in base_footprint"
    assert frames.zero["base_lidar_link"][2, 3] == pytest.approx(.11, abs=.001), "Preserve lowered mount; sensor plane is a separate frame"
    assert not frames.data["lidar"]["physical_yaw_verified"], "Source audit is not a hardware measurement"


def test_crop_preserves_unobserved_and_no_return_values_and_checks_frame(frames):
    original = [math.nan, math.inf, 3., 4., 5.]
    values, masked = crop_scan(original, math.pi, math.pi/180, frames.semantic["lidar"], frames.data)
    assert not masked and math.isnan(values[0]) and values[1] == math.inf and values[2:] == original[2:]
    assert original[2] == 3.
    with pytest.raises(ValueError, match="different frame"):
        crop_scan([1.], 0., .1, "base_link", frames.data)
    unverified = copy.deepcopy(frames.data)
    unverified["lidar"]["crop_audited"] = False
    with pytest.raises(ValueError, match="not been audited"):
        crop_scan([1.], 0., .1, frames.semantic["lidar"], unverified)


def test_negative_angle_scan_crops_same_physical_rays(frames):
    values, _ = crop_scan([1.]*450, -math.pi, 2*math.pi/450, frames.semantic["lidar"], frames.data)
    for index, value in enumerate(values):
        degrees = (-180+index*.8) % 360
        assert math.isnan(value) == (45 <= degrees <= 135)


def test_receiver_domes_keep_side_center_and_outward_hemisphere(frames):
    mods = load(WINGBOT/"config/wingbot_mods.yaml")
    for (center, radius, facing), side in zip(frames.receivers(), ("left", "right")):
        assert center == pytest.approx(mods["receivers"][side]["center"], abs=1e-9)
        assert radius == mods["receivers"]["dome_radius"]
        outward = tuple(c+f for c, f in zip(center, facing))
        inward = tuple(c-f for c, f in zip(center, facing))
        assert ray_hemisphere(outward, tuple(-f for f in facing), center, radius, facing) == pytest.approx(1-radius)
        assert ray_hemisphere(inward, facing, center, radius, facing) is None


@pytest.mark.parametrize("pan,tilt", [(0., 0.), (.3, .2), (-.4, -.1)])
def test_referee_ray_uses_laser_frame_not_camera_frame(frames, pan, tilt):
    spec = load(ARENA/"config/arena_spec.yaml")
    referee = ArenaReferee(spec, {"boxes": []}, load(ARENA/"config/simulation_assumptions.yaml"), frames=frames)
    referee.static_bounds = []
    origin, direction = frames.laser(pan, tilt)
    point = [o+d for o, d in zip(origin, direction)]
    referee.targets = [{"camp": "RED", "kind": "comms", "face": "W", "receiver": point, "diameter": .01}]
    referee.poses = {"B-2": (0., 0., 0., 0.)}
    referee.gimbals["B-2"] = pan, tilt
    assert referee._first_receiver("B-2") == ("target", 0)
    camera = frames.transforms(pan, tilt)[frames.semantic["camera"]]
    referee.targets[0]["receiver"] = [c+d for c, d in zip(camera[:3, 3], direction)]
    assert referee._first_receiver("B-2") is None


def test_actual_tf_edges_reconstruct_all_frames_without_legacy_mounts(frames):
    from wingbot_geometry import transform
    expected = frames.transforms(.3, .2)
    rebuilt = {frames.data["root"]: np.eye(4)}
    pending = list(frames.tf_edges(.3, .2))
    while pending:
        rest = []
        for parent, child, xyz, rpy in pending:
            if parent in rebuilt:
                rebuilt[child] = rebuilt[parent] @ transform(xyz, rpy)
            else:
                rest.append((parent, child, xyz, rpy))
        assert len(rest) < len(pending)
        pending = rest
    for name, matrix in expected.items():
        assert rebuilt[name] == pytest.approx(matrix, abs=1e-8)


@pytest.mark.parametrize("explicit",[False,True])
def test_ego_and_five_dummies_use_one_xacro_with_sensor_modes(monkeypatch,explicit):
    requests = []
    def instantiate(name, team, namespace, sensors, wheels, simulator, effective_separation):
        requests.append((name, team, namespace, sensors, wheels))
        model = ET.Element("model", name=name)
        link = ET.SubElement(model, "link", name="base_link")
        ET.SubElement(model, "joint", name="pt_base_link_to_pt_link1")
        if sensors == "full":
            for kind, topic in (("camera", "/camera/image"), ("gpu_lidar", "/scan"), ("imu", "/imu")):
                node = ET.SubElement(link, "sensor", type=kind)
                ET.SubElement(node, "topic").text = namespace+topic
        return model
    monkeypatch.setattr(robot_models, "instantiate_wingbot", instantiate)
    monkeypatch.setattr(robot_models, "wingbot_package", lambda: WINGBOT)
    world = ET.Element("world")
    metadata = robot_models.add_ugvs(world, load(ARENA/"config/arena_spec.yaml"),
        load(ARENA/"config/profiles/standard.yaml"), 6, "B-2", **({"robot_model":"wingbot"} if explicit else {}))
    assert [request[3] for request in requests] == ["full"]+["none"]*5
    assert [request[1] for request in requests].count("BLUE") == 3
    assert all(request[4] == 6 for request in requests)
    assert all(model.find("joint[@name='pt_base_link_to_pt_link1']") is not None for model in world)
    assert metadata["sensors"] == {"camera": 1, "gpu_lidar": 1, "imu": 1}
    assert metadata["model_authority"] == "wingbot_description"
    assert {node.text for node in world.findall(".//sensor/topic")} == {"/arena/ego/image", "/arena/ego/scan_raw", "/imu/data"}


def test_harmonic_wingbot_is_rejected_before_conversion_or_asset_writes(monkeypatch,tmp_path):
    def forbidden(*args,**kwargs):
        pytest.fail("An unsupported WINGBOT backend must not start xacro or Gazebo")
    monkeypatch.setattr(robot_models.subprocess,"run",forbidden)
    with pytest.raises(ValueError,match="Fortress only"):
        robot_models.instantiate_wingbot("test","BLUE","/test","full",6,"harmonic")
    with pytest.raises(ValueError,match="Fortress only"):
        robot_models.add_wingbots(ET.Element("world"),{}, {},6,"B-2","harmonic")
    import gen_world
    output=tmp_path/"must_not_exist"
    with pytest.raises(ValueError,match="Fortress only"):
        gen_world.generate_world({}, {}, {},package_root=output,robot_model="wingbot",simulator="harmonic")
    assert not output.exists()


def test_public_generation_defaults_to_wingbot_and_normal_launch_to_one_ego(monkeypatch,capsys):
    import gen_world
    observed=[]
    def write(*args,**kwargs):
        observed.append(kwargs)
        return {"world":"maicon_arena","counts":{}}
    monkeypatch.setattr(gen_world,"write_world",write)
    gen_world.main(["--ugvs","1"])
    assert observed[0]["robot_model"] == "wingbot"
    assert inspect.signature(gen_world.generate_world).parameters["robot_model"].default == "wingbot"
    assert inspect.signature(robot_models.add_ugvs).parameters["robot_model"].default == "wingbot"
    tree=ast.parse((ARENA/"launch/arena.launch.py").read_text(encoding="utf-8"))
    launch=next(node for node in tree.body if isinstance(node,ast.FunctionDef) and node.name=="generate_launch_description")
    assignment=next(node for node in launch.body if isinstance(node,ast.Assign)
                    and any(isinstance(target,ast.Name) and target.id=="defaults" for target in node.targets))
    defaults=ast.literal_eval(assignment.value)
    assert defaults["robot_model"] == "wingbot" and defaults["ugvs"] == "1"
    assert '"world": "maicon_arena"' in capsys.readouterr().out


def test_wingbot_ogre_one_is_rejected_before_asset_or_model_generation(monkeypatch,tmp_path):
    import gen_world
    def forbidden(*args,**kwargs):
        pytest.fail("Unsupported sensor rendering must not instantiate a model")
    monkeypatch.setattr(robot_models,"add_wingbots",forbidden)
    with pytest.raises(ValueError,match="Ogre2"):
        robot_models.add_ugvs(ET.Element("world"),{}, {},1,"B-2",render_engine="ogre")
    with pytest.raises(ValueError,match="Ogre2"):
        gen_world.generate_world({}, {}, {},package_root=tmp_path/"unwritten",ugvs=1,render_engine="ogre")
    assert not (tmp_path/"unwritten").exists()
