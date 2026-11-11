"""Shared-mesh/atlas orientation and YAML-driven static arena generation."""
import importlib.util
import copy
import math
from pathlib import Path
import shutil
import sys

import numpy as np
from PIL import Image
import pytest
import yaml


PACKAGE = Path(__file__).resolve().parents[1] / "ros2/maicon_arena_gz"


def load_script(name):
    spec = importlib.util.spec_from_file_location("arena_static_test_" + name, PACKAGE / "scripts" / (name + ".py"))
    module = importlib.util.module_from_spec(spec)
    sys.path.insert(0, str(PACKAGE / "scripts"))
    try:
        spec.loader.exec_module(module)
    finally:
        sys.path.pop(0)
    return module


atlas = load_script("make_atlas")


@pytest.mark.parametrize("face", atlas.FACES)
def test_shared_cube_face_uv_is_upright_unmirrored_and_outward(face):
    # Outside camera: viewing direction -normal, up +Z, right = forward x up.
    vertices = np.array(atlas.FACE_VERTICES[face])
    normal = np.array(atlas.NORMALS[face])
    right = np.cross(-normal, (0, 0, 1))
    assert np.dot(vertices[1] - vertices[0], right) > 0
    assert np.dot(vertices[3] - vertices[0], (0, 0, 1)) > 0
    assert np.allclose(np.cross(vertices[1]-vertices[0], vertices[2]-vertices[0]), normal)
    lines = atlas.cube_obj().splitlines()
    uvs = np.array([[float(x) for x in line.split()[1:]] for line in lines if line.startswith("vt ")])
    slot = atlas.FACES.index(face)
    uv = uvs[4*slot:4*slot+4]
    assert (uv[:, 0] > slot/5).all() and (uv[:, 0] < (slot+1)/5).all()
    assert uv[1, 0] > uv[0, 0] and uv[3, 1] > uv[0, 1]
    assert uv[0, 1] < .01 and uv[3, 1] > .99


def test_cube_top_and_bottom_map_only_to_black_slot_and_floor_top_left_is_world_top_left():
    lines = atlas.cube_obj().splitlines()
    uv = np.array([[float(x) for x in line.split()[1:]] for line in lines if line.startswith("vt ")])
    assert (uv[16:, 0] > .8).all()
    lines = atlas.floor_obj().splitlines()
    vertices = np.array([[float(x) for x in line.split()[1:]] for line in lines if line.startswith("v ")])
    tex = np.array([[float(x) for x in line.split()[1:]] for line in lines if line.startswith("vt ")])
    i = np.where((tex == (0, 1)).all(axis=1))[0][0]
    assert np.array_equal(vertices[i], (-.5, .5, 0))


def source_image(root, name, color):
    path = root / "faces" / name
    path.parent.mkdir(parents=True, exist_ok=True)
    image = Image.new("RGB", (16, 16), color)
    image.putpixel((0, 0), (255, 255, 255))
    image.putpixel((15, 0), (0, 0, 0))
    image.save(path, format="PNG")
    return path


def test_atlas_preserves_asymmetric_image_corners_and_deduplicates_effective_pixels(tmp_path):
    source_image(tmp_path, "m3_tank.jpg", (30, 70, 90))
    source_image(tmp_path, "m3_drum.jpg", (80, 20, 30))
    faces = {"N": "m3:tank", "E": "m3:drum", "S": "hidden", "W": "blank"}
    output = tmp_path / "atlases"
    first = atlas.make_atlas(faces, tmp_path, output, tile_px=16)
    equivalent = atlas.make_atlas({**faces, "S": "blank", "W": "hidden"}, tmp_path, output, tile_px=16)
    assert first == equivalent
    with Image.open(first) as image:
        assert image.size == (80, 16)
        assert image.getpixel((0, 0)) == (255, 255, 255)
        assert image.getpixel((15, 0)) == (0, 0, 0)
        assert image.getpixel((0, 15)) == (30, 70, 90)
        assert image.getpixel((32, 8)) == image.getpixel((79, 8)) == (0, 0, 0)
    swapped = atlas.make_atlas({**faces, "N": "m3:drum", "E": "m3:tank"}, tmp_path, output, tile_px=16)
    assert swapped != first
    source_image(tmp_path, "m3_tank.jpg", (99, 88, 77))
    replaced = atlas.make_atlas(faces, tmp_path, output, tile_px=16)
    assert replaced != first


def test_class_variant_stays_inside_package_texture_root(tmp_path):
    source_image(tmp_path, "m3_tank/variant.jpg", (11, 22, 33))
    path = atlas.make_atlas({"N": "m3:tank@variant.jpg"}, tmp_path, tmp_path / "out", 16)
    with Image.open(path) as image:
        assert image.getpixel((5, 5)) == (11, 22, 33)
    with pytest.raises(ValueError):
        atlas.make_atlas({"N": "m3:tank@../../../../outside.jpg"}, tmp_path, tmp_path / "out", 16)


def test_atlas_budget_accepts_standard_and_lite_but_rejects_oversized_texture(tmp_path):
    for size in (256, 512):
        path = atlas.make_atlas({}, tmp_path, tmp_path / "out", size)
        with Image.open(path) as image:
            assert image.size == (size*5, size)
    with pytest.raises(ValueError):
        atlas.make_atlas({}, tmp_path, tmp_path / "out", 513)


@pytest.fixture(scope="module")
def scene_inputs(tmp_path_factory):
    root = tmp_path_factory.mktemp("arena_static") / "maicon_arena_gz"
    # Include cached derived atlases if present; fixtures never modify source assets.
    shutil.copytree(PACKAGE / "media", root / "media")
    gen = load_script("gen_world")
    spec = yaml.safe_load((PACKAGE / "config/arena_spec.yaml").read_text(encoding="utf-8"))
    layout = yaml.safe_load((PACKAGE / "config/layouts/layout_default.yaml").read_text(encoding="utf-8"))
    profile = gen.load_profile("standard", PACKAGE)
    return root, gen, spec, layout, profile


@pytest.fixture(scope="module")
def static_scene(scene_inputs):
    root, gen, spec, layout, profile = scene_inputs
    tree, metadata = gen.generate_world(spec, layout, profile, package_root=root, uav_cam=False)
    return tree.getroot().find("world"), metadata


def numbers(node):
    return [float(v) for v in node.text.split()]


def test_measured_floor_collision_top_and_print_surface_are_exact(static_scene):
    world, _ = static_scene
    floor = world.find("model[@name='floor']/link")
    assert numbers(floor.find("collision/geometry/box/size")) == [6., 4.8, .02]
    assert numbers(floor.find("collision/pose"))[2] == -.01
    assert len(floor.findall("visual")) == 1
    assert numbers(floor.find("visual/geometry/mesh/scale")) == [6., 4.8, 1.]
    assert numbers(floor.find("visual/pose"))[2] == 0.


def test_static_geometry_budget_and_no_extra_lights_mesh_collisions_or_floor_shapes(static_scene):
    world, metadata = static_scene
    assert metadata["counts"] == {"collisions": 28, "visuals": 57, "textures": 22, "lights": 1, "sensors": {}}
    assert not world.findall(".//collision/geometry/mesh")
    assert all(model.findtext("static") == "true" for model in world.findall("model"))
    assert len(world.findall("light")) == 1 and world.find("light").get("type") == "directional"
    assert world.findtext("scene/shadows") == "false" and world.findtext("scene/grid") == "false"
    assert not world.findall(".//particle_emitter")
    for link in world.findall(".//link"):
        names = [child.get("name") for child in link if child.get("name")]
        assert len(names) == len(set(names)), "SDFormat requires unique collision/visual/sensor names within each link"
    plugins = [p.get("name").split("::")[-1] for p in world.findall("plugin")]
    assert plugins == ["Physics", "UserCommands", "SceneBroadcaster"]
    gui_plugins = {p.get("filename") for p in world.findall("gui/plugin")}
    assert {"MinimalScene", "GzSceneManager", "InteractiveViewControl", "CameraTracking"} <= gui_plugins
    assert world.findtext("gui/plugin[@filename='MinimalScene']/ignition-gui/property[@key='state']") == "docked"


def test_fixed_sensor_pad_and_target_receiver_positions_match_supplied_spec(static_scene):
    world, metadata = static_scene
    sensors = world.find("model[@name='arena']/link[@name='sensor_box']")
    centers = sorted(tuple(numbers(node.find("pose"))[:3]) for node in sensors.findall("collision"))
    expected = sorted((x, y, .2) for x in (-2.8, 2.8) for y in (-2.2, -.8, .8, 2.2))
    assert centers == expected
    assert len(sensors.findall("visual")) == 8 + 4 + 12  # boxes, poles, 3-color emissive stacks
    assert len(metadata["targets"]) == 6
    for target in metadata["targets"]:
        assert target["position"][2] == pytest.approx(.337)
        assert target["radius"] == pytest.approx(.0285)
        assert target["normal"] in ([1, 0, 0], [-1, 0, 0])
        assert target["camp"] in ("BLUE", "RED")
    cubes = world.find("model[@name='layout']").findall(".//visual")
    assert len([v for v in cubes if not v.get("name").endswith("_signal")]) == 20


def test_every_render_resource_is_package_local_and_shared_cube_mesh_is_reused(static_scene, scene_inputs):
    world, metadata = static_scene
    root = scene_inputs[0]
    prefix = "model://maicon_arena_gz/"
    for node in [*world.findall(".//uri"), *world.findall(".//albedo_map")]:
        assert node.text.startswith(prefix)
        assert (root / node.text.removeprefix(prefix)).exists()
    meshes = {node.text for node in world.find("model[@name='layout']").findall(".//mesh/uri")}
    assert meshes == {prefix + "models/cube_atlas/meshes/cube.obj"}
    assert not metadata["spawn_slots"]


def test_yaml_changes_move_geometry_without_editing_generator(scene_inputs):
    root, gen, original, layout, profile = scene_inputs
    spec = copy.deepcopy(original)
    spec["floor"]["size"][0] = 6.2
    spec["walls"]["inner_size"][0] = 6.2
    spec["sensor_boxes"]["centers"]["blue"][0][0] = -2.7
    tree, _ = gen.generate_world(spec, layout, profile, package_root=root, uav_cam=False)
    world = tree.getroot().find("world")
    assert numbers(world.find("model[@name='floor']/link/visual/geometry/mesh/scale"))[0] == 6.2
    sensor = world.find("model[@name='arena']/link[@name='sensor_box']/collision[@name='sensor_blue_0']/pose")
    assert numbers(sensor)[0] == -2.7
    assert original["floor"]["size"][0] == 6.


def test_camera_plugins_are_conditional_and_validation_camera_is_explicit(scene_inputs):
    root, gen, spec, layout, profile = scene_inputs
    tree, metadata = gen.generate_world(spec, layout, profile, package_root=root, uav_cam=True, validation_camera=True)
    world = tree.getroot().find("world")
    assert metadata["counts"]["sensors"] == {"camera": 2}
    assert len([p for p in world.findall("plugin") if p.get("name").endswith("::Sensors")]) == 1
    assert not [p for p in world.findall("plugin") if p.get("name").endswith("::Imu")]
    cameras = {s.findtext("topic"): s for s in world.findall(".//sensor")}
    assert set(cameras) == {"/uav/image_raw", "/arena/validation/image_raw"}
    assert cameras["/uav/image_raw"].findtext("update_rate") == "1.0"
    assert cameras["/uav/image_raw"].findtext("camera/image/width") == "1280"
    assert cameras["/arena/validation/image_raw"].findtext("camera/image/width") == "1500"


def test_overhead_camera_optical_mount_maps_image_right_up_to_arena_x_y_and_fits_floor(scene_inputs):
    root, gen, spec, layout, profile = scene_inputs
    source_pose = list(spec["uav_camera"]["pose"])
    tree, _ = gen.generate_world(spec, layout, profile, package_root=root, validation_camera=True)
    for name in ("uav_camera", "validation_camera"):
        model = tree.find(f".//model[@name='{name}']")
        x, y, z, roll, pitch, yaw = numbers(model.find("pose"))
        cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
        rotation = np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                             [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                             [-sp, cp*sr, cp*cr]])
        forward, right, up = rotation[:, 0], -rotation[:, 1], rotation[:, 2]
        assert np.allclose(forward, (0, 0, -1), atol=1e-7)
        assert np.allclose(right, (1, 0, 0), atol=1e-7)
        assert np.allclose(up, (0, 1, 0), atol=1e-7)
        camera = model.find("link/sensor/camera")
        width, height = int(camera.findtext("image/width")), int(camera.findtext("image/height"))
        focal = width / (2 * math.tan(float(camera.findtext("horizontal_fov")) / 2))
        for corner_x in (-spec["floor"]["size"][0]/2, spec["floor"]["size"][0]/2):
            for corner_y in (-spec["floor"]["size"][1]/2, spec["floor"]["size"][1]/2):
                relative = np.array([corner_x-x, corner_y-y, -z])
                u = width/2 + focal * np.dot(relative, right)/np.dot(relative, forward)
                v = height/2 - focal * np.dot(relative, up)/np.dot(relative, forward)
                assert 0 < u < width and 0 < v < height
        if name == "uav_camera":
            # The floor fits at123deg, but the elevated wall/corner-box tops did
            # not fit the actual720p frame. Check the complete raised enclosure.
            for corner_x in (-spec["walls"]["inner_size"][0]/2-spec["walls"]["thickness"],
                              spec["walls"]["inner_size"][0]/2+spec["walls"]["thickness"]):
                for corner_y in (-spec["walls"]["inner_size"][1]/2-spec["walls"]["thickness"],
                                  spec["walls"]["inner_size"][1]/2+spec["walls"]["thickness"]):
                    relative = np.array([corner_x-x, corner_y-y, spec["walls"]["height"]-z])
                    u = width/2 + focal * np.dot(relative, right)/np.dot(relative, forward)
                    v = height/2 - focal * np.dot(relative, up)/np.dot(relative, forward)
                    assert 0 < u < width and 0 < v < height
    assert spec["uav_camera"]["pose"] == source_pose


def test_generation_rejects_hidden_target_picture_before_writing_assets(scene_inputs, tmp_path):
    _, gen, spec, original_layout, profile = scene_inputs
    layout = copy.deepcopy(original_layout)
    # Every default L cluster has touching side faces. Altering one hidden label
    # must be rejected instead of silently masking an impossible photo target.
    altered = False
    for cube in layout["boxes"]:
        for face, label in cube["faces"].items():
            if label == "hidden":
                cube["faces"][face] = "m3:tank"
                altered = True
                break
        if altered:
            break
    assert altered
    output_root = tmp_path / "unwritten"
    with pytest.raises(ValueError, match="nonblank image on a hidden face"):
        gen.generate_world(spec, layout, profile, package_root=output_root, uav_cam=False)
    assert not output_root.exists()


@pytest.mark.parametrize("engine,enabled,expected", [("ogre2", True, False), ("ogre", True, True), ("ogre", False, False)])
def test_opaque_texture_workaround_is_optional_and_only_for_ogre_one(scene_inputs, engine, enabled, expected):
    root, gen, spec, layout, profile = scene_inputs
    tree, metadata = gen.generate_world(spec, layout, profile, package_root=root, render_engine=engine,
                                        opaque_material_compat=enabled, uav_cam=True)
    plugins = tree.findall(".//world/plugin[@name='maicon::OpaqueMaterialSystem']")
    assert len(plugins) == int(expected)
    assert metadata["opaque_material_compat"] is expected
    if plugins:
        assert plugins[0].get("filename") == "maicon_opaque_material_system"
    assert metadata["counts"]["visuals"] == 57
    assert metadata["counts"]["textures"] == 22


def test_cpu_ogre_two_sensor_world_can_select_separate_gpu_ogre_one_gui(scene_inputs):
    root, gen, spec, layout, profile = scene_inputs
    tree, metadata = gen.generate_world(spec, layout, profile, package_root=root, render_engine="ogre2",
                                        gui_render_engine="ogre", uav_cam=True)
    assert metadata["render_engine"] == "ogre2" and metadata["gui_render_engine"] == "ogre"
    assert not tree.findall(".//world/plugin[@name='maicon::OpaqueMaterialSystem']")
    assert tree.findtext(".//world/plugin[@name='ignition::gazebo::systems::Sensors']/render_engine") == "ogre2"
    assert tree.findtext(".//gui/plugin[@filename='MinimalScene']/engine") == "ogre"
    assert len(tree.findall(".//gui/plugin[@filename='MaiconOpaqueMaterialGui']")) == 1
    unchanged, _ = gen.generate_world(spec, layout, profile, package_root=root, render_engine="ogre2")
    assert not unchanged.findall(".//gui/plugin[@filename='MaiconOpaqueMaterialGui']")


def test_lite_profile_uses_half_size_atlases_and_reduced_uav_image(scene_inputs):
    root, gen, spec, layout, _ = scene_inputs
    profile = gen.load_profile("lite", PACKAGE)
    tree, metadata = gen.generate_world(spec, layout, profile, package_root=root, uav_cam=True)
    world = tree.getroot().find("world")
    for cube in metadata["cubes"]:
        with Image.open(root / cube["atlas"]) as image:
            assert image.size == (1280, 256)
    floor = world.find("model[@name='floor']/link/visual/material/pbr/metal/albedo_map").text
    assert floor.endswith("floor_print_1200x960_lite.png")
    sensor = world.find("model[@name='uav_camera']/link/sensor")
    assert sensor.findtext("camera/image/width") == "960"
    assert not world.find("model[@name='validation_camera']")
