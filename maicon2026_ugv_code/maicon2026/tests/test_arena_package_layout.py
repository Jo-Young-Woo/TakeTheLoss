"""SSOT layout constraints and independent generated-world acceptance tests."""
from copy import deepcopy
from pathlib import Path
import shutil
import sys
import xml.etree.ElementTree as ET

import pytest

PACKAGE = Path(__file__).resolve().parents[1] / "ros2/maicon_arena_gz"
sys.path.insert(0, str(PACKAGE / "scripts"))

from arena_common import (FACE_DIRS, accessible_faces, cube_extent, expand_boxes,
                          fixed_geometry, load_layout, load_spec, merged_collisions,
                          parse_face, recompute_truth, sector_of, target_metadata,
                          validate_layout)
from check_world import check_world, resolve_resource
from gen_layout import generate_layout, main as generate_main
from overlay import world_to_pixel


@pytest.fixture(scope="module")
def spec():
    return load_spec(PACKAGE / "config/arena_spec.yaml")


@pytest.fixture(scope="module")
def default():
    return load_layout(PACKAGE / "config/layouts/layout_default.yaml")


def test_supplied_default_is_valid_and_all_image_faces_accessible(spec, default):
    report = validate_layout(spec, default)
    assert report["ok"], report["errors"]
    assert report["metrics"] == {"physical_cubes": 20, "clusters": 9, "targets": 6, "m3_faces": 27}
    assert not report["accessibility"]["blocked_spawns"]
    assert default["m3_ground_truth"] == recompute_truth(spec, default)


def test_floor_center_frame_and_nonsector_bands_are_not_old_positive_frame(spec):
    assert world_to_pixel(spec, (2400, 1920), -3, 2.4) == (0, 0)
    assert world_to_pixel(spec, (2400, 1920), 3, -2.4) == (2400, 1920)
    assert world_to_pixel(spec, (2400, 1920), 0, 0) == (1200, 960)
    assert sector_of(spec, -1.2, 1.4) == "B-S1"
    assert sector_of(spec, 1.2, -1.4) == "R-S3"
    assert sector_of(spec, 0, 0) == "CM"
    assert sector_of(spec, 0, 2.2) is None
    assert sector_of(spec, -2.66, 0) is None


def test_fixed_geometry_uses_yaml_dimensions_and_has_exact_four_walls_eight_sensors_two_pads(spec):
    changed = deepcopy(spec)
    changed["walls"]["height"] = .53
    geometry = fixed_geometry(changed, floor_thickness=.03)
    floor = next(g for g in geometry if g["kind"] == "floor")
    assert floor["size"] == [6, 4.8, .03]
    assert floor["center"][2] == -.015
    walls = [g for g in geometry if g["kind"] == "wall"]
    assert len(walls) == 4 and all(g["size"][2] == .53 for g in walls)
    assert sum(g["kind"] == "sensor_box" for g in geometry) == 8
    assert sum(g["kind"] == "center_pad" for g in geometry) == 2


def test_target_receiver_faces_height_and_led_are_derived_not_world_oracle(spec, default):
    targets = target_metadata(spec, default)
    first = targets[0]
    assert first["camp"] == "B" and first["face"] == "E"
    assert first["receiver"] == pytest.approx([-1, 1.8, .337])
    assert first["diameter"] == .057
    assert first["led_center"] == pytest.approx([-1.2, 1.8, .41])
    changed = deepcopy(spec)
    changed["cubes"]["target_receiver"]["z"] = .3
    assert target_metadata(changed, default)[0]["receiver"][2] == .3


@pytest.mark.parametrize("seed", range(2026, 2046))
def test_twenty_seeds_preserve_clearance_all_sectors_targets_truth_and_robot_access(spec, default, seed):
    generated = generate_layout(spec, default, seed)
    report = validate_layout(spec, generated)
    assert report["ok"], (seed, report["errors"])
    assert report["metrics"]["physical_cubes"] == 20
    assert report["metrics"]["targets"] == 6
    assert set(b["sector"] for b in generated["boxes"]) == set(spec["sectors"])
    assert all(counts for counts in generated["m3_ground_truth"].values())
    assert generated["m3_ground_truth"] == recompute_truth(spec, generated)
    assert len(merged_collisions(spec, generated)) + len(fixed_geometry(spec)) <= 40


def test_seed_reproducibility_changes_positions_and_faces_without_mutating_template(spec, default):
    before = deepcopy(default)
    a, b = generate_layout(spec, default, 44), generate_layout(spec, default, 45)
    assert a == generate_layout(spec, default, 44)
    assert a["boxes"] != b["boxes"]
    assert [v["center"] for v in a["boxes"]] != [v["center"] for v in b["boxes"]]
    assert default == before


def test_variant_class_folder_selection_preserves_semantics(spec, default, tmp_path):
    folder = tmp_path / "m3_tank"
    folder.mkdir()
    (folder / "field2.jpg").write_bytes(b"selected only; image baking has its own decoder validation")
    generated = generate_layout(spec, default, 2026, faces_root=tmp_path)
    faces = [value for b in generated["boxes"] for value in b["faces"].values()]
    tanks = [value for value in faces if value.startswith("m3:tank")]
    assert tanks and all(value == "m3:tank@field2.jpg" for value in tanks)
    assert generated["m3_ground_truth"] == recompute_truth(spec, generated)
    with pytest.raises(ValueError, match="filename"):
        parse_face("m3:tank@../secret.jpg")


def test_default_yaml_cannot_be_overwritten_by_generator_cli(tmp_path):
    with pytest.raises(SystemExit) as exc:
        generate_main(["--output", str(tmp_path / "layout_default.yaml")])
    assert exc.value.code == 2
    assert not (tmp_path / "layout_default.yaml").exists()


def test_hidden_photo_is_rejected_and_never_in_ground_truth(spec, default):
    changed = deepcopy(default)
    changed["boxes"][0]["faces"]["S"] = "m3:drum"
    assert recompute_truth(spec, changed) == recompute_truth(spec, default)
    report = validate_layout(spec, changed, check_accessibility=False)
    assert any("nonblank image on a hidden face" in e for e in report["errors"])


@pytest.mark.parametrize("mutation,error", [
    (lambda b: b[0]["center"].__setitem__(0, -1.19), "grid"),
    (lambda b: b[0].__setitem__("sector", "R-S1"), "differs from coordinate sector"),
    (lambda b: b[0]["center"].__setitem__(1, 2.2), "wall clearance"),
    (lambda b: b[0].__setitem__("level", 1), "unsupported stack"),
    (lambda b: b[1].__setitem__("cluster", "separated"), "inter-cluster clearance"),
])
def test_constraint_violations_fail_instead_of_lowering_clearance(spec, default, mutation, error):
    changed = deepcopy(default)
    mutation(changed["boxes"])
    report = validate_layout(spec, changed, check_accessibility=False)
    assert not report["ok"]
    assert any(error in message for message in report["errors"])


def test_disconnected_same_name_is_not_an_exemption_from_cluster_rules(spec):
    layout = {"boxes": [dict(cluster="cheat", sector="CM", center=[0, y], faces={}) for y in (-.4, .4)]}
    report = validate_layout(spec, layout, require_all_sectors=False, require_targets=False,
                             check_truth=False, check_accessibility=False)
    assert any("disconnected cubes" in e for e in report["errors"])


def test_stacked_cube_support_truth_and_collision_union(spec):
    layout = {"boxes": [{"cluster": "stack", "sector": "CM", "center": [0, 0], "stack": 2,
                         "faces": {"N": "m3:tank"}}]}
    physical = expand_boxes(spec, layout)
    assert [b["center"][2] for b in physical] == pytest.approx([.2, .6])
    assert cube_extent(spec, layout["boxes"][0]) == pytest.approx([-.2, .2, -.2, .2, 0, .8])
    assert recompute_truth(spec, layout)["CM"] == {"tank": 2}
    merged = merged_collisions(spec, layout)
    assert len(merged) == 1
    assert merged[0]["size"] == pytest.approx([.4, .4, .8])
    report = validate_layout(spec, layout, require_all_sectors=False, require_targets=False, check_truth=False)
    assert report["ok"], report["errors"]
    assert any("camera-height/pitch" in warning for warning in report["warnings"])


def test_l_collision_does_not_fill_concave_viewing_space(spec, default):
    collisions = [g for g in merged_collisions(spec, default) if g["cluster"] == "B_S1_L"]
    assert len(collisions) == 2
    volume = sum(g["size"][0] * g["size"][1] * g["size"][2] for g in collisions)
    assert volume == pytest.approx(3 * .4 ** 3)
    # The L notch (-1.6,1.4) must remain empty, rather than a coarse AABB.
    assert not any(abs(-1.6 - g["center"][0]) < g["size"][0]/2 - 1e-8
                   and abs(1.4 - g["center"][1]) < g["size"][1]/2 - 1e-8 for g in collisions)


def test_ring_inside_face_is_exposed_but_not_robot_reachable(spec):
    positions = [(x, y) for x in (-.4, 0., .4) for y in (-.4, 0., .4) if x or y]
    layout = {"boxes": [dict(cluster="ring", sector="CM", center=[x, y], faces=dict.fromkeys(FACE_DIRS, "blank"))
                        for x, y in positions]}
    for physical in expand_boxes(spec, layout):
        layout["boxes"][physical["index"]]["faces"] = physical["faces"]
    south = next(i for i, b in enumerate(layout["boxes"]) if b["center"] == [0., -.4])
    layout["boxes"][south]["faces"]["N"] = "m3:missile"
    access = accessible_faces(spec, layout)
    assert access["views"][f"cube_{south}_0:N"] is None
    report = validate_layout(spec, layout, require_all_sectors=False, require_targets=False, check_truth=False)
    assert any("image has no reachable robot viewing pose" in e for e in report["errors"])


@pytest.fixture(scope="module")
def built_world(spec, default, tmp_path_factory):
    from gen_world import generate_world, load_profile
    package = tmp_path_factory.mktemp("arena_package")
    shutil.copytree(PACKAGE / "media/textures", package / "media/textures",
                    ignore=shutil.ignore_patterns("atlases"))
    profile = load_profile("standard")
    tree, _ = generate_world(spec, default, profile, package_root=package, ugvs=0, uav_cam=False)
    return package, profile, tree


def test_actual_sdf_and_resources_match_yaml_without_sidecar(spec, default, built_world):
    package, profile, tree = built_world
    report = check_world(spec, default, tree, package_root=package, profile=profile)
    assert report["ok"], report["errors"]
    assert report["counts"]["collisions"] == 28
    assert report["counts"]["visuals"] <= 80
    assert report["counts"]["textures"] <= 30


@pytest.mark.parametrize("change,error", [
    ("shift", "center differs"), ("size", "size differs"),
    ("rotate", "rotated geometry"), ("delete", "missing collision"),
    ("extra", "unexpected collision"), ("budget", "visuals budget exceeded"),
])
def test_checker_rejects_actual_sdf_tampering_even_with_unchanged_yaml(spec, default, built_world, change, error):
    package, profile, original = built_world
    root = deepcopy(original.getroot())
    floor_link = root.find("world/model[@name='floor']/link")
    floor = floor_link.find("collision")
    if change == "shift":
        floor.find("pose").text = "0.002 0 -0.01 0 0 0"
    elif change == "size":
        floor.find("geometry/box/size").text = "6.003 4.8 .02"
    elif change == "rotate":
        root.find("world/model[@name='layout']/link/visual/pose").text = "-1.2 1.8 .2 0 0 1.57079632679"
    elif change == "delete":
        floor_link.remove(floor)
    elif change == "extra":
        extra = deepcopy(floor)
        extra.set("name", "unexpected_obstacle")
        floor_link.append(extra)
    else:
        visual = floor_link.find("visual")
        for index in range(81):
            extra = deepcopy(visual)
            extra.set("name", f"unnecessary_grid_line_{index}")
            floor_link.append(extra)
    report = check_world(spec, default, ET.ElementTree(root), package_root=package, profile=profile)
    assert not report["ok"]
    assert any(error in e for e in report["errors"]), report["errors"]


def test_checker_uses_composed_model_link_pose_not_only_child_pose(spec, default, built_world):
    package, profile, original = built_world
    root = deepcopy(original.getroot())
    model = root.find("world/model[@name='layout']")
    ET.SubElement(model, "pose").text = "0.01 0 0 0 0 0"
    report = check_world(spec, default, root, package_root=package, profile=profile)
    assert not report["ok"]
    assert any("center differs" in e for e in report["errors"])


def test_checker_rejects_remote_and_path_escape_resources(tmp_path):
    with pytest.raises(ValueError, match="nonlocal"):
        resolve_resource("https://fuel.example/robot", tmp_path)
    with pytest.raises(ValueError, match="external"):
        resolve_resource("model://maicon_arena_gz/../private.png", tmp_path)
