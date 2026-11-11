"""Photo gate rejects missing pixels and separates observations from TF math."""
import copy
import importlib.util
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import cv2
import numpy as np
import pytest
import yaml


PACKAGE = Path(__file__).resolve().parents[1] / "ros2/wingbot_description"
SPEC = importlib.util.spec_from_file_location("wingbot_photo", PACKAGE / "scripts/photo_overlay_test.py")
photo = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(photo)


def inputs():
    root = PACKAGE / "config/photo_fit"
    return (json.loads((root / "cam_fit.json").read_text()),
            yaml.safe_load((root / "photo_keypoints.yaml").read_text()),
            np.load(root / "photo_mask.npy", allow_pickle=False))


def test_supplied_gazebo_camera_matches_independent_optical_matrix():
    camera, keypoints, mask = inputs()
    points = np.asarray([p["model_xyz"] for p in keypoints["keypoints"]])
    q = (points - camera["C"]) @ np.asarray(camera["R"]).T
    independent = q[:, :2]/q[:, 2:] * camera["f"] + [camera["W"]/2, camera["H"]/2]
    assert np.max(np.linalg.norm(photo.project_world(points, camera) - independent, axis=1)) < .04
    assert mask.shape == (1054, 1057) and mask.dtype == np.bool_
    assert mask[640:].sum() == 282437


def test_exact_photo_camera_world_has_one_sensor_no_ground_or_markers():
    camera, _, _ = inputs()
    sdf = photo.make_photo_world('<sdf version="1.9"><model name="wingbot"><link name="base_link"/></model></sdf>', camera)
    world = ET.fromstring(sdf).find("world")
    assert world.find("scene/background").text == "0 0 0 1"
    assert world.find("scene/shadows").text == "false"
    assert world.find("scene/grid").text == "false"
    assert world.find("scene/origin_visual").text == "false"
    assert world.find("light/visualize").text == "false"
    assert len(world.findall(".//sensor")) == 1
    model = world.find("model[@name='photo_camera']")
    assert list(map(float, model.findtext("pose").split())) == camera["gazebo_camera"]["pose_xyzrpy"]
    assert float(model.findtext(".//horizontal_fov")) == camera["gazebo_camera"]["hfov_rad"]
    assert (int(model.findtext(".//width")), int(model.findtext(".//height"))) == (1057, 1054)
    assert int(model.findtext(".//visibility_mask")) & 1 == 0
    assert world.find(".//visual") is None


def test_black_image_cannot_pass_even_with_perfect_reference_model_coordinates():
    camera, keypoints, mask = inputs()
    points = {p["name"]: p["model_xyz"] for p in keypoints["keypoints"]}
    report = photo.evaluate(np.zeros((1054, 1057, 3), np.uint8), mask, camera, keypoints, points)
    assert not report["passed"]
    assert report["observed_count"] == 0
    assert report["silhouette"]["iou"] == 0
    assert all(p["image_error_px"] is None and not p["passed"] for p in report["keypoints"])
    altered = copy.deepcopy(keypoints)
    for point in altered["keypoints"]:
        point["model_xyz"] = [999, 999, 999]
        point["model_px"] = [-900, -900]
    again = photo.evaluate(np.zeros((1054, 1057, 3), np.uint8), mask, camera, altered, points)
    assert report["keypoints"] == again["keypoints"]


@pytest.mark.parametrize("offset", [(0, 0), (30, 0), (0, -25)])
def test_feature_pixel_follows_actual_displaced_image_not_annotation(offset):
    image = np.zeros((1054, 1057, 3), np.uint8)
    reference = np.array([620, 213])
    observed = reference + offset
    cv2.circle(image, tuple(observed), 9, (220, 220, 220), -1)
    detection = photo.detect_feature(image, "camera_led", reference)
    assert detection["status"] == "observed"
    assert np.linalg.norm(np.asarray(detection["pixel"]) - observed) < .1
    if offset != (0, 0):
        assert np.linalg.norm(np.asarray(detection["pixel"]) - reference) > 12


def test_silhouette_comparison_does_not_fill_holes_or_translate_image():
    reference = np.zeros((1054, 1057), dtype=bool)
    reference[700:900, 400:600] = True
    observed = reference.copy()
    observed[740:860, 440:560] = False
    assert photo.lower_silhouette_iou(observed, reference) == pytest.approx(.64)
    assert photo.lower_silhouette_iou(np.roll(reference, 40, axis=1), reference) == pytest.approx(2/3)


def test_wrong_resolution_is_explicit_failure():
    camera, keypoints, mask = inputs()
    with pytest.raises(ValueError, match="resolution"):
        photo.evaluate(np.zeros((720, 1280, 3), np.uint8), mask, camera, keypoints, {})


@pytest.mark.parametrize("dx", [0, 30])
def test_depth_front_center_comes_from_actual_rectangle_corners(dx):
    image = np.zeros((1054, 1057, 3), np.uint8)
    corners = np.array([[811, 552], [595, 585], [595, 677], [811, 639]]) + [dx, 0]
    cv2.fillPoly(image, [corners], (70, 67, 67))
    detection = photo.detect_feature(image, "depth_camera_front_ctr", [712, 612])
    assert detection["method"] == "actual_front_rectangle_diagonal_intersection"
    assert detection["pixel"] == pytest.approx([706.0167597765363+dx, 612.754189944134])
    if dx:
        assert np.linalg.norm(np.asarray(detection["pixel"])-[712, 612]) > 12


@pytest.mark.parametrize("dx", [0, 30])
def test_lidar_cap_center_comes_from_pixels_despite_same_colour_side(dx):
    image = np.zeros((1054, 1057, 3), np.uint8)
    cv2.rectangle(image, (580+dx, 458), (603+dx, 506), (84, 81, 81), -1)
    cv2.ellipse(image, (612+dx, 458), (58, 8), 0, 0, 360, (84, 81, 81), -1)
    detection = photo.detect_feature(image, "lidar_top_center", [617, 462])
    assert detection["method"] == "actual_flat_cap_region_ellipse"
    assert np.linalg.norm(np.asarray(detection["pixel"])-[612+dx, 458]) < 1
    if dx:
        assert np.linalg.norm(np.asarray(detection["pixel"])-[617, 462]) > 12


def test_actual_rgb_measurements_do_not_change_when_tf_is_missing():
    camera, keypoints, mask = inputs()
    image = cv2.imread(str(PACKAGE / "docs/photo_render_RED.png"))
    recorded = json.loads((PACKAGE / "docs/photo_validation.json").read_text())
    points = {p["name"]: p["actual_tf_world"] for p in recorded["keypoints"]}
    actual = photo.evaluate(image, mask, camera, keypoints, points)
    without_tf = photo.evaluate(image, mask, camera, keypoints, {})
    assert without_tf["observed_count"] == actual["observed_count"] == 12
    assert not without_tf["passed"]
    for a, b in zip(actual["keypoints"], without_tf["keypoints"]):
        assert a["image_observation"] == b["image_observation"]
        assert b["tf_error_px"] is None and not b["passed"]


def test_blue_lenses_are_not_mistaken_for_antenna_sleeve_endpoints():
    image = np.zeros((1054, 1057, 3), np.uint8)
    cv2.rectangle(image, (672, 116), (696, 275), (230, 75, 25), -1)
    cv2.rectangle(image, (672, 167), (696, 174), (8, 8, 8), -1)
    cv2.rectangle(image, (672, 185), (696, 191), (8, 8, 8), -1)
    cv2.ellipse(image, (632, 118), (12, 18), 0, 0, 360, (170, 60, 30), -1)
    cv2.ellipse(image, (650, 273), (10, 14), 0, 0, 360, (170, 60, 30), -1)
    top = photo.detect_feature(image, "iff_left_sleeve_top", [681, 115], "BLUE")
    bottom = photo.detect_feature(image, "iff_left_sleeve_bottom", [681, 270], "BLUE")
    assert top["pixel"] == pytest.approx([684, 116])
    assert bottom["pixel"] == pytest.approx([684, 275])
