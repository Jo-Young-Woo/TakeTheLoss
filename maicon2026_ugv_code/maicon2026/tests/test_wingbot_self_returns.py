"""Measured mast hits are unknown; nearby external returns remain observations."""
import copy
import json
import math
from pathlib import Path
import sys

import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
ARENA = ROOT / "ros2/maicon_arena_gz"
WINGBOT = ROOT / "ros2/wingbot_description"
sys.path.insert(0, str(ARENA / "scripts"))
from wingbot_runtime import WingbotFrames, crop_scan, mask_self_returns  # noqa: E402
from arena_common import expand_boxes  # noqa: E402


@pytest.fixture
def frames():
    return WingbotFrames(WINGBOT / "config/frames.yaml")


@pytest.fixture
def measured_scan():
    """Fixture retains ten actual ranges, not an invented complete raw scan.

    Other bins deliberately use synthetic distant walls to test preservation.
    Infer the original float32 angular sampling from the recorded ray angles.
    """
    report = json.loads((WINGBOT / "validation/default_smoke/smoke.json").read_text(encoding="utf-8"))
    rows = report["remaining_close_rays"]
    first, second = rows[:2]
    increment = math.radians((second["scan_angle_deg"] - first["scan_angle_deg"])
                             / (second["index"] - first["index"]))
    angle_min = math.radians(first["scan_angle_deg"]) - first["index"] * increment
    ranges = [2.0 + index / 450.0 for index in range(450)]
    for row in rows:
        assert math.degrees(angle_min + row["index"] * increment) == pytest.approx(row["scan_angle_deg"], abs=1e-8)
        ranges[row["index"]] = row["range_m"]
    return ranges, angle_min, increment, rows, report["cropped_nan_count"]


def test_actual_ten_mast_returns_are_masked_without_removing_distant_walls(frames, measured_scan):
    ranges, angle_min, increment, rows, expected_crop_count = measured_scan
    original = ranges.copy()
    cropped, crop_indices = crop_scan(ranges, angle_min, increment, frames.semantic["lidar"], frames.data)
    filtered, self_indices = mask_self_returns(cropped, angle_min, increment, frames.semantic["lidar"], frames)
    expected = [276, 277, 278, 279, 280, 393, 394, 395, 396, 397]
    assert [row["index"] for row in rows] == expected
    assert self_indices == expected
    assert len(crop_indices) == expected_crop_count == 112
    assert not set(crop_indices).intersection(self_indices)
    assert len(filtered) == len(ranges) == 450
    for index, value in enumerate(filtered):
        if index in set(crop_indices + self_indices):
            assert math.isnan(value), "Occlusion must not become free space (+inf)"
        else:
            assert value == original[index], "Unoccluded wall ranges must remain exact"
    assert ranges == original, "Filtering must not mutate the incoming scan"


def test_actual_rotated_scan_removes_antenna_surfaces_and_preserves_nearby_obstacles(frames):
    report = json.loads((WINGBOT / "validation/sensor_calibration/live_near_returns.json").read_text(encoding="utf-8"))["raw"]
    ranges = [2.] * 450
    for row in report["close"]:
        ranges[row["index"]] = row["range"]
    cropped, crop_indices = crop_scan(ranges, report["amin"], report["inc"], report["frame"], frames.data)
    filtered, masked = mask_self_returns(cropped, report["amin"], report["inc"], report["frame"], frames)
    expected = [269, 270, 271, 272, 276, 277, 278, 279, 280, 393, 394, 395, 396, 397, 401, 402, 403, 404, 405]
    assert masked == expected
    obstacle_count = 0
    for index, distance in enumerate(ranges):
        if index not in crop_indices + expected:
            assert filtered[index] == distance
            obstacle_count += .15 < distance < .25
        else:
            assert math.isnan(filtered[index])
    assert obstacle_count >= 20, "Replay must include the real close external obstacle"


@pytest.mark.parametrize("distance", [.12, .14, .18, 2.])
def test_antenna_directions_do_not_create_a_broad_close_range_filter(frames, distance):
    report = json.loads((WINGBOT / "validation/sensor_calibration/live_near_returns.json").read_text(encoding="utf-8"))["raw"]
    values = [2.] * 450
    for index in [269, 270, 271, 272, 401, 402, 403, 404, 405]:
        values[index] = distance
    filtered, masked = mask_self_returns(values, report["amin"], report["inc"], report["frame"], frames)
    assert not masked and filtered == values


def test_visual_sections_include_fixed_antennas_but_no_unmeasured_head_pose(frames):
    sections = frames.data["lidar"]["self_return_sections"]
    sources = [section["source"] for section in sections]
    assert any("iff_left_rod" in source for source in sources)
    assert any("iff_right_rod" in source for source in sources)
    assert not any(source.startswith(("pt_link1/", "pt_link2/")) for source in sources)
    assert all(section["z"] == pytest.approx(frames.zero[frames.semantic["lidar"]][2, 3]) for section in sections)


@pytest.mark.parametrize("distance", [.020, .025, .040, .10, 1., 11.])
def test_same_angles_preserve_closer_obstacles_and_farther_returns(frames, measured_scan, distance):
    ranges, angle_min, increment, rows, _ = measured_scan
    for row in rows:
        ranges[row["index"]] = distance
    filtered, masked = mask_self_returns(ranges, angle_min, increment, frames.semantic["lidar"], frames)
    assert masked == []
    assert filtered == ranges


def test_short_range_alone_does_not_identify_robot_body(frames):
    # Beam +X in the body is clear of the mast even at the same measured range.
    matrix = frames.zero[frames.semantic["lidar"]]
    yaw = math.atan2(matrix[1, 0], matrix[0, 0])
    values, masked = mask_self_returns([.032], -yaw, .001, frames.semantic["lidar"], frames)
    assert not masked and values == [.032]


def test_invalid_and_no_return_values_remain_distinct(frames):
    source = [math.nan, math.inf, -math.inf, 0., -.1]
    result, masked = mask_self_returns(source, .75, .001, frames.semantic["lidar"], frames)
    assert not masked and math.isnan(result[0])
    assert result[1:] == source[1:]


def test_scan_order_reversal_preserves_physical_self_hit_selection(frames, measured_scan):
    source, angle_min, increment, rows, _ = measured_scan
    result, masked = mask_self_returns(list(reversed(source)), angle_min + 449 * increment,
                                      -increment, frames.semantic["lidar"], frames)
    assert masked == sorted(449 - row["index"] for row in rows)
    assert all(math.isnan(result[index]) for index in masked)


def test_scan_plane_outside_pillar_height_preserves_same_angle_and_distance(frames, measured_scan):
    changed = copy.deepcopy(frames)
    changed.zero[frames.semantic["lidar"]][2, 3] = max(c["z"][1] for c in frames.data["lidar"]["self_return_cylinders"]) + .01
    source, angle_min, increment, _, _ = measured_scan
    result, masked = mask_self_returns(source, angle_min, increment, frames.semantic["lidar"], changed)
    assert not masked and result == source


def test_generated_pillars_follow_the_single_mount_authority(frames):
    mods = yaml.safe_load((WINGBOT / "config/wingbot_mods.yaml").read_text(encoding="utf-8"))
    mast = mods["pan_mast"]
    pillars = mast["pillars"]
    shapes = frames.data["lidar"]["self_return_cylinders"]
    assert len(shapes) == pillars["count"] == 4
    for index, shape in enumerate(shapes):
        angle = math.pi / 4 + 2 * math.pi * index / pillars["count"]
        expected = np.asarray(mast["center_xy"]) + pillars["ring_radius"] * np.asarray([math.cos(angle), math.sin(angle)])
        assert shape["frame"] == "base_footprint"
        assert shape["source"] == "pan_mast.pillars"
        assert shape["center_xy"] == pytest.approx(expected, abs=1e-12)
        assert shape["radius"] == pillars["radius"]
        assert shape["z"] == pillars["z"]
    lidar = frames.zero[frames.semantic["lidar"]]
    assert lidar[:3, 2] == pytest.approx([0., 0., 1.], abs=1e-8), "The current cylinder intersection assumes a horizontal scan"
    assert frames.data["lidar"]["crop_deg"] == mods["lidar"]["crop_deg"] == [225., 315.]


@pytest.mark.parametrize("angle_min,increment", [(math.nan, .01), (0., math.inf), (0., 0.)])
def test_invalid_angular_sampling_fails_closed(frames, angle_min, increment):
    with pytest.raises(ValueError, match="Invalid scan geometry"):
        mask_self_returns([.032], angle_min, increment, frames.semantic["lidar"], frames)


def test_wrong_frame_fails_closed(frames):
    with pytest.raises(ValueError, match="different scan frame"):
        mask_self_returns([.032], 0., .01, "base_footprint", frames)


def test_self_surface_range_tolerance_is_bounded(frames):
    matrix = frames.zero[frames.semantic["lidar"]]
    center = np.asarray(frames.data["lidar"]["self_return_cylinders"][0]["center_xy"])
    radius = frames.data["lidar"]["self_return_cylinders"][0]["radius"]
    delta = center - matrix[:2, 3]
    angle = math.atan2(delta[1], delta[0]) - math.atan2(matrix[1, 0], matrix[0, 0])
    exact_surface = float(np.linalg.norm(delta)) - radius
    for error, expected in ((-.0009, True), (.0009, True), (-.0011, False), (.0011, False)):
        result, masked = mask_self_returns([exact_surface + error], angle, 1e-6, frames.semantic["lidar"], frames)
        assert bool(masked) == expected
        assert math.isnan(result[0]) == expected


def test_recording_profile_retains_standard_physics_and_robot_sensors():
    standard = yaml.safe_load((ARENA / "config/profiles/standard.yaml").read_text(encoding="utf-8"))
    recording = yaml.safe_load((ARENA / "config/profiles/wingbot_recording.yaml").read_text(encoding="utf-8"))
    assert recording["name"] == "wingbot_recording"
    assert recording["validation_camera"]["resolution"] == [1280, 720]
    assert recording["validation_camera"]["update_rate"] == 10.
    for key, value in standard.items():
        if key not in ("name", "validation_camera"):
            assert recording[key] == value, key


def test_recording_view_contains_enclosure_and_raised_structure_tops():
    profile = yaml.safe_load((ARENA / "config/profiles/wingbot_recording.yaml").read_text(encoding="utf-8"))
    spec = yaml.safe_load((ARENA / "config/arena_spec.yaml").read_text(encoding="utf-8"))
    layout = yaml.safe_load((ARENA / "config/layouts/layout_default.yaml").read_text(encoding="utf-8"))
    camera = profile["validation_camera"]
    x, y, z, roll, pitch, yaw = camera["pose"]
    assert roll == 0. and y < -spec["floor"]["size"][1] / 2
    assert 0. < pitch < math.pi / 2, "Use an oblique third-person view"
    cp, sp, cy, sy = math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    rotation = np.array([[cy * cp, -sy, cy * sp], [sy * cp, cy, sy * sp], [-sp, 0., cp]])
    width, height = camera["resolution"]
    focal = width / (2 * math.tan(camera["hfov"] / 2))
    half_x, half_y = np.asarray(spec["floor"]["size"]) / 2 + spec["walls"]["thickness"]
    tower = spec["tower_leds"]["geometry"]
    top_z = max(spec["sensor_boxes"]["size"][2] + tower["pole_height"]
                + len(tower["segments"]) * tower["segment_height"],
                *(box["center"][2] + box["size"][2] / 2
                  + spec["cubes"]["target_signal_led"]["size"][2]
                  for box in expand_boxes(spec, layout)))
    # Bounding the full enclosure at both heights also bounds every floor/cube corner.
    for corner_x in (-half_x, half_x):
        for corner_y in (-half_y, half_y):
            for corner_z in (0., top_z):
                forward, left, up = rotation.T @ (np.asarray([corner_x, corner_y, corner_z]) - [x, y, z])
                assert camera["clip"][0] < forward < camera["clip"][1]
                pixel_x, pixel_y = width / 2 - focal * left / forward, height / 2 - focal * up / forward
                assert 20. < pixel_x < width - 20.
                assert 20. < pixel_y < height - 20.
