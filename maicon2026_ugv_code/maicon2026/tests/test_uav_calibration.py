"""Labelled-corner projection, box-height parallax and independent validation."""
import json
from pathlib import Path

import numpy as np
import pytest

from maicon26.perception.uav_calibration import CORNER_ORDER, FourCornerCalibration
from maicon26.perception.uav_layout import FixedUavCalibration, FixedUavLayoutDetector
from scripts.calibrate_uav import main


def calibration(**overrides):
    options = dict(image_size=(1000, 800), pixel_corners=((100, 700), (900, 650), (800, 100), (150, 150)),
                   camera_center=(3.06, 2.5, 2.465))
    return FourCornerCalibration(**{**options, **overrides})


def pixel_for(cal, xy, z=0.):
    points = np.column_stack([np.asarray(xy, dtype=float), np.ones(len(xy))])
    projected = points @ np.linalg.inv(cal.plane_homography(z)).T
    return projected[:, :2] / projected[:, 2:]


def test_four_labelled_corners_map_to_arena_axes_without_silent_reordering():
    cal = calibration()
    expected = [[0, 0], [6.12, 0], [6.12, 5], [0, 5]]
    assert np.allclose(cal.project(cal.pixel_corners), expected, atol=1e-9)
    report = cal.validation_report()
    assert report["corner_fit_rms_m"] < 1e-9
    assert report["corner_fit_is_independent_validation"] is False
    assert report["status"] == "not_independently_validated" and report["checkpoint_rms_m"] is None


@pytest.mark.parametrize("corners", [
    ((100, 700), (800, 100), (900, 650), (150, 150)),  # crossed order
    ((100, 700), (900, 650), (500, 650), (150, 150)), # concave
    ((100, 100), (200, 200), (300, 300), (400, 400)), # collinear
    ((100, 700), (900, 650), (1000, 100), (150, 150)), # outside image
    ((100, 700), (900, 650), (float("nan"), 100), (150, 150)),
])
def test_bad_corner_geometry_is_rejected(corners):
    with pytest.raises(ValueError):
        calibration(pixel_corners=corners)


def test_box_top_height_correction_removes_radial_floor_projection_bias():
    cal = calibration()
    top = np.array([[4.5, 3.5], [1.2, 1.0]])
    pixels = pixel_for(cal, top, .4)
    mapped_as_floor = cal.project(pixels)
    assert np.min(np.linalg.norm(mapped_as_floor - top, axis=1)) > .25
    assert np.allclose(cal.project(pixels, plane_z=.4), top, atol=1e-9)
    # The relation uses the optical centre and remains valid for tilted cameras.
    cx, cy, cz = cal.camera_center
    expected = np.array([cx, cy]) + (cz-.4)/cz * (mapped_as_floor-np.array([cx, cy]))
    assert np.allclose(expected, top)


def test_reference_plane_height_is_explicit_and_not_assumed_floor():
    cal = calibration(reference_plane_z=.6)
    points = np.array([[2.0, 1.5], [4.0, 3.0]])
    pixels = pixel_for(cal, points, .4)
    assert np.allclose(cal.project(pixels, .4), points)
    assert not np.allclose(cal.project(pixels), points)
    with pytest.raises(ValueError, match="above"):
        calibration(camera_center=(3, 2, .35))
    with pytest.raises(ValueError, match="below"):
        cal.project(pixels, 3.0)


def test_out_of_frame_and_out_of_arena_points_fail_explicitly():
    cal = calibration()
    with pytest.raises(ValueError, match="image"):
        cal.project([[-1, 400]])
    with pytest.raises(ValueError, match="arena"):
        cal.project([[0, 0]])
    assert np.isfinite(cal.project([[0, 0]], require_inside=False)).all()


def test_independent_checks_do_not_reuse_fit_corners_and_measure_real_error():
    cal = calibration()
    xy = np.array([[2., 1.5], [4., 3.5]])
    pixels = pixel_for(cal, xy)
    checkpoints = tuple({"pixel": pixel.tolist(), "arena": (point + [.05, 0]).tolist(), "plane_z": 0.}
                        for pixel, point in zip(pixels, xy))
    checked = calibration(checkpoints=checkpoints)
    report = checked.validation_report()
    assert report["checkpoint_rms_m"] == pytest.approx(.05)
    assert report["independent_checkpoint_count"] == 2 and report["status"] == "within_tolerance"
    with pytest.raises(ValueError, match="fitted corner"):
        calibration(checkpoints=({"pixel": [100, 700], "arena": [0, 0]},))
    with pytest.raises(ValueError, match="distinct"):
        calibration(checkpoints=(checkpoints[0], checkpoints[0]))


def test_failed_checkpoint_report_cannot_be_bypassed_by_editing_metadata(tmp_path):
    cal = calibration()
    pixel = pixel_for(cal, [[3., 2.]])[0].tolist()
    bad = calibration(checkpoints=({"pixel": pixel, "arena": [4., 2.]},))
    data = bad.to_dict()
    assert data["validation"]["status"] == "outside_tolerance"
    data["validation"]["status"] = "within_tolerance"
    path = tmp_path / "bad.json"
    path.write_text(json.dumps(data))
    with pytest.raises(ValueError, match="failed"):
        FourCornerCalibration.load(path)


def test_json_roundtrip_and_cli_keep_units_provenance_and_no_invented_validation(tmp_path):
    output = tmp_path / "camera.json"
    assert main(["--image-width", "1000", "--image-height", "800", "--corners",
                 "100", "700", "900", "650", "800", "100", "150", "150",
                 "--camera-center", "3.06", "2.5", "2.465", "--output", str(output)]) == 0
    data = json.loads(output.read_text())
    assert data["corner_order"] == list(CORNER_ORDER)
    assert data["reference_plane_z_m"] == 0 and data["object_plane_z_m"] == .4
    assert data["input_optics"] == "pinhole_undistorted"
    loaded = FourCornerCalibration.load(output)
    assert loaded.validation_report()["status"] == "not_independently_validated"
    assert np.allclose(loaded.project([[500, 400]]), calibration().project([[500, 400]]))


def test_rectification_preserves_actual_gazebo_cube_positions_and_rejects_wrong_resolution():
    cv2 = pytest.importorskip("cv2")
    fixed = FixedUavCalibration()
    # Synthetic projection of known FLOOR corners is only this test's fixture.
    # Runtime CLI requires explicit manually labelled pixels instead.
    width, height = 1280, 720
    floor_scale = width / (2 * np.tan(fixed.horizontal_fov/2) * fixed.camera_z)
    arena = np.array([[0, 0], [6.12, 0], [6.12, 5], [0, 5]], dtype=float)
    pixels = np.column_stack([width/2 + (arena[:, 0]-fixed.camera_x)*floor_scale,
                              height/2 - (arena[:, 1]-fixed.camera_y)*floor_scale])
    cal = FourCornerCalibration((width, height), tuple(map(tuple, pixels)),
                                (fixed.camera_x, fixed.camera_y, fixed.camera_z))
    path = Path(__file__).resolve().parents[1] / "ros2/gazebo/assets/m3_random4_overhead.png"
    bgr = cv2.imdecode(np.frombuffer(path.read_bytes(), np.uint8), cv2.IMREAD_COLOR)
    legacy = FixedUavLayoutDetector().detect(bgr)
    rectified = FixedUavLayoutDetector(cal).detect(bgr)
    assert len(rectified) == len(legacy) == 17
    for old in legacy:
        assert any(np.hypot(old.cx-b.cx, old.cy-b.cy) < .035 and old.sx == b.sx and old.sy == b.sy
                   for b in rectified)
    with pytest.raises(ValueError, match="size"):
        FixedUavLayoutDetector(cal).detect(bgr[:, :1200])
