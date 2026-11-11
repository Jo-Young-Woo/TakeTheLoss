"""Black overhead tops supply geometry candidates, never M3 side labels."""
import numpy as np
import pytest
from pathlib import Path
import yaml

from maicon26.perception.uav_layout import FixedUavCalibration, FixedUavLayoutDetector
from maicon26.perception.uav_calibration import load_calibration


def draw_fixture(centres, shift=0):
    cv2 = pytest.importorskip("cv2")
    image = np.full((720, 1280, 3), 65, np.uint8)
    cal = FixedUavCalibration()
    scale = cal.pixels_per_metre(1280)
    for cx, cy in centres:
        left = round(640+(cx-cal.camera_x-.2)*scale)+shift
        right = round(640+(cx-cal.camera_x+.2)*scale)+shift
        top = round(360-(cy-cal.camera_y+.2)*scale)
        bottom = round(360-(cy-cal.camera_y-.2)*scale)
        cv2.rectangle(image, (left, top), (right-1, bottom-1), (0, 0, 0), -1)
    return image, scale


def test_touching_black_l_cluster_yields_every_cube_and_follows_only_image_motion():
    expected = [(1.4, 1.4), (1.8, 1.4), (1.8, 1.8)]
    detector = FixedUavLayoutDetector()
    image, scale = draw_fixture(expected)
    boxes = detector.detect(image)
    assert len(boxes) == 3
    for cx, cy in expected:
        assert any(abs(b.cx-cx) < .015 and abs(b.cy-cy) < .015 for b in boxes)
    assert all(b.sx == b.sy == .4 for b in boxes)
    moved, _ = draw_fixture(expected, shift=31)
    moved_boxes = detector.detect(moved)
    assert len(moved_boxes) == len(boxes)
    for first, second in zip(boxes, moved_boxes):
        assert second.cx-first.cx == pytest.approx(31/scale, abs=.005)
        assert second.cy == pytest.approx(first.cy, abs=.005)
        assert not hasattr(second, "cls") and not hasattr(second, "faces")


def test_red_signal_hole_preserves_candidate_but_robot_size_patch_is_not_cube():
    cv2 = pytest.importorskip("cv2")
    image, scale = draw_fixture([(3.0, 2.5)])
    cv2.rectangle(image, (626, 355), (637, 361), (0, 0, 150), -1)
    # A small .25 x .23 m vehicle-shaped dark patch must not be enlarged to .4.
    cv2.rectangle(image, (800, 300), (800+round(.25*scale), 300+round(.23*scale)), (0, 0, 0), -1)
    boxes = FixedUavLayoutDetector().detect(image)
    assert len(boxes) == 1
    assert boxes[0].cx == pytest.approx(3.0, abs=.015)
    assert boxes[0].cy == pytest.approx(2.5, abs=.015)


def test_near_black_channel_noise_does_not_turn_black_cubes_into_high_saturation_holes():
    image, _ = draw_fixture([(1.4, 1.4), (1.8, 1.4), (1.8, 1.8)])
    black_pixels = np.all(image == 0, axis=2)
    image[black_pixels] = np.random.default_rng(19).integers(0, 9, (np.count_nonzero(black_pixels), 3), dtype=np.uint8)
    boxes = FixedUavLayoutDetector().detect(image)
    assert len(boxes) == 3
    assert all(b.sx == b.sy == .4 for b in boxes)


@pytest.mark.parametrize("capture", ["uav_render_gpu_6ugv.png", "api_uav_live.jpg"])
def test_measured_arena_frame_recovers_all_twenty_tops_without_reading_layout(monkeypatch, capture):
    cv2 = pytest.importorskip("cv2")
    root = Path(__file__).resolve().parents[1]
    package = root / "ros2/maicon_arena_gz"
    calibration = load_calibration(package / "config/uav_grid_calibration.json")
    image = cv2.imdecode(np.frombuffer((package / "validation" / capture).read_bytes(), np.uint8), cv2.IMREAD_COLOR)
    # Layout is an offline evaluator input only; it never reaches the detector.
    layout = yaml.safe_load((package / "config/layouts/layout_default.yaml").read_text(encoding="utf-8"))
    expected = [(b["center"][0]+3, b["center"][1]+2.4) for b in layout["boxes"]]
    detector = FixedUavLayoutDetector(calibration)
    def forbidden_read(*args, **kwargs):
        raise AssertionError("Image inference must not read layout or other files")
    monkeypatch.setattr(Path, "read_text", forbidden_read)
    monkeypatch.setattr(Path, "read_bytes", forbidden_read)
    boxes = detector.detect(image)
    assert len(boxes) == len(expected) == 20
    assert all(b.sx == b.sy == .4 for b in boxes)
    for cx, cy in expected:
        matches = [b for b in boxes if np.hypot(b.cx-cx, b.cy-cy) < .012]
        assert len(matches) == 1
    assert detector.detect(np.zeros_like(image)) == []
    assert all(not hasattr(b, "cls") and not hasattr(b, "faces") for b in boxes)


def test_new_camera_calibration_uses_measured_points_and_independent_holdout():
    package = Path(__file__).resolve().parents[1] / "ros2/maicon_arena_gz"
    calibration = load_calibration(package / "config/uav_grid_calibration.json")
    report = calibration.validation_report()
    assert report["fit_point_count"] == 24
    assert report["fit_inlier_count"] == 23  # repeated print motif rejected, not hidden
    assert report["independent_checkpoint_count"] == 23
    assert report["checkpoint_max_m"] < .01
    assert report["fit_is_independent_validation"] is False
    assert calibration.arena_width == 6 and calibration.arena_height == 4.8
    fit_pixels = {tuple(p["pixel"]) for p in calibration.measurements}
    assert not fit_pixels.intersection(tuple(p["pixel"]) for p in calibration.checkpoints)
    assert all("reference_pixel" in p and p["source"] == "mutual_image_feature_in_measured_floor_print"
               for p in (*calibration.measurements, *calibration.checkpoints))
