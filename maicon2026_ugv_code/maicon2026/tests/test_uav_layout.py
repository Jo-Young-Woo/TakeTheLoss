"""The overhead camera supplies geometry, never unseen M3 object labels."""
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from maicon26.hal.gazebo_uav import decode_bgr
from maicon26.perception.uav_layout import FixedUavCalibration, FixedUavLayoutDetector


def test_actual_gazebo_image_recovers_box_tops_without_world_input():
    cv2 = pytest.importorskip("cv2")
    image_path = Path(__file__).resolve().parents[1] / "ros2/gazebo/assets/fixed_uav_first_frame.png"
    image = cv2.imdecode(np.frombuffer(image_path.read_bytes(), np.uint8), cv2.IMREAD_COLOR)
    detector = FixedUavLayoutDetector()
    boxes = detector.detect(image)
    expected = [(1.60, 3.80, .4, .4), (2.00, 3.80, .4, .8),
                (4.12, 3.80, .4, .8), (4.52, 3.80, .4, .4),
                (1.70, 2.50, .8, .4), (4.42, 2.50, .8, .4),
                (1.60, 1.00, .4, .4), (2.00, 1.15, .4, .8),
                (4.12, 1.15, .4, .8), (4.52, 1.00, .4, .4),
                (3.06, 4.40, .4, .4), (3.06, 2.50, .4, .4),
                (3.06, .60, .4, .4)]
    assert len(boxes) == len(expected)
    for cx, cy, sx, sy in expected:
        assert any(abs(box.cx - cx) < .025 and abs(box.cy - cy) < .025 and
                   box.sx == sx and box.sy == sy for box in boxes)
    assert all(not hasattr(box, "cls") and not hasattr(box, "faces") for box in boxes)


def test_actual_photo_cube_scene_detects_all_new_top_footprints():
    cv2 = pytest.importorskip("cv2")
    path = Path(__file__).resolve().parents[1] / "ros2/gazebo/assets/m3_random4_overhead.png"
    image = cv2.imdecode(np.frombuffer(path.read_bytes(), np.uint8), cv2.IMREAD_COLOR)
    boxes = FixedUavLayoutDetector().detect(image)
    assert len(boxes) == 17
    for cx, cy in ((.6, 1.4), (5.5, 3.1), (3.1, 1.6), (3.3, 3.5)):
        assert any(abs(b.cx-cx) < .025 and abs(b.cy-cy) < .025 and
                   b.sx == b.sy == .4 for b in boxes)


def test_camera_projection_moves_with_pixel_evidence_and_rejects_blank():
    cv2 = pytest.importorskip("cv2")
    detector = FixedUavLayoutDetector()
    image = np.full((720, 1280, 3), 30, np.uint8)
    scale = FixedUavCalibration().pixels_per_metre(1280)
    side = round(.4 * scale)
    cv2.rectangle(image, (800, 460), (800 + side - 1, 460 + side - 1), (96, 101, 110), -1)
    box, = detector.detect(image)
    assert box.sx == box.sy == .4
    assert box.cx == pytest.approx(3.06 + (800 + side / 2 - 640) / scale)
    assert box.cy == pytest.approx(2.5 - (460 + side / 2 - 360) / scale)
    assert detector.detect(np.zeros_like(image)) == []
    assert detector.detect(np.full_like(image, 105)) == []
    assert detector.detect(image[:, :1000]) == []


def test_rgb_decoding_honours_padding_and_rejects_corrupt_frames():
    msg = SimpleNamespace(width=2, height=1, step=8, encoding="rgb8",
                          data=bytes([255, 0, 0, 0, 0, 255, 90, 90]))
    assert decode_bgr(msg).tolist() == [[[0, 0, 255], [255, 0, 0]]]
    msg.data = msg.data[:-1]
    with pytest.raises(ValueError, match="stride"):
        decode_bgr(msg)
    msg.encoding = "mono8"
    with pytest.raises(ValueError, match="encoding"):
        decode_bgr(msg)


def test_near_wall_random_boxes_are_not_cropped_by_an_arena_roi():
    cv2 = pytest.importorskip("cv2")
    image = np.full((720, 1280, 3), 30, np.uint8)
    cal = FixedUavCalibration()
    scale = cal.pixels_per_metre(1280)
    side = round(.4 * scale)
    expected = [(.6, 1.4), (5.5, 3.1)]
    for cx, cy in expected:
        left = round(640 + (cx - cal.camera_x) * scale - side / 2)
        top = round(360 - (cy - cal.camera_y) * scale - side / 2)
        cv2.rectangle(image, (left, top), (left + side - 1, top + side - 1), (96, 101, 110), -1)
    boxes = FixedUavLayoutDetector().detect(image)
    assert len(boxes) == 2
    for cx, cy in expected:
        assert any(abs(box.cx - cx) < .015 and abs(box.cy - cy) < .015 for box in boxes)
