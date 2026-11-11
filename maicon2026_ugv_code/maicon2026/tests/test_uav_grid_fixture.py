"""Regression against the saved Gazebo image and separately held-out grid points."""
from pathlib import Path

import numpy as np
import pytest

from maicon26.perception.uav_calibration import GridCalibration, load_calibration
from maicon26.perception.uav_layout import FixedUavLayoutDetector


def test_released_measured_grid_projects_saved_image_without_hidden_object_classes():
    cv2 = pytest.importorskip("cv2")
    root = Path(__file__).resolve().parents[1]
    calibration = load_calibration(root / "ros2/gazebo/uav_grid_calibration.json")
    assert isinstance(calibration, GridCalibration)
    report = calibration.validation_report()
    assert report["independent_checkpoint_count"] == 13
    assert report["fit_is_independent_validation"] is False
    assert report["checkpoint_max_m"] < .02
    data = (root / "ros2/gazebo/assets/m3_random4_overhead.png").read_bytes()
    image = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
    detector = FixedUavLayoutDetector(calibration)
    boxes = detector.detect(image)
    assert len(boxes) == 17  # 13 fixed blocks plus four random cubes in this captured fixture.
    assert all(0 <= b.cx <= 6.12 and 0 <= b.cy <= 5. for b in boxes)
    assert all(not hasattr(b, "objects") and not hasattr(b, "label") for b in boxes)
    with pytest.raises(ValueError, match="size"):
        detector.detect(image[:, :1200])
