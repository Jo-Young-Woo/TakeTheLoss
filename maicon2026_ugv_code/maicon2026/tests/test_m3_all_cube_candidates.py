"""Every cube is an inspection candidate, regardless of hidden side contents."""
from pathlib import Path

import numpy as np
import pytest

from maicon26.config import Config
from maicon26.link.base import UavBox
from maicon26.perception.coverage import CoverageMap, CONFIRMED, EMPTY
from maicon26.perception.uav_layout import FixedUavLayoutDetector
from scripts.plot_m3_trial import is_cube_candidate


def test_actual_image_all_eleven_cubes_start_with_four_uninspected_faces():
    cv2 = pytest.importorskip("cv2")
    path = Path(__file__).resolve().parents[1] / "ros2/gazebo/assets/m3_random4_overhead.png"
    image = cv2.imdecode(np.frombuffer(path.read_bytes(), np.uint8), cv2.IMREAD_COLOR)
    footprints = FixedUavLayoutDetector().detect(image)
    assert len(footprints) == 17
    cov = CoverageMap(Config())
    cov.ingest_uav_boxes(footprints, 1.0)
    cov.ingest_uav_boxes(footprints, 2.0)
    cubes = [box for box in cov.boxes.values() if cov.m3_candidate_box(box)]
    assert len(cubes) == 11, "Include seven original cubes as well as four added photo cubes"
    assert all(box.sx == box.sy == .4 for box in cubes)
    assert len(cov.pending_faces()) == 44
    assert {face.bid for face in cov.pending_faces()} == {box.bid for box in cubes}
    assert all(face.views == face.det_n == 0 for box in cubes for face in box.faces)
    assert cov.m3_counts(None) == {}, "Geometry alone must never count an M3 object"
    # The original M2 cubes and blank common cubes are still candidates.
    for cx, cy in ((1.6, 3.8), (4.52, 3.8), (1.6, 1.0), (4.52, 1.0),
                   (3.06, 4.4), (3.06, 2.5), (3.06, .6)):
        assert any(abs(box.cx - cx) < .025 and abs(box.cy - cy) < .025 for box in cubes)


@pytest.mark.parametrize("finding", ["m2_target", "empty", "m3_object"])
def test_ground_finding_resolves_one_face_without_excluding_the_cube(finding):
    cov = CoverageMap(Config())
    cov.ingest_uav_boxes([UavBox(2., 2., .4, .4)], 1.)
    cov.ingest_uav_boxes([UavBox(2., 2., .4, .4)], 2.)
    box, = cov.boxes.values()
    face = box.faces[0]
    if finding == "empty":
        face.views = cov.ec.empty_min_views
        face.view_poses = [(2.8, 1.8, 0.), (2.8, 2.2, 0.)]
        expected_state = EMPTY
    else:
        face.det_kind = "target" if finding == "m2_target" else "object"
        face.det_n = max(2, cov.cfg.perception.min_votes)
        face.det_votes = {"BLUE" if finding == "m2_target" else "AMMO_BOX": float(face.det_n)}
        expected_state = CONFIRMED
    assert cov.face_state(face) == expected_state
    assert cov.m3_candidate_box(box)
    assert {f.key for f in cov.pending_faces()} == {f.key for f in box.faces[1:]}
    if finding != "m3_object":
        assert cov.m3_counts(None) == {}


def test_reference_plot_uses_only_cube_size_and_not_referee_labels():
    assert is_cube_candidate({"sx": .4, "sy": .4})
    assert is_cube_candidate({"sx": .4, "sy": .4, "m3": False})
    assert not is_cube_candidate({"sx": .8, "sy": .4, "m3": True})
    assert not is_cube_candidate({"sx": .4, "sy": .8, "m3": True})
