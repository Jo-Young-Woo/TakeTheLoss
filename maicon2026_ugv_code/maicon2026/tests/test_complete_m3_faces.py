"""Completion policy uses ground evidence, never overhead labels or truth counts."""
from maicon26.config import Config
from maicon26.link.base import UavBox
from maicon26.perception.coverage import CoverageMap, CONFIRMED, EMPTY, GIVEUP


def scene():
    cfg = Config()
    cfg.explore.complete_m3_faces = True
    cov = CoverageMap(cfg)
    cov.ingest_uav_boxes([UavBox(2., 2., .4, .4)], 1.)
    cov.ingest_uav_boxes([UavBox(2., 2., .4, .4)], 2.)
    return cov, next(iter(cov.boxes.values()))


def confirm(face, kind="object"):
    face.det_kind, face.det_n, face.det_votes = kind, 5, {"DRUM": 4.5}


def test_geometry_and_m2_do_not_identify_a_photo_cube():
    cov, box = scene()
    assert not cov.photo_box(box)
    confirm(box.faces[0], "target")
    assert not cov.photo_box(box)
    assert cov.summary()["photo_cubes"] == 0


def test_ground_picture_reopens_empty_and_exhausted_faces_until_classified():
    cov, box = scene()
    empty, exhausted = box.faces[1:3]
    empty.views = 3
    empty.view_poses = [(1., 2., 0.), (1.5, 2., 0.)]
    exhausted.arrive_attempts = 4
    assert cov.face_state(empty) == EMPTY
    assert cov.face_state(exhausted) == GIVEUP
    confirm(box.faces[0])
    assert cov.photo_box(box)
    assert {f.idx for f in cov.pending_faces()} == {1, 2, 3}
    assert cov.face_weight(empty) > 1 and cov.face_weight(exhausted) > 1
    for f in box.faces[1:]:
        confirm(f)
    assert not cov.pending_faces()
    assert all(cov.face_state(f) == CONFIRMED for f in box.faces)
    assert sum(sum(v.values()) for v in cov.m3_counts(None).values()) == 4


def test_one_occluded_normal_approach_does_not_hide_alternate_photo_viewpoints():
    cov, box = scene()
    confirm(box.faces[0])
    box.faces[1].exterior_open = False
    assert box.faces[1] in cov.pending_faces()
    # The planner still independently checks full path/visibility for every pose.
    assert cov.picture_pending(box.faces[1])
