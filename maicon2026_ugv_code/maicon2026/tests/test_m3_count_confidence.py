"""Only confident current photo classifications belong in the M3 snapshot."""
import pytest

from maicon26.config import Config
from maicon26.perception.coverage import BoxModel, CoverageMap, CONFIRMED, UNCERTAIN
from maicon26.perception.object_map import Face, ObjectMap


def scene():
    cfg = Config()
    cfg.explore.complete_m3_faces = True
    cov, omap = CoverageMap(cfg), ObjectMap(cfg)
    box = BoxModel(7, 3.05, 2.525, "full", [], 1., 1.)
    cov._make_faces(box)
    cov.boxes[box.bid] = box
    return cfg, cov, omap, box


def evidence(omap, face, count, votes):
    record = Face(face.cx, face.cy, face.cx-face.nx*.2, face.cy-face.ny*.2,
                  votes=dict(votes), n=count, vx=face.nx, vy=face.ny, geometry_key=face.key)
    omap.faces.append(record)
    return record


def test_two_vote_false_pictures_on_plain_cube_are_not_reported():
    _, cov, omap, box = scene()
    evidence(omap, box.faces[1], 2, {"TRUCK": 1.8})
    evidence(omap, box.faces[0], 2, {"MISSILE": 1.8})
    cov._sync_detections(omap)
    assert cov.face_state(box.faces[1]) == cov.face_state(box.faces[0]) == UNCERTAIN
    assert cov.m3_counts(omap) == {}
    for face in box.faces:
        face.arrive_attempts = 10  # Exhausted viewpoints do not strengthen the votes.
    assert cov.m3_counts(omap) == {}


def test_four_confident_votes_count_one_face_once_despite_objectmap_fallback():
    _, cov, omap, box = scene()
    evidence(omap, box.faces[1], 4, {"TRUCK": 2.6})
    cov._sync_detections(omap)
    assert cov.face_state(box.faces[1]) == CONFIRMED
    assert cov.m3_counts(omap) == {"COMMON": {"TRUCK": 1}}


@pytest.mark.parametrize("count,votes", [(3, {"TRUCK": 2.7}), (4, {"TRUCK": 2.56})])
def test_weak_ledger_and_unclustered_objectmap_evidence_are_both_excluded(count, votes):
    _, cov, omap, box = scene()
    evidence(omap, box.faces[0], count, votes)
    cov._sync_detections(omap)
    assert cov.m3_counts(omap) == {}
    cov.boxes.clear()  # Fallback cannot use ObjectMap's looser .45 confidence rule.
    assert cov.m3_counts(omap) == {}


def test_configured_vote_count_and_margin_apply_to_ledger_and_fallback():
    cfg, cov, omap, box = scene()
    cfg.perception.min_votes = 5
    record = evidence(omap, box.faces[0], 4, {"TRUCK": 4.})
    cov._sync_detections(omap)
    assert cov.m3_counts(omap) == {}
    cfg.perception.min_votes = 4
    cfg.explore.confirm_min_margin = .6
    record.votes = {"TRUCK": 2.8, "DRUM": 1.2}  # Mean .7, but margin .4 is insufficient.
    cov._sync_detections(omap)
    assert cov.m3_counts(omap) == {}
    cov.boxes.clear()
    assert cov.m3_counts(omap) == {}


def test_full_snapshot_can_remove_an_ambiguous_class_and_accept_a_correction():
    _, cov, omap, box = scene()
    record = evidence(omap, box.faces[1], 4, {"TRUCK": 3.6})
    cov._sync_detections(omap)
    first = cov.m3_counts(omap)
    assert first == {"COMMON": {"TRUCK": 1}}

    record.n, record.votes = 8, {"TRUCK": 3.6, "DRUM": 3.6}
    cov._sync_detections(omap)
    assert cov.m3_counts(omap) == {}, "Later contradictory votes must clear the old class"
    assert first == {"COMMON": {"TRUCK": 1}}, "Prior snapshots must remain independent values"

    record.n, record.votes = 12, {"TRUCK": 3.6, "DRUM": 8.4}
    cov._sync_detections(omap)
    assert cov.m3_counts(omap) == {"COMMON": {"DRUM": 1}}
    omap.faces.clear()
    cov._sync_detections(omap)
    assert cov.m3_counts(omap) == {}


@pytest.mark.parametrize("centre", [(3., 3.3), (4.35, 1.2)])
def test_wrong_normal_legacy_records_on_full_cube_cannot_add_an_extra_object(centre):
    _, cov, omap, box = scene()
    box.cx, box.cy = centre
    cov._position_faces(box)
    south = box.faces[3]
    south.det_n, south.det_votes = 4, {"MISSILE": 3.6}
    # These are two early sightings of the same south picture, with erroneous
    # west-pointing normals. Normal-based duplicate filtering alone misses them.
    for dx, dy, nx, ny, n in [(-.1095, -.2239, -.982, -.189, 7),
                              (.0988, -.1981, -.992, -.124, 5)]:
        omap.faces.append(Face(box.cx+dx, box.cy+dy, box.cx+dx+.2, box.cy+dy,
                               votes={"MISSILE": n*.9}, n=n, vx=nx, vy=ny))
    sector = cov.box_sector(box)
    assert cov.m3_counts(omap) == {sector: {"MISSILE": 1}}
    assert len(omap.faces) == 2 and all(f.geometry_key is None for f in omap.faces)


def test_valid_picture_on_full_cube_is_deferred_until_associated_then_counted_once():
    _, cov, omap, box = scene()
    north = box.faces[1]
    record = evidence(omap, north, 4, {"DRUM": 3.6})
    record.geometry_key = None
    assert cov.m3_counts(omap) == {}
    cov._sync_detections(omap)
    assert cov.m3_counts(omap) == {"COMMON": {"DRUM": 1}}


@pytest.mark.parametrize("kind", ["face", "uav"])
def test_partial_or_aerial_only_geometry_cannot_suppress_independent_fallback(kind):
    _, cov, omap, box = scene()
    box.kind = kind
    record = evidence(omap, box.faces[1], 4, {"DRUM": 3.6})
    record.geometry_key = None
    assert cov.m3_counts(omap) == {"COMMON": {"DRUM": 1}}


def test_separate_picture_just_outside_full_cube_envelope_keeps_fallback():
    _, cov, omap, box = scene()
    # Wrong estimated centre overlaps the cube: only the measured point decides
    # whether evidence is inside its footprint, not that uncertain centre.
    omap.faces.append(Face(box.cx+box.sx/2+.051, box.cy, box.cx, box.cy,
                           votes={"DRUM": 3.6}, n=4, vx=1., vy=0.))
    assert cov.m3_counts(omap) == {"COMMON": {"DRUM": 1}}
