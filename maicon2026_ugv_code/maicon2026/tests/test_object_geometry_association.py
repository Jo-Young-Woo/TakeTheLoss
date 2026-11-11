"""Camera classes attach to observed LiDAR planes, not hidden cube identities."""
from collections import defaultdict
import math

import numpy as np
import pytest

from maicon26.config import Config
from maicon26.perception.coverage import BoxModel, CoverageMap, FaceRec
from maicon26.perception.object_map import Face, ObjectMap
from maicon26.types import Detection, Frame, Pose2D, Scan


def observed_box(bid=1, cx=2., cy=2.):
    box = BoxModel(bid, cx, cy, "full", [(10, 10)], 0., 0., n_seen=3, full_seen=3)
    box.faces = [FaceRec(bid, i, nx, ny, cx+.2*nx, cy+.2*ny)
                 for i, (nx, ny) in enumerate(((1., 0.), (0., 1.), (-1., 0.), (0., -1.)))]
    return box


def box_scan(box, pose):
    angles = np.linspace(-math.pi, math.pi, 720, endpoint=False)
    dx, dy = np.cos(angles+pose.th), np.sin(angles+pose.th)
    with np.errstate(divide="ignore", invalid="ignore"):
        tx0, tx1 = (box.cx-.2-pose.x)/dx, (box.cx+.2-pose.x)/dx
        ty0, ty1 = (box.cy-.2-pose.y)/dy, (box.cy+.2-pose.y)/dy
    enter = np.maximum(np.minimum(tx0, tx1), np.minimum(ty0, ty1))
    leave = np.minimum(np.maximum(tx0, tx1), np.maximum(ty0, ty1))
    ranges = np.where((leave >= enter) & (enter > .05), enter, np.nan)
    return Scan(-math.pi, 2*math.pi/720, ranges, 1., 8.)


def image_detection(omap, face, pose, cls="DRUM", size_range_scale=1.):
    psi = math.atan2(face.cy-pose.y, face.cx-pose.x)
    distance = math.hypot(face.cx-pose.x, face.cy-pose.y)
    u = omap.cam.cx-omap.cam.f*math.tan(psi-pose.th)
    h = omap.cam.f*.4/(distance*size_range_scale*math.cos(psi-pose.th))
    return Detection(cls, .95, (u-25, 240-h/2, u+25, 240+h/2), 1.)


def test_four_oblique_face_views_keep_four_canonical_planes_and_cube_center():
    omap, box = ObjectMap(Config()), observed_box()
    poses = [Pose2D(3.2, 2.25, math.pi), Pose2D(1.75, 3.2, -math.pi/2),
             Pose2D(.8, 1.75, 0.), Pose2D(2.25, .8, math.pi/2)]
    for face, pose in zip(box.faces, poses):
        det = image_detection(omap, face, pose)
        for _ in range(5):
            omap.observe([det], pose, Frame(None, 1.), box_scan(box, pose), boxes=[box])
    confirmed = omap.confirmed_faces()
    assert len(confirmed) == 4
    assert {f.geometry_key for f in confirmed} == {(1, i) for i in range(4)}
    for face in confirmed:
        expected = box.faces[face.geometry_key[1]]
        assert (face.x, face.y, face.cx, face.cy) == pytest.approx((expected.cx, expected.cy, 2., 2.))
        assert (face.vx, face.vy) == (expected.nx, expected.ny)


def test_measured_cube_plane_overrides_wrong_photo_size_without_using_truth():
    omap, box = ObjectMap(Config()), observed_box()
    pose = Pose2D(.8, 2., 0.)
    det = image_detection(omap, box.faces[2], pose, size_range_scale=2.)
    frame, scan = Frame(None, 1.), box_scan(box, pose)
    assert abs(omap.locate(det, pose, frame, scan)[0]-box.faces[2].cx) > .8
    for _ in range(5):
        omap.observe([det], pose, frame, scan, boxes=[box])
    face = omap.confirmed_faces()[0]
    assert face.geometry_key == (1, 2) and face.x == pytest.approx(1.8)
    assert omap.association_stats["anchored"] == 5


def test_size_conflict_before_geometry_cannot_seed_a_false_distant_photo():
    omap, box = ObjectMap(Config()), observed_box()
    pose = Pose2D(.8, 2., 0.)
    det = image_detection(omap, box.faces[2], pose, size_range_scale=2.)
    omap.observe([det], pose, Frame(None, 1.), box_scan(box, pose), boxes=[])
    assert not omap.faces and omap.association_stats["size_range_conflict"] == 1


def test_conflicting_lidar_does_not_attach_size_only_vote_to_an_observed_plane():
    omap, box = ObjectMap(Config()), observed_box()
    pose = Pose2D(.8, 2., 0.)
    det = image_detection(omap, box.faces[2], pose)
    scan = box_scan(box, pose)
    scan.ranges[np.isfinite(scan.ranges)] *= .4
    omap.observe([det], pose, Frame(None, 1.), scan, boxes=[box])
    assert not omap.faces and omap.association_stats["range_rejected"] == 1


def test_ambiguous_corner_without_normal_support_is_not_assigned_to_either_face():
    omap, box = ObjectMap(Config()), observed_box()
    pose = Pose2D(3., 3., -3*math.pi/4)
    distance = math.hypot(.8, .8)
    anchor, reason = omap._geometry_anchor([box], pose, pose.th, distance, None)
    assert anchor is None and reason == "ambiguous"


def test_first_two_votes_on_adjacent_planes_cannot_merge_when_normals_are_measured():
    omap = ObjectMap(Config())
    det = Detection("DRUM", .95, (100, 100, 150, 150), 1.)
    omap._add(det, 2.12, 2., math.pi, 1., Pose2D(3., 2.), nrm=(1., 0.))
    omap._add(det, 2., 2.12, -math.pi/2, 1.1, Pose2D(2., 3.), nrm=(0., 1.))
    assert len(omap.faces) == 2


def test_anchoring_adopts_compatible_history_once_and_preserves_adjacent_photo():
    omap, box = ObjectMap(Config()), observed_box()
    omap.faces = [Face(1.86, 2.0, 2.06, 2., defaultdict(float, DRUM=4.), 4, vx=-1.),
                  Face(1.92, 2.0, 2.12, 2., defaultdict(float, DRUM=4.), 4, vx=-1.),
                  Face(2., 2.2, 2., 2., defaultdict(float, AMMO_BOX=4.), 4, vy=1.)]
    pose = Pose2D(.8, 2., 0.)
    det = image_detection(omap, box.faces[2], pose)
    for _ in range(2):
        omap.observe([det], pose, Frame(None, 1.), box_scan(box, pose), boxes=[box])
    assert len(omap.faces) == 2
    anchored = next(f for f in omap.faces if f.geometry_key is not None)
    assert anchored.n == 10 and anchored.votes["DRUM"] == pytest.approx(9.9)
    assert next(f for f in omap.faces if f.geometry_key is None).votes["AMMO_BOX"] == 4.


def test_coverage_honors_measured_plane_key_instead_of_closer_neighbor_and_counts_once():
    cfg, box, neighbor = Config(), observed_box(), observed_box(2, cx=2.1)
    cov, omap = CoverageMap(cfg), ObjectMap(cfg)
    cov.boxes = {1: box, 2: neighbor}
    omap.faces = [Face(1.89, 2., 2., 2., defaultdict(float, DRUM=5.), 5,
                      vx=-1., geometry_key=(1, 2))]
    cov._sync_detections(omap)
    assert box.faces[2].det_n == 5 and neighbor.faces[2].det_n == 0
    assert sum(sum(classes.values()) for classes in cov.m3_counts(omap).values()) == 1


def test_default_call_without_geometry_keeps_hardware_size_range_fallback():
    omap = ObjectMap(Config())
    det = Detection("DRUM", .95, (280, 160, 360, 300), 1.)
    omap.observe([det], Pose2D(1., 2.), Frame(None, 1.), None)
    assert len(omap.faces) == 1 and omap.faces[0].geometry_key is None


def test_exact_plane_votes_cannot_be_outvoted_by_adjacent_legacy_viewing_normal():
    cfg, box = Config(), observed_box()
    cov, omap = CoverageMap(cfg), ObjectMap(cfg)
    cov.boxes = {1: box}
    primary = Face(2., 2.2, 2., 2., defaultdict(float, TRUCK=7.6), 8,
                   vy=1., geometry_key=(1, 1))
    # A real point near the east edge with a legacy viewing-direction "normal"
    # looks north-facing. Nearest-point association puts its DRUM votes on the
    # north face and previously changed TRUCK to DRUM despite exact geometry.
    legacy = Face(2.18, 2.12, 2.18, 1.92, defaultdict(float, DRUM=26.), 30, vy=1.)
    east = Face(2.2, 2., 2., 2., defaultdict(float, DRUM=4.5), 5, vx=1.)
    omap.faces = [legacy, primary, east]  # Priority must not depend on list order.
    cov._sync_detections(omap)
    assert box.faces[1].det_n == 8 and box.faces[1].det_votes == {"TRUCK": 7.6}
    assert box.faces[0].det_n == 5 and box.faces[0].det_votes == {"DRUM": 4.5}
    assert omap.faces == [legacy, primary, east]  # Independent history is retained.


def test_legacy_spatial_votes_remain_available_without_a_confirmed_plane_anchor():
    cfg, box = Config(), observed_box()
    cov, omap = CoverageMap(cfg), ObjectMap(cfg)
    cov.boxes = {1: box}
    legacy = Face(2.18, 2.12, 2.18, 1.92, defaultdict(float, DRUM=26.), 30, vy=1.)
    omap.faces = [legacy]
    cov._sync_detections(omap)
    assert box.faces[1].det_n == 30 and box.faces[1].det_votes == {"DRUM": 26.}
    omap.faces.append(Face(2., 2.2, 2., 2., defaultdict(float, TRUCK=.8), 1,
                          vy=1., geometry_key=(1, 1)))
    cov._sync_detections(omap)
    assert box.faces[1].det_n == 31  # One new vote cannot hide established evidence.
