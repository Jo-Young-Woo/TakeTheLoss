"""Ground images can corroborate solid structures when UAV images are absent."""
from types import SimpleNamespace

import pytest

from maicon26.config import Config
from maicon26.nav.grid import OccupancyGrid
from maicon26.perception.coverage import BoxModel, CoverageMap
from maicon26.perception.object_map import Face
from maicon26.perception.lidar_blobs import Blob
from maicon26.perception.threat_board import ThreatBoard
from maicon26.perception.tracks import UgvTracker
from maicon26.types import Pose2D


def scene(kind="object"):
    cfg = Config()
    cfg.explore.complete_m3_faces = True
    cov = CoverageMap(cfg)
    box = BoxModel(1, 2., 2., "full", [], 1., 10., n_seen=5,
                   full_seen=3, last_full_t=10.)
    cov._make_faces(box)
    cov.boxes[1] = box
    box.faces[0].views, box.faces[0].last_view_t = 2, 10.
    cls = cfg.perception.object_classes[0] if kind == "object" else cfg.perception.target_classes[0]
    face = Face(2.2, 2., 2., 2., votes={cls: 3.6}, n=4, kind=kind,
                last_t=10., vx=1., vy=0.)
    return cfg, cov, box, face, SimpleNamespace(faces=[face])


@pytest.mark.parametrize("kind", ["object", "target"])
def test_recent_ground_class_and_repeated_full_lidar_qualify_without_uav(kind):
    cfg, cov, box, _, omap = scene(kind)
    assert box.uav_t is None
    assert cov.confirmed_static_footprints(10.1, omap) == [box]
    # Diagnostics use the same observations, not an independent invented truth.
    cov._last_refresh = 10.1
    assert cov.summary()["confirmed_static_boxes"] == 1
    assert cov.confirmed_static_footprints(16) == [], "Cached observations cannot renew themselves"
    cfg.explore.complete_m3_faces = False
    assert cov.confirmed_static_footprints(10.1, omap) == [], "Generic scenario contract remains unchanged"


@pytest.mark.parametrize("case", ["old_full", "partial", "few_full", "missing", "clearing",
                                 "suspect", "old_photo", "old_view", "few_views", "weak",
                                 "ambiguous", "wrong_class", "normal", "point"])
def test_uncertain_or_stale_ground_evidence_cannot_suppress_threats(case):
    cfg, cov, box, face, omap = scene()
    if case == "old_full":
        box.last_full_t = 7.; box.t_last = 10.
    elif case == "partial": box.kind = "face"
    elif case == "few_full": box.full_seen = 2
    elif case == "missing": box.miss_t = 10.
    elif case == "clearing": box.free_checks = 1
    elif case == "suspect": box.faces[0].suspect = True
    elif case == "old_photo": face.last_t = 4.
    elif case == "old_view": box.faces[0].last_view_t = 4.
    elif case == "few_views": box.faces[0].views = 1
    elif case == "weak": face.votes = {next(iter(face.votes)): 2.6}
    elif case == "ambiguous": face.votes["other"] = 3.
    elif case == "wrong_class": face.votes = {"RED_UGV": 3.6}
    elif case == "normal": face.vx, face.vy = 0., 1.
    elif case == "point": face.x += .121
    assert cov.confirmed_static_footprints(10.1, omap) == []


def test_ground_qualification_only_removes_interior_lidar_ghosts():
    cfg, cov, _, _, omap = scene()
    solid = cov.confirmed_static_footprints(10.1, omap)
    inside = Blob(1, 2.05, 2.01, .1, 0., .2, 4, 10., .1)
    outside = Blob(2, 2.3, 2.01, .1, 0., .2, 4, 10., .1)
    board = ThreatBoard(cfg, "BLUE")
    tracks = board.update(None, [inside, outside], 10.1, Pose2D(1, 2, 0), static_boxes=solid)
    assert len(tracks) == 1 and tracks[0].x == 2.3
    tracker = UgvTracker(cfg, "BLUE")
    tracker._upsert(2.05, 2.01, 10., "RED", 1., "cam")
    tracks = ThreatBoard(cfg, "BLUE").update(tracker, [inside], 10.1, Pose2D(1, 2, 0), static_boxes=solid)
    assert len(tracks) == 1 and not tracks[0].from_blob


def test_partial_refresh_does_not_renew_full_lidar_evidence(monkeypatch):
    cfg, cov, box, _, omap = scene()
    grid = OccupancyGrid(cfg.arena)
    monkeypatch.setattr(cov, "_clusters", lambda _: [("face", 2., 2., [], .4, .4)])
    cov.refresh(grid, omap, 13., force=True)
    assert box.t_last == 13. and box.last_full_t == 10.
    assert cov.confirmed_static_footprints(13., omap) == []
