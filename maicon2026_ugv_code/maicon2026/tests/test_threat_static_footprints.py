"""Confirmed solid interiors contradict only LiDAR-only moving-object ghosts."""
from types import SimpleNamespace

import pytest

from maicon26.config import Config
from maicon26.mission.threat_field import build_field
from maicon26.nav.grid import OccupancyGrid
from maicon26.perception.lidar_blobs import Blob
from maicon26.perception.threat_board import ThreatBoard
from maicon26.perception.tracks import UgvTracker
from maicon26.types import Pose2D


def structure():
    return SimpleNamespace(cx=1.6, cy=1.0, sx=.4, sy=.4)


def blob(ident, x, y=1.008, t=.5):
    return Blob(ident, x, y, .2, .0, .2, 4, t, .2)


def test_confirmed_structure_removes_stale_lidar_ghost_before_uncertainty_expansion():
    cfg = Config()
    board = ThreatBoard(cfg, "BLUE")
    pose = Pose2D(1.5, 2.0, 0.0)
    tracks = board.update(None, [blob(54, 1.693)], .5, pose)
    assert len(tracks) == 1 and tracks[0].from_blob
    # Reproduce the observed five-second ghost: its expanded body could cover
    # metres of otherwise empty arena despite no real opponent being present.
    assert tracks[0].predict(5.976, 0.0, cfg.avoid)[2] > 2.0
    tracks = board.update(None, [], 5.976, pose, static_boxes=[structure()])
    assert tracks == []
    assert board._blob_tracks == {}
    assert board.stats["static_ghosts_cleared"] == 1
    assert build_field(tracks, OccupancyGrid(cfg.arena), cfg.avoid,
                       cfg.laser.max_range, 5.976, cfg.nav.inflate_hard) is None


def test_only_strict_interior_blobs_are_suppressed():
    cfg = Config()
    pose = Pose2D(.4, 2.5, 0.0)
    for x, suppressed in [(1.693, True), (1.75, False), (1.79, False), (1.90, False)]:
        board = ThreatBoard(cfg, "BLUE")
        result = board.update(None, [blob(1, x)], .5, pose, static_boxes=[structure()])
        assert (len(result) == 0) == suppressed


@pytest.mark.parametrize("source", ["cam", "uav"])
def test_confirmed_ugv_and_its_lidar_association_survive_even_inside_structure(source):
    cfg = Config()
    tracker = UgvTracker(cfg, "BLUE")
    tracker._upsert(1.67, 1.01, .5, "RED", 1.0, source)
    board = ThreatBoard(cfg, "BLUE")
    tracks = board.update(tracker, [blob(54, 1.693)], .5, Pose2D(.4, 2.5, 0.0),
                           static_boxes=[structure()])
    assert len(tracks) == 1 and not tracks[0].from_blob
    assert tracks[0].t_src[source] == .5 and tracks[0].t_src["lidar"] == .5


def test_no_confirmed_footprints_preserves_existing_blob_behavior():
    board = ThreatBoard(Config(), "BLUE")
    result = board.update(None, [blob(54, 1.693)], .5, Pose2D(.4, 2.5, 0.0))
    assert len(result) == 1 and result[0].from_blob


def test_inside_phantom_cannot_move_a_nearby_outside_blob_into_the_structure():
    board = ThreatBoard(Config(), "BLUE")
    result = board.update(None, [blob(1, 1.90), blob(2, 1.693)], .5,
                           Pose2D(.4, 2.5, 0.0), static_boxes=[structure()])
    assert len(result) == 1 and result[0].x == 1.90
    assert board.stats["static_blobs_suppressed"] == 1
