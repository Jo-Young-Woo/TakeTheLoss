"""Delayed UAV self observations must never become a physical threat halo."""
import math

from maicon26.config import Config
from maicon26.link.base import UavDet, UavFrame
from maicon26.mission.threat_field import build_field, body_hit
from maicon26.nav.grid import OccupancyGrid
from maicon26.perception.threat_board import ThreatBoard
from maicon26.perception.tracks import UgvTracker
from maicon26.types import Pose2D


def test_capture_time_registered_self_pose_handles_motion_delay_and_drift():
    cfg = Config()
    tracker = UgvTracker(cfg, "BLUE")
    speed, delay, drift = 0.38, 0.4, 0.25
    for stamp in range(1, 6):
        x = 0.35 + speed * stamp
        arrival_slam_x = x + speed * delay + drift
        assert arrival_slam_x - x > 0.30  # The former arrival-time gate leaks self.
        captured_slam = Pose2D(x + drift, 2.5, 0.0)
        registered_capture = Pose2D(captured_slam.x - drift, 2.5, 0.0)
        frame = UavFrame(float(stamp), detections=[UavDet(x, 2.5, "BLUE"), UavDet(5.25, 4.16, "RED")])
        tracker.update_uav(frame, registered_capture, self_ident="B-UGV-2")
    assert tracker.self_filtered == 5
    assert len(tracker.tracks) == 1 and tracker.tracks[0].team == "RED"
    pose = Pose2D(0.35 + speed * 5, 2.5, 0.0)
    tracks = ThreatBoard(cfg, "BLUE").update(tracker, [], 5.4, pose)
    grid = OccupancyGrid(cfg.arena)
    field = build_field(tracks, grid, cfg.avoid, cfg.laser.max_range, 5.4, cfg.nav.inflate_hard)
    assert not body_hit(field, grid, pose.x, pose.y)


def test_relative_self_association_survives_gradual_unregistered_slam_drift():
    tracker = UgvTracker(Config(), "BLUE")
    for stamp in range(13):
        x = 0.4 + 0.2 * stamp
        drift = 0.025 * stamp
        tracker.update_uav(UavFrame(float(stamp), detections=[UavDet(x, 2.5, "BLUE")]),
                           Pose2D(x + drift, 2.5, 0.0))
    assert not tracker.tracks
    assert tracker.self_filtered == 13


def test_only_single_self_detection_is_removed_nearby_friend_and_enemy_remain():
    tracker = UgvTracker(Config(), "BLUE")
    frame = UavFrame(1.0, detections=[UavDet(2.0, 2.0, "BLUE"),
                                     UavDet(2.28, 2.0, "BLUE", "B-UGV-1"),
                                     UavDet(2.0, 2.2, "RED", "R-UGV-1")])
    tracker.update_uav(frame, Pose2D(2.0, 2.0, 0.0), self_ident="B-UGV-2")
    # Identity hints must not merge nearby opposing robots into one track.
    assert tracker.self_filtered == 1
    assert {tr.ident for tr in tracker.tracks} == {"B-UGV-1", "R-UGV-1"}
    assert all(tr.last_uav_t == 1.0 and math.hypot(tr.x - 2.0, tr.y - 2.0) > 0 for tr in tracker.tracks)


def test_reacquired_self_removes_old_uav_only_ghost_without_removing_camera_track():
    cfg = Config()
    tracker = UgvTracker(cfg, "BLUE")
    own = tracker._upsert(1.0, 2.0, 0.0, "BLUE", 1.0, "uav")
    camera = tracker._upsert(2.0, 2.0, 0.0, "RED", 1.0, "cam", "R-UGV-1")
    tracker.update_uav(UavFrame(1.0, detections=[UavDet(1.1, 2.0, "BLUE")]),
                       Pose2D(1.1, 2.0, 0.0), self_ident="B-UGV-2")
    assert tracker.self_pruned == 1
    assert own not in tracker.tracks and camera in tracker.tracks
    tracker.update_uav(UavFrame(2.0, detections=[UavDet(1.2, 2.0, "BLUE")]),
                       Pose2D(1.2, 2.0, 0.0), self_ident="B-UGV-2")
    assert len(tracker.tracks) == 1


def test_alignment_jump_reacquires_precise_capture_pose_and_preserves_other_blue():
    tracker = UgvTracker(Config(), "BLUE")
    for stamp in range(19):
        tracker.update_uav(UavFrame(float(stamp), detections=[UavDet(2.0, 2.5, "BLUE")]),
                           Pose2D(2.0 + stamp * 0.025, 2.5, 0.0))
    # The established pose reference has .45m drift; registration now fixes it.
    tracker.update_uav(UavFrame(19.0, detections=[UavDet(2.0, 2.5, "BLUE"),
                                                UavDet(2.35, 2.5, "BLUE", "B-UGV-1")]),
                       Pose2D(2.0, 2.5, 0.0), self_ident="B-UGV-2")
    assert tracker.self_reacquired == 1
    assert tracker.self_filtered == 20
    assert {tr.ident for tr in tracker.tracks} == {"B-UGV-1"}
