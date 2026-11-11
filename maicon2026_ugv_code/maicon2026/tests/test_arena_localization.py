"""Known static geometry localizes noisy, occluded LiDAR without hidden M3 data."""
import math
import numpy as np
import pytest

from maicon26.nav.arena_localization import ArenaEdgeMap, ArenaLocalizer, fixed_infrastructure_rectangles
from maicon26.types import Pose2D, Scan
from maicon26.util import wrap
from scripts.arena_localization import interpolate_odom


@pytest.fixture(scope="module")
def edge_map():
    return ArenaEdgeMap()


def ray_scan(pose, stamp=1., occluders=(), noise=0., seed=4, fov=2.792526):
    angles = np.linspace(-fov/2, fov/2, 360)
    dx, dy = np.cos(angles+pose.th), np.sin(angles+pose.th)
    dx = np.where(abs(dx) < 1e-10, 1e-10, dx)
    dy = np.where(abs(dy) < 1e-10, 1e-10, dy)
    ranges = np.full(len(angles), 8.)
    for x0, y0, x1, y1 in ((0, 0, 6.12, 5), *fixed_infrastructure_rectangles(), *occluders):
        tx0, tx1 = (x0-pose.x)/dx, (x1-pose.x)/dx
        ty0, ty1 = (y0-pose.y)/dy, (y1-pose.y)/dy
        entry = np.maximum(np.minimum(tx0, tx1), np.minimum(ty0, ty1))
        leave = np.minimum(np.maximum(tx0, tx1), np.maximum(ty0, ty1))
        distance = np.where(entry > 0, entry, leave)
        valid = (leave > np.maximum(entry, 0)) & (distance > 0)
        ranges = np.minimum(ranges, np.where(valid, distance, 8.))
    if noise:
        ranges += np.random.default_rng(seed).normal(0, noise, len(ranges))
    return Scan(float(angles[0]), float(angles[1]-angles[0]), ranges, stamp, 8.)


@pytest.mark.parametrize("true", [Pose2D(.35, 2.5, 0), Pose2D(2.65, 3., 1.2), Pose2D(5.7, 4.5, -2.)])
def test_known_base_tracking_corrects_small_position_and_heading_errors(edge_map, true):
    start = Pose2D(true.x+.07, true.y-.06, true.th+.055)
    tracker = ArenaLocalizer(start, edge_map)
    result = tracker.update(ray_scan(true, noise=.008), Pose2D())
    assert result["valid"], result
    assert math.hypot(tracker.pose.x-true.x, tracker.pose.y-true.y) < .045
    assert abs(wrap(tracker.pose.th-true.th)) < .025
    assert result["map_source"] == "published_fixed_infrastructure_only"


def test_unknown_m3_cubes_and_noise_are_outliers_not_a_hidden_map_input(edge_map):
    true = Pose2D(2.65, 3., -.5)
    unknown = ((3.0, 3.3, 3.4, 3.7), (5.3, 2.9, 5.7, 3.3), (.4, 1.2, .8, 1.6))
    tracker = ArenaLocalizer(Pose2D(2.72, 2.94, -.44), edge_map)
    result = tracker.update(ray_scan(true, occluders=unknown, noise=.012), Pose2D())
    assert result["valid"], result
    assert math.hypot(tracker.pose.x-true.x, tracker.pose.y-true.y) < .06
    assert abs(wrap(tracker.pose.th-true.th)) < .035
    assert len(edge_map.rectangles) == 13


def test_scan_correction_prevents_accumulating_encoder_yaw_skid(edge_map):
    tracker = ArenaLocalizer(Pose2D(.75, 2.5, 0.), edge_map)
    maximum = 0.
    for i in range(65):
        true = Pose2D(.75, 2.5, .04*i)
        odom = Pose2D(0., 0., .07*i)
        result = tracker.update(ray_scan(true, stamp=1+i*.1, noise=.004, seed=i), odom)
        assert result["valid"], (i, result)
        maximum = max(maximum, abs(wrap(tracker.pose.th-true.th)))
    assert maximum < .06
    assert abs(wrap(odom.th-true.th)) > 1.8


def test_single_wall_is_degenerate_and_does_not_claim_full_pose_confidence(edge_map):
    true = Pose2D(3.06, 2.5, math.pi/2)
    tracker = ArenaLocalizer(true, edge_map)
    angles = np.linspace(-.2, .2, 80)
    scan = Scan(float(angles[0]), float(angles[1]-angles[0]), 2.5/np.cos(angles), 1., 8.)
    report = tracker.update(scan, Pose2D())
    assert not report["valid"]
    assert report["status"] == "ambiguous_or_degenerate_geometry"


def east_wall_scan(pose, stamp, noise=.003, shift=0.):
    angles = np.linspace(-.3, .3, 140)
    ranges = (6.12-pose.x)/np.cos(angles+pose.th) + shift
    ranges += np.random.default_rng(round(stamp*100)).normal(0., noise, len(angles))
    return Scan(float(angles[0]), float(angles[1]-angles[0]), ranges, stamp, 8.)


def warm_wall_tracker(edge_map):
    true = Pose2D(5.42, .75, .56)
    tracker = ArenaLocalizer(true, edge_map)
    report = tracker.update(ray_scan(true, fov=2*math.pi, noise=.003), true)
    assert report["valid"] and report["observable_dof"] == 3, report
    return tracker, true


def test_partial_wall_tracking_preserves_tangent_odometry_and_bounds_its_drift(edge_map):
    tracker, start = warm_wall_tracker(edge_map)
    initial_y = tracker.pose.y
    for i in range(1, 41):
        true = Pose2D(start.x, start.y+.019*i, start.th)
        # 5% tangent encoder error plus normal drift; only the normal is
        # observable from this wall. The tangent must not be falsely snapped.
        odom = Pose2D(start.x+.001*i, start.y+.020*i, start.th)
        report = tracker.update(east_wall_scan(true, 1+i*.1), odom)
        assert report["valid"] and report["status"] == "tracking_partial", report
        assert abs(tracker.pose.x-true.x) < .015
        assert abs(tracker.pose.y-true.y) <= report["position_uncertainty_bound_m"]
    assert tracker.pose.y-initial_y == pytest.approx(.8, abs=.015)
    assert report["dead_reckoning_duration_s"] == pytest.approx(4.)
    recovered = tracker.update(ray_scan(true, stamp=5.1, fov=2*math.pi, noise=.003), odom)
    assert recovered["valid"] and recovered["observable_dof"] == 3
    assert recovered["dead_reckoning_duration_s"] == 0.
    assert abs(tracker.pose.y-true.y) < .015


def test_partial_wall_keeps_correcting_encoder_yaw_in_its_observable_subspace(edge_map):
    tracker, true = warm_wall_tracker(edge_map)
    for i in range(1, 31):
        odom = Pose2D(true.x, true.y, true.th+.003*i)
        report = tracker.update(east_wall_scan(true, 1+i*.1), odom)
        assert report["valid"], report
        assert abs(wrap(tracker.pose.th-true.th)) < .012
    assert report["observable_dof"] == 2 and report["partial_frames"] == 30


def test_partial_tracking_stops_when_uncertainty_budget_is_exhausted(edge_map):
    tracker, start = warm_wall_tracker(edge_map)
    rejected = None
    for i in range(1, 121):
        true = Pose2D(start.x, start.y+.02*i, start.th)
        report = tracker.update(east_wall_scan(true, 1+i*.1), true)
        if not report["valid"]:
            rejected = report
            break
    assert rejected is not None
    assert rejected["status"] == "partial_uncertainty_exhausted", rejected
    assert rejected["position_uncertainty_bound_m"] > .12


def test_recent_full_lock_does_not_authorize_a_conflicting_wall_fit(edge_map):
    tracker, true = warm_wall_tracker(edge_map)
    report = tracker.update(east_wall_scan(true, 1.1, shift=.18), true)
    assert not report["valid"]
    assert report["status"] == "partial_pose_inconsistent", report
    assert abs(tracker.pose.x-true.x) < .015


def test_telemetry_jump_and_lost_ranges_fail_closed_without_pose_teleport(edge_map):
    start = Pose2D(.35, 2.5, 0.)
    tracker = ArenaLocalizer(start, edge_map)
    assert tracker.update(ray_scan(start), Pose2D())["valid"]
    before = tracker.pose.copy()
    report = tracker.update(ray_scan(start, stamp=1.1), Pose2D(4., 3., 2.))
    assert not report["valid"] and report["status"] == "odometry_jump_or_gap"
    assert math.hypot(tracker.pose.x-before.x, tracker.pose.y-before.y) < 1e-9
    empty = Scan(-1., .01, np.full(200, np.inf), 1.7, 8.)
    report = tracker.update(empty, Pose2D(4., 3., 2.))
    assert not report["valid"] and report["status"] == "insufficient_ranges"


def test_map_to_odom_transform_preserves_the_single_nav2_coordinate_frame(edge_map):
    tracker = ArenaLocalizer(Pose2D(2., 3., 1.2), edge_map)
    odom = Pose2D(1., 2., -.7)
    tf = tracker.map_to_odom(odom)
    c, s = math.cos(tf.th), math.sin(tf.th)
    assert tf.x+c*odom.x-s*odom.y == pytest.approx(tracker.pose.x)
    assert tf.y+s*odom.x+c*odom.y == pytest.approx(tracker.pose.y)
    assert wrap(tf.th+odom.th) == pytest.approx(tracker.pose.th)


def test_odom_interpolation_matches_scan_time_and_wraps_yaw():
    samples = [(1., Pose2D(0, 0, math.radians(179))), (1.1, Pose2D(.1, 0, math.radians(-179)))]
    out = interpolate_odom(samples, 1.05)
    assert out.x == pytest.approx(.05) and abs(abs(out.th)-math.pi) < 1e-9
    assert interpolate_odom(samples, 1.3) is None
