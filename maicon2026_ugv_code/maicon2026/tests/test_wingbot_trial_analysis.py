"""Offline alignment must not compare a delayed checkpoint with latest truth."""
import importlib.util
import math
from pathlib import Path

import pytest

_spec = importlib.util.spec_from_file_location(
    "wingbot_trial_analysis", Path(__file__).resolve().parents[1] / "scripts/analyze_wingbot_trial.py")
analysis = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(analysis)


def test_matches_same_time_not_latest_truth_and_applies_declared_frame_offset():
    trace = [{"sim_s": 10., "estimated_nav_xy_yaw": [1., 2., 0.]}]
    truth = [{"sim_s": 10.02, "world_xy_yaw": [-2., -.4, 0.]},
             {"sim_s": 20., "world_xy_yaw": [1., 1., 1.]}]
    report, matches = analysis.align(trace, truth, (-3., -2.4))
    assert report["matched_samples"] == 1
    assert report["position_error_m"]["max"] == pytest.approx(0.)
    assert matches[0]["truth_sim_s"] == 10.02


def test_stale_truth_is_excluded_without_interpolation_or_extrapolation():
    trace = [{"sim_s": stamp, "estimated_nav_xy_yaw": [0., 0., 0.]} for stamp in (1., 2., 3.)]
    truth = [{"sim_s": stamp, "world_xy_yaw": [0., 0., 0.]} for stamp in (1.05, 1.94, 3.2)]
    report, matches = analysis.align(trace, truth, (0., 0.))
    assert len(matches) == 1  # The 0.05-second tolerance is inclusive.
    assert report["unmatched_sim_s"] == [2., 3.]
    assert report["matched_fraction"] == pytest.approx(1 / 3)


def test_wraps_heading_at_pi_and_reports_absolute_error_statistics():
    trace = [{"sim_s": 1., "estimated_nav_xy_yaw": [3., 4., math.pi - .01]}]
    truth = [{"sim_s": 1., "world_xy_yaw": [0., 0., -math.pi + .01]}]
    report, matches = analysis.align(trace, truth, (0., 0.))
    assert matches[0]["yaw_error_rad"] == pytest.approx(-.02)
    assert report["absolute_yaw_error_rad"]["mean"] == pytest.approx(.02)
    assert report["position_error_m"]["mean"] == report["position_error_m"]["median"] == 5.


def test_clock_reset_cannot_silently_match_another_episode():
    trace = [{"sim_s": 1., "estimated_nav_xy_yaw": [0., 0., 0.]}]
    truth = [{"sim_s": stamp, "world_xy_yaw": [0., 0., 0.]} for stamp in (2., 1.)]
    with pytest.raises(ValueError, match="backwards"):
        analysis.align(trace, truth, (0., 0.))


def test_nonfinite_pose_cannot_poison_statistics():
    trace = [{"sim_s": 1., "estimated_nav_xy_yaw": [math.nan, 0., 0.]}]
    truth = [{"sim_s": 1., "world_xy_yaw": [0., 0., 0.]}]
    with pytest.raises(ValueError, match="Nonfinite"):
        analysis.align(trace, truth, (0., 0.))
