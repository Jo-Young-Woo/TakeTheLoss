import math

import pytest

from scripts.probe_gazebo_odometry import summarize_segment


def test_turn_calibration_unwraps_yaw_and_preserves_sign_in_both_directions():
    for sign in (-1, 1):
        rows = [{"clock_s": t, "odom": [0., 0., math.atan2(math.sin(sign * yaw), math.cos(sign * yaw))],
                 "truth": [1., 2., sign * yaw / 2]} for t, yaw in enumerate((0., 1., 2., 3., 4.))]
        result = summarize_segment(rows, 0., sign * .5, .17452)
        assert result["odom_yaw_delta_rad"] == pytest.approx(sign * 4)
        assert result["truth_yaw_delta_rad"] == pytest.approx(sign * 2)
        assert result["effective_wheel_track_m"] == pytest.approx(.34904)


def test_straight_run_does_not_claim_a_turn_calibration():
    result = summarize_segment([{"clock_s": 0, "odom": [0, 0, 0], "truth": [1, 2, 0]},
                                {"clock_s": 2, "odom": [.3, 0, 0], "truth": [1.2, 2, 0]}], .12, 0., .17452)
    assert result["odom_displacement_m"] == pytest.approx(.3)
    assert result["truth_displacement_m"] == pytest.approx(.2)
    assert "effective_wheel_track_m" not in result
