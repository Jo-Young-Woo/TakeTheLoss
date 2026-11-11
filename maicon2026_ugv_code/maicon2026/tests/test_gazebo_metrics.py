"""Gazebo distance and final pose must not contain referee prediction motion."""
from types import SimpleNamespace

import pytest

from maicon26.types import Pose2D
from scripts.run_gazebo_match import sync_pose_measurements


def test_distance_uses_measured_path_when_referee_predicts_forward():
    ugvs = {"B2": SimpleNamespace(pose=Pose2D())}
    distances = {"B2": 0.0}
    previous = {"B2": Pose2D()}
    # Gazebo reports 0.3 m each sample; the referee also predicts 0.2 m.
    # Aliasing those poses used to count only the residual 0.1 m next time.
    for x in (.3, .6, .9):
        truth = {"B2": Pose2D(x, 0.0, 0.0)}
        previous = sync_pose_measurements(ugvs, truth, previous, distances)
        ugvs["B2"].pose.x += .2
        ugvs["B2"].pose.th = .5
        assert truth["B2"].x == x and truth["B2"].th == 0.0
        assert previous["B2"].x == x and previous["B2"].th == 0.0
    assert distances["B2"] == pytest.approx(.9)


def test_final_snapshot_replaces_prediction_and_counts_final_displacement():
    ugvs = {"B2": SimpleNamespace(pose=Pose2D(.8, .1, .4)),
            "R1": SimpleNamespace(pose=Pose2D(5.15, 1.0, 3.14))}
    distances = {"B2": .6, "R1": 0.0}
    previous = {"B2": Pose2D(.6, 0.0, 0.0)}
    final_truth = {"B2": Pose2D(.7, 0.0, .2)}
    sync_pose_measurements(ugvs, final_truth, previous, distances)
    assert vars(ugvs["B2"].pose) == vars(final_truth["B2"])
    assert distances == pytest.approx({"B2": .7, "R1": 0.0})
    # Virtual opponents have no Gazebo sample and retain their referee pose.
    assert vars(ugvs["R1"].pose) == {"x": 5.15, "y": 1.0, "th": 3.14}
    ugvs["B2"].pose.x = 10.0
    assert final_truth["B2"].x == .7


def test_existing_jump_filter_and_independent_robot_distances_are_preserved():
    ugvs = {i: SimpleNamespace(pose=Pose2D()) for i in ("B2", "R1", "B1")}
    distances = {i: 0.0 for i in ugvs}
    previous = {"B2": Pose2D(), "R1": Pose2D()}
    truth = {"B2": Pose2D(2.0, 0.0, 0.0), "R1": Pose2D(.3, .4, 0.0),
             "B1": Pose2D(4.0, 2.0, 0.0)}
    previous = sync_pose_measurements(ugvs, truth, previous, distances)
    assert distances == pytest.approx({"B2": 0.0, "R1": .5, "B1": 0.0})
    assert previous["B2"].x == 2.0  # New baseline after the rejected jump.
    truth["B2"].x = 9.0
    assert previous["B2"].x == 2.0 and ugvs["B2"].pose.x == 2.0
