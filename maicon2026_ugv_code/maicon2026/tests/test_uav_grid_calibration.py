"""Synthetic pinhole fixtures test grid mathematics, not real measurement accuracy.

Fixtures use an explicit known projective camera matrix. No four-corner fit is
used to synthesize evidence for the grid fitter or its independent checkpoints.
"""
import copy
import json

import numpy as np
import pytest

from maicon26.perception.uav_calibration import FourCornerCalibration, GridCalibration, load_calibration


ARENA_TO_IMAGE = np.array([[90., 15., 180.], [-8., -75., 600.], [.012, .008, 1.]])


def camera_pixels(arena):
    pts = np.column_stack((np.asarray(arena), np.ones(len(arena)))) @ ARENA_TO_IMAGE.T
    return pts[:, :2] / pts[:, 2:]


def fixture(noise=0., outliers=(), **options):
    arena = np.array([(x, y) for y in (.3, 1.7, 3.2, 4.7) for x in (.4, 1.7, 3.4, 5.7)])
    pixels = camera_pixels(arena)+np.random.default_rng(6).normal(0., noise, (len(arena), 2))
    for index in outliers:
        pixels[index] += [70., -55.]
    points = tuple({"pixel": p.tolist(), "arena": a.tolist()} for p, a in zip(pixels, arena))
    return GridCalibration((1280, 720), points, (3.06, 2.5, 4.), **options)


def independent_checks():
    arena = np.array([[.8, .9], [2.7, 2.5], [5.1, 4.1]])
    return tuple({"pixel": p.tolist(), "arena": a.tolist()} for p, a in zip(camera_pixels(arena), arena))


def test_exact_overdetermined_grid_fit_uses_all_sixteen_measured_points():
    cal = fixture()
    report = cal.validation_report()
    assert report["fit_point_count"] == report["fit_inlier_count"] == 16
    assert report["fit_inlier_rms_m"] < 1e-10
    assert report["fit_is_independent_validation"] is False
    assert report["status"] == "not_independently_validated"
    assert report["checkpoint_rms_m"] is None
    expected = np.array([[.8, 4.1], [4.5, .9], [3.06, 2.5]])
    assert np.allclose(cal.project(camera_pixels(expected)), expected, atol=1e-9)


def test_ransac_excludes_bad_pixel_pairs_and_validates_separate_checkpoints():
    cal = fixture(noise=.15, outliers=(1, 6, 10), checkpoints=independent_checks(), checkpoint_tolerance_m=.02)
    report = cal.validation_report()
    assert report["fit_inlier_count"] == 13
    assert np.flatnonzero(~np.array(report["fit_inlier_mask"])).tolist() == [1, 6, 10]
    assert report["fit_inlier_rms_m"] < .005 and report["fit_all_rms_m"] > .1
    assert report["independent_checkpoint_count"] == 3
    assert report["checkpoint_max_m"] < .01 and report["status"] == "within_tolerance"
    assert fixture(noise=.15, outliers=(1, 6, 10)).validation_report()["fit_inlier_mask"] == report["fit_inlier_mask"]


def test_distinct_independent_checkpoint_error_is_not_hidden_by_low_fit_residual():
    checks = list(independent_checks())
    checks[1] = {**checks[1], "arena": [3.1, 2.5]}
    cal = fixture(checkpoints=tuple(checks), checkpoint_tolerance_m=.1)
    report = cal.validation_report()
    assert report["fit_inlier_rms_m"] < 1e-10
    assert report["checkpoint_max_m"] == pytest.approx(.4)
    assert report["status"] == "outside_tolerance"
    data = cal.to_dict()
    data["validation"]["status"] = "within_tolerance"
    with pytest.raises(ValueError, match="independent checkpoints"):
        GridCalibration.from_dict(data)


def test_grid_plane_projection_keeps_floor_and_box_top_parallax_distinct():
    cal = fixture()
    floor = np.array([[1., 1.], [5., 4.]])
    pixels = camera_pixels(floor)
    c = np.array(cal.camera_center[:2])
    expected = c+(cal.camera_center[2]-.4)/cal.camera_center[2]*(floor-c)
    assert np.allclose(cal.project(pixels, .4), expected)
    assert np.max(np.linalg.norm(cal.project(pixels)-cal.project(pixels, .4), axis=1)) > .2
    assert np.allclose(cal.project(pixels, 0.), floor)


def test_roundtrip_loader_preserves_measured_points_and_four_corner_compatibility(tmp_path):
    cal = fixture(outliers=(3, 6), checkpoints=independent_checks())
    path = tmp_path/"grid.json"
    cal.save(path)
    loaded = load_calibration(path)
    assert isinstance(loaded, GridCalibration)
    assert loaded.measurements == cal.measurements
    assert loaded.validation_report()["fit_inlier_mask"] == cal.validation_report()["fit_inlier_mask"]
    legacy = FourCornerCalibration((1280, 720), ((331, 612), (949, 612), (949, 108), (331, 108)), (3.06, 2.5, 2.465))
    legacy.save(path)
    assert isinstance(load_calibration(path), FourCornerCalibration)
    path.write_text(json.dumps({"schema": "made_up"}))
    with pytest.raises(ValueError, match="Unsupported"):
        load_calibration(path)


@pytest.mark.parametrize("failure", ["few", "duplicate_pixel", "duplicate_arena", "collinear", "local_only", "different_plane", "out_of_image"])
def test_bad_grid_evidence_is_rejected_before_a_fit_can_be_used(failure):
    base = fixture()
    points = copy.deepcopy(list(base.measurements))
    if failure == "few":
        points = points[:7]
    elif failure == "duplicate_pixel":
        points[1]["pixel"] = points[0]["pixel"]
    elif failure == "duplicate_arena":
        points[1]["arena"] = points[0]["arena"]
    elif failure == "collinear":
        for i, point in enumerate(points):
            point["arena"] = [.3+i*.35, 2.]
            point["pixel"] = [200.+i*30., 400.]
    elif failure == "local_only":
        for point in points:
            point["arena"] = [v*.2+1. for v in point["arena"]]
    elif failure == "different_plane":
        points[0]["plane_z"] = .4
    elif failure == "out_of_image":
        points[0]["pixel"] = [-1., 300.]
    with pytest.raises(ValueError):
        GridCalibration(base.image_size, tuple(points), base.camera_center)


def test_too_many_outliers_and_fewer_than_eight_consistent_points_are_rejected():
    base = fixture()
    points = copy.deepcopy(list(base.measurements))
    for i in range(8):
        points[i]["pixel"] = [60.+i*55., 60.+i*i*4.]
    with pytest.raises(ValueError, match="consistent"):
        GridCalibration(base.image_size, tuple(points), base.camera_center)


def test_fit_points_cannot_double_as_independent_validation_and_optics_are_explicit():
    base = fixture()
    with pytest.raises(ValueError, match="independent checkpoint"):
        fixture(checkpoints=(copy.deepcopy(base.measurements[0]),))
    data = base.to_dict()
    data["input_optics"] = "uncorrected_wide_angle"
    with pytest.raises(ValueError, match="undistorted"):
        GridCalibration.from_dict(data)


def test_grid_rectification_preserves_marker_position_on_the_object_plane():
    cv2 = pytest.importorskip("cv2")
    cal = fixture()
    centre = np.array([4., 3.])
    p = np.r_[centre, 1.] @ np.linalg.inv(cal.plane_homography(.4)).T
    pixel = np.rint(p[:2]/p[2]).astype(int)
    image = np.zeros((720, 1280, 3), np.uint8)
    cv2.circle(image, tuple(pixel), 3, (0, 0, 255), -1)
    rectified, scale = cal.rectify_top(image)
    y, x = np.nonzero(rectified[:, :, 2])
    recovered = np.array([x.mean()/scale, cal.arena_height-y.mean()/scale])
    assert np.linalg.norm(recovered-centre) < .02
    with pytest.raises(ValueError, match="size"):
        cal.rectify_top(image[:600])
