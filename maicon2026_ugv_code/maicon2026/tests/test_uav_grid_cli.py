"""Synthetic unit-test geometry only; the CLI itself never invents fit points."""
import json

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")

from maicon26.perception.uav_calibration import GridCalibration, load_calibration
from scripts.calibrate_uav_grid import _measurement_label, arena_to_pixels, main, metre_grid, render_floor_view, render_overlay


def fixture_data():
    # Independently specified projective camera model for numerical tests.
    inverse = np.array([[120., 8., 100.], [10., -120., 700.], [.012, .01, 1.]])

    def pixel(xy):
        p = inverse @ np.array([*xy, 1.])
        return (p[:2]/p[2]).tolist()

    points = [{"pixel": pixel((x, y)), "arena": [x, y]}
              for x in (.5, 3., 5.6) for y in (.5, 2.5, 4.5)]
    bad_xy = [1.6, 3.8]
    points.append({"pixel": (np.asarray(pixel(bad_xy))+[50., -40.]).tolist(), "arena": bad_xy})
    checks = [{"pixel": (np.asarray(pixel(xy))+[.4, -.25]).tolist(), "arena": list(xy)}
              for xy in ((1.3, 1.4), (4.4, 3.6))]
    return points, checks, pixel


def calibration():
    points, checks, _ = fixture_data()
    return GridCalibration(image_size=(1000, 800), measurements=tuple(points),
                           camera_center=(3.06, 2.5, 3.), checkpoints=tuple(checks))


def input_files(tmp_path):
    points, checks, _ = fixture_data()
    image = tmp_path / "측정 영상.png"
    ok, data = cv2.imencode(".png", np.full((800, 1000, 3), 35, dtype=np.uint8))
    assert ok
    image.write_bytes(data.tobytes())
    fit, holdout = tmp_path / "points.json", tmp_path / "checkpoints.json"
    fit.write_text(json.dumps(points), encoding="utf-8")
    holdout.write_text(json.dumps(checks), encoding="utf-8")
    output, overlay, bird = tmp_path / "camera.json", tmp_path / "overlay.png", tmp_path / "floor.png"
    args = ["--image", str(image), "--points", str(fit), "--checkpoints", str(holdout),
            "--camera-center", "3.06", "2.5", "3", "--output", str(output), "--overlay", str(overlay),
            "--birdseye", str(bird)]
    return args, (image, fit, holdout, output, overlay, bird)


def test_overlay_grid_is_inverse_projection_and_does_not_add_measurements():
    cal = calibration()
    before = json.dumps(cal.to_dict(), sort_keys=True)
    for _, _, xy in metre_grid(cal, .5):
        pixels = arena_to_pixels(cal, xy)
        assert np.allclose(cal.project(pixels, require_inside=False), xy, atol=1e-8)
    source = np.full((800, 1000, 3), 35, dtype=np.uint8)
    overlay = render_overlay(source, cal)
    assert np.all(source == 35), "Never modify the actual input image"
    assert overlay.shape == (948, 1000, 3)
    assert json.dumps(cal.to_dict(), sort_keys=True) == before
    assert cal.validation_report()["fit_inlier_mask"] == [True]*9+[False]

    for point, color in ((cal.measurements[0]["pixel"], (80, 230, 80)),
                         (cal.measurements[-1]["pixel"], (60, 70, 255)),
                         (cal.checkpoints[0]["pixel"], (230, 100, 230))):
        x, y = np.rint(point).astype(int)
        patch = overlay[y-7:y+8, x-7:x+8]
        assert np.any(np.all(patch == color, axis=2)), "Every measured category must retain a visible marker"


def test_short_ids_for_nearby_measured_points_do_not_overlap():
    image = np.zeros((200, 400, 3), dtype=np.uint8)
    points = [(150, 100), (170, 100), (190, 100)]
    occupied = [(x-8, y-8, x+8, y+8) for x, y in points]
    boxes = [_measurement_label(image, f"F{i+1:02d}", point, (80, 230, 80), occupied)
             for i, point in enumerate(points)]
    for i, a in enumerate(boxes):
        for b in boxes[i+1:]:
            assert min(a[2], b[2]) <= max(a[0], b[0]) or min(a[3], b[3]) <= max(a[1], b[1])


def test_floor_view_projects_actual_pixels_to_metres_without_synthetic_scene_content():
    cal = calibration()
    _, _, pixel = fixture_data()
    xy = (1.3, 1.4)
    source = np.zeros((800, 1000, 3), dtype=np.uint8)
    cv2.circle(source, tuple(np.rint(pixel(xy)).astype(int)), 5, (0, 0, 255), -1)
    floor = render_floor_view(source, cal, pixels_per_metre=100.)
    assert floor.shape == (501, 613, 3)
    x, y = round(xy[0]*100), round((5.-xy[1])*100)
    assert floor[y-3:y+4, x-3:x+4, 2].max() > 220
    assert np.max(floor[y-2:y+3, x-2:x+3, :2]) < 40


def test_cli_saves_original_measured_pairs_holdout_report_and_diagnostic_images(tmp_path, capsys):
    args, (_, fit, _, output, overlay, bird) = input_files(tmp_path)
    assert main(args) == 0
    data = json.loads(output.read_text(encoding="utf-8"))
    assert data["schema"] == "maicon_uav_grid_v1"
    assert data["measurements"] == json.loads(fit.read_text(encoding="utf-8"))
    cal = load_calibration(output)
    assert cal.validation_report()["independent_checkpoint_count"] == 2
    assert cal.validation_report()["fit_inlier_count"] == 9
    assert cal.validation_report()["status"] == "within_tolerance"
    assert cv2.imdecode(np.frombuffer(overlay.read_bytes(), np.uint8), cv2.IMREAD_COLOR).shape == (948, 1000, 3)
    assert bird.exists()
    printed = json.loads(capsys.readouterr().out)
    assert printed["calibration_saved"] is True and printed["overlay_grid_is_measurement_evidence"] is False


def test_reusing_fit_point_as_holdout_is_rejected_without_publishing_calibration(tmp_path):
    args, (_, fit, holdout, output, overlay, _) = input_files(tmp_path)
    holdout.write_text(json.dumps([json.loads(fit.read_text())[0]]))
    with pytest.raises(SystemExit) as error:
        main(args)
    assert error.value.code == 2 and not output.exists() and not overlay.exists()


def test_failed_independent_checks_save_diagnostics_but_not_usable_calibration(tmp_path, capsys):
    args, (_, _, holdout, output, overlay, bird) = input_files(tmp_path)
    checks = json.loads(holdout.read_text())
    checks[0]["arena"][0] += .3
    holdout.write_text(json.dumps(checks))
    with pytest.raises(SystemExit) as error:
        main(args)
    assert error.value.code == 2 and not output.exists()
    assert overlay.exists() and bird.exists()
    report = json.loads(capsys.readouterr().out)
    assert report["calibration_saved"] is False and report["status"] == "outside_tolerance"


def test_cli_refuses_overwrite_and_never_replaces_input_files(tmp_path):
    args, (image, _, _, output, _, _) = input_files(tmp_path)
    original = image.read_bytes()
    assert main(args) == 0
    saved = output.read_bytes()
    with pytest.raises(SystemExit):
        main(args)
    assert output.read_bytes() == saved
    bad = list(args)
    bad[bad.index("--output")+1] = str(image)
    with pytest.raises(SystemExit):
        main(bad+["--overwrite"])
    assert image.read_bytes() == original


def test_cli_requires_eight_measured_pairs_not_a_generated_four_corner_grid(tmp_path):
    args, (_, fit, _, output, overlay, _) = input_files(tmp_path)
    fit.write_text(json.dumps(json.loads(fit.read_text())[:7]))
    with pytest.raises(SystemExit) as error:
        main(args)
    assert error.value.code == 2 and not output.exists() and not overlay.exists()
