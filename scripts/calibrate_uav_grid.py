#!/usr/bin/env python3
"""Fit measured UAV grid correspondences and render an inspectable overlay.

Input must be an already lens-undistorted pinhole image. --points is a JSON
list of at least eight measured {"pixel": [u,v], "arena": [x,y]} pairs on the
reference plane. --checkpoints is a separate, nonempty held-out measurement
list with optional plane_z. Never derive either list from an existing fit.

The rendered metre grid is an inverse projection of the fitted homography,
NOT additional calibration evidence. Circle = measured pixel; cross = model
prediction; arrow = residual. Floor rectification does not remove box-height
parallax. Camera motion, changed crop/resolution or changed optics require a
new calibration and independent checks.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from maicon26.perception.uav_calibration import GridCalibration

INLIER = (80, 230, 80)
OUTLIER = (60, 70, 255)
CHECKPOINT = (230, 100, 230)
GRID = (220, 190, 60)


def arena_to_pixels(calibration, points, plane_z=None):
    """Inverse projection for annotation only; never creates fitting points."""
    xy = np.asarray(points, dtype=float)
    if xy.ndim != 2 or xy.shape[1] != 2 or not np.isfinite(xy).all():
        raise ValueError("Overlay arena points must be finite XY pairs")
    z = calibration.reference_plane_z if plane_z is None else plane_z
    inverse = np.linalg.inv(calibration.plane_homography(z))
    hom = np.column_stack([xy, np.ones(len(xy))]) @ inverse.T
    if np.any(np.abs(hom[:, 2]) < 1e-10):
        raise ValueError("Overlay point projects onto the image horizon")
    pixels = hom[:, :2] / hom[:, 2:]
    if not np.isfinite(pixels).all():
        raise ValueError("Overlay projection is not finite")
    return pixels


def metre_grid(calibration, step):
    """Return labelled floor lines, including exact arena edges."""
    if not math.isfinite(step) or step < .1:
        raise ValueError("Grid spacing must be finite and at least 0.1 metre")
    width, height = calibration.arena_width, calibration.arena_height
    for axis, bound, other in (("x", width, height), ("y", height, width)):
        values = list(np.arange(0., bound, step)) + [bound]
        for value in values:
            points = [[value, 0.], [value, other]] if axis == "x" else [[0., value], [other, value]]
            yield axis, float(value), np.asarray(points)


def _point(pixel):
    # Keep malformed/horizon geometry out of OpenCV's fixed-width coordinates.
    return tuple(int(round(float(v))) for v in np.clip(pixel, -1e6, 1e6))


def _line(image, start, end, color, thickness=1, arrow=False):
    import cv2
    h, w = image.shape[:2]
    inside, a, b = cv2.clipLine((0, 0, w, h), _point(start), _point(end))
    if inside:
        if arrow:
            cv2.arrowedLine(image, a, b, color, thickness, cv2.LINE_AA, tipLength=.25)
        else:
            cv2.line(image, a, b, color, thickness, cv2.LINE_AA)


def _label(image, text, point, color, scale=.43):
    import cv2
    h, w = image.shape[:2]
    (tw, th), baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, 1)
    x = min(max(int(point[0]), 2), max(2, w-tw-3))
    y = min(max(int(point[1]), th+3), h-baseline-3)
    cv2.putText(image, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, (15, 15, 15), 3, cv2.LINE_AA)
    cv2.putText(image, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)


def _measurement_label(image, text, point, color, occupied):
    """Place short IDs clear of other point markers and previously drawn IDs."""
    import cv2
    (width, height), baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, .43, 1)
    options = []
    for dx, dy in ((10, -9), (10, 22), (-width-10, -9), (-width-10, 22),
                   (10, -27), (10, 40), (-width-10, -27), (-width-10, 40)):
        x = min(max(round(point[0]+dx), 3), image.shape[1]-width-3)
        y = min(max(round(point[1]+dy), height+3), image.shape[0]-baseline-3)
        box = (x-2, y-height-2, x+width+2, y+baseline+2)
        overlap = sum(max(0, min(box[2], b[2])-max(box[0], b[0])) *
                      max(0, min(box[3], b[3])-max(box[1], b[1])) for b in occupied)
        options.append((overlap, x, y, box))
        if overlap == 0:
            break
    _, x, y, box = min(options, key=lambda option: option[0])
    _label(image, text, (x, y), color)
    occupied.append(box)
    return box


def render_overlay(bgr, calibration, grid_step=.5):
    """Draw residuals on a copy of the actual image; preserve its pixel scale."""
    import cv2
    if bgr.shape[:2] != (calibration.image_size[1], calibration.image_size[0]):
        raise ValueError("Overlay image size differs from the calibration")
    image = bgr.copy()
    for axis, value, endpoints in metre_grid(calibration, grid_step):
        pixels = arena_to_pixels(calibration, endpoints)
        _line(image, *pixels, GRID)
        # Keep half-metre lines, but label whole metres outside the arena.
        # Avoid the near-coincident x=6 / x=6.12 and the two origin labels.
        bound = calibration.arena_width if axis == "x" else calibration.arena_height
        edge = math.isclose(value, bound)
        integer = math.isclose(value, round(value))
        if ((edge or (integer and bound-value >= .5)) and (axis == "x" or value > 0)):
            anchor = pixels[0] if axis == "x" else pixels[1]
            text = f"{axis}={value:g}m"
            if axis == "x":
                (tw, _), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, .36, 1)
                offset = [-tw/2, 17]
            else:
                offset = [9, 4]
            _label(image, text, anchor+offset, GRID, .36)
    report = calibration.validation_report()
    groups = [("F", calibration.measurements, report["fit_inlier_mask"]),
              ("C", calibration.checkpoints, None)]
    occupied = [(item["pixel"][0]-8, item["pixel"][1]-8, item["pixel"][0]+8, item["pixel"][1]+8)
                for _, measurements, _ in groups for item in measurements]
    for prefix, measurements, inliers in groups:
        for index, item in enumerate(measurements):
            pixel = np.asarray(item["pixel"], dtype=float)
            plane = float(item.get("plane_z", calibration.reference_plane_z))
            predicted = arena_to_pixels(calibration, [item["arena"]], plane)[0]
            color = CHECKPOINT if inliers is None else INLIER if inliers[index] else OUTLIER
            _line(image, pixel, predicted, color, 2, arrow=True)
            cv2.circle(image, _point(pixel), 6, color, 2, cv2.LINE_AA)
            if 0 <= predicted[0] < image.shape[1] and 0 <= predicted[1] < image.shape[0]:
                cv2.drawMarker(image, _point(predicted), (245, 245, 245), cv2.MARKER_CROSS, 9, 1, cv2.LINE_AA)
            _measurement_label(image, f"{prefix}{index+1:02d}", pixel, color, occupied)
    # The legend is appended rather than obscuring or resizing the input pixels.
    panel = np.full((148, max(image.shape[1], 900), 3), 24, dtype=np.uint8)
    rejected = [f"F{i+1:02d} ({report['fit_errors_m'][i]*1000:.1f}mm)"
                for i, valid in enumerate(report["fit_inlier_mask"]) if not valid]
    rejected_text = "; rejected " + ", ".join(rejected[:3]) + (" ..." if len(rejected) > 3 else "") if rejected else ""
    lines = [
        "PROJECTED FLOOR GRID - not extra measurement evidence | undistorted input required",
        "Green F: fit inlier | Red F: rejected fit | Magenta C: independent held-out check",
        "Circle: measured pixel | White cross: prediction | Arrow: measured -> prediction",
        f"Fit: {report['fit_inlier_count']}/{report['fit_point_count']} inliers, RMS {report['fit_inlier_rms_m']*1000:.2f} mm (training only){rejected_text}",
        f"Hold-out: {report['independent_checkpoint_count']} points, RMS {report['checkpoint_rms_m']*1000:.2f} mm, max {report['checkpoint_max_m']*1000:.2f} mm | {report['status']}"
        if report["checkpoint_rms_m"] is not None else "Hold-out: NONE - not independently validated",
        f"Floor z={calibration.reference_plane_z:g}m | grid={grid_step:g}m | camera changes invalidate calibration",
    ]
    for i, line in enumerate(lines):
        cv2.putText(panel, line, (10, 20+i*23), cv2.FONT_HERSHEY_SIMPLEX, .43, (235, 235, 235), 1, cv2.LINE_AA)
    output = np.full((image.shape[0]+panel.shape[0], panel.shape[1], 3), 24, dtype=np.uint8)
    output[:image.shape[0], :image.shape[1]] = image
    output[image.shape[0]:] = panel
    return output


def render_floor_view(bgr, calibration, pixels_per_metre=128., grid_step=.5):
    """Warp the actual image to its reference floor plane, not the box-top plane."""
    import cv2
    if not math.isfinite(pixels_per_metre) or not 32 <= pixels_per_metre <= 512:
        raise ValueError("Floor-view scale must be between 32 and 512 pixels/metre")
    scale = pixels_per_metre
    width, height = calibration.arena_width, calibration.arena_height
    to_pixels = np.array([[scale, 0., 0.], [0., -scale, height*scale], [0., 0., 1.]])
    size = (int(math.ceil(width*scale))+1, int(math.ceil(height*scale))+1)
    result = cv2.warpPerspective(bgr, to_pixels @ calibration.plane_homography(calibration.reference_plane_z),
                                 size, flags=cv2.INTER_LINEAR)
    for _, _, endpoints in metre_grid(calibration, grid_step):
        pixels = np.column_stack([endpoints[:, 0]*scale, (height-endpoints[:, 1])*scale])
        _line(result, *pixels, GRID)
    _label(result, f"Floor z={calibration.reference_plane_z:g}m; boxes retain height parallax", (9, 20), (245, 245, 245))
    return result


def _read_measurements(path, minimum, label):
    result = json.loads(path.read_text(encoding="utf-8-sig"))
    if not isinstance(result, list) or len(result) < minimum:
        raise ValueError(f"{label} JSON must be a list of at least {minimum} measured pixel/arena pairs")
    return tuple(result)


def _save_image(path, bgr):
    import cv2
    if path.suffix.lower() not in (".png", ".jpg", ".jpeg", ".webp"):
        raise ValueError("Diagnostic image extension must be PNG, JPG or WEBP")
    ok, encoded = cv2.imencode(path.suffix, bgr)
    if not ok:
        raise ValueError(f"Could not encode diagnostic image: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(encoded.tobytes())


def main(argv=None):
    import cv2
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--image", type=Path, required=True)
    parser.add_argument("--points", type=Path, required=True)
    parser.add_argument("--checkpoints", type=Path, required=True)
    parser.add_argument("--camera-center", nargs=3, type=float, required=True, metavar=("X_M", "Y_M", "Z_M"))
    parser.add_argument("--arena-size", nargs=2, type=float, default=(6.12, 5.), metavar=("WIDTH_M", "HEIGHT_M"))
    parser.add_argument("--reference-height", type=float, default=0.)
    parser.add_argument("--object-height", type=float, default=.4)
    parser.add_argument("--fit-tolerance", type=float, default=.05, help="RANSAC inlier threshold in metres")
    parser.add_argument("--checkpoint-tolerance", type=float, default=.10)
    parser.add_argument("--grid-step", type=float, default=.5, help="Projected display grid spacing in metres")
    parser.add_argument("--source", default="manual_measured_arena_grid")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--overlay", type=Path, required=True)
    parser.add_argument("--birdseye", type=Path, help="Optional rectified reference-floor image")
    parser.add_argument("--pixels-per-metre", type=float, default=128.)
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args(argv)
    outputs = [p for p in (args.output, args.overlay, args.birdseye) if p is not None]
    inputs = {p.resolve() for p in (args.image, args.points, args.checkpoints)}
    if len({p.resolve() for p in outputs}) != len(outputs) or any(p.resolve() in inputs for p in outputs):
        parser.error("Output paths must be distinct and must not replace input measurements or image")
    if not args.overwrite and any(p.exists() for p in outputs):
        parser.error("Output already exists; use other paths or --overwrite")
    try:
        image = cv2.imdecode(np.frombuffer(args.image.read_bytes(), dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError("Input image cannot be decoded")
        cal = GridCalibration(
            image_size=(image.shape[1], image.shape[0]),
            measurements=_read_measurements(args.points, 8, "Fit points"),
            camera_center=tuple(args.camera_center), arena_width=args.arena_size[0], arena_height=args.arena_size[1],
            reference_plane_z=args.reference_height, object_plane_z=args.object_height,
            source=args.source, checkpoints=_read_measurements(args.checkpoints, 1, "Held-out checkpoints"),
            checkpoint_tolerance_m=args.checkpoint_tolerance, ransac_threshold_m=args.fit_tolerance)
        report = cal.validation_report()
        overlay = render_overlay(image, cal, args.grid_step)
        floor = render_floor_view(image, cal, args.pixels_per_metre, args.grid_step) if args.birdseye else None
        _save_image(args.overlay, overlay)
        if floor is not None:
            _save_image(args.birdseye, floor)
        saved = report["status"] != "outside_tolerance"
        if saved:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            cal.save(args.output)
        print(json.dumps({"calibration_saved": saved, "output": str(args.output.resolve()) if saved else None,
                          "overlay": str(args.overlay.resolve()),
                          "birdseye": str(args.birdseye.resolve()) if args.birdseye else None,
                          "overlay_grid_is_measurement_evidence": False, **report}, indent=2))
        if not saved:
            parser.error("Independent checkpoints exceed tolerance; diagnostic images saved, calibration not saved")
    except (ValueError, TypeError, KeyError, OSError, np.linalg.LinAlgError, cv2.error) as error:
        parser.error(str(error))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
