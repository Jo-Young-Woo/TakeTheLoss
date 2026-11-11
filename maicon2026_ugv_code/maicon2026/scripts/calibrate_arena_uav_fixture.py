#!/usr/bin/env python3
"""Measure printed-floor image correspondences, fit, and independently check.

The bitmap's measured physical size supplies metric reference coordinates.
Every fitted and held-out pixel is a real mutually matched image feature.
No fitted corners, synthetic projected grid, box layout or simulator pose is
used to manufacture calibration correspondences. This is for already
undistorted pinhole images; a projected display grid is not lens correction.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import re
from pathlib import Path
import sys

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from maicon26.perception.uav_calibration import GridCalibration
from maicon26.perception.uav_layout import FixedUavLayoutDetector
from calibrate_uav_grid import arena_to_pixels, render_overlay, _save_image


def measured_matches(reference, image, arena_size, max_match_ratio=.5):
    """Return distinct mutual Lowe-ratio matches, before any geometry fit."""
    width, height = arena_size
    reference_width = min(720, image.shape[1])
    reference_height = round(reference.shape[0]*reference_width/reference.shape[1])
    scaled = cv2.resize(reference, (reference_width, reference_height), interpolation=cv2.INTER_AREA)
    sift = cv2.SIFT_create(nfeatures=5000, contrastThreshold=.015)
    keys1, desc1 = sift.detectAndCompute(cv2.cvtColor(scaled, cv2.COLOR_BGR2GRAY), None)
    keys2, desc2 = sift.detectAndCompute(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), None)
    if desc1 is None or desc2 is None or min(len(desc1), len(desc2)) < 2:
        raise ValueError("Not enough visible printed-floor image features")
    matcher = cv2.BFMatcher()
    forward = matcher.knnMatch(desc1, desc2, k=2)
    reverse = matcher.match(desc2, desc1)
    candidates = []
    for pair in forward:
        if len(pair) != 2:
            continue
        match, other = pair
        if match.distance >= max_match_ratio*other.distance or reverse[match.trainIdx].trainIdx != match.queryIdx:
            continue
        x, y = keys1[match.queryIdx].pt
        pixel = list(keys2[match.trainIdx].pt)
        arena = [x/reference_width*width, height-y/reference_height*height]
        candidates.append({"pixel": pixel, "arena": arena,
                           "reference_pixel": [x/reference_width*reference.shape[1], y/reference_height*reference.shape[0]],
                           "match_ratio": match.distance/other.distance,
                           "source": "mutual_image_feature_in_measured_floor_print"})
    distinct = []
    for item in sorted(candidates, key=lambda p: p["match_ratio"]):
        if all(np.linalg.norm(np.array(item["pixel"])-p["pixel"]) > 10 and
               np.linalg.norm(np.array(item["arena"])-p["arena"]) > .08 for p in distinct):
            distinct.append(item)
    if len(distinct) < 20:
        raise ValueError("Need at least 20 distinct printed-floor matches for fit and held-out checks")
    # The split is fixed before fitting: no hold-out is dropped for disagreeing
    # with the fit. GridCalibration independently checks spatial coverage.
    return tuple(distinct[::2]), tuple(distinct[1::2])


def candidate_overlay(image, calibration, boxes):
    result = image.copy()
    for index, box in enumerate(boxes, 1):
        corners = [[box.cx-box.sx/2, box.cy-box.sy/2], [box.cx+box.sx/2, box.cy-box.sy/2],
                   [box.cx+box.sx/2, box.cy+box.sy/2], [box.cx-box.sx/2, box.cy+box.sy/2]]
        pixels = arena_to_pixels(calibration, corners, calibration.object_plane_z)
        polygon = np.round(pixels).astype(np.int32)
        cv2.polylines(result, [polygon], True, (50, 240, 70), 2, cv2.LINE_AA)
        centre = tuple(np.round(pixels.mean(axis=0)).astype(int))
        cv2.putText(result, str(index), centre, cv2.FONT_HERSHEY_SIMPLEX, .45, (50, 240, 70), 1, cv2.LINE_AA)
    cv2.putText(result, f"{len(boxes)} visible 0.4 m cube candidates - side classes UNKNOWN",
                (12, 20), cv2.FONT_HERSHEY_SIMPLEX, .55, (50, 240, 70), 1, cv2.LINE_AA)
    return result


def nominal_floor_alignment(checks, camera, arena_size, image_size):
    """Check held-out pixels against known pose/FOV, without any fitted H."""
    if tuple(camera["resolution"]) != tuple(image_size):
        raise ValueError("Nominal camera resolution differs from the captured frame")
    width, height = image_size
    x, y, z, roll, pitch, yaw = camera["pose"]
    cr, sr, cp, sp, cy, sy = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch), np.cos(yaw), np.sin(yaw)
    rotation = np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                         [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])
    focal = width/(2*np.tan(camera["hfov"]/2))
    errors = []
    for item in checks:
        u, v = item["pixel"]
        ray = rotation @ np.array([1., -(u-(width-1)/2)/focal, -(v-(height-1)/2)/focal])
        if ray[2] >= -1e-9:
            raise ValueError("Nominal camera checkpoint ray does not meet the floor")
        point = np.array([x, y, z])+ray*(-z/ray[2])
        point[:2] += np.array(arena_size)/2
        errors.append(float(np.linalg.norm(point[:2]-item["arena"])))
    return {"uses_fitted_homography": False, "checkpoint_count": len(errors), "errors_m": errors,
            "pixel_center": "(width-1)/2, (height-1)/2",
            "rms_m": float(np.sqrt(np.mean(np.square(errors)))), "max_m": max(errors),
            "within_1cm": max(errors) <= .01}


def main(argv=None):
    package = ROOT / "ros2/maicon_arena_gz"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image", type=Path, required=True)
    parser.add_argument("--spec", type=Path, default=package / "config/arena_spec.yaml")
    parser.add_argument("--reference", type=Path, default=package / "media/textures/floor_print_2400x1920.png")
    parser.add_argument("--output", type=Path, default=package / "config/uav_grid_calibration.json")
    parser.add_argument("--evidence-dir", type=Path, default=package / "validation")
    parser.add_argument("--evidence-prefix", default="uav", help="Separate output prefix for other fixed camera views")
    parser.add_argument("--camera-height", type=float, help="Explicit optical height for a separate validation camera")
    parser.add_argument("--max-match-ratio", type=float, default=.5,
                        help="Photometric descriptor uniqueness criterion applied before splitting/fitting")
    parser.add_argument("--nominal-camera-profile", type=Path,
                        help="Additionally check against this profile's validation_camera pose/FOV, without fitting")
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args(argv)
    if not re.fullmatch(r"[a-zA-Z0-9_]+", args.evidence_prefix):
        parser.error("Evidence prefix must contain only letters, digits and underscores")
    if not 0 < args.max_match_ratio < 1:
        parser.error("Maximum descriptor match ratio must be between zero and one")
    if args.output.exists() and not args.overwrite:
        parser.error("Calibration exists; use a different output or --overwrite")
    spec = yaml.safe_load(args.spec.read_text(encoding="utf-8"))
    image = cv2.imdecode(np.frombuffer(args.image.read_bytes(), np.uint8), cv2.IMREAD_COLOR)
    reference = cv2.imdecode(np.frombuffer(args.reference.read_bytes(), np.uint8), cv2.IMREAD_COLOR)
    if image is None or reference is None:
        parser.error("Input camera image or floor print could not be decoded")
    width, height = spec["floor"]["size"]
    fit, checks = measured_matches(reference, image, (width, height), args.max_match_ratio)
    camera = spec["uav_camera"]["pose"]
    camera_height = camera[2] if args.camera_height is None else args.camera_height
    prefix = args.evidence_prefix
    calibration = GridCalibration((image.shape[1], image.shape[0]), fit,
        (camera[0]+width/2, camera[1]+height/2, camera_height), arena_width=width, arena_height=height,
        object_plane_z=spec["cubes"]["size"], checkpoints=checks,
        ransac_threshold_m=.02, checkpoint_tolerance_m=.01,
        source="measured_printed_floor_image_features; holdout_split_before_homography_fit")
    report = calibration.validation_report()
    if args.nominal_camera_profile:
        nominal = yaml.safe_load(args.nominal_camera_profile.read_text(encoding="utf-8"))["validation_camera"]
        report["nominal_pinhole_alignment"] = nominal_floor_alignment(checks, nominal, (width, height), calibration.image_size)
    args.evidence_dir.mkdir(parents=True, exist_ok=True)
    _save_image(args.evidence_dir / f"{prefix}_grid_overlay.png", render_overlay(image, calibration, .4))
    if report["status"] != "within_tolerance":
        parser.error("Held-out measurements failed 1 cm tolerance; diagnostic saved, calibration not saved")
    if args.nominal_camera_profile and not report["nominal_pinhole_alignment"]["within_1cm"]:
        parser.error("Nominal camera pose/FOV check failed 1 cm tolerance; diagnostic saved, calibration not saved")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    calibration.save(args.output)
    for name, points in ((f"{prefix}_grid_measurements.json", fit), (f"{prefix}_grid_checkpoints.json", checks)):
        (args.evidence_dir / name).write_text(json.dumps(points, indent=2)+"\n", encoding="utf-8")
    boxes = FixedUavLayoutDetector(calibration).detect(image)
    _save_image(args.evidence_dir / f"{prefix}_cube_candidates.png", candidate_overlay(image, calibration, boxes))
    report.update({"image": args.image.name, "image_sha256": hashlib.sha256(args.image.read_bytes()).hexdigest(),
                   "reference_sha256": hashlib.sha256(args.reference.read_bytes()).hexdigest(),
                   "reference_method": "mutual matched real image pixels; distinct holdout selected before fitting",
                   "max_descriptor_match_ratio": args.max_match_ratio,
                   "grid_overlay_is_extra_measurement_evidence": False,
                   "candidate_count": len(boxes), "candidate_classes": "unknown; require ground side inspection",
                   "candidates": [vars(box) for box in boxes]})
    (args.evidence_dir / f"{prefix}_calibration_detection.json").write_text(json.dumps(report, indent=2)+"\n", encoding="utf-8")
    print(json.dumps({key: report[key] for key in ("fit_inlier_count", "fit_point_count", "independent_checkpoint_count",
                                                 "checkpoint_rms_m", "checkpoint_max_m", "candidate_count")}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
