"""Four labelled arena corners calibrate an undistorted pinhole UAV image.

Corner order is the arena order (0,0), (W,0), (W,H), (0,H), irrespective of
where those points appear on screen. Four fitted points do not independently
validate a homography. Optional separate checkpoints report that uncertainty.

Floor pixels and box-top pixels lie on different planes. A camera optical
centre in the same arena frame permits ray/plane correction for any camera
orientation; no target locations, simulator poses or class labels are inputs.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import json
import math
import copy
from pathlib import Path
from typing import Any, Sequence

import numpy as np

CORNER_ORDER = ("origin_0_0", "x_corner_W_0", "opposite_W_H", "y_corner_0_H")


def _points(value, count=None):
    points = np.asarray(value, dtype=float)
    if points.ndim != 2 or points.shape[1] != 2 or (count is not None and len(points) != count):
        raise ValueError(f"Expected {'N' if count is None else count} two-dimensional points")
    if not np.isfinite(points).all():
        raise ValueError("Point coordinates must be finite")
    return points


def _normalise(points):
    centre = points.mean(axis=0)
    spread = np.mean(np.linalg.norm(points - centre, axis=1))
    if spread < 1e-8:
        raise ValueError("Coincident calibration points")
    s = math.sqrt(2.) / spread
    transform = np.array([[s, 0., -s * centre[0]], [0., s, -s * centre[1]], [0., 0., 1.]])
    return (points - centre) * s, transform


def _fit_homography(source, destination):
    src, ts = _normalise(source)
    dst, td = _normalise(destination)
    rows = []
    for (x, y), (u, v) in zip(src, dst):
        rows.extend(([-x, -y, -1, 0, 0, 0, u*x, u*y, u],
                     [0, 0, 0, -x, -y, -1, v*x, v*y, v]))
    _, singular, vt = np.linalg.svd(np.asarray(rows, dtype=float), full_matrices=True)
    if singular[-1] < 1e-9 or singular[0] / singular[-1] > 1e5:
        raise ValueError("Calibration quadrilateral is ill-conditioned")
    h = np.linalg.inv(td) @ vt[-1].reshape(3, 3) @ ts
    return h / np.linalg.norm(h)


def _project(matrix, points):
    homogeneous = np.column_stack([points, np.ones(len(points))]) @ matrix.T
    if np.any(np.abs(homogeneous[:, 2]) < 1e-10):
        raise ValueError("Pixel projects onto the homography horizon")
    result = homogeneous[:, :2] / homogeneous[:, 2:]
    if not np.isfinite(result).all():
        raise ValueError("Projection is not finite")
    return result


@dataclass(frozen=True)
class FourCornerCalibration:
    image_size: tuple[int, int]                    # width, height of the undistorted image
    pixel_corners: tuple[tuple[float, float], ...]  # CORNER_ORDER, never silently reordered
    camera_center: tuple[float, float, float]      # optical centre XYZ in arena metres
    arena_width: float = 6.12
    arena_height: float = 5.0
    reference_plane_z: float = 0.0
    object_plane_z: float = .4
    source: str = "manual_labelled_arena_corners"
    checkpoints: tuple[dict[str, Any], ...] = ()
    checkpoint_tolerance_m: float = .10
    _homography: np.ndarray = field(init=False, repr=False, compare=False)

    def __post_init__(self):
        if (len(self.image_size) != 2 or any(isinstance(v, bool) or int(v) != v or v < 32
                                             for v in self.image_size)):
            raise ValueError("image_size must contain positive integer width and height, at least 32 pixels")
        if len(self.camera_center) != 3 or not np.isfinite(self.camera_center).all():
            raise ValueError("camera_center must be finite arena XYZ coordinates")
        values = (self.arena_width, self.arena_height, self.reference_plane_z,
                  self.object_plane_z, self.checkpoint_tolerance_m)
        if not all(math.isfinite(v) for v in values) or min(self.arena_width, self.arena_height,
                                                           self.checkpoint_tolerance_m) <= 0:
            raise ValueError("Arena dimensions and checkpoint tolerance must be positive and finite")
        if self.camera_center[2] <= max(self.reference_plane_z, self.object_plane_z) + .05:
            raise ValueError("Camera optical centre must be above both calibrated planes")
        corners = _points(self.pixel_corners, 4)
        self._validate_pixels(corners)
        edges = np.roll(corners, -1, axis=0) - corners
        turn = edges[:, 0] * np.roll(edges, -1, axis=0)[:, 1] - edges[:, 1] * np.roll(edges, -1, axis=0)[:, 0]
        if not (np.all(turn > 1e-5) or np.all(turn < -1e-5)):
            raise ValueError("Corners must follow the labelled convex arena perimeter without crossings")
        area = abs(np.dot(corners[:, 0], np.roll(corners[:, 1], -1)) -
                   np.dot(corners[:, 1], np.roll(corners[:, 0], -1))) / 2
        if area < 100. or np.min(np.linalg.norm(edges, axis=1)) < 8.:
            raise ValueError("Calibration corners are degenerate or too close together")
        target = np.array([[0., 0.], [self.arena_width, 0.],
                           [self.arena_width, self.arena_height], [0., self.arena_height]])
        object.__setattr__(self, "_homography", _fit_homography(corners, target))
        # Checkpoints are separate measurements, not the four fitted corners.
        used = []
        for item in self.checkpoints:
            if not isinstance(item, dict) or "pixel" not in item or "arena" not in item:
                raise ValueError("Each checkpoint needs pixel and arena coordinate pairs")
            pixel, arena = _points([item["pixel"]], 1), _points([item["arena"]], 1)
            self._validate_pixels(pixel)
            if np.min(np.linalg.norm(corners - pixel[0], axis=1)) <= 1.:
                raise ValueError("A fitted corner cannot be reused as an independent checkpoint")
            if any(np.linalg.norm(pixel[0] - other) <= 1. for other in used):
                raise ValueError("Independent checkpoints must be distinct")
            used.append(pixel[0])
            if not (0 <= arena[0, 0] <= self.arena_width and 0 <= arena[0, 1] <= self.arena_height):
                raise ValueError("Checkpoint arena coordinates are outside the arena")
            z = float(item.get("plane_z", self.reference_plane_z))
            self.plane_homography(z)

    def _validate_pixels(self, points):
        width, height = self.image_size
        if np.any(points < 0) or np.any(points[:, 0] > width - 1) or np.any(points[:, 1] > height - 1):
            raise ValueError("Pixel coordinates are outside the calibrated image")

    def plane_homography(self, plane_z: float) -> np.ndarray:
        """Image pixels -> XY on a requested horizontal plane, in arena metres."""
        if not math.isfinite(plane_z) or plane_z >= self.camera_center[2] - .05:
            raise ValueError("Projection plane must be finite and below the camera")
        cx, cy, cz = self.camera_center
        ratio = (cz - plane_z) / (cz - self.reference_plane_z)
        correction = np.array([[ratio, 0., (1-ratio)*cx], [0., ratio, (1-ratio)*cy], [0., 0., 1.]])
        return correction @ self._homography

    def project(self, pixels: Sequence[Sequence[float]], plane_z: float | None = None,
                *, require_inside: bool = True) -> np.ndarray:
        points = _points(pixels)
        self._validate_pixels(points)
        out = _project(self.plane_homography(self.reference_plane_z if plane_z is None else plane_z), points)
        if require_inside and (np.any(out < -1e-7) or np.any(out[:, 0] > self.arena_width + 1e-7)
                               or np.any(out[:, 1] > self.arena_height + 1e-7)):
            raise ValueError("Projected point is outside the arena")
        return out

    def validation_report(self) -> dict[str, Any]:
        target = np.array([[0., 0.], [self.arena_width, 0.],
                           [self.arena_width, self.arena_height], [0., self.arena_height]])
        fit = self.project(self.pixel_corners) - target
        errors = []
        for item in self.checkpoints:
            estimate = self.project([item["pixel"]], float(item.get("plane_z", self.reference_plane_z)), require_inside=False)[0]
            errors.append(float(np.linalg.norm(estimate - np.asarray(item["arena"]))))
        return {"corner_fit_rms_m": float(np.sqrt(np.mean(np.sum(fit*fit, axis=1)))),
                "corner_fit_is_independent_validation": False,
                "independent_checkpoint_count": len(errors),
                "checkpoint_errors_m": errors,
                "checkpoint_rms_m": float(np.sqrt(np.mean(np.square(errors)))) if errors else None,
                "checkpoint_max_m": max(errors) if errors else None,
                "checkpoint_tolerance_m": self.checkpoint_tolerance_m,
                "status": ("not_independently_validated" if not errors else
                           "within_tolerance" if max(errors) <= self.checkpoint_tolerance_m else "outside_tolerance")}

    def rectify_top(self, bgr: np.ndarray, pixels_per_metre: float = 128.) -> tuple[np.ndarray, float]:
        """Rectify tilted/rotated images before axis-aligned box-top segmentation."""
        import cv2
        if bgr.shape[:2] != (self.image_size[1], self.image_size[0]):
            raise ValueError("UAV image size differs from the calibrated image")
        if self.validation_report()["status"] == "outside_tolerance":
            raise ValueError("UAV calibration failed its independent checkpoints")
        if not math.isfinite(pixels_per_metre) or not 32 <= pixels_per_metre <= 512:
            raise ValueError("Rectification scale must be between 32 and 512 pixels/metre")
        scale = pixels_per_metre
        to_pixels = np.array([[scale, 0., 0.], [0., -scale, self.arena_height*scale], [0., 0., 1.]])
        matrix = to_pixels @ self.plane_homography(self.object_plane_z)
        size = (int(math.ceil(self.arena_width*scale))+1, int(math.ceil(self.arena_height*scale))+1)
        return cv2.warpPerspective(bgr, matrix, size, flags=cv2.INTER_NEAREST), scale

    def to_dict(self) -> dict[str, Any]:
        return {"schema": "maicon_uav_four_corners_v1", "source": self.source,
                "image_size": list(self.image_size), "corner_order": list(CORNER_ORDER),
                "pixel_corners": np.asarray(self.pixel_corners).tolist(),
                "arena_size_m": [self.arena_width, self.arena_height],
                "camera_optical_center_m": list(self.camera_center),
                "reference_plane_z_m": self.reference_plane_z, "object_plane_z_m": self.object_plane_z,
                "input_optics": "pinhole_undistorted", "checkpoint_tolerance_m": self.checkpoint_tolerance_m,
                "checkpoints": list(self.checkpoints), "validation": self.validation_report()}

    @classmethod
    def from_dict(cls, data):
        if data.get("schema") != "maicon_uav_four_corners_v1" or data.get("corner_order") != list(CORNER_ORDER):
            raise ValueError("Unsupported UAV calibration schema or corner order")
        if data.get("input_optics") != "pinhole_undistorted":
            raise ValueError("Homography requires an undistorted pinhole image")
        width, height = data["arena_size_m"]
        result = cls(tuple(data["image_size"]), tuple(tuple(p) for p in data["pixel_corners"]),
                     tuple(data["camera_optical_center_m"]), width, height,
                     data["reference_plane_z_m"], data["object_plane_z_m"],
                     data.get("source", "manual_labelled_arena_corners"),
                     tuple(data.get("checkpoints", [])), data.get("checkpoint_tolerance_m", .10))
        if result.validation_report()["status"] == "outside_tolerance":
            raise ValueError("UAV calibration failed its independent checkpoints")
        return result

    @classmethod
    def load(cls, path: str | Path):
        return cls.from_dict(json.loads(Path(path).read_text(encoding="utf-8")))

    def save(self, path: str | Path) -> None:
        Path(path).write_text(json.dumps(self.to_dict(), indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def _grid_homography(source, destination):
    """Normalised DLT for four or more measured correspondences.

    With more than four exact pairs the final singular value is the wanted
    homography null space, not a degeneracy. The preceding value checks rank.
    """
    src, ts = _normalise(source)
    dst, td = _normalise(destination)
    rows = []
    for (x, y), (u, v) in zip(src, dst):
        rows.extend(([-x, -y, -1, 0, 0, 0, u*x, u*y, u],
                     [0, 0, 0, -x, -y, -1, v*x, v*y, v]))
    _, singular, vt = np.linalg.svd(np.asarray(rows, dtype=float), full_matrices=True)
    rank_value = singular[7]
    if rank_value < 1e-9 or singular[0] / rank_value > 1e6:
        raise ValueError("Grid correspondences are collinear or ill-conditioned")
    h = np.linalg.inv(td) @ vt[-1].reshape(3, 3) @ ts
    if not np.isfinite(h).all() or abs(np.linalg.det(h / np.linalg.norm(h))) < 1e-12:
        raise ValueError("Grid homography is singular")
    return h / np.linalg.norm(h)


def _convex_area(points):
    """Monotone-chain area; no vision dependency is needed for calibration."""
    pts = sorted(map(tuple, points))
    def cross(o, a, b):
        return (a[0]-o[0])*(b[1]-o[1]) - (a[1]-o[1])*(b[0]-o[0])
    lower, upper = [], []
    for point in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= 0:
            lower.pop()
        lower.append(point)
    for point in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= 0:
            upper.pop()
        upper.append(point)
    hull = np.asarray(lower[:-1]+upper[:-1], dtype=float)
    if len(hull) < 3:
        return 0.
    return float(abs(np.dot(hull[:, 0], np.roll(hull[:, 1], -1)) -
                     np.dot(hull[:, 1], np.roll(hull[:, 0], -1))) / 2.)


@dataclass(frozen=True)
class GridCalibration:
    """Robust image-to-arena calibration from measured floor grid intersections.

    A drawn grid is only an overlay. Every ``measurements`` entry must pair a
    measured image pixel with its independently known arena coordinate. This
    class never generates fitting evidence from four corners or its own model.
    Inputs must already be lens-undistorted; homography fitting cannot estimate
    radial lens distortion. All fit points lie on ``reference_plane_z``.
    """
    image_size: tuple[int, int]
    measurements: tuple[dict[str, Any], ...]
    camera_center: tuple[float, float, float]
    arena_width: float = 6.12
    arena_height: float = 5.0
    reference_plane_z: float = 0.0
    object_plane_z: float = .4
    source: str = "manual_measured_arena_grid"
    checkpoints: tuple[dict[str, Any], ...] = ()
    checkpoint_tolerance_m: float = .10
    ransac_threshold_m: float = .05
    min_inlier_ratio: float = .65
    ransac_trials: int = 1000
    random_seed: int = 0
    _homography: np.ndarray = field(init=False, repr=False, compare=False)
    _pixels: np.ndarray = field(init=False, repr=False, compare=False)
    _arena_points: np.ndarray = field(init=False, repr=False, compare=False)
    _inliers: np.ndarray = field(init=False, repr=False, compare=False)
    _errors: np.ndarray = field(init=False, repr=False, compare=False)

    # The plane/parallax model is shared with the four-corner implementation;
    # fitting and validation below use only the actual measured grid points.
    _validate_pixels = FourCornerCalibration._validate_pixels
    plane_homography = FourCornerCalibration.plane_homography
    project = FourCornerCalibration.project
    rectify_top = FourCornerCalibration.rectify_top
    save = FourCornerCalibration.save

    def __post_init__(self):
        if (len(self.image_size) != 2 or any(isinstance(v, bool) or int(v) != v or v < 32
                                             for v in self.image_size)):
            raise ValueError("image_size must contain integer width and height, at least 32 pixels")
        if len(self.camera_center) != 3 or not np.isfinite(self.camera_center).all():
            raise ValueError("camera_center must be finite arena XYZ coordinates")
        values = (self.arena_width, self.arena_height, self.reference_plane_z, self.object_plane_z,
                  self.checkpoint_tolerance_m, self.ransac_threshold_m, self.min_inlier_ratio)
        if not all(math.isfinite(v) for v in values) or min(self.arena_width, self.arena_height,
                 self.checkpoint_tolerance_m, self.ransac_threshold_m) <= 0:
            raise ValueError("Calibration dimensions and tolerances must be positive and finite")
        if not .5 <= self.min_inlier_ratio <= 1.:
            raise ValueError("min_inlier_ratio must be between .5 and 1")
        if (isinstance(self.ransac_trials, bool) or not isinstance(self.ransac_trials, int)
                or not 16 <= self.ransac_trials <= 10000):
            raise ValueError("ransac_trials must be an integer between 16 and 10000")
        if isinstance(self.random_seed, bool) or not isinstance(self.random_seed, int) or self.random_seed < 0:
            raise ValueError("random_seed must be a nonnegative integer")
        if self.camera_center[2] <= max(self.reference_plane_z, self.object_plane_z)+.05:
            raise ValueError("Camera optical centre must be above both calibrated planes")
        if len(self.measurements) < 8:
            raise ValueError("Grid calibration needs at least 8 measured pixel/arena correspondences")
        clean = []
        for item in self.measurements:
            if not isinstance(item, dict) or "pixel" not in item or "arena" not in item:
                raise ValueError("Each measured grid point needs pixel and arena coordinate pairs")
            if not math.isclose(float(item.get("plane_z", self.reference_plane_z)), self.reference_plane_z, abs_tol=1e-8):
                raise ValueError("All measured grid points must share reference_plane_z")
            pixel, arena = _points([item["pixel"]], 1)[0], _points([item["arena"]], 1)[0]
            clean.append({**copy.deepcopy(item), "pixel": pixel.tolist(), "arena": arena.tolist()})
        object.__setattr__(self, "measurements", tuple(clean))
        object.__setattr__(self, "checkpoints", tuple(copy.deepcopy(self.checkpoints)))
        pixels = _points([item["pixel"] for item in clean])
        arena = _points([item["arena"] for item in clean])
        self._validate_pixels(pixels)
        if np.any(arena < 0) or np.any(arena[:, 0] > self.arena_width) or np.any(arena[:, 1] > self.arena_height):
            raise ValueError("Measured grid arena coordinates are outside the arena")
        for pts, tolerance, label in ((pixels, 2., "pixel"), (arena, .02, "arena")):
            distances = np.linalg.norm(pts[:, None, :] - pts[None, :, :], axis=2)
            np.fill_diagonal(distances, np.inf)
            if np.min(distances) < tolerance:
                raise ValueError(f"Duplicate or too-close measured {label} grid points")
        self._validate_distribution(pixels, arena)
        homography, inliers, errors = self._fit(pixels, arena)
        self._validate_distribution(pixels[inliers], arena[inliers])
        for key, value in (("_homography", homography), ("_pixels", pixels), ("_arena_points", arena),
                           ("_inliers", inliers), ("_errors", errors)):
            object.__setattr__(self, key, value)
        used = []
        for item in self.checkpoints:
            if not isinstance(item, dict) or "pixel" not in item or "arena" not in item:
                raise ValueError("Each independent checkpoint needs pixel and arena coordinate pairs")
            pixel, point = _points([item["pixel"]], 1)[0], _points([item["arena"]], 1)[0]
            self._validate_pixels(np.array([pixel]))
            if np.min(np.linalg.norm(pixels-pixel, axis=1)) <= 2.:
                raise ValueError("A measured fit point cannot be reused as an independent checkpoint")
            if any(np.linalg.norm(pixel-other) <= 2. for other in used):
                raise ValueError("Independent checkpoints must be distinct")
            used.append(pixel)
            if not (0 <= point[0] <= self.arena_width and 0 <= point[1] <= self.arena_height):
                raise ValueError("Checkpoint arena coordinates are outside the arena")
            z = float(item.get("plane_z", self.reference_plane_z))
            self.plane_homography(z)
            if math.isclose(z, self.reference_plane_z, abs_tol=1e-8) and np.min(np.linalg.norm(arena-point, axis=1)) < .02:
                raise ValueError("A measured grid location cannot be reused as an independent checkpoint")

    def _validate_distribution(self, pixels, arena):
        normalised = arena / np.array([self.arena_width, self.arena_height])
        for points in (pixels, normalised):
            singular = np.linalg.svd(points-points.mean(axis=0), compute_uv=False)
            if len(singular) < 2 or singular[1] / max(singular[0], 1e-12) < .025:
                raise ValueError("Measured grid points are collinear or nearly collinear")
        if (np.any(np.ptp(normalised, axis=0) < .6) or _convex_area(normalised) < .25
                or _convex_area(pixels) < 100.):
            raise ValueError("Measured grid points must cover at least 60% of both arena axes and 25% of its area")

    def _fit(self, pixels, arena):
        rng = np.random.default_rng(self.random_seed)
        best_mask, best_key = None, (-1, -math.inf)
        tried = set()
        for _ in range(self.ransac_trials):
            sample = tuple(sorted(rng.choice(len(pixels), 4, replace=False).tolist()))
            if sample in tried:
                continue
            tried.add(sample)
            try:
                model = _grid_homography(pixels[list(sample)], arena[list(sample)])
                errors = np.linalg.norm(_project(model, pixels)-arena, axis=1)
            except (ValueError, np.linalg.LinAlgError):
                continue
            mask = errors <= self.ransac_threshold_m
            count = int(mask.sum())
            key = (count, -float(np.mean(errors[mask]**2)) if count else -math.inf)
            if key > best_key:
                best_mask, best_key = mask, key
            if count == len(pixels):
                break
        required = max(8, int(math.ceil(len(pixels)*self.min_inlier_ratio)))
        if best_mask is None or int(best_mask.sum()) < required:
            raise ValueError(f"RANSAC found fewer than {required} consistent measured grid points")
        mask = best_mask
        for _ in range(8):
            model = _grid_homography(pixels[mask], arena[mask])
            errors = np.linalg.norm(_project(model, pixels)-arena, axis=1)
            updated = errors <= self.ransac_threshold_m
            if int(updated.sum()) < required:
                raise ValueError("Refined grid fit has too few consistent measurements")
            if np.array_equal(updated, mask):
                return model, mask, errors
            mask = updated
        raise ValueError("Grid inlier refinement did not converge")

    def validation_report(self) -> dict[str, Any]:
        errors = []
        for item in self.checkpoints:
            estimate = self.project([item["pixel"]], float(item.get("plane_z", self.reference_plane_z)),
                                    require_inside=False)[0]
            errors.append(float(np.linalg.norm(estimate-np.asarray(item["arena"]))))
        fit = self._errors[self._inliers]
        return {"fit_point_count": len(self.measurements), "fit_inlier_count": int(self._inliers.sum()),
                "fit_inlier_mask": self._inliers.tolist(), "fit_errors_m": self._errors.tolist(),
                "fit_inlier_rms_m": float(np.sqrt(np.mean(fit**2))), "fit_inlier_max_m": float(np.max(fit)),
                "fit_all_rms_m": float(np.sqrt(np.mean(self._errors**2))),
                "fit_is_independent_validation": False, "input_optics": "pinhole_undistorted",
                "ransac_threshold_m": self.ransac_threshold_m,
                "independent_checkpoint_count": len(errors), "checkpoint_errors_m": errors,
                "checkpoint_rms_m": float(np.sqrt(np.mean(np.square(errors)))) if errors else None,
                "checkpoint_max_m": max(errors) if errors else None,
                "checkpoint_tolerance_m": self.checkpoint_tolerance_m,
                "status": ("not_independently_validated" if not errors else
                           "within_tolerance" if max(errors) <= self.checkpoint_tolerance_m else "outside_tolerance")}

    def to_dict(self) -> dict[str, Any]:
        return {"schema": "maicon_uav_grid_v1", "source": self.source,
                "image_size": list(self.image_size), "measurements": copy.deepcopy(list(self.measurements)),
                "arena_size_m": [self.arena_width, self.arena_height],
                "camera_optical_center_m": list(self.camera_center),
                "reference_plane_z_m": self.reference_plane_z, "object_plane_z_m": self.object_plane_z,
                "input_optics": "pinhole_undistorted", "checkpoint_tolerance_m": self.checkpoint_tolerance_m,
                "checkpoints": copy.deepcopy(list(self.checkpoints)),
                "ransac_threshold_m": self.ransac_threshold_m, "min_inlier_ratio": self.min_inlier_ratio,
                "ransac_trials": self.ransac_trials, "random_seed": self.random_seed,
                "validation": self.validation_report()}

    @classmethod
    def from_dict(cls, data):
        if data.get("schema") != "maicon_uav_grid_v1":
            raise ValueError("Unsupported UAV grid calibration schema")
        if data.get("input_optics") != "pinhole_undistorted":
            raise ValueError("Grid homography requires an undistorted pinhole image")
        width, height = data["arena_size_m"]
        result = cls(tuple(data["image_size"]), tuple(data["measurements"]),
                     tuple(data["camera_optical_center_m"]), width, height,
                     data["reference_plane_z_m"], data["object_plane_z_m"],
                     data.get("source", "manual_measured_arena_grid"), tuple(data.get("checkpoints", [])),
                     data.get("checkpoint_tolerance_m", .10), data.get("ransac_threshold_m", .05),
                     data.get("min_inlier_ratio", .65), data.get("ransac_trials", 1000), data.get("random_seed", 0))
        if result.validation_report()["status"] == "outside_tolerance":
            raise ValueError("UAV grid calibration failed its independent checkpoints")
        return result

    @classmethod
    def load(cls, path: str | Path):
        return cls.from_dict(json.loads(Path(path).read_text(encoding="utf-8")))


def load_calibration(path: str | Path) -> FourCornerCalibration | GridCalibration:
    """Load either explicit calibration format; never infer fitting data."""
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    schema = data.get("schema") if isinstance(data, dict) else None
    if schema == "maicon_uav_four_corners_v1":
        return FourCornerCalibration.from_dict(data)
    if schema == "maicon_uav_grid_v1":
        return GridCalibration.from_dict(data)
    raise ValueError("Unsupported UAV calibration schema")
