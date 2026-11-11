"""Merge three co-located physical Gazebo scan sectors; never invent rays."""
from __future__ import annotations

from dataclasses import dataclass, replace
import math

import numpy as np

from .gazebo_model import LIDAR_FRAME, LIDAR_RPY, LIDAR_XYZ, lidar_frame

SECTOR_YAWS = (0., 2 * math.pi / 3, -2 * math.pi / 3)
SECTOR_HALF_FOV = math.pi / 3


def sector_samples(output_samples: int) -> int:
    if output_samples not in (360, 640):
        raise ValueError("Merged LiDAR samples must be 360 or 640")
    return math.ceil(output_samples / 3) + 1


def sector_frame(prefix: str, index: int) -> str:
    return f"{lidar_frame(prefix)}_sector_{index}"


def sector_topic(prefix: str, index: int) -> str:
    prefix = prefix.strip("/")
    return f"/{prefix}/scan_sector_{index}" if prefix else f"/scan_sector_{index}"


@dataclass(frozen=True)
class SectorScan:
    stamp: float
    frame_id: str
    angle_min: float
    angle_max: float
    angle_increment: float
    range_min: float
    range_max: float
    ranges: np.ndarray


def mask_rover_self_returns(scan: SectorScan, *, length=.25, width=.23, inset=.005):
    """Remove only measured endpoints strictly inside the known Rover body.

    Scan bearings are in the common lidar frame. Transform endpoints using the
    known mount into base_link, then test the PDF 250x230 mm footprint with
    a 5 mm inset. Body-surface/outside obstacles remain measurements, and hidden
    directions stay NaN rather than being declared free behind the robot mesh.
    This is robot geometry only; no arena map or simulator pose is consulted.
    """
    if scan.frame_id != LIDAR_FRAME and not scan.frame_id.endswith("/" + LIDAR_FRAME):
        raise ValueError("Self-return mask requires the common LiDAR frame")
    if (not all(math.isfinite(v) for v in (length, width, inset))
            or inset < 0 or min(length, width) <= 2 * inset):
        raise ValueError("Invalid Rover footprint mask dimensions")
    ranges = np.asarray(scan.ranges, dtype=np.float32).copy()
    angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment + LIDAR_RPY[2]
    valid = np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges <= scan.range_max)
    xs = LIDAR_XYZ[0] + ranges * np.cos(angles)
    ys = LIDAR_XYZ[1] + ranges * np.sin(angles)
    # Numerical tolerance preserves endpoints exactly on the inset boundary.
    inside = valid & (np.abs(xs) < length / 2 - inset - 1e-7) & (np.abs(ys) < width / 2 - inset - 1e-7)
    ranges[inside] = np.nan
    return replace(scan, ranges=ranges), int(np.count_nonzero(inside))


class LidarSectorMerger:
    """Fresh synchronized sectors -> one standard full-circle LaserScan.

    Output bearings are -pi + i*2pi/N. Each beam is rotated by its fixed sensor
    yaw and assigned to the closest angular bin; overlaps use the nearest real
    measured return. Invalid/missing bins stay NaN. No interpolation, synthetic
    free-space, arena geometry, odometry or simulator poses enter this class.
    """

    def __init__(self, samples=360, prefix="", max_skew_s=.02,
                 max_sensor_age_s=.3, max_receive_age_s=.5):
        self.samples = samples
        self.input_samples = sector_samples(samples)
        limits = (max_skew_s, max_sensor_age_s, max_receive_age_s)
        if any(not math.isfinite(v) or v <= 0 for v in limits):
            raise ValueError("LiDAR synchronization/freshness limits must be positive and finite")
        self.prefix = prefix
        self.max_skew_s, self.max_sensor_age_s, self.max_receive_age_s = limits
        self.latest = {}
        self.consumed = {}
        self.last_clock = None
        self.stats = {"received": 0, "rejected": 0, "published": 0, "clock_resets": 0}
        self.status = "waiting_for_sectors"
        self.last_error = ""

    def _validate(self, index, scan):
        if index not in range(3):
            raise ValueError("Unknown LiDAR sector index")
        if scan.frame_id != sector_frame(self.prefix, index):
            raise ValueError(f"Wrong sector frame: expected {sector_frame(self.prefix, index)}")
        if not math.isfinite(scan.stamp) or scan.stamp < 0:
            raise ValueError("Invalid sector timestamp")
        n = self.input_samples
        if np.asarray(scan.ranges).shape != (n,):
            raise ValueError(f"Wrong sector ray count: expected {n}")
        expected = (-SECTOR_HALF_FOV, SECTOR_HALF_FOV, 2 * SECTOR_HALF_FOV / (n - 1))
        actual = (scan.angle_min, scan.angle_max, scan.angle_increment)
        if any(not math.isfinite(v) or not math.isclose(v, e, rel_tol=2e-4, abs_tol=1e-5)
               for v, e in zip(actual, expected)):
            raise ValueError("Sector angular geometry differs from the configured 120-degree sensor")
        if (not math.isfinite(scan.range_min) or not math.isfinite(scan.range_max)
                or not 0 < scan.range_min < scan.range_max):
            raise ValueError("Invalid sector range limits")
        ranges = np.asarray(scan.ranges, dtype=float)
        if np.all(np.isfinite(ranges)) and np.all(np.abs(ranges - scan.range_min) <= 1e-6):
            # Observed OGRE failure: every beam is exactly the near clipping
            # plane. Reject this unhealthy sensor packet, not individual valid
            # near returns mixed with other ranges. No scan means HAL freshness
            # stops driving; publishing false obstacles would hide the failure.
            raise ValueError("LiDAR sector collapsed to range_min; renderer or sensor placement is unhealthy")

    def push(self, index, scan: SectorScan, received_wall: float):
        try:
            self._validate(index, scan)
            if not math.isfinite(received_wall):
                raise ValueError("Invalid receive time")
        except (ValueError, TypeError) as error:
            self.latest.pop(index, None)
            self.stats["rejected"] += 1
            self.last_error = str(error)
            raise
        previous = self.latest.get(index)
        if previous is not None and scan.stamp <= previous[0].stamp:
            self.stats["rejected"] += 1
            return False
        self.latest[index] = (scan, received_wall)
        self.stats["received"] += 1
        return True

    def merge(self, now_sim: float, now_wall: float) -> SectorScan | None:
        if not math.isfinite(now_sim) or not math.isfinite(now_wall):
            self.status = "invalid_clock"
            return None
        if self.last_clock is not None and now_sim < self.last_clock:
            self.reset_clock(now_sim)
            return None
        self.last_clock = now_sim
        if len(self.latest) != 3:
            self.status = "waiting_for_sectors"
            return None
        scans = [self.latest[i][0] for i in range(3)]
        if any(scan.stamp <= self.consumed.get(i, -math.inf) for i, scan in enumerate(scans)):
            self.status = "waiting_for_new_sectors"
            return None
        if any(not -.05 <= now_sim - scan.stamp <= self.max_sensor_age_s for scan in scans):
            self.status = "stale_sensor_timestamp"
            return None
        if any(not 0 <= now_wall - self.latest[i][1] <= self.max_receive_age_s for i in range(3)):
            self.status = "stale_receive_time"
            return None
        if max(s.stamp for s in scans) - min(s.stamp for s in scans) > self.max_skew_s + 1e-9:
            self.status = "unsynchronized_sectors"
            return None
        rmin, rmax = scans[0].range_min, scans[0].range_max
        if any(not math.isclose(s.range_min, rmin, abs_tol=1e-6) or
               not math.isclose(s.range_max, rmax, abs_tol=1e-6) for s in scans[1:]):
            self.status = "range_limits_disagree"
            return None
        output = np.full(self.samples, np.inf, dtype=np.float32)
        step = 2 * math.pi / self.samples
        for index, scan in enumerate(scans):
            ranges = np.asarray(scan.ranges, dtype=np.float32)
            valid = np.isfinite(ranges) & (ranges >= rmin) & (ranges <= rmax)
            angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment + SECTOR_YAWS[index]
            bins = np.floor(((angles + math.pi) % (2 * math.pi)) / step + .5).astype(int) % self.samples
            np.minimum.at(output, bins[valid], ranges[valid])
        output[~np.isfinite(output)] = np.nan
        self.consumed = {i: scan.stamp for i, scan in enumerate(scans)}
        self.stats["published"] += 1
        self.status = "ok"
        return SectorScan(min(s.stamp for s in scans), lidar_frame(self.prefix), -math.pi,
                          math.pi-step, step, rmin, rmax, output)

    def reset_clock(self, now_sim):
        self.latest.clear()
        self.consumed.clear()
        self.stats["clock_resets"] += 1
        self.last_clock = now_sim
        self.status = "clock_reset"

    def diagnostics(self):
        return {"status": self.status, "prefix": self.prefix, "samples": self.samples,
                "input_samples_per_sector": self.input_samples, "last_error": self.last_error, **self.stats}
