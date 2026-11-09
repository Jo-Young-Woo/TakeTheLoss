"""Bounded LiDAR localization against the explicitly fixed MAICON infrastructure.

Only the published 13 fixed footprints and outside walls are map inputs. Random
M3 cubes, robots and their classes are unknown outliers. This is local tracking
from a known starting base, never global localization in a symmetric arena.
"""
from __future__ import annotations

import math
import numpy as np

from ..types import Pose2D, Scan
from ..util import wrap


def fixed_infrastructure_rectangles():
    from ..sim.fixed_maicon import STRUCTURES
    return [(s.x-s.sx/2, s.y-s.sy/2, s.x+s.sx/2, s.y+s.sy/2) for s in STRUCTURES]


class ArenaEdgeMap:
    def __init__(self, width=6.12, height=5.0, rectangles=None, resolution=.025):
        self.width, self.height, self.res = width, height, resolution
        self.rectangles = tuple(fixed_infrastructure_rectangles() if rectangles is None else rectangles)
        self.segments = []
        for x0, y0, x1, y1 in ((0., 0., width, height), *self.rectangles):
            self.segments.extend(((x0, y0, x0, y1), (x1, y0, x1, y1),
                                  (x0, y0, x1, y0), (x0, y1, x1, y1)))
        self.ox = self.oy = -.3
        xs = np.arange(self.ox, width+.3+resolution, resolution)
        ys = np.arange(self.oy, height+.3+resolution, resolution)
        xx, yy = np.meshgrid(xs, ys)
        distances, _, _ = self.nearest(np.column_stack([xx.ravel(), yy.ravel()]))
        self.field = distances.reshape(xx.shape)

    def nearest(self, points):
        points = np.asarray(points, dtype=float)
        best = np.full(len(points), np.inf)
        closest = np.zeros_like(points)
        normals = np.zeros_like(points)
        for x0, y0, x1, y1 in self.segments:
            vertical = x0 == x1
            qx = np.full(len(points), x0) if vertical else np.clip(points[:, 0], x0, x1)
            qy = np.clip(points[:, 1], y0, y1) if vertical else np.full(len(points), y0)
            dx, dy = points[:, 0]-qx, points[:, 1]-qy
            dd = dx*dx+dy*dy
            take = dd < best
            best[take] = dd[take]
            closest[take, 0], closest[take, 1] = qx[take], qy[take]
            # Orthogonal point-to-edge error; use radial normals at endpoints.
            normal = np.column_stack([np.ones(len(points)) if vertical else np.zeros(len(points)),
                                      np.zeros(len(points)) if vertical else np.ones(len(points))])
            corner = ((points[:, 1] < y0) | (points[:, 1] > y1)) if vertical else ((points[:, 0] < x0) | (points[:, 0] > x1))
            denom = np.sqrt(np.maximum(dd, 1e-18))
            normal[corner, 0], normal[corner, 1] = dx[corner]/denom[corner], dy[corner]/denom[corner]
            normals[take] = normal[take]
        return np.sqrt(best), closest, normals

    def distances(self, points):
        shape = points.shape[:-1]
        flat = points.reshape(-1, 2)
        fx, fy = (flat[:, 0]-self.ox)/self.res, (flat[:, 1]-self.oy)/self.res
        ix, iy = np.floor(fx).astype(int), np.floor(fy).astype(int)
        ny, nx = self.field.shape
        valid = (ix >= 0) & (ix < nx-1) & (iy >= 0) & (iy < ny-1)
        out = np.full(len(flat), 1.0)
        a, b, xx, yy = fx[valid]-ix[valid], fy[valid]-iy[valid], ix[valid], iy[valid]
        out[valid] = ((1-a)*(1-b)*self.field[yy, xx] + a*(1-b)*self.field[yy, xx+1]
                      + (1-a)*b*self.field[yy+1, xx] + a*b*self.field[yy+1, xx+1])
        return out.reshape(shape)


class ArenaLocalizer:
    """Encoder prediction plus robust, bounded matching to fixed arena edges."""
    def __init__(self, start: Pose2D, edge_map: ArenaEdgeMap | None = None):
        self.map = edge_map or ArenaEdgeMap()
        self.pose = start.copy()
        self._odom = None
        self._stamp = None
        self._last_good = None
        self._last_full = None
        self._since_full_distance = self._since_full_rotation = 0.
        self._full_position_bound = .02
        self.partial_frames = 0
        self.report = {"valid": False, "status": "waiting_for_scan"}
        self.accepted = self.rejected = 0

    @staticmethod
    def _transform(points, pose):
        c, s = math.cos(pose[2]), math.sin(pose[2])
        return points @ np.array([[c, s], [-s, c]]) + pose[:2]

    def _predict(self, odom, dt):
        if self._odom is None:
            self._odom = odom.copy()
            return True
        old, self._odom = self._odom, odom.copy()
        dx, dy, da = odom.x-old.x, odom.y-old.y, wrap(odom.th-old.th)
        # Reject telemetry resets and implausible jumps, never compensate by a
        # global map search. Ordinary skid is corrected by the next LiDAR frame.
        if math.hypot(dx, dy) > .04 + .8*dt or abs(da) > .08 + 3.*dt:
            return False
        self._since_full_distance += math.hypot(dx, dy)
        self._since_full_rotation += abs(da)
        c, s = math.cos(old.th), math.sin(old.th)
        fx, fy = c*dx+s*dy, -s*dx+c*dy
        c, s = math.cos(self.pose.th), math.sin(self.pose.th)
        self.pose = Pose2D(self.pose.x+c*fx-s*fy, self.pose.y+s*fx+c*fy, wrap(self.pose.th+da))
        return True

    def update(self, scan: Scan, odom: Pose2D):
        stamp = float(scan.stamp)
        if not all(math.isfinite(v) for v in (stamp, odom.x, odom.y, odom.th)):
            return self._reject(stamp, "invalid_input", immediate=True)
        if self._stamp is not None and stamp <= self._stamp:
            return self._reject(stamp, "nonmonotonic_scan", immediate=True)
        dt = .1 if self._stamp is None else stamp-self._stamp
        self._stamp = stamp
        if dt > 1. or not self._predict(odom, dt):
            self._odom = odom.copy()
            return self._reject(stamp, "odometry_jump_or_gap", immediate=True)
        valid = scan.valid() & (scan.ranges > .15) & (scan.ranges < 7.8)
        indices = np.flatnonzero(valid)
        if len(indices) < 24:
            return self._reject(stamp, "insufficient_ranges")
        indices = indices[np.linspace(0, len(indices)-1, min(140, len(indices))).astype(int)]
        points = np.column_stack([scan.ranges[indices]*np.cos(scan.angles[indices]),
                                   scan.ranges[indices]*np.sin(scan.angles[indices])])
        prediction = np.array([self.pose.x, self.pose.y, self.pose.th])
        # Search only near a known start / previous accepted location. A .4m
        # repeated cube cannot justify a jump across the arena.
        offsets = np.array([(x, y, a) for x in (-.16, -.08, 0., .08, .16)
                            for y in (-.16, -.08, 0., .08, .16) for a in (-.16, -.08, 0., .08, .16)])
        proposals = prediction + offsets
        angle = proposals[:, 2:3]
        ex = proposals[:, 0:1] + np.cos(angle)*points[None, :, 0] - np.sin(angle)*points[None, :, 1]
        ey = proposals[:, 1:2] + np.sin(angle)*points[None, :, 0] + np.cos(angle)*points[None, :, 1]
        distances = self.map.distances(np.stack([ex, ey], axis=-1))
        scores = np.mean(np.exp(-.5*(distances/.055)**2), axis=1)
        scores -= .004 * np.sum((offsets/.16)**2, axis=1)
        inside = ((proposals[:, 0] >= .05) & (proposals[:, 0] <= self.map.width-.05)
                  & (proposals[:, 1] >= .05) & (proposals[:, 1] <= self.map.height-.05))
        scores[~inside] = -1.
        best = int(np.argmax(scores))
        estimate = proposals[best].copy()
        far = ((np.linalg.norm(proposals[:, :2]-estimate[:2], axis=1) >= .15)
               | (np.abs(proposals[:, 2]-estimate[2]) >= .15))
        margin = float(scores[best] - scores[far].max()) if far.any() else 1.
        for _ in range(8):
            transformed = self._transform(points, estimate)
            d, q, normal = self.map.nearest(transformed)
            keep = d < .13
            if keep.sum() < 20:
                break
            p, n = points[keep], normal[keep]
            c, s = math.cos(estimate[2]), math.sin(estimate[2])
            jt = n[:, 0]*(-s*p[:, 0]-c*p[:, 1]) + n[:, 1]*(c*p[:, 0]-s*p[:, 1])
            jac = np.column_stack([n, jt])
            residual = np.sum(n*(transformed[keep]-q[keep]), axis=1)
            weight = np.minimum(1., .035/np.maximum(np.abs(residual), 1e-9))
            prior = np.diag([.15, .15, .1])
            step = np.linalg.solve(jac.T @ (weight[:, None]*jac) + prior,
                                   -jac.T @ (weight*residual) - prior @ (estimate-prediction))
            step[:2] = np.clip(step[:2], -.06, .06)
            step[2] = np.clip(step[2], -.05, .05)
            estimate += step
            estimate = np.clip(estimate, prediction-np.array([.22, .22, .25]), prediction+np.array([.22, .22, .25]))
            if np.linalg.norm(step) < 1e-4:
                break
        transformed = self._transform(points, estimate)
        d, _, normal = self.map.nearest(transformed)
        inlier = d < .075
        ratio = float(np.mean(inlier))
        rms = float(np.sqrt(np.mean(d[inlier]**2))) if inlier.any() else 1.
        if inlier.sum() >= 20:
            p, n = points[inlier], normal[inlier]
            c, s = math.cos(estimate[2]), math.sin(estimate[2])
            jt = n[:, 0]*(-s*p[:, 0]-c*p[:, 1]) + n[:, 1]*(c*p[:, 0]-s*p[:, 1])
            jac = np.column_stack([n, jt])
            eigen, axes = np.linalg.eigh(jac.T @ jac / len(jac))
            condition = float(eigen[-1]/max(eigen[0], 1e-12))
        else:
            eigen, axes, condition = np.array([0., 0., 0.]), np.eye(3), 1e12
        metrics = {"inlier_ratio": round(ratio, 4), "rms_m": round(rms, 5),
                   "condition": round(condition, 2), "ambiguity_margin": round(margin, 4),
                   "matched_rays": int(inlier.sum()), "total_rays": len(points)}
        if ratio < .28 or inlier.sum() < 24 or rms > .055:
            return self._reject(stamp, "insufficient_static_support", metrics)
        partial = condition > 500. or eigen[0] < .01 or (margin < .006 and condition > 80.)
        if partial:
            # A single wall observes distance normal to the wall and yaw, but
            # cannot observe translation along it. Only a previous full lock
            # licenses bounded encoder prediction in that one null direction.
            observable = eigen >= .02
            if (self._last_full is None or observable.sum() != 2
                    or abs(axes[2, 0]) > .15 or ratio < .60 or rms > .035):
                return self._reject(stamp, "ambiguous_or_degenerate_geometry", metrics)
            projection = axes[:, observable] @ axes[:, observable].T
            correction = projection @ (estimate-prediction)
            # Compare alternative fits only in the observed subspace. Equal
            # scores along the unobserved wall tangent are expected, whereas a
            # competing wall-normal/yaw solution still invalidates tracking.
            alternatives = (proposals-proposals[best]) @ projection
            separated = ((np.linalg.norm(alternatives[:, :2], axis=1) >= .12)
                         | (np.abs(alternatives[:, 2]) >= .12))
            observed_margin = float(scores[best]-scores[separated].max()) if separated.any() else 1.
            age = stamp-self._last_full
            # Engineering uncertainty budget, not a statistical confidence
            # claim: 6% of encoder travel, 2mm/s, 4mm/rad plus the last full
            # scan residual. Both travel and time are bounded independently.
            position_bound = (self._full_position_bound + .06*self._since_full_distance
                              + .002*age + .004*self._since_full_rotation)
            metrics.update(observed_ambiguity_margin=round(observed_margin, 4),
                           dead_reckoning_duration_s=round(age, 3),
                           unobserved_travel_m=round(self._since_full_distance, 4),
                           position_uncertainty_bound_m=round(position_bound, 4),
                           observable_dof=2)
            if observed_margin < .015 or np.linalg.norm(correction[:2]) > .08 or abs(correction[2]) > .08:
                return self._reject(stamp, "partial_pose_inconsistent", metrics, immediate=True)
            if age > 12. or position_bound > .12:
                return self._reject(stamp, "partial_uncertainty_exhausted", metrics, immediate=True)
            estimate = prediction + correction
            # Projection must not sacrifice the measured wall fit to retain
            # the encoder tangent; check the resulting endpoints again.
            projected_d, _, _ = self.map.nearest(self._transform(points, estimate))
            projected_inlier = projected_d < .075
            projected_rms = (float(np.sqrt(np.mean(projected_d[projected_inlier]**2)))
                             if projected_inlier.any() else 1.)
            if projected_inlier.mean() < .60 or projected_rms > .035:
                return self._reject(stamp, "partial_projection_inconsistent", metrics, immediate=True)
            metrics["rms_m"] = round(projected_rms, 5)
        else:
            metrics.update(observable_dof=3, dead_reckoning_duration_s=0.,
                           unobserved_travel_m=0., position_uncertainty_bound_m=round(max(.02, 2*rms), 4))
        if not (.05 <= estimate[0] <= self.map.width-.05 and .05 <= estimate[1] <= self.map.height-.05):
            return self._reject(stamp, "outside_arena", metrics, immediate=True)
        if partial:
            self.partial_frames += 1
        else:
            self._last_full = stamp
            self._since_full_distance = self._since_full_rotation = 0.
            self._full_position_bound = max(.02, 2*rms)
        self.pose = Pose2D(float(estimate[0]), float(estimate[1]), wrap(float(estimate[2])))
        self._last_good = stamp
        self.accepted += 1
        self.report = {"valid": True, "accepted": True, "stamp": stamp,
                       "status": "tracking_partial" if partial else "tracking", **metrics,
                       "pose": [self.pose.x, self.pose.y, self.pose.th], "accepted_frames": self.accepted,
                       "partial_frames": self.partial_frames, "rejected_frames": self.rejected,
                       "map_source": "published_fixed_infrastructure_only"}
        return dict(self.report)

    def _reject(self, stamp, reason, metrics=None, immediate=False):
        self.rejected += 1
        age = math.inf if self._last_good is None else stamp-self._last_good
        self.report = {"valid": bool(not immediate and 0 <= age <= .5), "accepted": False,
                       "stamp": stamp if math.isfinite(stamp) else None, "status": reason,
                       "last_good_age_s": age if math.isfinite(age) else None,
                       "pose": [self.pose.x, self.pose.y, self.pose.th], "accepted_frames": self.accepted,
                       "rejected_frames": self.rejected, **(metrics or {})}
        return dict(self.report)

    def map_to_odom(self, odom: Pose2D) -> Pose2D:
        """TF map<-odom such that map<-odom<-base equals the accepted estimate."""
        yaw = wrap(self.pose.th-odom.th)
        c, s = math.cos(yaw), math.sin(yaw)
        return Pose2D(self.pose.x-c*odom.x+s*odom.y, self.pose.y-s*odom.x-c*odom.y, yaw)
