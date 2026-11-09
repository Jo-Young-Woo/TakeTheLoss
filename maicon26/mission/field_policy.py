"""Time-dependent field policy, separate from sensed obstacle geometry.

S9.7 says any UGV can trigger the opponent's capture in its own base. The
225-second keepout is the requested team strategy, not an organizer prohibition.
All coordinates come from ArenaGeometry, including measured YAML-backed bases.
"""
from __future__ import annotations

import math
import numpy as np

from ..footprint import aabb_corners


def _penetration(corners, rectangle):
    """Shortest translation to separate an OBB from an AABB; zero if separate.

    Projection endpoint distances handle complete containment correctly. Using
    intersection width instead would allow a fully-inside robot to go deeper.
    """
    other = aabb_corners(*rectangle)
    edges = (corners[1] - corners[0], corners[2] - corners[1])
    axes = (np.array([1., 0.]), np.array([0., 1.]), *(v / np.linalg.norm(v) for v in edges))
    distances = []
    for axis in axes:
        a, b = corners @ axis, other @ axis
        if a.max() <= b.min() or b.max() <= a.min():
            return 0.
        distances.append(min(float(b.max() - a.min()), float(a.max() - b.min())))
    return min(distances)


class OwnBasePolicy:
    def __init__(self, cfg):
        self.cfg = cfg
        self.fp = cfg.budget().fp
        self.margin = cfg.budget().pose_term
        self.active = False
        self.team = None
        self.rectangles = {}
        self.escape_cuts = {}
        self.blocked_commands = 0

    def planning_mask(self, grid, pose, team, elapsed):
        """Already footprint-expanded centre keepout; never put it in SLAM.

        An initially overlapping base gets only a one-way outward half-corridor
        until the robot fully exits the expanded region. Its old/deeper half
        stays blocked to the planner. Output checks additionally require every
        sampled footprint to move no deeper into each actual base.
        """
        threshold = self.cfg.mission.forbid_own_bases_at
        enabled = (team in ("BLUE", "RED") and elapsed is not None
                   and math.isfinite(elapsed) and elapsed >= threshold >= 0)
        if not enabled:
            self.active, self.team = False, team
            self.rectangles, self.escape_cuts = {}, {}
            return None
        initial = not self.active or team != self.team
        self.active, self.team = True, team
        self.rectangles = {name: self.cfg.arena.base_rect(name) for name in self.cfg.arena.bases(team)}
        radius = self.fp.circumradius + self.margin + grid.res * math.sqrt(2) / 2
        xs = grid.ox + (np.arange(grid.nx) + .5) * grid.res
        ys = grid.oy + (np.arange(grid.ny) + .5) * grid.res
        x, y = np.meshgrid(xs, ys)
        mask = np.zeros(grid.L.shape, dtype=bool)
        if initial:
            self.escape_cuts = {}
        for name, (x0, y0, x1, y1) in self.rectangles.items():
            inside = x0-radius <= pose.x <= x1+radius and y0-radius <= pose.y <= y1+radius
            if initial and inside:
                self.escape_cuts[name] = pose.x
            elif name in self.escape_cuts and not inside:
                del self.escape_cuts[name]
            region = (x >= x0-radius) & (x <= x1+radius) & (y >= y0-radius) & (y <= y1+radius)
            if name in self.escape_cuts:
                cut = self.escape_cuts[name]
                region &= x < cut-grid.res/2 if team == "BLUE" else x > cut+grid.res/2
            mask |= region
        return mask

    def _safe_motion(self, pose, v, w, dt):
        horizon = max(float(dt), self.cfg.mission.own_base_guard_horizon_s,
                      abs(v) / max(self.cfg.robot.a_max, 1e-6))
        step = max(1e-4, self.cfg.mission.own_base_guard_step_m)
        # Half-step padding covers the maximum unsampled corner displacement.
        padding = self.margin + step/2
        corner_radius = math.hypot(self.fp.half_l + padding, self.fp.half_w + padding)
        reach = corner_radius + abs(v)*horizon
        rectangles = {name: rect for name, rect in self.rectangles.items()
                      if math.hypot(max(rect[0]-pose.x, pose.x-rect[2], 0.),
                                    max(rect[1]-pose.y, pose.y-rect[3], 0.)) <= reach}
        if not rectangles:
            return True
        previous = {name: self._base_depth(self.fp.corners(pose.x, pose.y, pose.th, padding), rect)
                    for name, rect in rectangles.items()}
        samples = max(1, math.ceil((abs(v) + abs(w) * corner_radius) * horizon / step))
        for index in range(1, samples + 1):
            t = horizon * index / samples
            th = pose.th + w*t
            if abs(w) < 1e-9:
                x, y = pose.x + v*t*math.cos(pose.th), pose.y + v*t*math.sin(pose.th)
            else:
                x = pose.x + v/w*(math.sin(th)-math.sin(pose.th))
                y = pose.y - v/w*(math.cos(th)-math.cos(pose.th))
            corners = self.fp.corners(x, y, th, padding)
            for name, rect in rectangles.items():
                depth = self._base_depth(corners, rect)
                if depth > previous[name] + 1e-8:
                    return False
                previous[name] = depth
        return True

    def _base_depth(self, corners, rectangle):
        if _penetration(corners, rectangle) <= 0.:
            return 0.
        # The outer wall is not an exit. Measure intrusion from the field-side
        # edge, so moving inward toward the arena is allowed even near that wall
        # and moving deeper cannot hide behind a closer parallel side edge.
        return (max(0., rectangle[2]-float(corners[:, 0].min())) if self.team == "BLUE"
                else max(0., float(corners[:, 0].max())-rectangle[0]))

    def limit_velocity(self, pose, v, w, dt):
        """Check the complete translating/rotating footprint after command ramps."""
        if not self.active or (abs(v) < 1e-12 and abs(w) < 1e-12):
            return v, w
        if not all(math.isfinite(n) for n in (pose.x, pose.y, pose.th, v, w, dt)):
            self.blocked_commands += 1
            return 0., 0.
        if self._safe_motion(pose, v, w, dt):
            return v, w
        self.blocked_commands += 1
        # Preserve a safe alignment turn instead of immobilising the planner.
        return (0., w) if abs(w) > 1e-12 and self._safe_motion(pose, 0., w, dt) else (0., 0.)

    def escape_goal(self, grid, pose):
        """Outward centreline goal only while an initial overlap needs clearing."""
        if not self.active or not self.escape_cuts:
            return None
        radius = self.fp.circumradius + self.margin + 2*grid.res
        name = min(self.escape_cuts, key=lambda n: abs(pose.x - self.escape_cuts[n]))
        x0, y0, x1, y1 = self.rectangles[name]
        return ((x1 + radius if self.team == "BLUE" else x0 - radius),
                min(max(pose.y, y0), y1))
