"""Calibrated box-top detection for the fixed Gazebo overhead camera.

This is an image detector, not access to the simulator's object list. It uses
the known camera pose/intrinsics and the 0.4 m top plane to map image rectangles
to arena footprints. Neutral grey (legacy world) and black (measured arena)
tops and fixed lighting are Gazebo assumptions; this detector is deliberately
not enabled for real UAV images without validating those assumptions.
Side pictures and their classes are never inferred from an overhead view.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List

import numpy as np

from ..link.base import UavBox
from .uav_calibration import FourCornerCalibration, GridCalibration


@dataclass(frozen=True)
class FixedUavCalibration:
    arena_width: float = 6.12
    arena_height: float = 5.0
    camera_x: float = 3.06
    camera_y: float = 2.5
    camera_z: float = 2.465  # model height 2.5 minus sensor mount offset .035
    top_z: float = .4
    horizontal_fov: float = 2.4

    def pixels_per_metre(self, width: int) -> float:
        return width / (2.0 * math.tan(self.horizontal_fov / 2.0)
                        * (self.camera_z - self.top_z))


class FixedUavLayoutDetector:
    """Find square/rectangular neutral tops using only a BGR camera frame.

    Touching fixed blocks can form an L-shaped component. Splitting at changes
    in its vertical span recovers the separate rectangular tops. Candidates
    must agree with the supported .4/.8 m dimensions and occupy their fitted
    rectangle; unrelated markings and partly occluded tops are rejected.
    """

    def __init__(self, calibration: FixedUavCalibration | FourCornerCalibration | GridCalibration | None = None) -> None:
        self.calibration = calibration or FixedUavCalibration()

    def detect(self, bgr: np.ndarray) -> List[UavBox]:
        import cv2

        if (not isinstance(bgr, np.ndarray) or bgr.dtype != np.uint8 or
                bgr.ndim != 3 or bgr.shape[2] != 3 or min(bgr.shape[:2]) < 240):
            return []
        height, width = bgr.shape[:2]
        cal = self.calibration
        if isinstance(cal, (FourCornerCalibration, GridCalibration)):
            bgr, scale = cal.rectify_top(bgr)
            height, width = bgr.shape[:2]
            origin_x, origin_y = 0., cal.arena_height
        else:
            # The legacy pose/intrinsics calibration is specific to the nadir
            # 16:9 camera. Explicit four-corner calibrations allow other views.
            if abs(width / height - 16.0 / 9.0) > .02:
                return []
            scale = cal.pixels_per_metre(width)
            origin_x = cal.camera_x - width / (2. * scale)
            origin_y = cal.camera_y + height / (2. * scale)
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        grey = ((hsv[:, :, 1] <= 60) & (hsv[:, :, 2] >= 75) &
                (hsv[:, :, 2] <= 155)).astype(np.uint8) * 255
        # Saturation is a relative channel difference and is unstable near
        # zero: JPEG noise such as BGR(0, 1, 0) has saturation255 despite being
        # visually black. Use absolute brightness for black tops, with a lower
        # ceiling to keep the darker Ogre2 floor out of the same component.
        black = (hsv[:, :, 2] <= 18).astype(np.uint8) * 255
        # Process separately. Combining the two ranges would connect grey tops
        # to the dark legacy floor and erase their measured boundaries.
        boxes = self._components(grey, scale, origin_x, origin_y, cal)
        for candidate in self._black_components(black, scale, origin_x, origin_y, cal):
            if not any(abs(candidate.cx-b.cx) < .08 and abs(candidate.cy-b.cy) < .08
                       and candidate.sx == b.sx and candidate.sy == b.sy for b in boxes):
                boxes.append(candidate)
        return sorted(boxes, key=lambda box: (round(box.cx, 2), round(box.cy, 2)))

    @staticmethod
    def _black_components(mask, scale, origin_x, origin_y, cal):
        """Tile opaque top evidence from the edge farthest from the camera.

        A blank black vertical side projects toward the optical centre and can
        join its black top. The far contour edge still lies on the known top
        plane. Anchoring .4 m cells there prevents that side from shifting or
        inflating the footprint. Only cells actually filled by image evidence
        survive; absent/occluded cells are never inferred from a layout.
        """
        import cv2

        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if isinstance(cal, (FourCornerCalibration, GridCalibration)):
            camera_x, camera_y = cal.camera_center[:2]
        else:
            camera_x, camera_y = cal.camera_x, cal.camera_y
        camera_pixel = ((camera_x-origin_x)*scale, (origin_y-camera_y)*scale)
        side = .4*scale
        boxes = []
        for contour in contours:
            x, y, width, height = cv2.boundingRect(contour)
            # Walls and unseen pixels form arena-sized components; filling
            # their interior holes would otherwise invent obstacle evidence.
            if max(width, height) > 3.2*scale or cv2.contourArea(contour) < .12*scale**2:
                continue
            component = np.zeros_like(mask)
            cv2.drawContours(component, [contour], -1, 255, thickness=cv2.FILLED)
            from_right = x+width/2 > camera_pixel[0]
            from_bottom = y+height/2 > camera_pixel[1]
            for ix in range(math.ceil(width/side)):
                for iy in range(math.ceil(height/side)):
                    left = x+width-(ix+1)*side if from_right else x+ix*side
                    top = y+height-(iy+1)*side if from_bottom else y+iy*side
                    x0, x1, y0, y1 = round(left), round(left+side), round(top), round(top+side)
                    if x0 < 0 or y0 < 0 or x1 > mask.shape[1] or y1 > mask.shape[0]:
                        continue
                    filled = np.count_nonzero(component[y0:y1, x0:x1]) / ((x1-x0)*(y1-y0))
                    if filled < .93:
                        continue
                    cx, cy = origin_x+(left+side/2)/scale, origin_y-(top+side/2)/scale
                    if .35 <= cx <= cal.arena_width-.35 and .3 <= cy <= cal.arena_height-.3:
                        boxes.append(UavBox(cx, cy, .4, .4, min(.98, filled)))
        return boxes

    @staticmethod
    def _components(mask, scale, origin_x, origin_y, cal):
        import cv2

        # Segment complete tops first: cropping a base-side ROI here would
        # remove half of a valid .4 m cube centred only .6 m from the wall.
        # Arena bounds are applied to candidate centres below instead.
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        boxes = []
        for contour in contours:
            if cv2.contourArea(contour) < (.28 * scale) ** 2:
                continue
            component = np.zeros_like(mask)
            # The circular receiver on a target top is a hole, not a new box.
            cv2.drawContours(component, [contour], -1, 255, thickness=cv2.FILLED)
            x0, y0, cw, ch = cv2.boundingRect(contour)
            runs = []
            for x in range(x0, x0 + cw):
                active = np.flatnonzero(component[y0:y0 + ch, x])
                if active.size:
                    runs.append((x, int(active[0]) + y0, int(active[-1]) + y0 + 1))
            groups = []
            for x, lo, hi in runs:
                if groups and x == groups[-1][1] and abs(lo - groups[-1][2]) <= 2 and abs(hi - groups[-1][3]) <= 2:
                    groups[-1][1] = x + 1
                else:
                    groups.append([x, x + 1, lo, hi])
            for left, right, top, bottom in groups:
                sx_raw, sy_raw = (right - left) / scale, (bottom - top) / scale
                sx = min((.4, .8), key=lambda size: abs(sx_raw - size))
                sy = min((.4, .8), key=lambda size: abs(sy_raw - size))
                error = max(abs(sx - sx_raw), abs(sy - sy_raw))
                if error > .065 or (sx == .8 and sy == .8):
                    continue
                filled = np.count_nonzero(component[top:bottom, left:right]) / ((right - left) * (bottom - top))
                if filled < .93:
                    continue
                cx = origin_x + (left + right) / (2. * scale)
                cy = origin_y - (top + bottom) / (2. * scale)
                if not (.35 <= cx <= cal.arena_width - .35 and
                        .3 <= cy <= cal.arena_height - .3):
                    continue
                confidence = max(.5, .98 - 3.0 * error)
                boxes.append(UavBox(cx, cy, sx, sy, confidence))
        return boxes
