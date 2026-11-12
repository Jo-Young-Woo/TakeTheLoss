"""An aerial face inventory built from actual RGB frames, without SLAM/truth.

The top contour establishes candidate side geometry, not side-picture identity.
Optional ground-camera ledger states are attached as a separate evidence field.
"""
from __future__ import annotations

import math
import numpy as np

from ..config import Config
from ..link.schema import server_sector
from .coverage import CoverageMap
from .uav_layout import FixedUavLayoutDetector
from .wingbot_vision import WingbotUavDetector


SECTOR_ORDER = ("B-S1","B-S2","B-S3","CM","R-S1","R-S2","R-S3","OUTSIDE")


def sector_box_summary(faces):
    """Count unique boxes, not faces. U/H/missing evidence never completes one."""
    rows = {s:dict(boxes=0,identified_boxes=0,resolved_boxes=0,
                   identified_faces=0,eligible_faces=0,box_details=[]) for s in SECTOR_ORDER}
    by_box = {}
    for face in faces:
        by_box.setdefault(face['box'], {})[face['face']] = face
    for bid, directions in sorted(by_box.items()):
        box_faces = list(directions.values())
        sector = box_faces[0].get('sector') or 'OUTSIDE'
        eligible = [f for f in box_faces if f['image_state'] != 'excluded']
        identified = sum(f.get('ground_state') == 'identified' for f in eligible)
        resolved = bool(eligible) and all(f.get('ground_state') in ('identified','empty') for f in eligible)
        row = rows[sector]
        row['boxes'] += 1
        row['identified_boxes'] += int(identified > 0)
        row['resolved_boxes'] += int(resolved)
        row['identified_faces'] += identified
        row['eligible_faces'] += len(eligible)
        row['box_details'].append(dict(id=bid,identified_faces=identified,
            eligible_faces=len(eligible),all_eligible_resolved=resolved,
            ground_states={d:f.get('ground_state') for d,f in directions.items()}))
    return rows


class UavImageFaceLedger:
    def __init__(self, calibration, arena):
        if calibration.validation_report()["status"] != "within_tolerance":
            raise ValueError("UAV ledger requires independently validated calibration")
        if not (math.isclose(calibration.arena_width, arena.W) and math.isclose(calibration.arena_height, arena.H)):
            raise ValueError("UAV calibration and arena dimensions differ")
        cfg = Config()
        cfg.arena = arena
        cfg.explore.uav_face_registry = True
        cfg.explore.uav_census = True
        cfg.explore.uav_outer_faces = True
        cfg.explore.exhaustive_scoring_faces = True
        cfg.explore.uav_exclude_ugv = True
        cfg.explore.uav_map_freeze = False
        self.coverage = CoverageMap(cfg)
        self.arena = arena
        self.calibration = calibration
        self.detector = FixedUavLayoutDetector(calibration)
        self.ugv_detector = WingbotUavDetector(calibration)
        self.last_stamp = None
        self.frames = 0

    def update(self, image, stamp, ground=None, ground_stamp=None):
        if not math.isfinite(stamp) or stamp < 0:
            raise ValueError("Invalid image timestamp")
        if self.last_stamp is not None and stamp <= self.last_stamp:
            return None
        if image.shape[:2] != (self.calibration.image_size[1], self.calibration.image_size[0]):
            raise ValueError("UAV frame does not match calibration resolution")
        boxes = self.detector.detect(image)
        ugvs = self.ugv_detector.detect(image)
        # No occupancy grid, odometry, simulator layout or hidden model poses.
        self.coverage.ingest_uav_boxes(boxes, stamp, ugv_xy=[(u.x, u.y) for u in ugvs])
        self.last_stamp = stamp
        self.frames += 1
        snapshot = self.coverage.aerial_face_map()
        # Ground state must not come from later than the photographed frame.
        usable_ground = (ground is not None and ground_stamp is not None
                         and 0 <= stamp - ground_stamp <= 3.)
        ground_faces = ground.get("uav_map", {}).get("outer_faces", {}).get("faces", []) if usable_ground else []
        camera = self.calibration.camera_center
        for f in snapshot["faces"]:
            f["sector"] = server_sector(self.arena.sector_of(*f["center"])) or "OUTSIDE"
            f["key"] = f"U{f['box']:02d}-{f['face']}"
            f["image_state"] = "excluded" if f["state"] == "excluded" else "side_identity_unobserved"
            f["ground_state"] = None
            f["ground_box"] = None
            f["image_age_s"] = round(stamp - f["last_image_s"], 3)
            nx, ny = f["normal"]
            cx, cy = f["center"]
            sx, sy = f["size"]
            # Facing the optical centre is necessary but not sufficient for
            # visibility. Do not label an occluded side as seen/empty.
            facing = nx*(camera[0]-cx-nx*sx/2) + ny*(camera[1]-cy-ny*sy/2)
            f["camera_facing"] = facing > .01
            f["side_visibility"] = "unverified" if f["camera_facing"] else "back_facing"
            f["top_pixel_segments"] = [self.calibration.arena_to_pixels(s).tolist() for s in f["segments"]]
            f["floor_pixel_segments"] = [self.calibration.arena_to_pixels(s, 0.).tolist() for s in f["segments"]]
            candidates = [g for g in ground_faces if g["face"] == f["face"] and "center" in g and "size" in g
                          and math.dist(g["center"], f["center"]) < .12
                          and max(abs(a-b) for a,b in zip(g.get("size", []),f["size"])) < .1]
            if len(candidates) == 1:
                g = candidates[0]
                f["ground_state"], f["ground_box"] = g["state"], g["box"]
        snapshot.update(image_stamp_s=stamp, image_size=list(self.calibration.image_size),
                        frames_processed=self.frames, raw_box_candidates=len(boxes),
                        boxes_confirmed=len({f["box"] for f in snapshot["faces"]}),
                        source="/uav/image_raw RGB + measured calibration only",
                        geometry_uses_lidar=False, geometry_uses_world_truth=False,
                        camera_optical_center_arena_m=list(camera),
                        object_plane_z_m=self.calibration.object_plane_z,
                        layout=self.coverage.layout_diagnostics(),
                        ground_state_stamp_s=ground_stamp if usable_ground else None,
                        classification="UAV contour does not identify side pictures; ground_state is separate evidence")
        snapshot["sector_summary"] = sector_box_summary(snapshot["faces"])
        snapshot["ground_observer"] = "B-2"
        snapshot["sector_floor_polygons"] = {}
        for sector in SECTOR_ORDER[:-1]:
            x0,y0,x1,y1 = self.arena.sector_rect(sector)
            snapshot["sector_floor_polygons"][sector] = self.calibration.arena_to_pixels(
                [(x0,y0),(x1,y0),(x1,y1),(x0,y1)],0.).tolist()
        return snapshot


def draw_uav_image_ledger(image, snapshot, calibration, camera_description=None, video_stamp=None):
    """Preserve the native perspective; project top and floor planes separately."""
    import cv2
    h, w = image.shape[:2]
    canvas = np.full((h, w+420, 3), 24, np.uint8)
    canvas[:, :w] = image
    panel = w+12
    font = cv2.FONT_HERSHEY_SIMPLEX

    def text(label, y, color=(230,230,230), scale=.43):
        cv2.putText(canvas,label,(panel,y),font,scale,color,1,cv2.LINE_AA)

    text("UAV IMAGE FACE LEDGER",43,(80,220,255),.58)
    text(f"Camera arena XYZ: {tuple(round(v,2) for v in calibration.camera_center)}",58)
    if camera_description:
        text(f"Downward | FOV {camera_description['horizontal_fov_deg']:.0f} deg | {camera_description['update_rate_hz']:g} Hz",79)
    t=snapshot['image_stamp_s'] if video_stamp is None else video_stamp
    text(f"Video t={t:.2f}s | {snapshot['boxes_confirmed']} UAV boxes",103)
    text("Yellow: outer top edge (z=0.4m)",126,(0,220,255))
    text("Cyan dots: floor footprint (z=0m)",146,(255,200,70))
    text("UAV ? unknown side | X excluded",168)
    text("Ground: B-2 only | I identified / E empty",189)

    for sector, polygon in snapshot.get("sector_floor_polygons",{}).items():
        pts=np.rint(polygon).astype(np.int32)
        cv2.polylines(canvas,[pts],True,(120,120,110),1,cv2.LINE_AA)
        at=np.mean(pts,axis=0).astype(int)
        cv2.putText(canvas,sector,(int(at[0])-20,int(at[1])-5),font,.42,(220,220,180),1,cv2.LINE_AA)

    faces_by_box = {}
    for f in snapshot["faces"]:
        faces_by_box.setdefault(f["box"], []).append(f)
        color=(0,220,255) if f["image_state"] != "excluded" else (150,150,150)
        for top, floor in zip(f["top_pixel_segments"],f["floor_pixel_segments"]):
            pts=np.rint(top).astype(int)
            cv2.line(canvas,tuple(pts[0]),tuple(pts[1]),color,2,cv2.LINE_AA)
            for t in np.linspace(0.,1.,8):
                p=np.rint((1-t)*np.array(floor[0])+t*np.array(floor[1])).astype(int)
                if 0<=p[0]<w and 0<=p[1]<h:
                    cv2.circle(canvas,tuple(p),1,(255,200,70),-1)
    for bid, faces in sorted(faces_by_box.items()):
        f=faces[0]
        pixel=np.rint(calibration.arena_to_pixels([f["center"]])[0]).astype(int)
        if 0<=pixel[0]<w and 0<=pixel[1]<h:
            cv2.putText(canvas,f"U{bid:02d}",tuple(pixel),font,.43,(60,255,255),1,cv2.LINE_AA)
    text("SECTOR     BOXES   WITH I   ALL I/E",220,(80,220,255),.48)
    yy=248
    for sector,row in snapshot.get("sector_summary",{}).items():
        if sector=="OUTSIDE" and not row["boxes"]:
            continue
        text(f"{sector:7s}     {row['boxes']:2d}       {row['identified_boxes']:2d}       {row['resolved_boxes']:2d}",yy,scale=.46)
        ids="  ".join(f"U{b['id']:02d}:{b['identified_faces']}/{b['eligible_faces']}" for b in row["box_details"])
        text(ids or "(no confirmed box)",yy+18,(165,190,200),.38)
        yy+=46
    text("WITH I: at least one identified face",h-100,(150,190,220),.4)
    text("ALL I/E: every eligible side identified/empty",h-81,(150,190,220),.4)
    nadir=np.rint(calibration.arena_to_pixels([calibration.camera_center[:2]],0.))[0].astype(int)
    cv2.drawMarker(canvas,tuple(nadir),(255,90,230),cv2.MARKER_CROSS,16,1)
    text("Magenta cross: point below UAV",h-57,(255,140,230))
    text("Contour geometry is not M3 recognition.",h-36,(150,190,220))
    footer=(f"FINAL ledger @ t={snapshot['image_stamp_s']:.1f}s | mission ended"
            if snapshot.get("mission_complete") else "Real Gazebo frames | ledger sampled at 1 Hz")
    text(footer,h-15,(150,190,220),.4)
    return canvas
