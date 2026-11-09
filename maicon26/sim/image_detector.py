"""Image-only reference-photo matcher for the local Gazebo fixture.

This is a simulation test detector, not a trained real-world object detector.
It sees only RGB pixels and the public reference photographs: no layout,
model IDs, poses, face labels from the world, or synthetic Frame metadata.
"""
from pathlib import Path
import time
import cv2
import numpy as np

from ..perception.detector import Detector
from ..types import Detection

LABELS = {
    "m3_ammo_box": "AMMO_BOX", "m3_drum": "DRUM",
    "m3_military_truck": "TRUCK", "m3_missile": "MISSILE",
    "m3_mortar": "MORTAR", "m3_tank": "TANK",
    "target_comms": "COMM_FACILITY", "target_outpost": "GUARD_POST",
    "target_radar": "RADAR_FACILITY",
    "deco_barrier_wall": "BARRIER", "deco_city_tree": "TREE",
    "deco_field_bunker": "POSITION", "deco_ruined_building": "BUILDING",
}


def overlap(a, b):
    x = max(0., min(a[2], b[2])-max(a[0], b[0]))
    y = max(0., min(a[3], b[3])-max(a[1], b[1]))
    common = x*y
    return common / max(1., (a[2]-a[0])*(a[3]-a[1])+(b[2]-b[0])*(b[3]-b[1])-common)


class ReferencePhotoDetector(Detector):
    def __init__(self, directory):
        cv2.setNumThreads(1)
        self.sift = cv2.SIFT_create(nfeatures=1600, contrastThreshold=.018)
        self.matcher = cv2.BFMatcher(cv2.NORM_L2)
        self.templates = []
        for stem, label in LABELS.items():
            gray = cv2.imread(str(Path(directory)/(stem+".jpg")), cv2.IMREAD_GRAYSCALE)
            if gray is None:
                raise ValueError("Missing reference photograph: "+stem)
            canonical = cv2.resize(gray, (256, 256), interpolation=cv2.INTER_AREA)
            points, descriptors = [], []
            for size in (128, 256):
                kp, desc = self.sift.detectAndCompute(cv2.resize(gray, (size, size)), None)
                if desc is not None:
                    points.extend([(k.pt[0]*256/size, k.pt[1]*256/size) for k in kp])
                    descriptors.extend(desc)
            self.templates.append((label, canonical, np.float32(points), np.float32(descriptors)))
        self.stats = {"frames": 0, "detections": 0, "max_duration_s": 0., "classes": {}}
        from ..perception.wingbot_vision import WingbotGroundDetector
        self.ugv_detector = WingbotGroundDetector()

    def detect(self, frame):
        if frame.image is None:
            return []
        started = time.monotonic()
        gray = cv2.cvtColor(frame.image, cv2.COLOR_BGR2GRAY)
        kp, desc = self.sift.detectAndCompute(gray, None)
        candidates = []
        if desc is not None and len(desc) >= 8:
            scene = np.float32([k.pt for k in kp])
            for label, template, ref_points, ref_desc in self.templates:
                matches = self.matcher.knnMatch(desc, ref_desc, k=2)
                good = [m for pair in matches if len(pair)==2 for m,n in [pair] if m.distance < .70*n.distance]
                # Multiple visible copies of a photograph are fitted separately.
                for _ in range(5):
                    if len(good) < 8:
                        break
                    src = np.float32([ref_points[m.trainIdx] for m in good])
                    dst = np.float32([scene[m.queryIdx] for m in good])
                    H, mask = cv2.findHomography(src, dst, cv2.RANSAC, 2.5, maxIters=1200, confidence=.995)
                    if H is None or mask is None or int(mask.sum()) < 8:
                        break
                    inliers = mask.ravel().astype(bool)
                    good = [m for m, keep in zip(good, inliers) if not keep]
                    quad = cv2.perspectiveTransform(np.float32([[[0,0],[255,0],[255,255],[0,255]]]), H)[0]
                    if not np.isfinite(quad).all() or not cv2.isContourConvex(quad):
                        continue
                    area = abs(cv2.contourArea(quad))
                    edges = np.linalg.norm(quad-np.roll(quad,1,axis=0),axis=1)
                    if area < 220 or area > gray.size*.85 or edges.min() < 9 or edges.max()/edges.min() > 8:
                        continue
                    # Feature inliers must cover the photo, not one shared
                    # patch of background among two different class pictures.
                    hull = cv2.convexHull(src[inliers])
                    if cv2.contourArea(hull) < 256*256*.045:
                        continue
                    warped = cv2.warpPerspective(gray, np.linalg.inv(H), (256,256))
                    visibility = cv2.warpPerspective(np.ones_like(gray),np.linalg.inv(H),(256,256)) > 0
                    if visibility.mean() < .88:
                        continue
                    a=cv2.resize(warped,(48,48)).astype(float).ravel()
                    b=cv2.resize(template,(48,48)).astype(float).ravel()
                    if min(a.std(),b.std()) < 8:
                        continue
                    ncc=float(np.corrcoef(a,b)[0,1])
                    if not np.isfinite(ncc) or ncc < .60:
                        continue
                    lo,hi=quad.min(axis=0),quad.max(axis=0)
                    box=tuple(float(v) for v in (*lo,*hi))
                    extra={"method":"rgb_reference_homography", "image_correlation":ncc,
                           "inliers":int(inliers.sum()), "quad":quad.tolist()}
                    candidates.append(Detection(label,min(.99,.50+.48*ncc),box,frame.stamp,extra=extra))
        accepted = []
        for det in sorted(candidates,key=lambda d:d.conf,reverse=True):
            if any(overlap(det.box,old.box) > .35 for old in accepted):
                continue
            if det.cls in ("GUARD_POST","RADAR_FACILITY","COMM_FACILITY"):
                from ..perception.receiver import find_receivers
                from ..perception.led import find_signal_led
                sensors=find_receivers(frame.image,det.box,max_n=1,kind="target")
                if sensors:
                    det.extra["sensors"]=sensors
                det.extra["led_on"]=find_signal_led(frame.image,det.box)
            accepted.append(det)
        accepted.extend(self.ugv_detector.detect(frame))
        self.stats["frames"]+=1
        self.stats["detections"]+=len(accepted)
        self.stats["max_duration_s"]=max(self.stats["max_duration_s"],time.monotonic()-started)
        for d in accepted:
            self.stats["classes"][d.cls]=self.stats["classes"].get(d.cls,0)+1
        return accepted


def make_wingbot_detector(cfg):
    return ReferencePhotoDetector(Path(__file__).resolve().parents[2]/"ros2/maicon_arena_gz/media/textures/faces")
