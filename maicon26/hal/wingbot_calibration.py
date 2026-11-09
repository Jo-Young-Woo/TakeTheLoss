"""WINGBOT Fortress/Ogre2 camera contract, not a physical-camera calibration.

Fortress's Ogre2 RenderPassSystem does not implement ImageBrownDistortion.
CameraInfo still advertises SDF D: applying that D would distort pinhole pixels.
This explicit simulation profile preserves the requested D as provenance while
using effective D=0. Changing renderers requires a new measured profile.
"""
from __future__ import annotations

from collections import deque
import hashlib
import json
import math
from pathlib import Path

import numpy as np


def _package():
    local = Path(__file__).resolve().parents[2] / "ros2/wingbot_description"
    if local.is_dir():
        return local
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory("wingbot_description"))


def configure_wingbot_simulation(cfg, frames_path=None):
    """Apply before constructing Agent/CameraModel. No map/layout inputs.

    Geometry comes from generated model metadata, checked against its immutable
    YAML. The runtime consumes actual CameraInfo and measured TF/joint feedback.
    Output K is fixed before Agent creation; runtime K changes remap pixels into
    that same coordinate system, never silently invalidate cached CameraModels.
    """
    import yaml
    frames_path = Path(frames_path) if frames_path else _package()/"config/frames.yaml"
    data = yaml.safe_load(frames_path.read_text(encoding="utf-8"))
    package = frames_path.parent if (frames_path.parent/"config").is_dir() else frames_path.parent.parent
    source = package/"config/wingbot_mods.yaml"
    if data.get("schema") != "wingbot_frames_v1" or hashlib.sha256(source.read_bytes()).hexdigest() != data["source_sha256"]:
        raise ValueError("WINGBOT camera metadata does not match immutable model YAML")
    mods = yaml.safe_load(source.read_text(encoding="utf-8"))
    width, height = mods["camera"]["resolution"]
    f = width / (2 * math.tan(mods["camera"]["hfov_rad"] / 2))
    K = (f, 0., width/2, 0., f, height/2, 0., 0., 1.)
    camera, laser = (data["frames"][data["semantic"][key]] for key in ("camera", "laser"))
    # Exported zero-pose lens axes are canonical +X forward, +Y left, +Z up.
    if max(abs(v) for item in (camera, laser) for v in item["rpy"]) > 1e-5:
        raise ValueError("WINGBOT zero-pose camera/laser axes require a new profile")
    offset = (np.asarray(laser["xyz"])-camera["xyz"]).tolist()
    settings_path = Path(__file__).resolve().parents[2]/"config/wingbot_sim_camera.json"
    settings = json.loads(settings_path.read_text(encoding="utf-8"))
    profile = dict(settings, source_sha256=data["source_sha256"],
                   frames_sha256=hashlib.sha256(frames_path.read_bytes()).hexdigest(),
                   camera_frame=data["semantic"]["camera"],
                   optical_frame=data["semantic"]["camera_optical"],
                   laser_frame=data["semantic"]["laser"],
                   pan_joint=data["pan_tilt"]["pan_joint"], tilt_joint=data["pan_tilt"]["tilt_joint"],
                   camera_zero_xyz=camera["xyz"], laser_from_camera_xyz=offset,
                   width=width, height=height, output_K=list(K))
    cfg.camera.width, cfg.camera.height, cfg.camera.K = width, height, K
    cfg.camera.hfov_deg = math.degrees(mods["camera"]["hfov_rad"])
    cfg.camera.calibration_profile = profile
    cfg.robot.cam_height = float(camera["xyz"][2])
    cfg.aim.laser_lateral_m, cfg.aim.laser_vertical_m = offset[1], offset[2]
    cfg.aim.laser_offset_pan_deg = cfg.aim.laser_offset_tilt_deg = 0.
    cfg.hal.pan_sign, cfg.hal.tilt_sign = -1., 1.
    return profile


def transform_parts(transform):
    p, q = transform.translation, transform.rotation
    t = np.asarray([p.x, p.y, p.z], dtype=float)
    xyzw = np.asarray([q.x, q.y, q.z, q.w], dtype=float)
    if not np.isfinite(t).all() or not np.isfinite(xyzw).all() or abs(np.linalg.norm(xyzw)-1) > 1e-3:
        raise ValueError("invalid measured camera TF")
    x, y, z, w = xyzw / np.linalg.norm(xyzw)
    R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                  [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                  [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    return t, R


class WingbotCameraCalibration:
    """Measured camera metadata and timestamped joints, independent of commands."""
    def __init__(self, profile):
        self.profile = profile
        if profile.get("schema") != "wingbot_fortress_ogre2_camera_v1" or profile.get("scope") != "simulation_only":
            raise ValueError("unsupported camera calibration profile")
        self.output_K = np.asarray(profile["output_K"], dtype=float).reshape(3, 3)
        self.info = None
        self._map = None
        self.joints = deque(maxlen=150)

    def camera_info(self, msg):
        """Consume raw K, never rectified P; D is recorded, not reapplied."""
        self.info, self._map = None, None
        k = np.asarray(msg.k, dtype=float)
        d = np.asarray(msg.d, dtype=float)
        p = self.profile
        if ((int(msg.width), int(msg.height)) != (p["width"], p["height"]) or
                msg.header.frame_id != p["optical_frame"]):
            raise ValueError("CameraInfo resolution/frame does not match WINGBOT profile")
        if k.shape != (9,) or not np.isfinite(k).all() or min(k[0], k[4]) <= 0 or not np.allclose(k[6:], [0, 0, 1]):
            raise ValueError("invalid raw CameraInfo K")
        if not np.isfinite(d).all() or str(msg.distortion_model) not in ("plumb_bob", ""):
            raise ValueError("unsupported CameraInfo distortion metadata")
        if not (d.size == 0 or d.size == 5 and
                (np.allclose(d, p["requested_D"], atol=1e-8) or np.allclose(d, 0., atol=1e-8))):
            raise ValueError("CameraInfo D changed; renderer verification required")
        K = k.reshape(3, 3)
        # A large intrinsic change cannot silently masquerade as this model.
        if max(abs(K[0, 0]/self.output_K[0, 0]-1), abs(K[1, 1]/self.output_K[1, 1]-1)) > .01:
            raise ValueError("CameraInfo focal length differs from generated model")
        if max(abs(K[0, 2]-self.output_K[0, 2]), abs(K[1, 2]-self.output_K[1, 2])) > 1.:
            raise ValueError("CameraInfo principal point differs from generated model")
        self.info = {"raw_K": k.tolist(), "advertised_D": d.tolist(), "effective_D": [0.]*5,
                     "distortion_model": str(msg.distortion_model), "output_K": self.output_K.ravel().tolist()}
        if not np.allclose(K, self.output_K, rtol=0, atol=1e-7):
            import cv2
            self._map = cv2.initUndistortRectifyMap(K, np.zeros(5), np.eye(3), self.output_K,
                                                 (p["width"], p["height"]), cv2.CV_32FC1)

    def image(self, image, frame_id):
        p = self.profile
        if self.info is None:
            raise ValueError("waiting for valid CameraInfo")
        if frame_id != p["optical_frame"] or image.shape != (p["height"], p["width"], 3):
            raise ValueError("Image/CameraInfo frame or resolution mismatch")
        if self._map is None:
            return image
        import cv2
        return cv2.remap(image, *self._map, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

    def joint_state(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        values = {n.split("::")[-1]: v for n, v in zip(msg.name, msg.position)}
        p = self.profile
        if not all(k in values for k in (p["pan_joint"], p["tilt_joint"])):
            raise ValueError("missing measured WINGBOT pan/tilt joints")
        pan, tilt = -math.degrees(values[p["pan_joint"]]), math.degrees(values[p["tilt_joint"]])
        if stamp <= 0 or not all(math.isfinite(v) for v in (stamp, pan, tilt)):
            raise ValueError("invalid measured WINGBOT joint feedback")
        if self.joints and stamp < self.joints[-1][0]:
            self.joints.clear()  # simulation reset invalidates the old epoch
        self.joints.append((stamp, pan, tilt))
        return pan, tilt

    def joints_at(self, stamp):
        if not self.joints:
            raise ValueError("waiting for measured joints")
        closest = min(self.joints, key=lambda row: abs(row[0]-stamp))
        if abs(closest[0]-stamp) > self.profile["joint_max_age_s"]:
            raise ValueError("measured joints stale at image timestamp")
        for a, b in zip(self.joints, list(self.joints)[1:]):
            if a[0] <= stamp <= b[0] and b[0] > a[0]:
                k = (stamp-a[0])/(b[0]-a[0])
                return np.asarray(a[1:])*(1-k)+np.asarray(b[1:])*k
        return np.asarray(closest[1:])

    def measured_pose(self, stamp, base_camera, camera_laser):
        """Actual +X lens TF yields internal left-positive pan/up-positive tilt."""
        t, R = transform_parts(base_camera)
        offset, laser_R = transform_parts(camera_laser)
        axis = R[:, 0]
        angles = np.degrees([math.atan2(axis[1], axis[0]), math.atan2(axis[2], np.hypot(axis[0], axis[1]))])
        if np.max(abs(angles-self.joints_at(stamp))) > self.profile["tf_joint_tolerance_deg"]:
            raise ValueError("camera TF and measured joints disagree")
        if np.linalg.norm(offset-self.profile["laser_from_camera_xyz"]) > .002 or not np.allclose(laser_R, np.eye(3), atol=1e-4):
            raise ValueError("measured camera/laser boresight differs from model profile")
        return tuple(angles), {"camera_xyz_base": t.tolist(), "camera_rotation_base": R.tolist(),
                               "laser_from_camera_xyz": offset.tolist(), "pan_tilt_source": "image_timestamp_tf_and_joint_feedback"}
