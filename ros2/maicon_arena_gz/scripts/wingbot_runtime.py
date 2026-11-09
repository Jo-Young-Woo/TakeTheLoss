#!/usr/bin/env python3
"""Read the model's exported frames; no independently maintained mount values."""
from __future__ import annotations

import importlib.util
import hashlib
import math
from pathlib import Path

import numpy as np
import yaml


def rotation_rpy(matrix):
    """RPY for a proper rotation, including the URDF optical-frame singularity."""
    pitch = math.atan2(-matrix[2, 0], math.hypot(matrix[0, 0], matrix[1, 0]))
    if abs(math.cos(pitch)) > 1e-10:
        return (math.atan2(matrix[2, 1], matrix[2, 2]), pitch,
                math.atan2(matrix[1, 0], matrix[0, 0]))
    return (math.atan2(-matrix[1, 2], matrix[1, 1]), pitch, 0.)


class WingbotFrames:
    def __init__(self, path):
        self.path = Path(path).resolve()
        self.data = yaml.safe_load(self.path.read_text(encoding="utf-8"))
        if self.data.get("schema") != "wingbot_frames_v1":
            raise ValueError("Expected generated wingbot_frames_v1 metadata")
        package_root = (self.path.parent if (self.path.parent/"config/wingbot_mods.yaml").is_file()
                        else self.path.parent.parent)
        source = package_root/"config/wingbot_mods.yaml"
        if hashlib.sha256(source.read_bytes()).hexdigest() != self.data["source_sha256"]:
            raise ValueError("WINGBOT frames are stale; regenerate from wingbot_mods.yaml")
        module_path = package_root/"scripts/wingbot_geometry.py"
        spec = importlib.util.spec_from_file_location("maicon_wingbot_geometry", module_path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        self.forward = module.metadata_forward_kinematics
        self.semantic = self.data["semantic"]
        self.gimbal = self.data["pan_tilt"]
        self.joint_names = self.gimbal["pan_joint"], self.gimbal["tilt_joint"]
        self.zero = self.transforms()
        for key in ("laser", "camera", "lidar", "receiver_left", "receiver_right"):
            if self.semantic[key] not in self.zero:
                raise ValueError("Missing generated WINGBOT frame: "+key)

    def positions(self, pan=0., tilt=0.):
        return dict(zip(self.joint_names, (pan, tilt)))

    def transforms(self, pan=0., tilt=0.):
        return self.forward(self.data, self.positions(pan, tilt))

    def laser(self, pan=0., tilt=0.):
        matrix = self.transforms(pan, tilt)[self.semantic["laser"]]
        return tuple(matrix[:3, 3]), tuple(matrix[:3, 0])

    def receivers(self):
        for key in ("receiver_left", "receiver_right"):
            name = self.semantic[key]
            frame = self.data["frames"][name]
            facing = np.asarray(frame["facing"], dtype=float)
            facing /= np.linalg.norm(facing)
            yield tuple(self.zero[name][:3, 3]), float(frame["radius"]), tuple(facing)

    def tf_edges(self, pan, tilt):
        transforms = self.transforms(pan, tilt)
        for joint in self.data["joints"]:
            parent, child = joint["parent"], joint["child"]
            relative = np.linalg.inv(transforms[parent]) @ transforms[child]
            yield parent, child, tuple(relative[:3, 3]), rotation_rpy(relative[:3, :3])

    def collision_transforms(self, pan=0., tilt=0.):
        """Actual primitive boxes used to reject rays occluded by another UGV."""
        transforms = self.transforms(pan, tilt)
        for box in self.data["collision_boxes"]:
            # The export includes each collision's fixed local transform, so
            # current head rotation is inherited from its actual parent link.
            xyz, rpy = box["local_xyz"], box["local_rpy"]
            cr, cp, cy = (math.cos(v) for v in rpy)
            sr, sp, sy = (math.sin(v) for v in rpy)
            local = np.eye(4)
            local[:3, :3] = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                            [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                            [-sp, cp*sr, cp*cr]]
            local[:3, 3] = xyz
            yield transforms[box["parent"]] @ local, box["size"]


def crop_scan(ranges, angle_min, angle_increment, frame_id, metadata):
    """Mask the vendor clockwise sector in the actual ROS scan frame.

    ugv_ws ldlidar/src/demo.cpp crops point.angle before reversing the index
    when laser_scan_dir=True (launch/stl27l.launch.py). Thus raw225..315 is
    ROS45..135 degrees, not ROS225..315. Combined with the preserved URDF
    +pi/2 lidar yaw, this masks the rear135..225 body sector. NaN means unknown;
    neither missing observations nor body occlusion establish free space.
    """
    lidar = metadata["lidar"]
    if not lidar.get("crop_audited"):
        raise ValueError("WINGBOT LiDAR crop convention has not been audited")
    if frame_id != lidar["crop_frame"] or frame_id != metadata["semantic"]["lidar"]:
        raise ValueError("LiDAR crop received a different frame")
    sign = float(lidar["ros_angle_sign"])
    if sign not in (-1., 1.) or not math.isfinite(angle_min) or not math.isfinite(angle_increment) or angle_increment == 0:
        raise ValueError("Invalid LiDAR angle convention")
    lower, upper = (float(v) % 360 for v in lidar["crop_deg"])
    output, masked = list(ranges), []
    for index in range(len(output)):
        raw = (math.degrees(angle_min+index*angle_increment)/sign) % 360
        inside = lower-1e-7 <= raw <= upper+1e-7 if lower <= upper else raw >= lower-1e-7 or raw <= upper+1e-7
        if inside:
            output[index] = math.nan
            masked.append(index)
    return output, masked


def mask_self_returns(ranges, angle_min, angle_increment, frame_id, frames):
    """Reject finite hits on generated body surfaces; leave other rays intact.

    GPU rays quantize the azimuth to a render pixel. Sample within half a scan
    bin and allow 1 mm surface tolerance. Occluded rays remain unknown (NaN),
    never free. Fixed visual triangle sections cover antennae and other parts
    seen by GPU rays; the moving head is not frozen into this static geometry.
    No arbitrary rear sector or broad minimum-range clipping.
    """
    if frame_id != frames.semantic['lidar']:
        raise ValueError('Self-return filter received a different scan frame')
    if not math.isfinite(angle_increment) or angle_increment == 0 or not math.isfinite(angle_min):
        raise ValueError('Invalid scan geometry')
    matrix = frames.zero[frame_id]
    origin = matrix[:3, 3]
    yaw = math.atan2(matrix[1, 0], matrix[0, 0])
    output, masked = list(ranges), []
    primitives = frames.data['lidar'].get('self_return_cylinders', [])
    sections = frames.data['lidar'].get('self_return_sections', [])
    segments = []
    for section in sections:
        if section['frame'] != 'base_footprint':
            raise ValueError('Self-return geometry must use base_footprint')
        if abs(float(section['z'])-origin[2]) < 1e-8:
            segments.extend(section['segments'])
    if segments:
        lines = np.asarray(segments, dtype=float)-origin[:2]
        starts, vectors = lines[:, 0], lines[:, 1]-lines[:, 0]
        max_self_range = float(np.linalg.norm(lines, axis=2).max())+.001
    else:
        max_self_range = 0.
    for index, distance in enumerate(output):
        if not math.isfinite(distance) or distance <= 0:
            continue
        angle = angle_min + index * angle_increment + yaw
        if distance <= max_self_range:
            angles = angle + np.linspace(-abs(angle_increment)/2, abs(angle_increment)/2, 9)
            directions = np.stack((np.cos(angles), np.sin(angles)), axis=1)
            # Ray t*d = segment start + u*vector. Only actual finite visual
            # surfaces can match; a close obstacle in front remains intact.
            denominator = directions[:, 0, None]*vectors[None, :, 1]-directions[:, 1, None]*vectors[None, :, 0]
            good = np.abs(denominator)>1e-12
            divisor = np.where(good, denominator, 1.)
            ray_t = (starts[:, 0]*vectors[:, 1]-starts[:, 1]*vectors[:, 0])[None, :]/divisor
            segment_u = (starts[None, :, 0]*directions[:, 1, None]-starts[None, :, 1]*directions[:, 0, None])/divisor
            valid = good & (ray_t > 0) & (segment_u >= 0) & (segment_u <= 1)
            near = np.min(np.where(valid, ray_t, np.inf), axis=0)
            far = np.max(np.where(valid, ray_t, -np.inf), axis=0)
            if np.any((near-.001 <= distance) & (distance <= far+.001)):
                output[index] = math.nan
                masked.append(index)
                continue
        for shape in primitives:
            if shape['frame'] != 'base_footprint':
                raise ValueError('Self-return geometry must use base_footprint')
            if not shape['z'][0] <= origin[2] <= shape['z'][1]:
                continue
            dx, dy = np.asarray(shape['center_xy']) - origin[:2]
            radius = float(shape['radius'])
            if abs(distance-math.hypot(dx, dy)) > radius+.001:
                continue
            # Bound all first intersections in the angular pixel footprint.
            near = []
            for offset in np.linspace(-abs(angle_increment)/2, abs(angle_increment)/2, 9):
                a = angle + offset
                along = dx*math.cos(a) + dy*math.sin(a)
                cross = dx*math.sin(a) - dy*math.cos(a)
                if along > 0 and abs(cross) <= radius:
                    near.append(along - math.sqrt(max(0., radius*radius-cross*cross)))
            if near and min(near)-.001 <= distance <= max(near)+.001:
                output[index] = math.nan
                masked.append(index)
                break
    return output, masked
