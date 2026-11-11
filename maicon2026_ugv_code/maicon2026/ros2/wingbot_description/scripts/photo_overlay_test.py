#!/usr/bin/env python3
"""Compare actual Gazebo RGB and actual robot TF with the supplied PDF photo.

Image observations never consume model_xyz, model_px or projected TF positions.
Missing/ambiguous observations fail separately from excessive pixel error.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import cv2
import numpy as np
import yaml


PACKAGE = Path(__file__).resolve().parents[1]
PALETTE = {"laser": (255, 255, 0), "camera": (0, 160, 255),
           "receiver": (0, 255, 0), "iff": (255, 0, 255),
           "headlight": (0, 255, 255), "lidar": (90, 90, 255),
           "depth": (0, 160, 255)}  # OpenCV BGR, matching the supplied legend.


def rotation_rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])


def quaternion_matrix(q):
    x, y, z, w = map(float, q)
    norm = math.sqrt(x*x+y*y+z*z+w*w)
    if norm < 1e-9:
        raise ValueError("Invalid observed quaternion")
    x, y, z, w = x/norm, y/norm, z/norm, w/norm
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def project_world(points, camera):
    """Gazebo camera +X forward/+Y left/+Z up -> image right/down."""
    c = camera["gazebo_camera"]
    pose = np.asarray(c["pose_xyzrpy"], dtype=float)
    rotation = rotation_rpy(*pose[3:])
    optical = np.vstack((-rotation[:, 1], -rotation[:, 2], rotation[:, 0]))
    camera_points = (np.asarray(points, dtype=float)-pose[:3]) @ optical.T
    if np.any(camera_points[:, 2] <= 0):
        raise ValueError("Observed TF point is behind the camera")
    width, height = map(int, c["image_wh"])
    focal = width/(2*math.tan(float(c["hfov_rad"])/2))
    # Pixel-centre convention: the supplied fit uses W/2,H/2. Ogre raster
    # centres introduce at most half a pixel, retained rather than refitted.
    return camera_points[:, :2]/camera_points[:, 2:] * focal + [width/2, height/2]


def image_silhouette(image):
    if image is None or image.ndim != 3 or image.shape[2] != 3:
        raise ValueError("Expected an actual 3-channel camera image")
    # Fixed threshold, no optimized overlap, hole filling or shape projection.
    return image.max(axis=2) > 3


def lower_silhouette_iou(observed, reference, row=640):
    if observed.shape != reference.shape or reference.dtype != np.bool_:
        raise ValueError("Photo mask shape/type differs from actual camera image")
    a, b = observed[row:], reference[row:]
    union = int(np.count_nonzero(a | b))
    return float(np.count_nonzero(a & b)/union) if union else 0.


def _roi(image, point, rx, ry=None):
    ry = rx if ry is None else ry
    x, y = map(float, point)
    x0, x1 = max(0, int(x-rx)), min(image.shape[1], int(x+rx+1))
    y0, y1 = max(0, int(y-ry)), min(image.shape[0], int(y+ry+1))
    return image[y0:y1, x0:x1], x0, y0


def _components(mask, minimum=12):
    count, labels, stats, centers = cv2.connectedComponentsWithStats(mask.astype(np.uint8), 8)
    return [(centers[i], int(stats[i, cv2.CC_STAT_AREA]), labels == i, stats[i])
            for i in range(1, count) if stats[i, cv2.CC_STAT_AREA] >= minimum]


def _point_result(point, method, count, pixels, contour=None):
    return {"status": "observed", "pixel": [float(v) for v in point],
            "method": method, "candidate_count": int(count), "evidence_pixels": int(pixels),
            "contour": contour or []}


def detect_feature(image, name, photo_pixel, team="RED"):
    """Measure RGB features in an annotation-defined ROI, never a TF-defined ROI.

    The ROI is wider than the acceptance tolerance. Its centre is not returned
    on detection failure; real displaced pixels therefore fail the pixel gate.
    """
    radius = 100 if name.startswith("receiver") else 65
    crop, x0, y0 = _roi(image, photo_pixel, radius)
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    if name in ("laser_lens_center", "camera_lens_center"):
        b, g, r = np.moveaxis(crop.astype(float), 2, 0)
        mask = (b > 15) & (b > 1.25*g) & (b > 1.5*r)
        method, maximum = "actual_blue_glass_component_centroid", 4500
    elif name.startswith(("receiver", "headlight")) or name == "camera_led":
        # Fixed photo-scene lighting makes white parts >135 while neighbouring
        # gray CAD faces can exceed 100. Do not join them into the white object.
        mask = (hsv[:, :, 1] < 55) & (hsv[:, :, 2] > 135)
        method, maximum = "actual_pale_surface_component_centroid", 25000 if name.startswith("receiver") else 5000
    elif name.startswith("iff_left_sleeve"):
        crop, x0, y0 = _roi(image, [photo_pixel[0], 190], 45, 190)
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        hue = hsv[:, :, 0]
        hue_mask = ((hue < 12) | (hue > 168)) if team == "RED" else ((hue > 95) & (hue < 135))
        mask = hue_mask & (hsv[:, :, 1] > 100) & (hsv[:, :, 2] > 25)
        components = _components(mask, 30)
        # BLUE sleeves share the lens hue. Require the measured tall coloured
        # sleeve sections rather than including nearby circular lens patches.
        components = [c for c in components if c[3][cv2.CC_STAT_HEIGHT] >= 20
                      and c[3][cv2.CC_STAT_HEIGHT] >= c[3][cv2.CC_STAT_WIDTH]*.8]
        if not components:
            return {"status": "missing", "method": "actual_team_sleeve_boundary", "pixel": None}
        main = max(components, key=lambda c: c[1])
        components = [c for c in components if abs(c[0][0]-main[0][0]) <= max(8, main[3][cv2.CC_STAT_WIDTH]*.5)]
        # All disconnected sections of the same coloured sleeve, including
        # its black-band gaps, are measured from pixels in the narrow column.
        points = np.column_stack(np.where(np.logical_or.reduce([v[2] for v in components])))
        edge_y = int(points[:, 0].min() if name.endswith("top") else points[:, 0].max())
        edge = points[np.abs(points[:, 0]-edge_y) <= 1]
        point = [x0+float(np.median(edge[:, 1])), y0+edge_y]
        return _point_result(point, "actual_team_sleeve_boundary", len(components), len(points))
    elif name == "iff_left_tip":
        crop, x0, y0 = _roi(image, [photo_pixel[0], 38], 28, 38)
        parts = _components(crop.max(axis=2) > 3, 10)
        parts = [p for p in parts if p[3][cv2.CC_STAT_HEIGHT] >= 10]
        if not parts:
            return {"status": "missing", "method": "actual_rod_top_silhouette", "pixel": None}
        best = min(parts, key=lambda p: abs(x0+p[0][0]-photo_pixel[0]))
        yy, xx = np.where(best[2])
        top = int(yy.min())
        if y0+top == 0:
            return {"status": "clipped", "method": "actual_rod_top_silhouette", "pixel": None}
        return _point_result([x0+float(np.median(xx[yy <= top+1])), y0+top],
                             "actual_rod_top_silhouette", len(parts), best[1])
    else:
        return _detect_edge_feature(image, name, photo_pixel)
    parts = [p for p in _components(mask) if p[1] <= maximum]
    if not parts:
        return {"status": "missing", "method": method, "pixel": None}
    best = min(parts, key=lambda p: np.linalg.norm(p[0]+[x0, y0]-photo_pixel))
    contours, _ = cv2.findContours(best[2].astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    outline = max(contours, key=cv2.contourArea).reshape(-1, 2)+[x0, y0]
    return _point_result(best[0]+[x0, y0], method, len(parts), best[1], outline.tolist())


def _flat_face_feature(image, name, photo_pixel):
    """Measure a visible flat CAD face, using its RGB boundary, not TF.

    The depth camera has no painted centre mark in the supplied STL. Its front
    rectangle's diagonal intersection is observable instead. The LiDAR cap is
    an ellipse; horizontal opening separates equal-colour vertical side faces.
    """
    x, y = np.rint(photo_pixel).astype(int)
    color = image[y, x].astype(int)
    if color.max() <= 5:
        return None
    mask = (np.abs(image.astype(int)-color).max(axis=2) <= 2).astype(np.uint8)
    if name == "lidar_top_center":
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((1, 31), np.uint8))
    count, labels, stats, centers = cv2.connectedComponentsWithStats(mask, 8)
    candidates = []
    for i in range(1, count):
        left, top, width, height, area = stats[i]
        if area < 100 or np.linalg.norm(centers[i]-photo_pixel) > 80:
            continue
        contours, _ = cv2.findContours((labels == i).astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        contour = max(contours, key=cv2.contourArea)
        if name == "lidar_top_center":
            if not (35 <= width <= 180 and 5 <= height <= 40 and width/height >= 3 and len(contour) >= 15):
                continue
            ellipse = cv2.fitEllipse(contour)
            point = np.asarray(ellipse[0])
            method = "actual_flat_cap_region_ellipse"
        elif name == "depth_camera_front_ctr":
            if not (80 <= width <= 300 and 30 <= height <= 170 and area >= 1500):
                continue
            polygon = cv2.approxPolyDP(contour, .015*cv2.arcLength(contour, True), True)
            if len(polygon) != 4 or not cv2.isContourConvex(polygon):
                continue
            corners = polygon.reshape(4, 2).astype(float)
            # Projective rectangle centre = intersection of observed diagonals.
            first = np.cross([*corners[0], 1], [*corners[2], 1])
            second = np.cross([*corners[1], 1], [*corners[3], 1])
            crossing = np.cross(first, second)
            if abs(crossing[2]) < 1e-9:
                continue
            point = crossing[:2]/crossing[2]
            contour = polygon
            method = "actual_front_rectangle_diagonal_intersection"
        else:
            return None
        candidates.append((point, method, int(area), contour))
    if not candidates:
        return None
    point, method, area, contour = min(candidates, key=lambda c: np.linalg.norm(c[0]-photo_pixel))
    return _point_result(point, method, len(candidates), area, contour.reshape(-1, 2).tolist())


def _detect_edge_feature(image, name, photo_pixel):
    """Require actual closed elliptical image edges for the CAD-only features."""
    flat = _flat_face_feature(image, name, photo_pixel)
    if flat is not None:
        return flat
    crop, x0, y0 = _roi(image, photo_pixel, 82 if name == "lidar_top_center" else 55)
    gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 4, 14)
    contours, _ = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    candidates = []
    for contour in contours:
        if len(contour) < 15:
            continue
        ellipse = cv2.fitEllipse(contour)
        center, axes, angle = ellipse
        minor, major = sorted(axes)
        if not (5 <= minor <= 75 and 15 <= major <= 155):
            continue
        if name == "lidar_top_center" and minor/major > .65:
            continue
        if name != "lidar_top_center" and (minor/major < .3 or major > 70):
            continue
        points = contour.reshape(-1, 2).astype(float)-center
        rotation = rotation_rpy(0, 0, math.radians(angle))[:2, :2]
        unit = (points @ rotation)/np.asarray(axes)*2
        residual = float(np.median(np.abs(np.linalg.norm(unit, axis=1)-1)))
        if residual > .12:
            continue
        candidates.append((np.asarray(center)+[x0, y0], contour, residual))
    if not candidates:
        return {"status": "missing", "method": "actual_closed_ellipse_edges", "pixel": None}
    best = min(candidates, key=lambda c: np.linalg.norm(c[0]-photo_pixel))
    result = _point_result(best[0], "actual_closed_ellipse_edges", len(candidates), len(best[1]),
                           (best[1].reshape(-1, 2)+[x0, y0]).tolist())
    result["ellipse_normalized_residual"] = best[2]
    return result


def evaluate(image, photo_mask, camera, keypoints, world_points, team="RED"):
    expected_shape = tuple(reversed(camera["gazebo_camera"]["image_wh"]))
    if image.shape[:2] != expected_shape:
        raise ValueError("Camera resolution is not the exact supplied photo resolution")
    observations = []
    for keypoint in keypoints["keypoints"]:
        name, photo = keypoint["name"], np.asarray(keypoint["photo_px"], dtype=float)
        tolerance = float(keypoints["tolerance"]["receiver_px" if name.startswith("receiver") else "default_px"])
        point = world_points.get(name)
        projected = None if point is None else project_world([point], camera)[0]
        measured = detect_feature(image, name, photo, team)
        tf_error = None if projected is None else float(np.linalg.norm(projected-photo))
        image_error = None if measured["pixel"] is None else float(np.linalg.norm(np.asarray(measured["pixel"])-photo))
        observations.append({"name": name, "photo_pixel": photo.tolist(), "tolerance_px": tolerance,
            "actual_tf_world": point, "tf_projected_pixel": None if projected is None else projected.tolist(),
            "tf_error_px": tf_error, "image_error_px": image_error, "image_observation": measured,
            "passed": bool(tf_error is not None and image_error is not None and
                           tf_error <= tolerance and image_error <= tolerance)})
    iou = lower_silhouette_iou(image_silhouette(image), photo_mask)
    debug_pixels = int(np.count_nonzero((image[:, :, 1] > 180) & (image[:, :, 0] < 30) & (image[:, :, 2] < 30)))
    return {"passed": bool(len(observations) == 12 and iou >= .93 and debug_pixels == 0 and all(o["passed"] for o in observations)),
            "silhouette": {"row_start": 640, "iou": iou, "minimum": .93, "rgb_threshold": 3, "passed": iou >= .93},
            "scene_artifacts": {"unexpected_green_debug_pixels": debug_pixels, "passed": debug_pixels == 0},
            "keypoints": observations, "observed_count": sum(o["image_observation"]["pixel"] is not None for o in observations),
            "method_limit": "RGB component/edge measurements in photo-annotation ROIs; no fitted camera, TF-guided pixel snapping, or YAML parameter optimization."}


def write_comparison(image, photo, report, output, team):
    output.mkdir(parents=True, exist_ok=True)
    if team == "BLUE":
        cv2.imwrite(str(output / "model_blue.png"), image)
        return
    cv2.imwrite(str(output / "photo_render_RED.png"), image)
    h, w = image.shape[:2]
    side = np.zeros((h+50, 2*w+20, 3), dtype=np.uint8)
    side[50:, :w], side[50:, w+20:] = photo, image
    cv2.putText(side, "PDF p21 original", (12, 32), cv2.FONT_HERSHEY_SIMPLEX, .8, (255, 255, 255), 2)
    cv2.putText(side, "Actual Gazebo RGB - fixed supplied camera", (w+32, 32), cv2.FONT_HERSHEY_SIMPLEX, .7, (255, 255, 255), 2)
    cv2.imwrite(str(output / "model_vs_pdf.png"), side)
    overlay = photo.copy()
    silhouette = image_silhouette(image).astype(np.uint8)
    boundary = cv2.morphologyEx(silhouette, cv2.MORPH_GRADIENT, np.ones((3, 3), np.uint8)) > 0
    overlay[boundary] = (255, 160, 80)
    for point in report["keypoints"]:
        name, observation = point["name"], point["image_observation"]
        color = next((value for prefix, value in PALETTE.items() if name.startswith(prefix)), (255, 255, 255))
        if observation.get("contour"):
            cv2.polylines(overlay, [np.asarray(observation["contour"], dtype=np.int32)], True, color, 1)
        if observation["pixel"] is not None:
            cv2.drawMarker(overlay, tuple(np.rint(observation["pixel"]).astype(int)), color, cv2.MARKER_CROSS, 12, 2)
        if point["tf_projected_pixel"] is not None:
            cv2.circle(overlay, tuple(np.rint(point["tf_projected_pixel"]).astype(int)), 5, color, 1)
    cv2.putText(overlay, "Actual RGB contours/crosses; TF projections=circle", (10, h-18),
                cv2.FONT_HERSHEY_SIMPLEX, .55, (255, 255, 255), 1)
    cv2.imwrite(str(output / "outline_on_photo.png"), overlay)


def make_photo_world(robot_sdf, camera, renderer="ogre2"):
    root = ET.Element("sdf", version="1.9")
    world = ET.SubElement(root, "world", name="wingbot_photo")
    ET.SubElement(world, "gravity").text = "0 0 0"
    physics = ET.SubElement(world, "physics", name="photo", type="dart")
    ET.SubElement(physics, "max_step_size").text = ".004"
    ET.SubElement(physics, "real_time_factor").text = "1"
    for filename, name in (("physics", "Physics"), ("user-commands", "UserCommands"),
                           ("scene-broadcaster", "SceneBroadcaster"), ("sensors", "Sensors")):
        plugin = ET.SubElement(world, "plugin", filename=f"ignition-gazebo-{filename}-system", name=f"ignition::gazebo::systems::{name}")
        if name == "Sensors":
            ET.SubElement(plugin, "render_engine").text = renderer
    scene = ET.SubElement(world, "scene")
    ET.SubElement(scene, "ambient").text = ".8 .8 .8 1"
    ET.SubElement(scene, "background").text = "0 0 0 1"
    ET.SubElement(scene, "shadows").text = "false"
    ET.SubElement(scene, "grid").text = "false"
    ET.SubElement(scene, "origin_visual").text = "false"
    light = ET.SubElement(world, "light", name="photo_key_light", type="directional")
    ET.SubElement(light, "cast_shadows").text = "false"
    # Light SDF defaults visualize=true. Fortress can render its green gizmo
    # into sensor images even when the camera excludes the GUI visibility bit.
    ET.SubElement(light, "visualize").text = "false"
    ET.SubElement(light, "diffuse").text = ".6 .6 .6 1"
    ET.SubElement(light, "specular").text = ".1 .1 .1 1"
    ET.SubElement(light, "direction").text = "-.3 .5 -.8"
    source = ET.fromstring(robot_sdf)
    model = source if source.tag == "model" else source.find("model")
    if model is None:
        raise ValueError("URDF conversion did not produce a Gazebo model")
    model.set("name", "wingbot")
    pose = model.find("pose")
    if pose is None:
        pose = ET.SubElement(model, "pose")
    pose.text = "0 0 0 0 0 0"
    world.append(model)
    model = ET.SubElement(world, "model", name="photo_camera")
    ET.SubElement(model, "static").text = "true"
    spec = camera["gazebo_camera"]
    ET.SubElement(model, "pose").text = " ".join(map(str, spec["pose_xyzrpy"]))
    link = ET.SubElement(model, "link", name="camera_link")
    sensor = ET.SubElement(link, "sensor", name="photo", type="camera")
    ET.SubElement(sensor, "always_on").text = "true"
    ET.SubElement(sensor, "update_rate").text = "2"
    ET.SubElement(sensor, "topic").text = "/wingbot/photo/image"
    camera_node = ET.SubElement(sensor, "camera")
    ET.SubElement(camera_node, "horizontal_fov").text = str(spec["hfov_rad"])
    # Local RenderTypes.hh: ALL=0x0fffffff, GUI=0x1. Exclude render-only
    # axes/light gizmos from sensor pixels while retaining physical visuals.
    ET.SubElement(camera_node, "visibility_mask").text = str(0x0FFFFFFF & ~0x1)
    image = ET.SubElement(camera_node, "image")
    ET.SubElement(image, "width").text = str(spec["image_wh"][0])
    ET.SubElement(image, "height").text = str(spec["image_wh"][1])
    ET.SubElement(image, "format").text = "R8G8B8"
    clip = ET.SubElement(camera_node, "clip")
    ET.SubElement(clip, "near").text = ".01"
    ET.SubElement(clip, "far").text = "10"
    return ET.tostring(root, encoding="unicode")


def capture(output, package=PACKAGE, team="RED", timeout=35.):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import Image, JointState
    from tf2_msgs.msg import TFMessage
    from tf2_ros import Buffer, TransformListener

    camera = json.loads((package / "config/photo_fit/cam_fit.json").read_text())
    keypoints = yaml.safe_load((package / "config/photo_fit/photo_keypoints.yaml").read_text())
    frames = yaml.safe_load((package / "config/frames.yaml").read_text())
    mappings = frames["photo_keypoints"]
    state = {"image": None, "frames": 0, "joints": {}, "pose": None}
    rclpy.init()
    node = Node("wingbot_photo_acceptance")
    buffer = Buffer()
    listener = TransformListener(buffer, node)

    def receive_image(message):
        if message.encoding not in ("rgb8", "bgr8"):
            raise ValueError("Unexpected photo camera encoding")
        pixels = np.frombuffer(message.data, np.uint8).reshape(message.height, message.step)
        pixels = pixels[:, :message.width*3].reshape(message.height, message.width, 3)
        state["image"] = cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR) if message.encoding == "rgb8" else pixels.copy()
        state["frames"] += 1
        state["image_stamp"] = message.header.stamp.sec+message.header.stamp.nanosec*1e-9

    def receive_joints(message):
        state["joints"] = dict(zip(message.name, message.position))

    def receive_pose(message):
        for t in message.transforms:
            if t.child_frame_id == "wingbot":
                p, q = t.transform.translation, t.transform.rotation
                state["pose"] = {"xyz": [p.x, p.y, p.z], "xyzw": [q.x, q.y, q.z, q.w]}

    node.create_subscription(Image, "/wingbot/photo/image", receive_image, qos_profile_sensor_data)
    node.create_subscription(JointState, "/joint_states", receive_joints, qos_profile_sensor_data)
    node.create_subscription(TFMessage, "/wingbot/photo/poses", receive_pose, qos_profile_sensor_data)
    error, points, transforms = None, {}, {}
    try:
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            if state["frames"] < 3 or not state["pose"] or not state["joints"]:
                continue
            points, transforms = {}, {}
            pose = state["pose"]
            world_rotation = quaternion_matrix(pose["xyzw"])
            for keypoint in keypoints["keypoints"]:
                name = keypoint["name"]
                # Photo annotations include explanatory (+y)/(-y) suffixes;
                # metadata frame identifiers use the canonical side name.
                mapping = mappings.get(name, mappings.get(name.split("(", 1)[0]))
                if mapping is None:
                    continue
                try:
                    t = buffer.lookup_transform("base_footprint", mapping["frame"], Time()).transform
                except Exception:
                    continue
                position = np.array([t.translation.x, t.translation.y, t.translation.z])
                quaternion = [t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w]
                local = np.asarray(mapping.get("local_xyz", [0, 0, 0]), dtype=float)
                point = position + quaternion_matrix(quaternion) @ local
                points[name] = (np.asarray(pose["xyz"]) + world_rotation @ point).tolist()
                transforms[name] = {"frame": mapping["frame"], "translation": position.tolist(),
                                     "quaternion": quaternion, "local_xyz": local.tolist()}
            if len(points) == 12:
                break
        if state["frames"] < 3 or not state["pose"] or len(points) != 12:
            error = "Missing actual camera frames, Gazebo pose, joint state, or one of the 12 TF transforms"
    finally:
        del listener
        node.destroy_node()
        rclpy.try_shutdown()
    output.mkdir(parents=True, exist_ok=True)
    if state["image"] is None:
        report = {"passed": False, "capture_error": error, "observed_count": 0}
    else:
        mask = np.load(package / "config/photo_fit/photo_mask.npy", allow_pickle=False)
        report = evaluate(state["image"], mask, camera, keypoints, points, team)
        report["capture_error"] = error
        report["passed"] = report["passed"] and error is None
        photo = cv2.imread(str(package / "config/photo_fit/pdf_p21_robot_raw.png"))
        write_comparison(state["image"], photo, report, output, team)
    report.update(team=team, actual_camera_frames=state["frames"], actual_joint_positions=state["joints"],
                  actual_model_pose=state["pose"], actual_tf=transforms,
                  image_stamp_s=state.get("image_stamp"), camera_spec=camera["gazebo_camera"])
    report["source_sha256"] = {name: hashlib.sha256((package / name).read_bytes()).hexdigest()
        for name in ("config/wingbot_mods.yaml", "config/photo_fit/cam_fit.json",
                     "config/photo_fit/photo_keypoints.yaml", "config/photo_fit/photo_mask.npy")}
    path = output / ("photo_validation.json" if team == "RED" else "photo_validation_BLUE.json")
    path.write_text(json.dumps(report, indent=2)+"\n", encoding="utf-8")
    print(json.dumps({"passed": report["passed"], "observed": report["observed_count"],
                      "iou": report.get("silhouette", {}).get("iou"), "capture_error": error}))
    return 0 if report["passed"] else 1


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--capture", action="store_true", required=True)
    parser.add_argument("--package", type=Path, default=PACKAGE)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--team", choices=("RED", "BLUE"), default="RED")
    parser.add_argument("--timeout", type=float, default=35.)
    args = parser.parse_args(argv)
    return capture(args.output_dir, args.package, args.team, args.timeout)


if __name__ == "__main__":
    raise SystemExit(main())
