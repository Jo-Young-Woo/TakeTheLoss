#!/usr/bin/env python3
"""Capture two real Gazebo face close-ups without moving arena or robot models.

This explicit acceptance probe adds collision-free temporary camera models, then
removes them. It compares camera pixels with supplied source photographs; the
source images are never substituted for the captured frame. Requires a running
Fortress world and a sourced ROS environment. YAML is evaluator-only input.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import xml.etree.ElementTree as ET

import cv2
import numpy as np


REPO = Path(__file__).resolve().parents[1]
PACKAGE = REPO / "ros2/maicon_arena_gz"
sys.path.insert(0, str(PACKAGE / "scripts"))
from arena_common import expand_boxes, load_layout, load_spec  # noqa: E402
from make_atlas import NORMALS, image_for_face  # noqa: E402


def select_faces(spec, layout):
    """Choose exposed target and M3 sides; derive all geometry from the YAML."""
    choices = []
    for box in expand_boxes(spec, layout):
        for face, label in box["faces"].items():
            if not label.startswith(("target:", "m3:")):
                continue
            normal = np.asarray(NORMALS[face], dtype=float)
            center = np.asarray(box["center"], dtype=float)
            face_center = center + normal * float(box["size"][0]) / 2
            position = face_center + normal * .65
            choices.append({"kind": label.split(":")[0], "label": label,
                            "box": box["name"], "face": face,
                            "size": float(box["size"][0]),
                            "face_center": face_center.tolist(),
                            "camera_position": position.tolist(),
                            "yaw": math.atan2(-normal[1], -normal[0])})
    selected = []
    for kind, preferred in (("target", "target:outpost"), ("m3", "m3:military_truck")):
        candidates = [c for c in choices if c["kind"] == kind]
        if not candidates:
            raise ValueError(f"Layout has no exposed {kind} image face")
        candidates.sort(key=lambda c: (c["label"] != preferred, c["box"]))
        selected.append(candidates[0])
    return selected


def camera_sdf(name, topic, selection, width=640, hfov=.85):
    sdf = ET.Element("sdf", version="1.9")
    model = ET.SubElement(sdf, "model", name=name)
    ET.SubElement(model, "static").text = "true"
    position = selection["camera_position"]
    ET.SubElement(model, "pose").text = " ".join(map(str, [*position, 0, 0, selection["yaw"]]))
    link = ET.SubElement(model, "link", name="camera_link")
    sensor = ET.SubElement(link, "sensor", name="closeup_camera", type="camera")
    ET.SubElement(sensor, "always_on").text = "true"
    ET.SubElement(sensor, "update_rate").text = "3"
    ET.SubElement(sensor, "topic").text = topic
    camera = ET.SubElement(sensor, "camera")
    ET.SubElement(camera, "horizontal_fov").text = str(hfov)
    image = ET.SubElement(camera, "image")
    ET.SubElement(image, "width").text = str(width)
    ET.SubElement(image, "height").text = str(width)
    ET.SubElement(image, "format").text = "R8G8B8"
    clip = ET.SubElement(camera, "clip")
    ET.SubElement(clip, "near").text = ".02"
    ET.SubElement(clip, "far").text = "5"
    return ET.tostring(sdf, encoding="unicode")


def orientation_scores(frame, source, side, distance=.65, hfov=.85):
    """NCC at a precomputed physical-face crop; no fitted/reflected registration."""
    focal = frame.shape[1] / (2 * math.tan(hfov / 2))
    extent = focal * side / distance
    center_x, center_y = (frame.shape[1] - 1) / 2, (frame.shape[0] - 1) / 2
    bounds = [round(center_x - extent/2), round(center_y - extent/2),
              round(center_x + extent/2), round(center_y + extent/2)]
    x0, y0, x1, y1 = bounds
    if min(x0, y0) < 0 or x1 > frame.shape[1] or y1 > frame.shape[0]:
        raise ValueError("Physical face is outside camera frame")
    crop = frame[y0:y1, x0:x1]
    observed = cv2.resize(cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY), (160, 160)).astype(float)
    expected = cv2.resize(cv2.cvtColor(source, cv2.COLOR_BGR2GRAY), (160, 160)).astype(float)
    variants = {"upright": expected, "mirror_horizontal": np.fliplr(expected),
                "mirror_vertical": np.flipud(expected), "rotated_180": np.rot90(expected, 2),
                "rotated_90": np.rot90(expected, 1), "rotated_270": np.rot90(expected, 3)}
    # Exclude cube edges / UV gutters consistently from every comparison.
    observed = observed[8:-8, 8:-8].ravel()
    scores = {}
    for name, candidate in variants.items():
        reference = candidate[8:-8, 8:-8].ravel()
        scores[name] = float(np.corrcoef(observed, reference)[0, 1])
    runner_up = max(v for k, v in scores.items() if k != "upright")
    passed = bool(np.isfinite(list(scores.values())).all() and
                  scores["upright"] >= .75 and scores["upright"] - runner_up >= .15)
    return {"passed": passed, "crop_xyxy": bounds, "ncc": scores,
            "upright_margin": scores["upright"] - runner_up}, crop


def _service(world, action, request_type, request):
    result = subprocess.run(["ign", "service", "-s", f"/world/{world}/{action}",
        "--reqtype", f"ignition.msgs.{request_type}", "--reptype", "ignition.msgs.Boolean",
        "--timeout", "4000", "--req", request], capture_output=True, text=True, timeout=6)
    if result.returncode or "data: true" not in result.stdout:
        raise RuntimeError(f"Gazebo did not acknowledge close-up camera {action}")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--spec", type=Path, default=PACKAGE / "config/arena_spec.yaml")
    parser.add_argument("--layout", type=Path, default=PACKAGE / "config/layouts/layout_default.yaml")
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--world", default="maicon_arena")
    parser.add_argument("--timeout", type=float, default=20.)
    args = parser.parse_args(argv)
    if not 1 <= args.timeout <= 60:
        parser.error("timeout must be in [1,60] seconds")
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image

    selected = select_faces(load_spec(args.spec), load_layout(args.layout))
    args.output_dir.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = Node("maicon_closeup_acceptance")
    frames, counts, spawned, rows, cleanup_errors = {}, {}, [], [], []
    bridge, error = None, None

    def receive(kind, message):
        if message.encoding not in ("rgb8", "bgr8"):
            return
        raw = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.step)
        pixels = raw[:, :message.width*3].reshape(message.height, message.width, 3)
        frames[kind] = cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR) if message.encoding == "rgb8" else pixels.copy()
        counts[kind] = counts.get(kind, 0) + 1

    try:
        topics = []
        with tempfile.TemporaryDirectory(prefix="maicon_closeups_") as temporary:
            for selection in selected:
                kind = selection["kind"]
                name = f"acceptance_closeup_{kind}_{int(time.time())}"
                topic = f"/arena/validation/closeup_{kind}"
                path = Path(temporary) / f"{name}.sdf"
                path.write_text(camera_sdf(name, topic, selection), encoding="utf-8")
                node.create_subscription(Image, topic, lambda message, key=kind: receive(key, message),
                                         qos_profile_sensor_data)
                _service(args.world, "create", "EntityFactory", "sdf_filename: " + json.dumps(str(path)))
                spawned.append(name)
                topics.append(topic)
            bridge = subprocess.Popen(["ros2", "run", "ros_gz_image", "image_bridge", *topics],
                                      stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            deadline = time.monotonic() + args.timeout
            while not all(counts.get(s["kind"], 0) >= 3 for s in selected):
                if time.monotonic() >= deadline:
                    raise TimeoutError("No three actual frames received for both close-up cameras")
                if bridge.poll() is not None:
                    raise RuntimeError("Close-up ROS image bridge stopped")
                rclpy.spin_once(node, timeout_sec=.1)
            for selection in selected:
                kind = selection["kind"]
                frame = frames[kind]
                source_path = image_for_face(selection["label"], PACKAGE / "media/textures")
                source = cv2.imread(str(source_path))
                comparison, crop = orientation_scores(frame, source, selection["size"])
                captured_path = args.output_dir / f"{kind}_closeup.png"
                crop_path = args.output_dir / f"{kind}_face_crop.png"
                comparison_path = args.output_dir / f"{kind}_source_vs_capture.png"
                sheet = np.hstack((cv2.resize(source, (512, 512)), cv2.resize(crop, (512, 512))))
                for path, pixels in ((captured_path, frame), (crop_path, crop), (comparison_path, sheet)):
                    if not cv2.imwrite(str(path), pixels):
                        raise OSError("Unable to save close-up capture")
                rows.append({**selection, **comparison, "actual_frames_received": counts[kind],
                    "source_image": source_path.relative_to(PACKAGE).as_posix(),
                    "source_sha256": hashlib.sha256(source_path.read_bytes()).hexdigest(),
                    "capture": captured_path.name, "crop": crop_path.name,
                    "source_left_capture_right": comparison_path.name,
                    "capture_sha256": hashlib.sha256(captured_path.read_bytes()).hexdigest()})
    except Exception as exc:
        error = str(exc)
    finally:
        if bridge is not None:
            bridge.terminate()
            try:
                bridge.wait(timeout=4)
            except subprocess.TimeoutExpired:
                bridge.kill()
                bridge.wait(timeout=2)
        for name in spawned:
            try:
                _service(args.world, "remove", "Entity", f'name: "{name}", type: MODEL')
            except Exception as exc:
                cleanup_errors.append(str(exc))
        node.destroy_node()
        rclpy.try_shutdown()
    report = {"passed": error is None and not cleanup_errors and len(rows) == 2 and
                        all(row["passed"] for row in rows),
              "scope": "actual Gazebo camera frames, one target and one M3 face; evaluator-only YAML",
              "world": args.world, "spec_sha256": hashlib.sha256(args.spec.read_bytes()).hexdigest(),
              "layout_sha256": hashlib.sha256(args.layout.read_bytes()).hexdigest(),
              "error": error, "cleanup_errors": cleanup_errors, "checks": rows,
              "limits": "Two sampled side faces, not all face instances or recognition accuracy."}
    (args.output_dir / "closeup_report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"passed": report["passed"], "checks": len(rows), "error": error}))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
