"""Acceptance evaluator must distinguish real upright renders from UV faults."""
import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

import cv2
import numpy as np
import pytest


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("closeup_probe", ROOT / "scripts/probe_arena_closeups.py")
probe = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(probe)
PACKAGE = ROOT / "ros2/maicon_arena_gz"


@pytest.mark.parametrize("kind,source_name", [("target", "target_outpost"), ("m3", "m3_military_truck")])
def test_actual_camera_images_pass_and_reflected_render_fails(kind, source_name):
    frame = cv2.imread(str(PACKAGE / "validation/closeups" / f"{kind}_closeup.png"))
    source = cv2.imread(str(PACKAGE / "media/textures/faces" / f"{source_name}.jpg"))
    assert frame is not None and source is not None
    result, _ = probe.orientation_scores(frame, source, .4)
    assert result["passed"]
    assert result["ncc"]["upright"] > .97
    for faulty in (np.fliplr(frame).copy(), np.flipud(frame).copy(), np.rot90(frame, 2).copy()):
        result, _ = probe.orientation_scores(faulty, source, .4)
        assert not result["passed"]


def test_probe_only_adds_sensors_and_uses_yaml_face_geometry():
    spec = probe.load_spec(PACKAGE / "config/arena_spec.yaml")
    layout = probe.load_layout(PACKAGE / "config/layouts/layout_default.yaml")
    selected = probe.select_faces(spec, layout)
    assert {s["kind"] for s in selected} == {"target", "m3"}
    for selection in selected:
        tree = ET.fromstring(probe.camera_sdf("acceptance_only", "/acceptance/image", selection))
        assert tree.find(".//collision") is None
        assert tree.find(".//visual") is None
        assert tree.find(".//plugin") is None
        assert tree.find(".//sensor").get("type") == "camera"
        assert np.linalg.norm(np.asarray(selection["camera_position"]) - selection["face_center"]) == pytest.approx(.65)
        assert selection["camera_position"][2] == pytest.approx(selection["face_center"][2])
