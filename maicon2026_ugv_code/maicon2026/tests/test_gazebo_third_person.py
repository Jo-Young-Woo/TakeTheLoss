"""The observer video camera must remain separate from robot perception."""
import math
from pathlib import Path

import yaml

from scripts.generate_gazebo_world import generate

ROOT = Path(__file__).resolve().parents[1]


def test_optional_overview_is_an_actual_rendered_camera_without_collisions():
    scene, world = generate(2, "B2", third_person=True)
    model = scene.find("world/model[@name='recording_overview']")
    assert model is not None and model.findtext("static") == "true"
    assert not model.findall(".//collision") and not model.findall(".//visual")
    sensor = model.find("link/sensor")
    assert sensor.get("type") == "camera"
    assert sensor.findtext("topic") == "/recording/arena/image"
    assert sensor.findtext("update_rate") == "5"
    assert sensor.findtext("camera/image/width") == "960"
    assert sensor.findtext("camera/image/height") == "720"
    # Leave a visible margin at all outer arena corners, including wall tops.
    cx, cy, cz, _, pitch, _ = map(float, model.findtext("pose").split())
    hfov = float(sensor.findtext("camera/horizontal_fov"))
    for x in (-.15, world.geo.W + .15):
        for y in (-.15, world.geo.H + .15):
            for z in (0, world.geo.wall_h):
                forward = (y - cy) * math.cos(pitch) - (z - cz) * math.sin(pitch)
                vertical = (y - cy) * math.sin(pitch) + (z - cz) * math.cos(pitch)
                assert forward > 0
                assert abs(x - cx) / forward < math.tan(hfov / 2) * .98
                assert abs(vertical) / forward < math.tan(hfov / 2) * .75 * .98
    # Third-person recording does not need the extra onboard camera workload.
    assert scene.find(".//sensor[@name='fpv_camera']") is None


def test_overview_default_off_and_distinct_read_only_bridge_topic():
    scene, _ = generate(2, "B2")
    assert scene.find("world/model[@name='recording_overview']") is None
    for filename in ("bridge_agent.yaml", "bridge_match.yaml"):
        bridges = yaml.safe_load((ROOT / "ros2/gazebo" / filename).read_text())
        observer, = [b for b in bridges if b["ros_topic_name"] == "/recording/arena/image"]
        assert observer["gz_topic_name"] == "/recording/arena/image"
        assert observer["direction"] == "GZ_TO_ROS"
        assert observer["ros_type_name"] == "sensor_msgs/msg/Image"
        uav, = [b for b in bridges if b["ros_topic_name"] == "/uav/image"]
        assert observer["gz_topic_name"] != uav["gz_topic_name"]
