"""A recording must come from a robot camera, without six extra render loads."""
from pathlib import Path

from scripts.generate_gazebo_world import generate


def test_optional_camera_is_b2_only_in_six_robot_world():
    root, _ = generate(2, "all", fpv=True)
    cameras = root.findall(".//sensor[@name='fpv_camera']")
    assert len(cameras) == 1
    camera = root.find("world/model[@name='wingbot_ugv']/link/sensor[@name='fpv_camera']")
    assert camera is not None and camera.get("type") == "camera"
    assert camera.findtext("topic") == "/B2/fpv/image"
    assert camera.findtext("update_rate") == "10"
    assert camera.findtext("camera/image/width") == "640"
    assert camera.findtext("camera/image/height") == "360"
    assert camera.findtext("pose").split()[-3:] == ["0", "0", "0"]
    for bridge in ("bridge_agent.yaml", "bridge_match.yaml"):
        text = (Path(__file__).resolve().parents[1] / "ros2/gazebo" / bridge).read_text()
        assert "ros_topic_name: /B2/fpv/image" in text
        assert "gz_topic_name: /B2/fpv/image" in text


def test_normal_single_robot_trial_has_no_recording_render_cost():
    root, _ = generate(2, "B2")
    assert root.findall(".//sensor[@name='fpv_camera']") == []
