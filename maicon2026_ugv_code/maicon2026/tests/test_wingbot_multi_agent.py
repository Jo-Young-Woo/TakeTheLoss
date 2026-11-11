"""Six independent sensor/command APIs must not alias the ego or one another."""
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
ARENA = ROOT/"ros2/maicon_arena_gz"
sys.path.insert(0, str(ARENA/"scripts"))
import robot_models
from multi_agent_wiring import SLOTS, agent_remappings, emulator_remappings, full_sensor_bridges, namespaces


def test_full_sensor_models_use_six_distinct_topics_and_keep_model_authority(monkeypatch):
    requests = []
    def instantiate(name, team, namespace, sensors, wheels, simulator, effective_separation):
        requests.append((name, namespace, sensors))
        model = ET.Element("model", name=name)
        link = ET.SubElement(model, "link", name="base_link")
        for kind, suffix in (("camera", "/camera/image"), ("gpu_lidar", "/scan"), ("imu", "/imu")):
            node = ET.SubElement(link, "sensor", type=kind)
            ET.SubElement(node, "topic").text = namespace+suffix
        return model
    monkeypatch.setattr(robot_models, "instantiate_wingbot", instantiate)
    monkeypatch.setattr(robot_models, "wingbot_package", lambda: ROOT/"ros2/wingbot_description")
    load = lambda name: yaml.safe_load((ARENA/"config"/name).read_text(encoding="utf8"))
    world = ET.Element("world")
    result = robot_models.add_ugvs(world, load("arena_spec.yaml"), load("profiles/standard.yaml"), 6, "B-2", full_sensors=True)
    assert result["sensors"] == {"camera": 6, "gpu_lidar": 6, "imu": 6}
    assert all(row[2] == "full" for row in requests)
    topics = [node.text for node in world.findall(".//sensor/topic")]
    assert len(set(topics)) == len(topics) == 18
    assert not any("/ego/" in topic for topic in topics)
    for slot in SLOTS:
        raw, _ = namespaces(slot)
        assert {raw+"/image", raw+"/scan_raw", raw+"/imu/data"} <= set(topics)


def test_bridge_isolation_and_agent_emulator_api_pairing():
    base = yaml.safe_load((ARENA/"config/bridge.yaml").read_text(encoding="utf8"))
    bridges = full_sensor_bridges(base, SLOTS)
    names = [item["ros_topic_name"] for item in bridges]
    assert len(names) == len(set(names))
    assert names.count("/clock") == names.count("/arena/dynamic_pose") == 1
    assert "/scan" not in names and "/imu/data" not in names
    for slot in SLOTS:
        raw, api = namespaces(slot)
        bridge = {item["ros_topic_name"]: item for item in bridges}
        for name in ("camera_info", "joint_states", "odom", "cmd_vel", "pan_cmd", "tilt_cmd", "scan_raw", "imu/data"):
            assert raw+"/"+name in bridge
        assert bridge[raw+"/camera_info"]["gz_topic_name"] == raw+"/camera/camera_info"
        assert bridge[api+"/imu/data"]["gz_topic_name"] == raw+"/imu/data"
        agent, emulator = dict(agent_remappings(slot)), dict(emulator_remappings(slot))
        for name in ("/scan", "/cmd_vel", "/odom", "/image_raw", "/mil/state", "/mil/gimbal", "/tf", "/tf_static"):
            assert agent[name] == emulator[name] == api+name
        assert agent["/arena/ego/joint_states"] == emulator["/arena/ego/joint_states"] == raw+"/joint_states"
        assert "/arena/dynamic_pose" not in agent and "/arena/state" not in agent


def test_full_sensor_rejects_legacy_and_invalid_namespace():
    with pytest.raises(ValueError, match="WINGBOT"):
        robot_models.add_ugvs(ET.Element("world"), {}, {}, 6, "B-2", robot_model="legacy", full_sensors=True)
    with pytest.raises(ValueError):
        namespaces("unexpected")


def test_larger_transport_is_explicit_and_has_six_camera_queue_headroom():
    from arena_transport import image_transport_environment
    overrides = image_transport_environment("shm256", ARENA, {})
    assert overrides["FASTRTPS_DEFAULT_PROFILES_FILE"].endswith("fastdds_shm256.xml")
    ns = {"d": "http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles"}
    tree = ET.parse(ARENA/"config/fastdds_shm256.xml")
    capacity = int(tree.findtext(".//d:segment_size", namespaces=ns))
    # Six raw cameras plus their BGR API copies, full UAV and third person.
    worst_frame_bytes = (12*640*480 + 1920*1080 + 1920*1080)*3
    assert capacity > worst_frame_bytes*5*1.25
