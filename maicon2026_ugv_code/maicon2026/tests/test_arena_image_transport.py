"""Transport selection and image memory budget, without ROS or a live server."""
import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[1]/"ros2/maicon_arena_gz"
SPEC = importlib.util.spec_from_file_location("arena_transport", ROOT/"scripts/arena_transport.py")
transport = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(transport)


def test_default_inherits_user_profile_and_middleware_without_mutating_environment():
    env = {"FASTRTPS_DEFAULT_PROFILES_FILE": "/user/custom.xml",
           "FASTDDS_DEFAULT_PROFILES_FILE": "/user/new.xml", "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp"}
    before = dict(env)
    assert transport.image_transport_environment("inherit", ROOT, env) == {}
    assert env == before


@pytest.mark.parametrize("rmw", ["", "rmw_fastrtps_cpp", "rmw_fastrtps_dynamic_cpp"])
def test_explicit_shm64_selects_local_xml_but_leaves_parent_environment_unchanged(rmw):
    env = {"FASTRTPS_DEFAULT_PROFILES_FILE": "/user/old.xml", "RMW_IMPLEMENTATION": rmw}
    before = dict(env)
    chosen = transport.image_transport_environment("shm64", ROOT, env)
    expected = str((ROOT/"config/fastdds_shm64.xml").resolve())
    assert chosen["FASTRTPS_DEFAULT_PROFILES_FILE"] == expected
    assert chosen["FASTDDS_DEFAULT_PROFILES_FILE"] == expected
    assert chosen["RMW_IMPLEMENTATION"] == (rmw or "rmw_fastrtps_cpp")
    assert env == before


def test_other_explicit_middleware_is_not_silently_replaced_and_missing_profile_fails(tmp_path):
    with pytest.raises(ValueError, match="another RMW"):
        transport.image_transport_environment("shm64", ROOT, {"RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp"})
    with pytest.raises(ValueError, match="missing"):
        transport.image_transport_environment("shm64", tmp_path, {})
    with pytest.raises(ValueError, match="inherit or shm64"):
        transport.image_transport_environment("unrecognized", ROOT, {})


@pytest.mark.parametrize("profile", ["lite", "standard", "high"])
def test_shm_capacity_covers_maximum_rgb_frames_depth_five_and_ego_republish(profile):
    document = ET.parse(ROOT/"config/fastdds_shm64.xml").getroot()
    ns = {"d": "http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles"}
    descriptors = {node.findtext("d:transport_id", namespaces=ns): node
                   for node in document.findall("d:transport_descriptors/d:transport_descriptor", ns)}
    shm = descriptors["maicon_shm64"]
    assert shm.findtext("d:type", namespaces=ns) == "SHM"
    capacity = int(shm.findtext("d:segment_size", namespaces=ns))
    spec = yaml.safe_load((ROOT/"config/arena_spec.yaml").read_text(encoding="utf-8"))
    settings = yaml.safe_load((ROOT/"config/profiles"/(profile+".yaml")).read_text(encoding="utf-8"))
    # Include both RGB bridge and BGR emulator publications, plus the largest
    # optional validation image. The allowance is engineering headroom, not an
    # assertion about every DDS allocation or arbitrary additional subscribers.
    resolutions = [spec["ugv"]["camera"]["resolution"]]*2 + [
        settings["uav_camera"]["resolution"], settings["validation_camera"]["resolution"]]
    sizes = [width*height*3 for width, height in resolutions]
    assert capacity >= max(sizes)
    assert capacity >= sum(sizes)*5*1.25
    rtps = document.find("d:participant/d:rtps", ns)
    selected = {entry.text for entry in rtps.findall("d:userTransports/d:transport_id", ns)}
    assert selected == {"maicon_shm64", "maicon_udp"}
    assert descriptors["maicon_udp"].findtext("d:type", namespaces=ns) == "UDPv4"
    assert rtps.findtext("d:useBuiltinTransports", namespaces=ns) == "false"
