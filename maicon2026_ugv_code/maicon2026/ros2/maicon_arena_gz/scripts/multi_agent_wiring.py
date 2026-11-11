"""Opt-in topic isolation for six independent WINGBOT sensor/control stacks.

Frame IDs remain the calibrated names; TF topics, MIL APIs and sensor topics
are isolated per robot. Global referee state is consumed only by emulators.
"""
from copy import deepcopy


SLOTS = ("B-1", "B-2", "B-3", "R-1", "R-2", "R-3")


def namespaces(slot):
    slot = slot.replace("-", "")
    if slot not in {s.replace("-", "") for s in SLOTS}:
        raise ValueError("Unknown WINGBOT slot")
    return "/arena/robots/" + slot, "/robots/" + slot


def emulator_remappings(slot):
    raw, api = namespaces(slot)
    remaps = [("/arena/ego/"+name, raw+"/"+name)
              for name in ("cmd_vel", "pan_cmd", "tilt_cmd", "odom", "image", "joint_states", "scan_raw")]
    remaps += [("/"+name, api+"/"+name) for name in
               ("cmd_vel", "odom", "image_raw", "scan", "mil/state", "mil/match", "mil/oled", "mil/gimbal", "mil/fire", "tf", "tf_static")]
    return remaps


def agent_remappings(slot):
    """Use as ROS --remap source:=target arguments on each independent Agent."""
    raw, api = namespaces(slot)
    return [("/"+name, api+"/"+name) for name in
            ("cmd_vel", "odom", "image_raw", "scan", "imu/data", "mil/state", "mil/match", "mil/oled", "mil/gimbal", "mil/fire", "tf", "tf_static")] + [
            ("/arena/ego/camera_info", raw+"/camera_info"),
            ("/arena/ego/joint_states", raw+"/joint_states")]


def full_sensor_bridges(base_entries, slots):
    """Replicate only actual per-robot sensors and commands, keeping globals once."""
    global_names = ("/clock", "/arena/dynamic_pose")
    result = [deepcopy(item) for item in base_entries if item["ros_topic_name"] in global_names]
    robot_entries = [item for item in base_entries if item["ros_topic_name"].startswith("/arena/ego/")]
    scan = next(item for item in base_entries if item["ros_topic_name"] == "/scan")
    imu = next(item for item in base_entries if item["ros_topic_name"] == "/imu/data")
    for slot in slots:
        raw, api = namespaces(slot)
        for item in robot_entries:
            bridge = deepcopy(item)
            for name in ("ros_topic_name", "gz_topic_name"):
                bridge[name] = bridge[name].replace("/arena/ego", raw)
            result.append(bridge)
        bridge = deepcopy(scan)
        bridge["ros_topic_name"] = bridge["gz_topic_name"] = raw+"/scan_raw"
        result.append(bridge)
        # Raw diagnostics and the public IMU API share the same measured source.
        for destination in (raw+"/imu/data", api+"/imu/data"):
            bridge = deepcopy(imu)
            bridge["ros_topic_name"], bridge["gz_topic_name"] = destination, raw+"/imu/data"
            result.append(bridge)
    return result
