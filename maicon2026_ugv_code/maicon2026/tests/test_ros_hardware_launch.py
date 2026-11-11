"""ROS-free launch contract tests; hardware/ROS execution remains a field check."""
import importlib.util
from pathlib import Path
import sys
import types

import pytest

ROOT = Path(__file__).resolve().parents[1]


class Action:
    def __init__(self, *args, **kwargs):
        self.args, self.kwargs = args, kwargs


class LaunchConfiguration:
    def __init__(self, name):
        self.name = name

    def perform(self, context):
        return context[self.name]


@pytest.fixture
def launch_modules(monkeypatch):
    mapping = {
        "launch": {"LaunchDescription": Action},
        "launch.actions": {k: type(k, (Action,), {}) for k in
                           ("DeclareLaunchArgument", "ExecuteProcess", "OpaqueFunction", "IncludeLaunchDescription")},
        "launch.substitutions": {"LaunchConfiguration": LaunchConfiguration},
        "launch.launch_description_sources": {"PythonLaunchDescriptionSource": Action},
        "launch_ros": {},
        "launch_ros.actions": {"Node": type("Node", (Action,), {})},
        "nav2_common": {},
        "nav2_common.launch": {"RewrittenYaml": type("RewrittenYaml", (Action,), {})},
    }
    for name, attrs in mapping.items():
        module = types.ModuleType(name)
        module.__dict__.update(attrs)
        monkeypatch.setitem(sys.modules, name, module)

    def load(name):
        spec = importlib.util.spec_from_file_location(name, ROOT / "ros2" / "launch" / (name + ".launch.py"))
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    return load


def context(**overrides):
    return {"slam": "cartographer", "use_sim_time": "false", "fast_replan": "true",
            "params_file": "field-tuned.yaml", "base_frame": "base_footprint",
            "cartographer_config": "cartographer_waveshare.lua", "odom_tf": "off",
            "nav2": "true", "depth_obstacles": "true", "bt_xml": "",
            "scan_topic": "/scan", "odom_topic": "/odom", "nav_cmd_topic": "/cmd_vel_nav2", **overrides}


def test_hardware_overlay_has_no_driver_or_tf_publishers(launch_modules):
    actions = launch_modules("maicon_waveshare").generate_launch_description().args[0]
    assert not any(type(a).__name__ in ("Node", "ExecuteProcess") for a in actions)
    include = [a for a in actions if type(a).__name__ == "IncludeLaunchDescription"]
    assert len(include) == 1
    args = dict(include[0].kwargs["launch_arguments"])
    assert args["use_sim_time"] == "false"
    defaults = {a.args[0]: a.kwargs["default_value"] for a in actions if type(a).__name__ == "DeclareLaunchArgument"}
    assert defaults[args["odom_tf"].name] == "off"                 # 기본은 끔; 로봇이 TF 를 안 낼 때만 odom_tf:=auto 로 켠다
    assert defaults[args["base_frame"].name] == "base_footprint"
    assert defaults[args["cartographer_config"].name] == ""
    assert defaults[args["scan_topic"].name] == "/scan"
    assert defaults[args["odom_topic"].name] == "/odom"
    assert defaults[args["nav_cmd_topic"].name] == "/cmd_vel_nav2"


def test_every_velocity_producer_is_intercepted_by_agent(launch_modules):
    actions = launch_modules("maicon_nav")._launch_setup(context())
    nodes = {a.kwargs["executable"]: a for a in actions if type(a).__name__ == "Node"}
    assert set(nodes) == {"cartographer_node", "cartographer_occupancy_grid_node",
                          "controller_server", "planner_server", "behavior_server", "bt_navigator", "lifecycle_manager"}
    for executable in ("controller_server", "behavior_server"):
        assert ("cmd_vel", "/cmd_vel_nav2") in nodes[executable].kwargs["remappings"]
    for executable in ("controller_server", "planner_server", "behavior_server", "bt_navigator"):
        rewrite = nodes[executable].kwargs["parameters"][0]
        assert rewrite.kwargs["source_file"] == "field-tuned.yaml"
        assert rewrite.kwargs["param_rewrites"]["robot_base_frame"] == "base_footprint"


def test_external_slam_option_does_not_create_second_slam(launch_modules):
    actions = launch_modules("maicon_nav")._launch_setup(context(slam="none"))
    assert all(not a.kwargs.get("package", "").startswith(("cartographer", "slam_toolbox")) for a in actions)


def test_optional_fixed_arena_localization_is_the_only_map_to_odom_source(launch_modules):
    actions = launch_modules("maicon_nav")._launch_setup(context(
        slam="arena", base_frame="base_link", use_sim_time="true", scan_topic="/scan_filtered",
        arena_start_x="0.35", arena_start_y="2.5", arena_start_yaw="0.0"))
    assert all(not a.kwargs.get("package", "").startswith(("cartographer", "slam_toolbox")) for a in actions)
    commands = [a.kwargs["cmd"] for a in actions if type(a).__name__ == "ExecuteProcess"]
    assert len(commands) == 1
    cmd = commands[0]
    assert Path(cmd[1]).is_file() and Path(cmd[1]).name == "arena_localization.py"
    assert cmd[cmd.index("--start-y")+1] == "2.5"
    assert cmd[cmd.index("--scan-topic")+1] == "/scan_filtered"
    assert "--use-sim-time" in cmd
    with pytest.raises(RuntimeError, match="calibrated Gazebo"):
        launch_modules("maicon_nav")._launch_setup(context(slam="arena"))


def test_toolbox_and_cartographer_use_same_hardware_frame(launch_modules):
    module = launch_modules("maicon_nav")
    actions = module._launch_setup(context(slam="slam_toolbox"))
    slam = next(a for a in actions if a.kwargs.get("executable") == "async_slam_toolbox_node")
    assert {"base_frame": "base_footprint", "scan_topic": "/scan"} in slam.kwargs["parameters"]
    actions = module._launch_setup(context())
    slam = next(a for a in actions if a.kwargs.get("executable") == "cartographer_node")
    assert slam.kwargs["arguments"][-1] == "cartographer_waveshare.lua"
    lua = (ROOT / "ros2/config/cartographer_waveshare.lua").read_text(encoding="utf-8")
    assert 'options.tracking_frame = "base_footprint"' in lua


def test_simulation_defaults_remain_base_link(launch_modules):
    actions = launch_modules("maicon_nav").generate_launch_description().args[0]
    defaults = {a.args[0]: a.kwargs["default_value"] for a in actions if type(a).__name__ == "DeclareLaunchArgument"}
    assert defaults["base_frame"] == "base_link"
    assert defaults["cartographer_config"] == ""
    assert launch_modules("maicon_nav")._cartographer_config("base_link", "") == "cartographer_2d.lua"
    assert defaults["odom_tf"] == "off"


def test_explicit_gazebo_localization_profile_reaches_cartographer_only(launch_modules):
    actions = launch_modules("maicon_nav")._launch_setup(context(
        base_frame="base_link", cartographer_config="cartographer_gazebo_2d.lua", use_sim_time="true"))
    slam = next(a for a in actions if a.kwargs.get("executable") == "cartographer_node")
    assert slam.kwargs["arguments"][-1] == "cartographer_gazebo_2d.lua"
    assert {"use_sim_time": True} in slam.kwargs["parameters"]
    with pytest.raises(RuntimeError, match="tracking frame mismatch"):
        launch_modules("maicon_nav")._launch_setup(context(cartographer_config="cartographer_gazebo_2d.lua"))


def test_gazebo_profile_excludes_observed_arena_scale_loop_aliases():
    import re
    text = (ROOT / "ros2/config/cartographer_gazebo_2d.lua").read_text(encoding="utf-8")
    def value(name):
        return float(re.search(r"^" + re.escape(name) + r"\s*=\s*([\d.]+)", text, re.M).group(1))
    prefix = "POSE_GRAPH.constraint_builder."
    # Run3 accepted 6.33m and 5.86m corrections at .693/.770 scores. The new
    # profile's local search must be smaller than one 40cm arena structure;
    # global search cannot bypass that geometric restriction.
    assert value(prefix + "fast_correlative_scan_matcher.linear_search_window") < .4
    assert value(prefix + "max_constraint_distance") < 2.0
    assert value(prefix + "min_score") > .77
    assert value("POSE_GRAPH.global_sampling_ratio") == 0.0
    assert value("POSE_GRAPH.optimization_problem.huber_scale") <= 10.
    assert 'include "cartographer_2d.lua"' in text and text.strip().endswith("return options")


def test_explicit_odom_bridge_uses_selected_frame(launch_modules):
    actions = launch_modules("maicon_nav")._launch_setup(context(odom_tf="auto"))
    cmd = next(a.kwargs["cmd"] for a in actions if type(a).__name__ == "ExecuteProcess")
    assert cmd[cmd.index("--base-frame") + 1] == "base_footprint"


def test_uncalibrated_depth_source_removed_but_virtual_walls_kept(launch_modules):
    actions = launch_modules("maicon_nav")._launch_setup(context(depth_obstacles="false"))
    node = next(a for a in actions if a.kwargs.get("executable") == "controller_server")
    rewrites = node.kwargs["parameters"][0].kwargs["param_rewrites"]
    assert rewrites["local_costmap.local_costmap.ros__parameters.obstacle_layer.observation_sources"] == "scan"
    assert rewrites["global_costmap.global_costmap.ros__parameters.obstacle_layer.observation_sources"] == "scan virtual_walls"


def test_filter_topics_and_clock_reach_all_consumers(launch_modules):
    actions = launch_modules("maicon_nav")._launch_setup(context(
        scan_topic="/scan_filtered", odom_topic="/wheel_imu/odom", nav_cmd_topic="/agent/nav_cmd",
        odom_tf="auto", use_sim_time="true", cartographer_config=""))
    slam = next(a for a in actions if a.kwargs.get("executable") == "cartographer_node")
    assert ("scan", "/scan_filtered") in slam.kwargs["remappings"]
    assert ("odom", "/wheel_imu/odom") in slam.kwargs["remappings"]
    bridge = next(a.kwargs["cmd"] for a in actions if type(a).__name__ == "ExecuteProcess")
    assert bridge[bridge.index("--odom-topic") + 1] == "/wheel_imu/odom"
    assert "--use-sim-time" in bridge
    for executable in ("controller_server", "planner_server", "behavior_server", "bt_navigator"):
        node = next(a for a in actions if a.kwargs.get("executable") == executable)
        rewrites = node.kwargs["parameters"][0].kwargs["param_rewrites"]
        assert rewrites["use_sim_time"] == "true"
        assert rewrites["local_costmap.local_costmap.ros__parameters.obstacle_layer.scan.topic"] == "/scan_filtered"
        assert rewrites["global_costmap.global_costmap.ros__parameters.obstacle_layer.scan.topic"] == "/scan_filtered"
        assert rewrites["controller_server.ros__parameters.odom_topic"] == "/wheel_imu/odom"
        assert rewrites["bt_navigator.ros__parameters.odom_topic"] == "/wheel_imu/odom"
        if executable == "controller_server":
            assert {"odom_topic": "/wheel_imu/odom"} in node.kwargs["parameters"]
        if executable in ("controller_server", "behavior_server"):
            assert ("cmd_vel", "/agent/nav_cmd") in node.kwargs["remappings"]


def test_toolbox_uses_filtered_scan_too(launch_modules):
    actions = launch_modules("maicon_nav")._launch_setup(context(slam="slam_toolbox", scan_topic="/scan_filtered"))
    node = next(a for a in actions if a.kwargs.get("executable") == "async_slam_toolbox_node")
    assert {"base_frame": "base_footprint", "scan_topic": "/scan_filtered"} in node.kwargs["parameters"]


@pytest.mark.parametrize("frame,expected", [("base_link", "cartographer_2d.lua"),
                                            ("base_footprint", "cartographer_waveshare.lua")])
def test_cartographer_default_tracks_selected_body(launch_modules, frame, expected):
    actions = launch_modules("maicon_nav")._launch_setup(context(base_frame=frame, cartographer_config=""))
    slam = next(a for a in actions if a.kwargs.get("executable") == "cartographer_node")
    assert slam.kwargs["arguments"][-1] == expected


@pytest.mark.parametrize("overrides", [
    {"base_frame": "base_footprint", "cartographer_config": "cartographer_2d.lua"},
    {"base_frame": "custom_body", "cartographer_config": ""},
    {"cartographer_config": "../config/cartographer_waveshare.lua"},
    {"nav_cmd_topic": "/cmd_vel"},
    {"scan_topic": "scan_filtered"},
])
def test_bad_hardware_wiring_fails_before_start(launch_modules, overrides):
    with pytest.raises(RuntimeError):
        launch_modules("maicon_nav")._launch_setup(context(**overrides))
