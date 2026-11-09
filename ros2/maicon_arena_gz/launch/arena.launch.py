"""Generate a YAML-defined local arena and launch Fortress with minimal bridges."""
from pathlib import Path
import os
import subprocess
import sys
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _boolean(context, key):
    value = LaunchConfiguration(key).perform(context).lower()
    if value not in ("true", "false"):
        raise ValueError(key+" must be true or false")
    return value == "true"


def _critical(node, reason):
    # DiffDrive may retain its previous command if the emulator/bridge dies.
    # Stop the entire simulator when the command safety path is lost.
    return [node, RegisterEventHandler(OnProcessExit(target_action=node,
             on_exit=[EmitEvent(event=Shutdown(reason=reason))]))]


def _setup(context):
    root = Path(get_package_share_directory("maicon_arena_gz"))
    sys.path.insert(0, str(root/"scripts"))
    from arena_transport import image_transport_environment
    from multi_agent_wiring import namespaces, full_sensor_bridges, emulator_remappings
    image_env = image_transport_environment(
        LaunchConfiguration("image_transport_profile").perform(context), root, context.environment)
    profile = LaunchConfiguration("profile").perform(context)
    profile_path = Path(profile)
    if not profile_path.is_file():
        profile_path = root/"config/profiles"/(profile+".yaml")
    layout = Path(LaunchConfiguration("layout").perform(context))
    if not layout.is_file():
        layout = root/"config/layouts"/layout
    if not layout.is_file():
        raise ValueError("layout must name an existing YAML file")
    engine = LaunchConfiguration("render_engine").perform(context)
    gui_engine = LaunchConfiguration("gui_render_engine").perform(context) or engine
    gui_software = LaunchConfiguration("gui_software_rendering").perform(context).lower()
    simulator = LaunchConfiguration("simulator").perform(context)
    robot_model = LaunchConfiguration("robot_model").perform(context)
    if robot_model not in ("legacy", "wingbot"):
        raise ValueError("robot_model must be legacy or wingbot")
    if robot_model == "wingbot" and simulator != "fortress":
        raise ValueError("WINGBOT currently supports Gazebo Fortress only; select legacy explicitly for Harmonic")
    wingbot_root = (Path(get_package_share_directory("wingbot_description"))
                    if robot_model == "wingbot" else None)
    if engine not in ("ogre", "ogre2") or gui_engine not in ("ogre", "ogre2") or simulator not in ("fortress", "harmonic"):
        raise ValueError("Unsupported render engine or simulator")
    if gui_software not in ("", "true", "false"):
        raise ValueError("gui_software_rendering must be empty, true or false")
    ugvs = int(LaunchConfiguration("ugvs").perform(context))
    full_sensors = _boolean(context, "full_sensors")
    if full_sensors and robot_model != "wingbot":
        raise ValueError("full_sensors requires WINGBOT")
    if ugvs not in (0, 1, 6):
        raise ValueError("ugvs must be 0, 1, or 6")
    if robot_model == "wingbot" and ugvs and engine == "ogre":
        raise ValueError("WINGBOT full360 LiDAR requires the verified Ogre2 sensor server; use a separate Ogre1 GUI")
    ego = LaunchConfiguration("ego_slot").perform(context)
    uav, validation = _boolean(context, "uav_cam"), _boolean(context, "validation_camera")
    # Process-local compatibility opt-in; never change the user's shell or
    # pretend a requested GL version guarantees every driver feature.
    graphics_env = ({"LIBGL_ALWAYS_SOFTWARE": "0", "MESA_GL_VERSION_OVERRIDE": "4.3",
                     "MESA_GLSL_VERSION_OVERRIDE": "430"}
                    if _boolean(context, "wsl_gpu_compat") else {})
    output = Path(tempfile.mkdtemp(prefix="maicon_arena_"))
    spec = root/"config/arena_spec.yaml"
    base_map = LaunchConfiguration("base_map").perform(context)
    if base_map:
        document = yaml.safe_load(spec.read_text(encoding="utf-8"))
        labels = yaml.safe_load(Path(base_map).read_text(encoding="utf-8"))
        labels = labels.get("label_map", labels)
        if set(labels) != set(document["bases"]["label_map"]):
            raise ValueError("base_map must contain exactly the three existing row keys")
        document["bases"]["label_map"] = labels
        spec = output/"arena_spec.yaml"
        spec.write_text(yaml.safe_dump(document, allow_unicode=True), encoding="utf-8")
    world = output/"maicon_arena.sdf"
    subprocess.run([sys.executable, str(root/"scripts/gen_world.py"), "--spec", str(spec),
                    "--layout", str(layout), "--profile", profile, "--output", str(world),
                    "--render-engine", engine, "--gui-render-engine", gui_engine,
                    "--simulator", simulator, "--ugvs", str(ugvs),
                    "--robot-model", robot_model, "--full-sensors", str(full_sensors).lower(),
                    "--ego-slot", ego, "--uav-cam", str(uav).lower(),
                    "--validation-camera", str(validation).lower()], check=True)
    bridges = yaml.safe_load((root/"config/bridge.yaml").read_text(encoding="utf-8"))
    all_slots = yaml.safe_load(spec.read_text(encoding="utf-8"))["bases"]["start_pose"]
    active = ([ego]+[slot for slot in all_slots if slot != ego])[:ugvs]
    dummy_topics = {"/ugv_"+slot.replace("-", "")+"/cmd_vel" for slot in active if slot != ego}
    bridges = [b for b in bridges if (not b["ros_topic_name"].startswith("/ugv_") or b["ros_topic_name"] in dummy_topics)
               and (ugvs or b["ros_topic_name"] in ("/clock", "/arena/dynamic_pose"))]
    if robot_model == "wingbot":
        bridges = [b for b in bridges if not b["ros_topic_name"].startswith("/arena/scan_sector_")]
        for item in bridges:
            if item["ros_topic_name"] == "/scan":
                item["ros_topic_name"] = item["gz_topic_name"] = "/arena/ego/scan_raw"
        if ugvs == 6:
            # The five lightweight WINGBOTs retain real head joints. Their
            # training controller needs bridge access to aim those joints.
            for slot in active:
                if slot == ego:
                    continue
                namespace = "/ugv_" + slot.replace("-", "")
                for axis in ("pan", "tilt"):
                    bridges.append(dict(ros_topic_name=namespace+"/"+axis+"_cmd",
                                        gz_topic_name=namespace+"/"+axis+"_cmd",
                                        ros_type_name="std_msgs/msg/Float64",
                                        gz_type_name="ignition.msgs.Double", direction="ROS_TO_GZ"))
    else:
        bridges = [b for b in bridges if b["ros_topic_name"] != "/arena/ego/camera_info"
                   and not (engine == "ogre" and b["ros_topic_name"] == "/scan")
                   and not (engine != "ogre" and b["ros_topic_name"].startswith("/arena/scan_sector_"))]
    if full_sensors:
        bridges = full_sensor_bridges(yaml.safe_load((root/"config/bridge.yaml").read_text(encoding="utf-8")), active)
    if simulator == "harmonic":
        for item in bridges:
            item["gz_type_name"] = item["gz_type_name"].replace("ignition.msgs.", "gz.msgs.")
    bridge_path = output/"bridge.yaml"
    bridge_path.write_text(yaml.safe_dump(bridges), encoding="utf-8")
    search_path = os.pathsep.join(filter(None, (str(root.parent), str(wingbot_root.parent) if wingbot_root else "",
                                               os.environ.get("IGN_GAZEBO_RESOURCE_PATH", ""))))
    server = ExecuteProcess(cmd=(["ign", "gazebo"] if simulator == "fortress" else ["gz", "sim"])+
                            ["-r", "-s", str(world), "--render-engine", engine], output="screen",
                            additional_env=graphics_env)
    # These launch-local environment actions precede every ROS process, including
    # image bridge, emulator, referee and optional merger. External clients must
    # opt in separately; an ordinary launch preserves their inherited settings.
    actions = [SetEnvironmentVariable(key, value) for key, value in image_env.items()]
    actions += [SetEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", search_path),
               SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", search_path),
               server,
               RegisterEventHandler(OnProcessExit(target_action=server,
                    on_exit=[EmitEvent(event=Shutdown(reason="Gazebo server exited"))]))]
    bridge_groups = [("arena_bridge", bridge_path)]
    if full_sensors:
        # Separate participants avoid one process owning every robot's Gazebo
        # transport subscription and DDS queue. The global clock/referee bridge
        # remains independent of any individual sensor stream.
        global_path = output/"bridge_global.yaml"
        global_path.write_text(yaml.safe_dump([b for b in bridges if b["ros_topic_name"] in ("/clock","/arena/dynamic_pose")]), encoding="utf-8")
        bridge_groups = [("arena_bridge_global",global_path)]
        for slot in active:
            raw, api = namespaces(slot)
            robot_bridge = output/("bridge_"+slot.replace("-", "")+".yaml")
            robot_bridge.write_text(yaml.safe_dump([b for b in bridges if b["ros_topic_name"].startswith((raw+"/",api+"/"))]), encoding="utf-8")
            bridge_groups.append(("arena_bridge_"+slot.replace("-", ""),robot_bridge))
    for bridge_name, path in bridge_groups:
        actions += _critical(Node(package="ros_gz_bridge", executable="parameter_bridge", name=bridge_name,
                        parameters=[{"config_file": str(path), "use_sim_time": True}], output="screen"), "Arena command bridge exited: "+bridge_name)
    images = ([namespaces(slot)[0]+"/image" for slot in active] if full_sensors else
              ["/arena/ego/image"] if ugvs else [])+(["/uav/image_raw"] if uav else [])
    if validation:
        images.append("/arena/validation/image_raw")
    if images:
        actions.append(Node(package="ros_gz_image", executable="image_bridge", name="arena_images",
                            arguments=images, parameters=[{"use_sim_time": True}], output="screen"))
    if ugvs:
        if engine == "ogre" and robot_model == "legacy":
            actions += _critical(Node(package="maicon_arena_gz", executable="lidar_sector_merger.py", name="arena_scan_merger",
                                arguments=["--spec", str(spec), "--profile", str(profile_path),
                                           "--assumptions", str(root/"config/robot_assumptions.yaml")],
                                parameters=[{"use_sim_time": True}], output="screen"), "LiDAR sector merger exited")
        engineering = root/"config"/("wingbot_engineering.yaml" if wingbot_root else "robot_assumptions.yaml")
        frame_args = ["--frames", str(wingbot_root/"config/frames.yaml")] if wingbot_root else []
        for slot in active if full_sensors else [ego]:
            suffix = "_"+slot.replace("-", "") if full_sensors else ""
            actions += _critical(Node(package="maicon_arena_gz", executable="mil_emulator.py", name="arena_mil"+suffix,
                                arguments=["--spec", str(spec), "--assumptions", str(engineering),
                                           "--ego-slot", slot, "--interface-package", LaunchConfiguration("interface_package")]+frame_args,
                                remappings=emulator_remappings(slot) if full_sensors else [],
                                parameters=[{"use_sim_time": True}], output="screen"), "MIL command watchdog exited"+suffix)
    if _boolean(context, "referee"):
        geometry_args = (["--frames", str(wingbot_root/"config/frames.yaml")] if wingbot_root else
                         ["--robot-assumptions", str(root/"config/robot_assumptions.yaml")])
        actions += _critical(Node(package="maicon_arena_gz", executable="arena_referee.py", name="arena_referee",
                            arguments=["--spec", str(spec), "--layout", str(layout),
                                       "--assumptions", str(root/"config/simulation_assumptions.yaml")]+geometry_args,
                            parameters=[{"use_sim_time": True}], output="screen"), "Arena referee exited")
    if _boolean(context, "gui"):
        gui_env = dict(graphics_env)
        if gui_software:
            gui_env["LIBGL_ALWAYS_SOFTWARE"] = "1" if gui_software == "true" else "0"
        actions.append(ExecuteProcess(cmd=(["ign", "gazebo"] if simulator == "fortress" else ["gz", "sim"])+
                                      ["-g", "--render-engine", gui_engine], output="screen",
                                      additional_env=gui_env))
    return actions


def generate_launch_description():
    defaults = {"layout": "layout_default.yaml", "profile": "standard", "gui": "false",
                "render_engine": "ogre2", "simulator": "fortress", "ugvs": "1", "ego_slot": "B-2",
                "uav_cam": "true", "base_map": "", "validation_camera": "false", "referee": "true",
                "interface_package": "mil_interfaces_sim", "gui_render_engine": "", "gui_software_rendering": "",
                "wsl_gpu_compat": "false", "image_transport_profile": "inherit", "robot_model": "wingbot", "full_sensors": "false"}
    return LaunchDescription([DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()]+[OpaqueFunction(function=_setup)])
