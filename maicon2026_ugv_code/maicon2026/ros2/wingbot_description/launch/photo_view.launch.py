"""Actual Gazebo photo pose, actual joint/TF observation and bounded capture."""
from pathlib import Path
import json
import os
import subprocess
import sys
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context):
    package = Path(get_package_share_directory("wingbot_description"))
    sys.path.insert(0, str(package / "scripts"))
    from photo_overlay_test import make_photo_world
    team = LaunchConfiguration("team").perform(context).upper()
    engine = LaunchConfiguration("render_engine").perform(context)
    if team not in ("RED", "BLUE") or engine not in ("ogre", "ogre2"):
        raise ValueError("Unsupported team or renderer")
    output = Path(LaunchConfiguration("output_dir").perform(context)).resolve()
    output.mkdir(parents=True, exist_ok=True)
    temporary = Path(tempfile.mkdtemp(prefix="wingbot_photo_"))
    description = subprocess.check_output(["xacro", str(package / "urdf/wingbot.urdf.xacro"),
        "team:=" + team, "model_name:=wingbot", "namespace:=/wingbot", "prefix:=",
        "sensors:=none", "depth:=false", "wheels:=6"], text=True)
    urdf = temporary / "wingbot.urdf"
    urdf.write_text(description, encoding="utf-8")
    sdf = subprocess.check_output(["ign", "sdf", "-p", str(urdf)], text=True)
    camera = json.loads((package / "config/photo_fit/cam_fit.json").read_text())
    world = temporary / "photo_world.sdf"
    world.write_text(make_photo_world(sdf, camera, engine), encoding="utf-8")
    # One RGB image is 3.34 MB. A process-local 64 MiB pool avoids the WSL
    # default transport truncating repeated frames. No global settings change.
    dds = temporary / "photo_dds.xml"
    dds.write_text('''<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
      <transport_descriptors>
        <transport_descriptor><transport_id>photo_shm</transport_id><type>SHM</type><segment_size>67108864</segment_size></transport_descriptor>
        <transport_descriptor><transport_id>photo_udp</transport_id><type>UDPv4</type></transport_descriptor>
      </transport_descriptors>
      <participant profile_name="photo_local" is_default_profile="true"><rtps>
        <userTransports><transport_id>photo_shm</transport_id><transport_id>photo_udp</transport_id></userTransports>
        <useBuiltinTransports>false</useBuiltinTransports>
      </rtps></participant></profiles>''', encoding="utf-8")
    resource = os.pathsep.join(filter(None, (str(package.parent), context.environment.get("IGN_GAZEBO_RESOURCE_PATH", ""))))
    software = LaunchConfiguration("software_rendering").perform(context).lower()
    if software not in ("true", "false"):
        raise ValueError("software_rendering must be true or false")
    server = ExecuteProcess(cmd=["ign", "gazebo", "-r", "-s", str(world), "--render-engine", engine],
                            output="screen", additional_env={"LIBGL_ALWAYS_SOFTWARE": "1" if software == "true" else "0"})
    bridge = Node(package="ros_gz_bridge", executable="parameter_bridge", name="photo_bridge",
                  arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
                             "/wingbot/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model",
                             "/world/wingbot_photo/dynamic_pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V"],
                  remappings=[("/wingbot/joint_states", "/joint_states"),
                              ("/world/wingbot_photo/dynamic_pose/info", "/wingbot/photo/poses")], output="screen")
    capture = ExecuteProcess(cmd=[sys.executable, str(package / "scripts/photo_overlay_test.py"),
        "--capture", "--package", str(package), "--output-dir", str(output), "--team", team], output="screen")
    actions = [SetEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", resource),
               SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource),
               SetEnvironmentVariable("FASTRTPS_DEFAULT_PROFILES_FILE", str(dds)),
               SetEnvironmentVariable("FASTDDS_DEFAULT_PROFILES_FILE", str(dds)),
               SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp"),
               server, bridge,
               Node(package="robot_state_publisher", executable="robot_state_publisher",
                    parameters=[{"robot_description": description, "use_sim_time": True}], output="screen"),
               Node(package="ros_gz_image", executable="image_bridge", arguments=["/wingbot/photo/image"], output="screen"),
               capture,
               RegisterEventHandler(OnProcessExit(target_action=capture,
                   on_exit=[EmitEvent(event=Shutdown(reason="Photo capture/evaluation finished; inspect its JSON gate result"))])),
               RegisterEventHandler(OnProcessExit(target_action=server,
                   on_exit=[EmitEvent(event=Shutdown(reason="Gazebo photo server stopped"))]))]
    # Preserve the exact converted runtime model and world as provenance. These
    # are diagnostic outputs, never substitutions for the RGB gate evidence.
    (output / f"photo_runtime_{team}.urdf").write_text(description, encoding="utf-8")
    (output / f"photo_runtime_{team}.sdf").write_text(world.read_text(), encoding="utf-8")
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("team", default_value="RED"),
        DeclareLaunchArgument("render_engine", default_value="ogre2"),
        DeclareLaunchArgument("software_rendering", default_value="true"),
        DeclareLaunchArgument("output_dir", default_value=str(Path.cwd() / "wingbot_photo_report")),
        OpaqueFunction(function=_setup)])
