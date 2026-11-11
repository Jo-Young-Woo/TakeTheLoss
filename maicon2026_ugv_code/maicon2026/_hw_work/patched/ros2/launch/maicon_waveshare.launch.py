"""Competition overlay on an already running Waveshare/MIL base.

Starts only our SLAM/Nav2. Never starts serial drivers, EKF, robot_state_publisher
or a /cmd_vel publisher. The competition base must provide /scan, /odom,
odom->base_footprint and sensor TF. Only if it publishes /odom without that TF,
opt in to the odom TF bridge with odom_tf:=auto (rests when a TF publisher exists);
the default is off. Agent is the sole final velocity publisher.
Use slam:=none if the robot already runs SLAM (map->odom).
scan_self_mask:="170:15,-170:15" (+ scan_self_mask_range, scan_min_range) starts ros2/scripts/scan_self_mask.py, which republishes
/scan as /scan_filtered with the robot's own mast/antenna returns set to NaN; SLAM and Nav2 then read the filtered topic. The Agent keeps
reading the raw /scan and applies its own SelfMask. Empty mask (default) = no node, nothing changes.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

HERE = os.path.dirname(os.path.abspath(__file__))
ROS2_DIR = os.path.dirname(HERE)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("slam", default_value="cartographer",
                              description="cartographer | slam_toolbox | none (existing SLAM)"),
        DeclareLaunchArgument("nav2", default_value="true"),
        DeclareLaunchArgument("fast_replan", default_value="true"),
        DeclareLaunchArgument("base_frame", default_value="base_footprint"),
        DeclareLaunchArgument("cartographer_config", default_value=""),
        DeclareLaunchArgument("scan_topic", default_value="/scan"),
        DeclareLaunchArgument("scan_self_mask", default_value="",
                              description="LiDAR 자기 가림 구간 '중심deg:반폭deg,…' (예 '170:15,-170:15'). 비어 있으면 필터 노드 없이 "
                                          "SLAM·Nav2 가 scan_topic 을 그대로 쓴다. 있으면 <scan_topic>_filtered 로 바꾼다(Agent 는 원본 사용)"),
        DeclareLaunchArgument("scan_self_mask_range", default_value="0.35", description="구간 안에서 이보다 가까운 반사만 지운다 [m]"),
        DeclareLaunchArgument("scan_min_range", default_value="0", description="모든 방향에서 이보다 가까운 반사를 지운다 [m] (0 = 끔)"),
        DeclareLaunchArgument("odom_topic", default_value="/odom"),
        DeclareLaunchArgument("nav_cmd_topic", default_value="/cmd_vel_nav2"),
        DeclareLaunchArgument("odom_tf", default_value="off",
                              description="off | auto | force. 로봇이 /odom 만 내고 odom->base_footprint TF 를 안 낼 때 auto "
                                          "(이미 TF 가 있으면 브리지가 쉰다). 기본 off = 로봇 베이스가 TF 를 낸다고 믿는다"),
        DeclareLaunchArgument("depth_obstacles", default_value="false",
                              description="Enable only after depth extrinsics/intrinsics calibration and Agent depth profile"),
        DeclareLaunchArgument("params_file", default_value=os.path.join(ROS2_DIR, "config", "nav2_params.yaml")),
        DeclareLaunchArgument("bt_xml", default_value=""),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(HERE, "maicon_nav.launch.py")),
            launch_arguments={
                "slam": LaunchConfiguration("slam"),
                "nav2": LaunchConfiguration("nav2"),
                "fast_replan": LaunchConfiguration("fast_replan"),
                "depth_obstacles": LaunchConfiguration("depth_obstacles"),
                "params_file": LaunchConfiguration("params_file"),
                "bt_xml": LaunchConfiguration("bt_xml"),
                "use_sim_time": "false",
                "odom_tf": LaunchConfiguration("odom_tf"),
                "base_frame": LaunchConfiguration("base_frame"),
                "cartographer_config": LaunchConfiguration("cartographer_config"),
                "scan_topic": LaunchConfiguration("scan_topic"),
                "scan_self_mask": LaunchConfiguration("scan_self_mask"),
                "scan_self_mask_range": LaunchConfiguration("scan_self_mask_range"),
                "scan_min_range": LaunchConfiguration("scan_min_range"),
                "odom_topic": LaunchConfiguration("odom_topic"),
                "nav_cmd_topic": LaunchConfiguration("nav_cmd_topic"),
            }.items(),
        ),
    ])
