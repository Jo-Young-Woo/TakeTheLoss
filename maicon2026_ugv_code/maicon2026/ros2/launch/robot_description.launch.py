"""보정된 URDF 로 TF(base_link→laser/imu/camera…)를 발행한다 (robot_state_publisher).

    ros2 launch /path/to/ros2/launch/robot_description.launch.py            # ros2/urdf/wingbot_ugv.urdf
URDF 는 scripts/calib_generate.py 가 robot_profile.yaml 에서 만든다. 로봇 드라이버가 이미 같은 정적 TF 를 내고 있으면
중복 발행(충돌)되므로 둘 중 하나만 쓴다. 팬/틸트는 고정(0°) 조인트 — 동적 각도는 Agent 가 따로 관리한다.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ROS2_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_URDF = os.path.join(ROS2_DIR, "urdf", "wingbot_ugv.urdf")


def _read(path: str) -> str:
    with open(path, "r", encoding="utf-8") as f:
        return f.read()


def generate_launch_description():
    from launch.actions import OpaqueFunction

    def setup(context, *args, **kwargs):
        urdf = LaunchConfiguration("urdf").perform(context)
        sim = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
        return [Node(package="robot_state_publisher", executable="robot_state_publisher", output="screen",
                     parameters=[{"robot_description": _read(urdf), "use_sim_time": sim}])]

    return LaunchDescription([
        DeclareLaunchArgument("urdf", default_value=DEFAULT_URDF),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        OpaqueFunction(function=setup),
    ])
