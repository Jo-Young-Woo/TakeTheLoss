"""Inspect the authoritative model in RViz with movable original pan/tilt joints."""
from pathlib import Path
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    package=Path(get_package_share_directory('wingbot_description'))
    team=LaunchConfiguration('team').perform(context).upper()
    prefix=LaunchConfiguration('prefix').perform(context)
    if team not in ('BLUE','RED'):
        raise ValueError('team must be BLUE or RED')
    description=subprocess.check_output(['xacro',str(package/'urdf/wingbot.urdf.xacro'),f'team:={team}',f'prefix:={prefix}','sensors:=none'],text=True)
    return [Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[{'robot_description':description}]),
            Node(package='joint_state_publisher_gui',executable='joint_state_publisher_gui'),
            Node(package='rviz2',executable='rviz2',arguments=['-d',str(package/'config/wingbot.rviz')])]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('team',default_value='BLUE'),DeclareLaunchArgument('prefix',default_value=''),OpaqueFunction(function=setup)])
