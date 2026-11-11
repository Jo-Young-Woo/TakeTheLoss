"""Spawn WINGBOT into an existing Fortress world; never starts a second simulator."""
from pathlib import Path
import math
import re
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    def get(name):return LaunchConfiguration(name).perform(context)
    package=Path(get_package_share_directory('wingbot_description'))
    team,slot,sensors,wheels,depth=get('team').upper(),get('slot'),get('sensors'),get('wheels'),get('depth')
    if team not in ('BLUE','RED') or sensors not in ('full','none') or wheels not in ('2','6') or depth not in ('true','false'):
        raise ValueError('Invalid team/sensors/wheels/depth model option')
    if not re.fullmatch(r'[A-Za-z][A-Za-z0-9_-]*',slot):raise ValueError('Invalid slot')
    pose=[float(get(n)) for n in ('x','y','z','yaw')]
    if not all(math.isfinite(v) for v in pose):raise ValueError('Spawn pose must be finite')
    model='wingbot_'+slot.replace('-','')
    namespace=get('namespace') or '/'+model
    prefix=get('prefix')
    description=subprocess.check_output(['xacro',str(package/'urdf/wingbot.urdf.xacro'),f'model_name:={model}',f'team:={team}',f'prefix:={prefix}',f'namespace:={namespace}',f'sensors:={sensors}',f'wheels:={wheels}',f'depth:={depth}',f'effective_separation:={get("effective_separation")}'],text=True)
    return [Node(package='robot_state_publisher',executable='robot_state_publisher',namespace=namespace,parameters=[{'robot_description':description,'use_sim_time':True}],remappings=[('joint_states',namespace+'/joint_states')]),
            Node(package='ros_gz_sim',executable='create',arguments=['-world',get('world'),'-name',model,'-string',description,'-x',str(pose[0]),'-y',str(pose[1]),'-z',str(pose[2]),'-Y',str(pose[3])],output='screen'),
            Node(package='ros_gz_bridge',executable='parameter_bridge',arguments=[namespace+'/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model'],parameters=[{'use_sim_time':True}])]


def generate_launch_description():
    defaults={'team':'BLUE','slot':'B-2','namespace':'','prefix':'','sensors':'full','depth':'false','wheels':'6','effective_separation':'0.1745','world':'default','x':'0','y':'0','z':'0.002','yaw':'0'}
    return LaunchDescription([*(DeclareLaunchArgument(k,default_value=v) for k,v in defaults.items()),OpaqueFunction(function=setup)])
