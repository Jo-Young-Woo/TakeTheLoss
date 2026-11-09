"""깊이 카메라(OAK Stereo) 3D 지도용 선택 launch (ROS2 Humble). !! 실기에서 아직 실행해 보지 못했다 — 토픽·프레임 이름은 장비에서 확인.

maicon26 Agent 는 3D 지도를 **자체적으로**(perception/voxel_map.py) 만든다: 깊이 영상(config.depth.topic) + 카메라 보정(camera_info)
+ 카메라 자세(차체→팬→틸트→깊이 TF 는 코드가 URDF 와 같은 값으로 계산) + 로봇 자세(오도메트리·SLAM)로. 그래서 이 launch 는 필수가 아니다.
이 launch 는 같은 깊이 영상으로 ROS 쪽 3D 도구를 함께 쓰고 싶을 때(눈으로 확인·오프라인 저장) 쓴다:

    ros2 launch /path/to/ros2/launch/maicon_depth.launch.py cloud:=true          # 깊이 → 점군 (RViz 로 확인)
    ros2 launch /path/to/ros2/launch/maicon_depth.launch.py rtabmap:=true        # RTAB-Map 3D 지도 (오프라인 저장용, 임무 루프에는 쓰지 않음)

전제: 로봇 드라이버가 depth 이미지(16UC1 mm 또는 32FC1 m)와 camera_info 를 내고, TF 가 base_link→depth_optical 로 이어져 있다
(ros2/urdf/wingbot_ugv.urdf: 깊이 카메라는 팬/틸트 '아래' 차체에 고정이라 depth_link 의 부모가 base_link 다 — 팬/틸트가 움직여도 TF 는 변하지 않는다).
RTAB-Map 은 이 PC 에서 설치·실행해 보지 못했다(ros-humble-rtabmap-ros 필요).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    depth = LaunchConfiguration("depth_topic").perform(context)
    info = LaunchConfiguration("info_topic").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    sim = {"use_sim_time": use_sim_time}
    actions = []
    if LaunchConfiguration("cloud").perform(context).lower() == "true":
        # 깊이 영상 → 점군 (depth_image_proc). 시각화·오프라인 확인용
        actions.append(Node(package="depth_image_proc", executable="point_cloud_xyz_node", name="depth_to_cloud", output="screen",
                            parameters=[sim], remappings=[("image_rect", depth), ("camera_info", info), ("points", "/depth_points")]))
    if LaunchConfiguration("rtabmap").perform(context).lower() == "true":
        # RTAB-Map: RGB 없이 깊이만 쓰는 구성은 지원이 제한적이라 RGB-D 입력(rgb 토픽)을 같이 줘야 할 수 있다 — 장비에서 확인
        actions.append(Node(package="rtabmap_slam", executable="rtabmap", name="rtabmap", output="screen",
                            parameters=[{"frame_id": "base_link", "odom_frame_id": "odom", "subscribe_depth": True,
                                         "subscribe_rgb": False, "approx_sync": True, "Grid/FromDepth": "true",
                                         "Grid/MaxObstacleHeight": "0.5", "Grid/RangeMax": "3.5", "Mem/IncrementalMemory": "true"}, sim],
                            remappings=[("depth/image", depth), ("depth/camera_info", info), ("odom", "/odom")]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("depth_topic", default_value="/oak/stereo/image_raw"),
        DeclareLaunchArgument("info_topic", default_value="/oak/stereo/camera_info"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("cloud", default_value="true", description="깊이 → 점군(/depth_points)"),
        DeclareLaunchArgument("rtabmap", default_value="false", description="RTAB-Map 3D 지도(선택, 미검증)"),
        OpaqueFunction(function=_setup),
    ])
