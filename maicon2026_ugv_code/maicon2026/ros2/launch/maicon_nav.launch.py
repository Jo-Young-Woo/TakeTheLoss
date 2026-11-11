"""SLAM + Nav2 를 한 번에 띄운다 (ROS2 Humble). !! 로봇에서 아직 실행해 보지 못했다 — ros2/README.md 의 순서로 단계별 확인.

    ros2 launch /path/to/ros2/launch/maicon_nav.launch.py slam:=cartographer
    ros2 launch /path/to/ros2/launch/maicon_nav.launch.py slam:=slam_toolbox fast_replan:=false

전제(로봇 드라이버가 이미 내는 것): /scan (D500), /odom 과 TF odom→base_link, base_link→(라이다 프레임) 정적 TF.
  2차 사전교육 PDF 에는 /odom 만 있다 → TF 가 없으면 `odom_tf:=auto`(이 launch 가 scripts/odom_tf_bridge.py 를 띄움; 이미 TF 가 있으면 쉰다)
  와 `ros2 launch ros2/launch/robot_description.launch.py`(URDF 정적 TF) 를 같이 쓴다.
이 launch 가 내는 것: TF map→odom (SLAM), 액션 /navigate_to_pose, 속도 /cmd_vel_nav2 (※ /cmd_vel 아님).
maicon26 Agent 가 /cmd_vel_nav2 를 읽어 LiDAR 안전 제한을 적용한 뒤 로봇에 보낸다.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

HERE = os.path.dirname(os.path.abspath(__file__))
ROS2_DIR = os.path.dirname(HERE)
CFG = os.path.join(ROS2_DIR, "config")
BT_FAST = os.path.join(ROS2_DIR, "behavior_trees", "navigate_replan_fast.xml")


def _cartographer_config(base_frame, requested):
    builtins = {"base_link": "cartographer_2d.lua", "base_footprint": "cartographer_waveshare.lua"}
    profile_frames = {name: frame for frame, name in builtins.items()}
    profile_frames["cartographer_gazebo_2d.lua"] = "base_link"
    chosen = requested or builtins.get(base_frame)
    if not chosen:
        raise RuntimeError("Custom base_frame requires an explicit matching cartographer_config Lua basename")
    if chosen in profile_frames and profile_frames[chosen] != base_frame:
        raise RuntimeError(f"Cartographer tracking frame mismatch: {base_frame} / {chosen}")
    if os.path.basename(chosen) != chosen or not chosen.endswith(".lua") or not os.path.isfile(os.path.join(CFG, chosen)):
        raise RuntimeError("cartographer_config must be an existing Lua basename in ros2/config")
    return chosen


def _opt(context, name, default=""):
    """선언되지 않았거나 비어 있는 선택 인자는 기본값으로 읽는다(시뮬·옛 호출은 이 인자들을 모른다)."""
    try:
        value = LaunchConfiguration(name).perform(context)
    except Exception:
        return default
    return value if value != "" else default


def _launch_setup(context, *args, **kwargs):
    slam = LaunchConfiguration("slam").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    fast = LaunchConfiguration("fast_replan").perform(context).lower() == "true"
    params = LaunchConfiguration("params_file").perform(context)
    base_frame = LaunchConfiguration("base_frame").perform(context)
    cartographer_config = LaunchConfiguration("cartographer_config").perform(context)
    scan_topic = LaunchConfiguration("scan_topic").perform(context)
    odom_topic = LaunchConfiguration("odom_topic").perform(context)
    nav_cmd_topic = LaunchConfiguration("nav_cmd_topic").perform(context)
    for name, value in (("scan_topic", scan_topic), ("odom_topic", odom_topic), ("nav_cmd_topic", nav_cmd_topic)):
        if not value.startswith("/") or len(value) < 2:
            raise RuntimeError(f"{name} must be an absolute ROS topic name")
    # LiDAR 자기 가림 필터(선택): 마스크가 비어 있으면 아무것도 바꾸지 않는다. 있으면 ros2/scripts/scan_self_mask.py 가
    # scan_topic → <scan_topic>_filtered 로 낸다(Agent 는 계속 원본을 받는다). SLAM·Nav2 는 필터된 토픽을 쓴다.
    self_mask = _opt(context, "scan_self_mask").strip()
    try:
        mask_range, min_range = float(_opt(context, "scan_self_mask_range", "0.35")), float(_opt(context, "scan_min_range", "0"))
    except ValueError as e:
        raise RuntimeError(f"scan_self_mask_range / scan_min_range 는 숫자여야 합니다: {e}")
    scan_filter = bool(self_mask) or min_range > 0.0
    scan_raw = scan_topic
    if scan_filter:
        if scan_raw.endswith("_filtered"):
            raise RuntimeError("scan_topic 이 이미 필터된 토픽(_filtered)입니다 — scan_self_mask 와 함께 쓰면 입력과 출력이 같아집니다")
        scan_topic = scan_raw + "_filtered"
    if nav_cmd_topic.rstrip("/") == "/cmd_vel":
        raise RuntimeError("Nav2 must not publish /cmd_vel directly; Agent owns the final velocity output")
    if not base_frame or base_frame.startswith("/"):
        raise RuntimeError("base_frame must be a nonempty TF frame without a leading slash")
    # The same body frame must be used by BT, both costmaps and behaviors. The
    # nested costmap nodes do not inherit a controller_server parameter override.
    rewrites = {
        "robot_base_frame": base_frame,
        "use_sim_time": str(use_sim_time).lower(),
        "bt_navigator.ros__parameters.odom_topic": odom_topic,
        "controller_server.ros__parameters.odom_topic": odom_topic,
        "local_costmap.local_costmap.ros__parameters.obstacle_layer.scan.topic": scan_topic,
        "global_costmap.global_costmap.ros__parameters.obstacle_layer.scan.topic": scan_topic,
    }
    if LaunchConfiguration("depth_obstacles").perform(context).lower() != "true":
        rewrites.update({
            "local_costmap.local_costmap.ros__parameters.obstacle_layer.observation_sources": "scan",
            "global_costmap.global_costmap.ros__parameters.obstacle_layer.observation_sources": "scan virtual_walls",
        })
    params = RewrittenYaml(source_file=params, param_rewrites=rewrites, convert_types=True)
    sim = {"use_sim_time": use_sim_time}
    actions = []

    # ---------------------------------------------------------------- TF odom→base_link (로봇이 /odom 만 낼 때)
    mode = LaunchConfiguration("odom_tf").perform(context).lower()
    if mode in ("auto", "force"):                       # auto: 이미 다른 발행자가 있으면 쉰다. force: 항상 발행
        bridge = os.path.join(os.path.dirname(ROS2_DIR), "scripts", "odom_tf_bridge.py")
        cmd = ["python3", bridge, "--base-frame", base_frame, "--odom-topic", odom_topic] + (["--force"] if mode == "force" else []) + (["--use-sim-time"] if use_sim_time else [])
        actions.append(ExecuteProcess(cmd=cmd, output="screen"))
    elif mode != "off":
        raise RuntimeError(f"odom_tf 인자는 auto | force | off (받은 값: {mode})")

    # ---------------------------------------------------------------- LiDAR 자기 가림 필터 (마스크가 있을 때만)
    if scan_filter:
        script = os.path.join(ROS2_DIR, "scripts", "scan_self_mask.py")
        cmd = ["python3", script, "--input", scan_raw, "--output", scan_topic, "--base-frame", base_frame,
               "--sectors", self_mask, "--mask-range", str(mask_range), "--min-range", str(min_range)]
        if use_sim_time:
            cmd.append("--use-sim-time")
        actions.append(ExecuteProcess(cmd=cmd, output="screen"))

    # ---------------------------------------------------------------- SLAM (위치추정 + 지도)
    if slam == "cartographer":
        cartographer_config = _cartographer_config(base_frame, cartographer_config)
        actions += [
            Node(package="cartographer_ros", executable="cartographer_node", name="cartographer_node",
                 output="screen", parameters=[sim],
                 arguments=["-configuration_directory", CFG, "-configuration_basename", cartographer_config],
                 remappings=[("scan", scan_topic), ("odom", odom_topic)]),
            Node(package="cartographer_ros", executable="cartographer_occupancy_grid_node",
                 name="cartographer_occupancy_grid_node", output="screen", parameters=[sim],
                 arguments=["-resolution", "0.05", "-publish_period_sec", "0.5"]),
        ]
    elif slam == "slam_toolbox":
        actions += [
            Node(package="slam_toolbox", executable="async_slam_toolbox_node", name="slam_toolbox",
                 output="screen", parameters=[os.path.join(CFG, "slam_toolbox.yaml"), sim,
                                              {"base_frame": base_frame, "scan_topic": scan_topic}]),
        ]
    elif slam == "arena":
        if base_frame != "base_link":
            raise RuntimeError("arena localization requires the calibrated Gazebo base_link/LiDAR frames")
        script = os.path.join(os.path.dirname(ROS2_DIR), "scripts", "arena_localization.py")
        cmd = ["python3", script, "--scan-topic", scan_topic, "--odom-topic", odom_topic,
               "--start-x", LaunchConfiguration("arena_start_x").perform(context),
               "--start-y", LaunchConfiguration("arena_start_y").perform(context),
               "--start-yaw", LaunchConfiguration("arena_start_yaw").perform(context)]
        if use_sim_time:
            cmd.append("--use-sim-time")
        actions.append(ExecuteProcess(cmd=cmd, output="screen"))
    elif slam != "none":
        raise RuntimeError(f"slam 인자는 cartographer | slam_toolbox | arena | none (받은 값: {slam})")
    # slam:=none → Gmapping 등 다른 SLAM 을 직접 띄운다. map→odom TF 만 나오면 Nav2 쪽은 그대로 동작한다.

    # ---------------------------------------------------------------- Nav2 (cmd_vel → cmd_vel_nav2 로 리맵)
    if LaunchConfiguration("nav2").perform(context).lower() != "true":
        return actions                                  # SLAM 만(README 1단계 확인 / 폴백 시험)
    bt_xml = LaunchConfiguration("bt_xml").perform(context) or BT_FAST     # 보정본: navigate_replan_fast.calibrated.xml
    bt_override = {"default_nav_to_pose_bt_xml": bt_xml} if fast else {}
    nav_remap = [("cmd_vel", nav_cmd_topic)]
    actions += [
        Node(package="nav2_controller", executable="controller_server", output="screen",
             # Some Humble RewrittenYaml versions cannot insert absent keys.
             # controller_server's default odom topic must be overridden too.
             parameters=[params, sim, {"odom_topic": odom_topic}], remappings=nav_remap),
        Node(package="nav2_planner", executable="planner_server", name="planner_server", output="screen",
             parameters=[params, sim]),
        Node(package="nav2_behaviors", executable="behavior_server", name="behavior_server", output="screen",
             parameters=[params, sim], remappings=nav_remap),
        Node(package="nav2_bt_navigator", executable="bt_navigator", name="bt_navigator", output="screen",
             parameters=[params, sim, bt_override]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_navigation",
             output="screen", parameters=[params, sim]),
    ]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("slam", default_value="cartographer",
                              description="cartographer | slam_toolbox | arena (explicit fixed-field prior) | none"),
        DeclareLaunchArgument("arena_start_x", default_value="0.35"),
        DeclareLaunchArgument("arena_start_y", default_value="2.5"),
        DeclareLaunchArgument("arena_start_yaw", default_value="0.0"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("base_frame", default_value="base_link",
                              description="Nav2/slam_toolbox body frame; hardware wrapper uses base_footprint"),
        DeclareLaunchArgument("cartographer_config", default_value="",
                              description="Lua basename in ros2/config; empty selects body-frame default; Gazebo uses cartographer_gazebo_2d.lua"),
        DeclareLaunchArgument("scan_topic", default_value="/scan", description="Same calibrated/filtered LaserScan used by Agent"),
        DeclareLaunchArgument("scan_self_mask", default_value="",
                              description="LiDAR 자기 가림 구간 '중심deg:반폭deg,…' (예 '170:15,-170:15'). 비어 있으면 필터 노드를 띄우지 않고 SLAM·Nav2 는 scan_topic 을 그대로 쓴다"),
        DeclareLaunchArgument("scan_self_mask_range", default_value="0.35", description="구간 안에서 이보다 가까운 반사만 지운다 [m]"),
        DeclareLaunchArgument("scan_min_range", default_value="0", description="모든 방향에서 이보다 가까운 반사를 지운다 [m] (0 = 끔)"),
        DeclareLaunchArgument("odom_topic", default_value="/odom"),
        DeclareLaunchArgument("nav_cmd_topic", default_value="/cmd_vel_nav2", description="Agent input; never direct /cmd_vel"),
        DeclareLaunchArgument("depth_obstacles", default_value="true",
                              description="Enable calibrated depth obstacle source (hardware wrapper defaults false)"),
        DeclareLaunchArgument("odom_tf", default_value="off",
                              description="off | auto | force — /odom 으로 TF odom→base_link 를 낸다(로봇이 TF 를 안 낼 때). auto 는 이미 있으면 쉰다"),
        DeclareLaunchArgument("nav2", default_value="true", description="false 면 SLAM 만 띄운다"),
        DeclareLaunchArgument("fast_replan", default_value="true",
                              description="2 Hz 재계획 BT 사용(false 면 Nav2 기본 BT, 1 Hz)"),
        DeclareLaunchArgument("params_file", default_value=os.path.join(CFG, "nav2_params.yaml"),
                              description="보정본: ros2/config/nav2_params.calibrated.yaml"),
        DeclareLaunchArgument("bt_xml", default_value="", description="BT XML 경로(빈 값이면 navigate_replan_fast.xml)"),
        OpaqueFunction(function=_launch_setup),
    ])
