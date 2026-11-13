-- Cartographer 2D (WINGBOT 이 지원하는 SLAM 중 하나). ROS2 Humble, 2D LiDAR(D500) + 바퀴 오도메트리.
-- !! 로봇에서 아직 실행해 보지 못한 시작값이다. 프레임 이름은 로봇 드라이버의 TF 와 맞춰야 한다(ros2/README.md).
--
-- 이 설정에서 SLAM 이 하는 일: TF map→odom 발행(= 위치추정). Nav2 는 /map 을 쓰지 않는다(nav2_params.yaml 설명 참고).
-- 구조물이 바뀌는 경기장이라 지도 갱신이 빠르도록 miss_probability 를 기본(0.49)보다 낮췄다.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "odom",          -- 드라이버가 odom→base_link TF 를 내고, Cartographer 는 map→odom 만 낸다
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,
  use_pose_extrapolator = true,
  use_odometry = true,               -- /odom 이 없거나 프레임이 안 맞으면 false
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.min_range = 0.12
TRAJECTORY_BUILDER_2D.max_range = 8.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.0
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.2)
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 40
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.insert_free_space = true
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = 0.55
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = 0.42

-- 경기장은 6x5 m 로 작고 구조물이 바뀔 수 있다: 루프 클로저는 보수적으로(잘못된 제약이 지도를 비틀지 않게)
POSE_GRAPH.optimize_every_n_nodes = 35
POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.75
POSE_GRAPH.optimization_problem.huber_scale = 1e2

return options
