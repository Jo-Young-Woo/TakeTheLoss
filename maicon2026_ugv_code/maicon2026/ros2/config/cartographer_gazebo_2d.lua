-- Fixed 6.12 x 5 m Gazebo arena, known starting base, 160-degree LiDAR.
-- This profile has NO random M3 positions, labels or simulator truth poses.
-- Keep real hardware on its separately calibrated profile.
include "cartographer_2d.lua"

-- Repeated, symmetric arena structures produced accepted 5-6 m loop matches
-- with scores up to .77 in b2_no_uav_fpv_20261109_run3.launch.log. Raising the
-- score alone does not reject those aliases: constrain geometric correction.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 0.25
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(8.)
POSE_GRAPH.constraint_builder.max_constraint_distance = 1.5
POSE_GRAPH.constraint_builder.min_score = 0.80
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.90
-- We track from a known start. Global relocalization over the whole symmetric
-- arena must not silently replace the current pose with a distant lookalike.
POSE_GRAPH.global_sampling_ratio = 0.
-- Restore upstream robust-loss strength rather than the generic profile's 100.
POSE_GRAPH.optimization_problem.huber_scale = 10.

return options
