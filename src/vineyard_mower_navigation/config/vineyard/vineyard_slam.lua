-- Cartographer Configuration for Vineyard Environment
-- Optimized for structured environments with parallel rows

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = true,
  use_odometry = false,
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

-- Trajectory builder configuration optimized for vineyard rows
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 35
TRAJECTORY_BUILDER_2D.min_range = 0.3
TRAJECTORY_BUILDER_2D.max_range = 20.
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 1.
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1

-- Ceres scan matcher settings for vineyard environments
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 1.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = false
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 20
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.num_threads = 1

-- Motion filter settings for agricultural vehicles
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 5.
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.2
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(1.)

-- IMU configuration for outdoor environments
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 10.

-- Submap settings optimized for structured environments
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.grid_type = "PROBABILITY_GRID"
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.range_data_inserter_type = "PROBABILITY_GRID_INSERTER_2D"
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.insert_free_space = true
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = 0.55
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = 0.49

-- Map builder settings for vineyard mapping
MAP_BUILDER.use_trajectory_builder_2d = true

-- Pose graph optimization settings
MAP_BUILDER.pose_graph.optimize_every_n_nodes = 90
MAP_BUILDER.pose_graph.constraint_builder.sampling_ratio = 0.3
MAP_BUILDER.pose_graph.optimization_problem.ceres_solver_options.max_num_iterations = 10
MAP_BUILDER.pose_graph.constraint_builder.min_score = 0.62
MAP_BUILDER.pose_graph.constraint_builder.global_localization_min_score = 0.66

-- Fast correlative scan matcher for loop closure
MAP_BUILDER.pose_graph.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 7.
MAP_BUILDER.pose_graph.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(30.)
MAP_BUILDER.pose_graph.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 7

-- Ceres scan matcher for refined loop closure
MAP_BUILDER.pose_graph.constraint_builder.ceres_scan_matcher.occupied_space_weight = 20.
MAP_BUILDER.pose_graph.constraint_builder.ceres_scan_matcher.translation_weight = 10.
MAP_BUILDER.pose_graph.constraint_builder.ceres_scan_matcher.rotation_weight = 1.
MAP_BUILDER.pose_graph.constraint_builder.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = true
MAP_BUILDER.pose_graph.constraint_builder.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 10
MAP_BUILDER.pose_graph.constraint_builder.ceres_scan_matcher.ceres_solver_options.num_threads = 1

-- Huber loss for robust optimization in agricultural environments
MAP_BUILDER.pose_graph.optimization_problem.huber_scale = 5e2
MAP_BUILDER.pose_graph.optimization_problem.acceleration_weight = 1e1
MAP_BUILDER.pose_graph.optimization_problem.rotation_weight = 3e2

-- Matcher translation and rotation weights for vineyard structure
MAP_BUILDER.pose_graph.constraint_builder.loop_closure_translation_weight = 1.1e4
MAP_BUILDER.pose_graph.constraint_builder.loop_closure_rotation_weight = 1e5

return options
