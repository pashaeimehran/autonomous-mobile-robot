-- Include required configuration files for map building and trajectory building
include "map_builder.lua"
include "trajectory_builder.lua"

-- Define options table to configure Cartographer SLAM behavior
options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",  -- Frame for publishing the map
  tracking_frame = "base_link",  -- Frame used to track the robot's position
  published_frame = "base_link",  -- Frame used for publishing the robot's pose
  odom_frame = "odom",  -- Frame used for odometry
  provide_odom_frame = false,  -- Whether to provide an odometry frame (false disables)
  publish_frame_projected_to_2d = false,  -- Whether to project the frame to 2D (set to false)
  use_pose_extrapolator = true,  -- Whether to use pose extrapolation to predict the robot's pose
  use_odometry = false,  -- Whether to use odometry data for SLAM
  use_nav_sat = false,  -- Whether to use GPS data (disabled here)
  use_landmarks = false,  -- Whether to use landmarks for SLAM (disabled here)
  num_laser_scans = 1,  -- Number of laser scans to use (set to 1 for a single 2D laser scanner)
  num_multi_echo_laser_scans = 0,  -- Number of multi-echo laser scans to use (set to 0 for no multi-echo data)
  num_subdivisions_per_laser_scan = 10,  -- Number of subdivisions to apply to each laser scan
  num_point_clouds = 0,  -- Number of point clouds to use (set to 0 as we are using laser scans)
  lookup_transform_timeout_sec = 0.2,  -- Timeout for looking up transforms
  submap_publish_period_sec = 0.3,  -- Period for publishing submaps
  pose_publish_period_sec = 5e-3,  -- Period for publishing poses
  trajectory_publish_period_sec = 30e-3,  -- Period for publishing the robot's trajectory
  rangefinder_sampling_ratio = 1.,  -- Sampling ratio for rangefinder data (set to 1 for full data usage)
  odometry_sampling_ratio = 1.,  -- Sampling ratio for odometry data (set to 1 for full data usage)
  fixed_frame_pose_sampling_ratio = 1.,  -- Sampling ratio for fixed frame poses
  imu_sampling_ratio = 0.,  -- Sampling ratio for IMU data (disabled here, set to 0)
  landmarks_sampling_ratio = 1.,  -- Sampling ratio for landmarks
}

-- Enable 2D trajectory building in Cartographer
MAP_BUILDER.use_trajectory_builder_2d = true

-- Configure the 2D trajectory builder
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 7  -- Number of laser scans to accumulate before performing scan matching
TRAJECTORY_BUILDER_2D.min_range = 0.1  -- Minimum range for the laser scanner
TRAJECTORY_BUILDER_2D.max_range = 10  -- Maximum range for the laser scanner
TRAJECTORY_BUILDER_2D.use_imu_data = false  -- Disable use of IMU data for scan matching
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true  -- Enable real-time correlative scan matching

-- Configure the real-time correlative scan matcher
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1  -- Linear search window for scan matching
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 0.1  -- Weight for translation error in scan matching
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1  -- Weight for rotation error in scan matching

-- Return the options table to configure Cartographer
return options
