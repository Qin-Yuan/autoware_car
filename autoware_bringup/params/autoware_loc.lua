-- optimize_every_n_nodes：推荐参数修改范围为30-70，官方文档中建议减少该值来提高速度，推荐30左右配置较为良好

-- MAP_BUILDER.num_background_threads：建议优化至每个机器的CPU线程数量

-- global_sampling_ratio：在0.001，0.002时平均误差处于较小位置，优化时可以考虑的值为0.001和0.002

-- constraint_builder.sampling_ratio：该参数在0.2-0.25左右平均误差处于相对低位。最小方差的位置在在0.25左右，推荐参数修改范围为0.2-0.3

-- constraint_builder.min_score：该参数在0.75-0.95误差的方差和最大误差处于相对低位，推荐0.75-0.95为修改参数范围

-- search_windows_sizes.linear_xy_search_window：该参数在10-20左右误差的方差和最大误差上升较快，在10左右时处于较低点，推荐5-10为修改参数范围

-- search_windows_sizes.linear_z_search_window：该参数在4以后误差的方差和最大误差上升较快，在4左右时仍处于较低点，推荐1-4为修改参数范围

-- search_windows_sizes.angular_search_window：该参数随着角度的增加，误差的方差和最大误差减小，推荐60左右为修改参数范围

-- global_constraint_search_after_n_seconds：该参数随着间隔时间的增加误差的方差和最大误差逐渐下降，但最大误差逐渐上升，在30-40时处于两者的平衡点。推荐30-40为修改参数范围

-- ceres_scan_matcher.max_num_iterations：该参数误差波动较大，平均误差大时最大误差小，平均误差小时最大误差大，在7左右处于相对平衡点，推荐7-10为修改参数范围


include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu",               -- 如果用了 imu ，这里改成 imu_link , 否则会经常报错无法启动，概率性成功启动
  published_frame = "base_link",      -- base_link
  odom_frame = "odom",                -- odom
  provide_odom_frame = true,                 -- 供 odom -> base_link 变换
  use_pose_extrapolator = true,
  publish_tracked_pose  = true,              -- 发布pose
  publish_frame_projected_to_2d = false,
  publish_to_tf = true,                      -- 发布 map -> odom tf2 变换
  use_odometry = false,                      -- 是否用里程计，不用可以有效避免撞墙等现象
  use_nav_sat = false,  
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.5,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,          -- 1.
  odometry_sampling_ratio = 1.,             -- 1.
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,

  -- location 初始化，只有在纯定位时设置为true 
  localization = false,
  inital_pose_x = 2.02,
  inital_pose_y = -3.52,
  inital_pose_z = 0.0,
  inital_orientation_x = 0.0,
  inital_orientation_y = 0.0,
  inital_orientation_z = 0.62,
  inital_orientation_w = 0.78,
  
}


-- MAP
MAP_BUILDER.use_trajectory_builder_2d = true

MAP_BUILDER.num_background_threads = 7                    -- 建议优化至每个机器的CPU线程数量
POSE_GRAPH.global_sampling_ratio = 0.002                  -- 在 0.001 - 0.003 之间调
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3        -- 0.3
POSE_GRAPH.global_constraint_search_after_n_seconds = 30  -- 提供全局闭环优化间隔的时间 ，默认值 10 ，推荐30-40
POSE_GRAPH.constraint_builder.max_constraint_distance = 15
-- 降低资源消耗
POSE_GRAPH.pure_localization = true
POSE_GRAPH.log_residual_histograms = false
-- POSE_GRAPH.overlapping_submaps_trimmer_2d = {
--     fresh_submaps_count = 1,
--     min_covered_area = 2,
--     min_added_submaps_count = 5,
--   }

TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 2,      -- 保存两张sub_map就够了
}
-- POSE_GRAPH.optimize_every_n_nodes = 10

-- Lidar
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 50
TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 12.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 1.

-- IMU
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 9.7883

TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1

TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(1.0)
TRAJECTORY_BUILDER_2D.motion_filter. max_distance_meters = 0.2

POSE_GRAPH.optimization_problem.huber_scale = 1e2
POSE_GRAPH.optimize_every_n_nodes = 30                              -- 全局SLAM定位效果，如果设置为0就表示禁用，光靠local_slam , 之前设置的35
POSE_GRAPH.constraint_builder.min_score = 0.6                       -- 得分要求，高一点会更准
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7   -- 同上
POSE_GRAPH.constraint_builder.log_matches = false

return options
