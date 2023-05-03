/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "absl/memory/memory.h"
#include "cartographer/mapping/map_builder.h"
#include "cartographer_ros/node.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/ros_log_sink.h"
#include "gflags/gflags.h"
#include "tf2_ros/transform_listener.h"

#include "cartographer_ros/msg_conversion.h"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"


DEFINE_bool(collect_metrics, false,
            "Activates the collection of runtime metrics. If activated, the "
            "metrics can be accessed via a ROS service.");
DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");
DEFINE_string(load_state_filename, "",
              "If non-empty, filename of a .pbstream file to load, containing "
              "a saved SLAM state.");
DEFINE_bool(load_frozen_state, true,
            "Load the saved state as frozen (non-optimized) trajectories.");
DEFINE_bool(
    start_trajectory_with_default_topics, true,
    "Enable to immediately start the first trajectory with default topics.");
DEFINE_string(
    save_state_filename, "",
    "If non-empty, serialize state and write it to disk before shutting down.");

namespace cartographer_ros {
namespace {

rclcpp::Node::SharedPtr cartographer_node ;
cartographer_ros::Node* node;
cartographer_ros::TrajectoryOptions trajectory_options;

// 订阅位置重置的话题
void Reset_InitPose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg) {
  RCLCPP_WARN(cartographer_node->get_logger(),"RESET CARTO POSE !");
  // 关闭当前运行的Trajectories
  node->FinishAllTrajectories();
  // 给轨迹设置起点 msg->pose.pose
  // start trajectory with initial pose
  *trajectory_options.trajectory_builder_options.mutable_initial_trajectory_pose()->mutable_relative_pose() 
      = cartographer::transform::ToProto(cartographer_ros::ToRigid3d(msg->pose.pose));
  // 重新开启Trajectory
  if (FLAGS_start_trajectory_with_default_topics) 
  {
    node->StartTrajectoryWithDefaultTopics(trajectory_options);
  }
  return ;
}

void Run() {
  
  cartographer_node = rclcpp::Node::make_shared("cartographer_node");
  auto initPose_sub = cartographer_node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose_carto",1,std::bind(&Reset_InitPose_callback, std::placeholders::_1));
  
  constexpr double kTfBufferCacheTimeInSeconds = 10.;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer =
      std::make_shared<tf2_ros::Buffer>(
        cartographer_node->get_clock(),
        tf2::durationFromSec(kTfBufferCacheTimeInSeconds),
        cartographer_node);

  std::shared_ptr<tf2_ros::TransformListener> tf_listener =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  NodeOptions node_options;
  // TrajectoryOptions trajectory_options;
  std::tie(node_options, trajectory_options) =
      LoadOptions(FLAGS_configuration_directory, FLAGS_configuration_basename);

  auto map_builder =
    cartographer::mapping::CreateMapBuilder(node_options.map_builder_options);

  // auto node = std::make_shared<cartographer_ros::Node>(
  auto node_1 = cartographer_ros::Node(
    node_options, std::move(map_builder), tf_buffer, cartographer_node,
    FLAGS_collect_metrics);
  node = &(node_1);

  /** begin 位置初始化 **/
  if(trajectory_options.localization == true) {
    geometry_msgs::msg::Pose init_pose;
    init_pose.position.x = trajectory_options.inital_pose_x ;
    init_pose.position.y = trajectory_options.inital_pose_y ;
    init_pose.position.z = trajectory_options.inital_pose_z ;
    init_pose.orientation.x = trajectory_options.inital_orientation_x ;
    init_pose.orientation.y = trajectory_options.inital_orientation_y ;
    init_pose.orientation.z = trajectory_options.inital_orientation_z ;
    init_pose.orientation.w = trajectory_options.inital_orientation_w ;
    
    *trajectory_options.trajectory_builder_options.mutable_initial_trajectory_pose()->mutable_relative_pose() 
      = cartographer::transform::ToProto(cartographer_ros::ToRigid3d(init_pose));;
  }
  /** end 位置初始化 **/

  if (!FLAGS_load_state_filename.empty()) {
    node->LoadState(FLAGS_load_state_filename, FLAGS_load_frozen_state);
  }

  if (FLAGS_start_trajectory_with_default_topics) {
    node->StartTrajectoryWithDefaultTopics(trajectory_options);
  }

  rclcpp::spin(cartographer_node);

  node->FinishAllTrajectories();
  node->RunFinalOptimization();

  if (!FLAGS_save_state_filename.empty()) {
    node->SerializeState(FLAGS_save_state_filename,
                        true /* include_unfinished_submaps */);
  }
}

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {
  // Init rclcpp first because gflags reorders command line flags in argv
  rclcpp::init(argc, argv);

  google::AllowCommandLineReparsing();
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);

  CHECK(!FLAGS_configuration_directory.empty())
      << "-configuration_directory is missing.";
  CHECK(!FLAGS_configuration_basename.empty())
      << "-configuration_basename is missing.";

  cartographer_ros::ScopedRosLogSink ros_log_sink;
  cartographer_ros::Run();
  ::rclcpp::shutdown();
}
