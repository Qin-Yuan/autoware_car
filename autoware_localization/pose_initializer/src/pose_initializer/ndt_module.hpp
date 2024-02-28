// Copyright 2022 The Autoware Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef POSE_INITIALIZER__NDT_MODULE_HPP_
#define POSE_INITIALIZER__NDT_MODULE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tier4_localization_msgs/srv/pose_with_covariance_stamped.hpp>

class NdtModule
{
private:
  using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
  // 自定义的定位数据话题 ，req 和 res 都是 geometry_msgs/PoseWithCovarianceStamped 类型
  using RequestPoseAlignment = tier4_localization_msgs::srv::PoseWithCovarianceStamped;   

public:
  // 指定构造函数或转换函数 (C++11起)为显式, 即它不能用于隐式转换和复制初始化 ； explicit 关键字
  explicit NdtModule(rclcpp::Node * node);
  // ndt pose client端函数
  PoseWithCovarianceStamped align_pose(const PoseWithCovarianceStamped & pose);

private:
  // 先声明，便于后面赋初值 
  rclcpp::Logger logger_;
  rclcpp::Client<RequestPoseAlignment>::SharedPtr cli_align_;
};

#endif  // POSE_INITIALIZER__NDT_MODULE_HPP_
