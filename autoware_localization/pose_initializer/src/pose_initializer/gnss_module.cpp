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

#include "gnss_module.hpp"

#include <component_interface_specs/localization.hpp>
#include <component_interface_utils/rclcpp/exceptions.hpp>

#include <memory>

GnssModule::GnssModule(rclcpp::Node * node) : fitter_(node)
{
  // 创建 gnss 话题订阅者 ，回调函数直接赋值给 pose_ = msg
  sub_gnss_pose_ = node->create_subscription<PoseWithCovarianceStamped>(
    "gnss_pose_cov", 1, [this](PoseWithCovarianceStamped::ConstSharedPtr msg) { pose_ = msg; });

  clock_ = node->get_clock();
  timeout_ = node->declare_parameter<double>("gnss_pose_timeout");
}

// gnss 数据获取
geometry_msgs::msg::PoseWithCovarianceStamped GnssModule::get_pose()
{
  using Initialize = localization_interface::Initialize;
  // 如果 pose_ 为空，抛出异常
  if (!pose_) {
    // 没有收到 gnss 数据
    throw component_interface_utils::ServiceException(
      Initialize::Service::Response::ERROR_GNSS, "The GNSS pose has not arrived.");
  }
  // 检测 gnss 的时效性 
  const auto elapsed = rclcpp::Time(pose_->header.stamp) - clock_->now();
  // 抛出超时异常
  if (timeout_ < elapsed.seconds()) {
    throw component_interface_utils::ServiceException(
      Initialize::Service::Response::ERROR_GNSS, "The GNSS pose is out of date.");
  }
  // 赋值给result,然后返回
  auto result = *pose_;
  result.pose.pose.position = fitter_.fit(result.pose.pose.position, result.header.frame_id);
  return result;
}
