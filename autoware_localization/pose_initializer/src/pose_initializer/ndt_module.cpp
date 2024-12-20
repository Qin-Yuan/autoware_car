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

#include "ndt_module.hpp"

#include <component_interface_specs/localization.hpp>
#include <component_interface_utils/rclcpp/exceptions.hpp>

#include <memory>

using ServiceException = component_interface_utils::ServiceException;
using Initialize = localization_interface::Initialize;
using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;

// ndt定位点云初始化
// 给定一个大概位置，经过ndt scan smatcher 匹配后返一个较为准确的位置信息
NdtModule::NdtModule(rclcpp::Node * node) : logger_(node->get_logger())
{
  // 创建一个ndt请求端，对ndt_can_matcher发布初始位置请求
  cli_align_ = node->create_client<RequestPoseAlignment>("ndt_align");
}
// 请求函数
PoseWithCovarianceStamped NdtModule::align_pose(const PoseWithCovarianceStamped & pose)
{
  // 创建请求端得到请求数据变量
  const auto req = std::make_shared<RequestPoseAlignment::Request>();
  // ndt 初始化位置赋值
  req->pose_with_covariance = pose;
  // 检测NDT align服务端是否在线，不在线抛出异常
  if (!cli_align_->service_is_ready()) {
    throw component_interface_utils::ServiceUnready("NDT align server is not ready.");
  }
  // 对 ndt align 发送请求并等待返回结果 res , 其中 :
  //  - 1、success true/false 表示ndt定位是否有效
  //  - 2、pose_with_covariance 返回的ndt匹配定位信息
  RCLCPP_INFO(logger_, "Call NDT align server.");
  const auto res = cli_align_->async_send_request(req).get();
  // 如果返回success为false抛出异常
  if (!res->success) {
    RCLCPP_INFO(logger_, "NDT align server failed.");
    throw ServiceException(
      Initialize::Service::Response::ERROR_ESTIMATION, "NDT align server failed.");
  }
  RCLCPP_INFO(logger_, "NDT align server succeeded.");

  // 返回 带有协方差的 pose 
  return res->pose_with_covariance;
}
