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

#include "ekf_localization_trigger_module.hpp"

#include <component_interface_specs/localization.hpp>
#include <component_interface_utils/rclcpp/exceptions.hpp>

#include <memory>
#include <string>

using ServiceException = component_interface_utils::ServiceException;
using Initialize = localization_interface::Initialize;

EkfLocalizationTriggerModule::EkfLocalizationTriggerModule(rclcpp::Node * node)
: logger_(node->get_logger())
{
  // 激活 ekf_trigger_node 的服务请求端
  client_ekf_trigger_ = node->create_client<SetBool>("ekf_trigger_node");
}

void EkfLocalizationTriggerModule::send_request(bool flag) const
{
  // 创建请求端的数据变量 req
  const auto req = std::make_shared<SetBool::Request>();
  std::string command_name;
  // 赋予 req 请求的data数据值，true / false
  req->data = flag;
  // 判断是激活还是挂起，用于后面日志打印
  if (flag) {
    command_name = "Activation";
  } else {
    command_name = "Deactivation";
  }
  // ekf ekf_trigger_node 服务端未在线，抛出异常
  if (!client_ekf_trigger_->service_is_ready()) {
    throw component_interface_utils::ServiceUnready("EKF triggering service is not ready");
  }
  // 发起请求，等待反馈结果 res
  auto future_ekf = client_ekf_trigger_->async_send_request(req);
  // 日志打印
  if (future_ekf.get()->success) {
    RCLCPP_INFO(logger_, "EKF %s succeeded", command_name.c_str());
  } else {
    RCLCPP_INFO(logger_, "EKF %s failed", command_name.c_str());
    throw ServiceException(
      Initialize::Service::Response::ERROR_ESTIMATION, "EKF " + command_name + " failed");
  }
}
