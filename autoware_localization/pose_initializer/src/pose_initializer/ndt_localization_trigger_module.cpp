// Copyright 2023 The Autoware Contributors
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

#include "ndt_localization_trigger_module.hpp"

#include <component_interface_specs/localization.hpp>
#include <component_interface_utils/rclcpp/exceptions.hpp>

#include <memory>
#include <string>

using ServiceException = component_interface_utils::ServiceException;
using Initialize = localization_interface::Initialize;

NdtLocalizationTriggerModule::NdtLocalizationTriggerModule(rclcpp::Node * node)
: logger_(node->get_logger())
{
  client_ndt_trigger_ = node->create_client<SetBool>("ndt_trigger_node");
}

// 激活/挂起 ndt 点云匹配算法 ，由传入参数 flag 为 True/false 决定
void NdtLocalizationTriggerModule::send_request(bool flag) const
{
  // client请求数据变量
  const auto req = std::make_shared<SetBool::Request>();
  std::string command_name;
  // 请求数据变量data数据
  req->data = flag;
  if (flag) {
    command_name = "Activation";
  } else {
    command_name = "Deactivation";
  }
  // 检测ndt服务端是否在线
  if (!client_ndt_trigger_->service_is_ready()) {
    throw component_interface_utils::ServiceUnready("NDT triggering service is not ready");
  }
  // 发起请求并等待结果，然后日志打印
  auto future_ndt = client_ndt_trigger_->async_send_request(req);

  if (future_ndt.get()->success) {
    RCLCPP_INFO(logger_, "NDT %s succeeded", command_name.c_str());
  } else {
    RCLCPP_INFO(logger_, "NDT %s failed", command_name.c_str());
    throw ServiceException(
      Initialize::Service::Response::ERROR_ESTIMATION, "NDT " + command_name + " failed");
  }
}
