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

#include "stop_check_module.hpp"

StopCheckModule::StopCheckModule(rclcpp::Node * node, double buffer_duration)
: VehicleStopCheckerBase(node, buffer_duration)
{
  // twist停止检查订阅函数 ， stop_check_twist 可以remap重映射改变订阅话题名称
  sub_twist_ = node->create_subscription<TwistWithCovarianceStamped>(
    "stop_check_twist", 1, std::bind(&StopCheckModule::on_twist, this, std::placeholders::_1));
}
// 回调处理函数
void StopCheckModule::on_twist(TwistWithCovarianceStamped::ConstSharedPtr msg)
{
  // addTwist buffer 队列中的成员类型为geometry_msgs::msg::TwistStamped;
  // 因此创建一个该类型的变量用于存储stop_check_twist
  TwistStamped twist;
  twist.header = msg->header;
  twist.twist = msg->twist.twist;
  // 将stop_check_twist收到的控制命令存储到buffer中，同时pop出不符合buffer_duration的历史twist数据
  addTwist(twist);
}
