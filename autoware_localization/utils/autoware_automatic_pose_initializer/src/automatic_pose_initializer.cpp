// Copyright 2022 TIER IV, Inc.
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

#include "automatic_pose_initializer.hpp"

#include <memory>

namespace autoware_automatic_pose_initializer
{

AutomaticPoseInitializer::AutomaticPoseInitializer() : Node("autoware_automatic_pose_initializer")
{
  RCLCPP_INFO(get_logger(), "autoware_automatic_pose_initializer running ...");
  const auto adaptor = component_interface_utils::NodeAdaptor(this);
  group_cli_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  adaptor.init_cli(cli_initialize_, group_cli_);
  adaptor.init_sub(sub_state_, [this](const State::Message::ConstSharedPtr msg) { 
    state_ = *msg; 
    RCLCPP_INFO(get_logger(), "The state of the location is %d",state_.state);
  });

  const auto period = rclcpp::Rate(1.0).period();
  timer_ = rclcpp::create_timer(this, get_clock(), period, [this]() { on_timer(); });

  state_.stamp = now();
  state_.state = State::Message::UNKNOWN;
}

void AutomaticPoseInitializer::on_timer()
{
  timer_->cancel();
  // RCLCPP_INFO(get_logger(), "The state of the location is UNKNOWN ...");
  if (state_.state == State::Message::UNINITIALIZED) {
    // RCLCPP_WARN(get_logger(), "**** the state of the location is UNINITIALIZED. Try to initialize the PoseInitializer ****");
    try {
      const auto req = std::make_shared<Initialize::Service::Request>();
      cli_initialize_->call(req);
    } catch (const component_interface_utils::ServiceException & error) {
      RCLCPP_ERROR(get_logger(), "PoseInitializer error !");
    }
  }
  timer_->reset();
}

}  // namespace autoware_automatic_pose_initializer

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<autoware_automatic_pose_initializer::AutomaticPoseInitializer>();
  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);
  rclcpp::shutdown();
}
