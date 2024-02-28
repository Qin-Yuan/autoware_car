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

#include "motion_utils/vehicle/vehicle_state_checker.hpp"

#include "motion_utils/trajectory/trajectory.hpp"

#include <string>

namespace motion_utils
{
VehicleStopCheckerBase::VehicleStopCheckerBase(rclcpp::Node * node, double buffer_duration)
: clock_(node->get_clock()), logger_(node->get_logger())
{
  buffer_duration_ = buffer_duration;
}

// 将新的twist数据存储到队列中
void VehicleStopCheckerBase::addTwist(const TwistStamped & twist)
{
  // 从队列头部存储新数据
  twist_buffer_.push_front(twist);
  const auto now = clock_->now();
  // 将以前所有的twist命名从尾部开始，凡是不满足 buffer_duration 
  // 的 twist 数据全部pop出去
  while (true) {
    // 检查最旧数据时间
    const auto time_diff = now - twist_buffer_.back().header.stamp;

    // 满足 buffer_duration 时就跳出，因为后面的数据肯定比这个数据更新
    if (time_diff.seconds() <= buffer_duration_) {
      break;
    }
    // 移除旧的不满足buffer_duration的 twist 数据
    twist_buffer_.pop_back();
  }
}

bool VehicleStopCheckerBase::isVehicleStopped(const double stop_duration) const
{
  // twist_buffer_ 为空则直接返回false，无法判断当前车辆的是否停止
  if (twist_buffer_.empty()) {
    return false;
  }
  // 定义一个接近于 0 的停止速度值
  constexpr double squared_stop_velocity = 1e-3 * 1e-3;
  const auto now = clock_->now();
  // back() ：此函数用于引用双端队列容器的最后一个元素。此函数可用于从双端队列的后面获取第一个元素。
  // front() ：此函数用于引用双端队列容器的第一个元素。此函数可用于获取双端队列的第一个元素。
  // 检测最旧的一个 twist 命令与现在的时间是否间隔满足 stop_duration
  // 因为如果历史最旧的一个数据时间戳都不满足停止时间大于stop_duration，那后面肯定也不满足
  const auto time_buffer_back = now - twist_buffer_.back().header.stamp;
  // 间隔时间太短直接返回 false
  if (time_buffer_back.seconds() < stop_duration) {
    return false;
  }

  // 遍历 twist_buffer_ 缓存区内的速度
  for (const auto & velocity : twist_buffer_) {
    double x = velocity.twist.linear.x;
    double y = velocity.twist.linear.y;
    double z = velocity.twist.linear.z;
    // 简单计算xyz方向的速度平方之和
    double v = (x * x) + (y * y) + (z * z);
    // 如果任意数据计算出来的速度比 0 大，判定为未停止，返回false
    if (squared_stop_velocity <= v) {
      return false;
    }
    // 当满足 stop_duration 时间断跳出 for 循环，返回 true
    const auto time_diff = now - velocity.header.stamp;
    if (time_diff.seconds() >= stop_duration) {
      break;
    }
  }
  // 汽车处于停止状态
  return true;
}

VehicleStopChecker::VehicleStopChecker(rclcpp::Node * node)
: VehicleStopCheckerBase(node, velocity_buffer_time_sec)
{
  using std::placeholders::_1;
  // 订阅 odom 数据
  sub_odom_ = node->create_subscription<Odometry>(
    "/localization/kinematic_state", rclcpp::QoS(1),
    std::bind(&VehicleStopChecker::onOdom, this, _1));
}
// 该odom回调函数中，将 odom 中的速度控制部分添加到bufferz中
void VehicleStopChecker::onOdom(const Odometry::SharedPtr msg)
{
  odometry_ptr_ = msg;
  // 创建一个geometry_msgs::msg::TwistStamped变量，便于使用addTwist()函数添加到buffer中
  TwistStamped current_velocity;
  current_velocity.header = msg->header;
  current_velocity.twist = msg->twist.twist;
  addTwist(current_velocity);
}

VehicleArrivalChecker::VehicleArrivalChecker(rclcpp::Node * node) : VehicleStopChecker(node)
{
  using std::placeholders::_1;

  sub_trajectory_ = node->create_subscription<Trajectory>(
    "/planning/scenario_planning/trajectory", rclcpp::QoS(1),
    std::bind(&VehicleArrivalChecker::onTrajectory, this, _1));
}

bool VehicleArrivalChecker::isVehicleStoppedAtStopPoint(const double stop_duration) const
{
  if (!odometry_ptr_ || !trajectory_ptr_) {
    return false;
  }

  if (!isVehicleStopped(stop_duration)) {
    return false;
  }

  const auto & p = odometry_ptr_->pose.pose.position;
  const auto idx = motion_utils::searchZeroVelocityIndex(trajectory_ptr_->points);

  if (!idx) {
    return false;
  }

  return std::abs(motion_utils::calcSignedArcLength(trajectory_ptr_->points, p, idx.get())) <
        th_arrived_distance_m;
}

void VehicleArrivalChecker::onTrajectory(const Trajectory::SharedPtr msg) { trajectory_ptr_ = msg; }
}  // namespace motion_utils
