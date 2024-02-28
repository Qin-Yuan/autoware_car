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

#include "pose_initializer_core.hpp"

#include "copy_vector_to_array.hpp"
#include "ekf_localization_trigger_module.hpp"
#include "gnss_module.hpp"
#include "ndt_localization_trigger_module.hpp"
#include "ndt_module.hpp"
#include "stop_check_module.hpp"

#include <memory>
#include <vector>

PoseInitializer::PoseInitializer() : Node("pose_initializer")
{
  const auto node = component_interface_utils::NodeAdaptor(this);
  group_srv_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  node.init_pub(pub_state_);
  node.init_srv(srv_initialize_, this, &PoseInitializer::on_initialize, group_srv_);
  // 创建 pose_reset 发布者
  pub_reset_ = create_publisher<PoseWithCovarianceStamped>("pose_reset", 1);
  // 获取 协方差 covariance ，这里是做参数声明，在launch启动时会将config文件夹中的
  // pose_initializer.param.yaml协方差配置文件传入对应的名称中，返回获取返回协方差
  // 对应的为一个36长度的数组
  output_pose_covariance_ = get_covariance_parameter(this, "output_pose_covariance");
  gnss_particle_covariance_ = get_covariance_parameter(this, "gnss_particle_covariance");
  // 检测是否需要对 下面 几个智能指针分配内存
  if (declare_parameter<bool>("ekf_enabled")) {
    // make_unique 为智能指针分配内存
    ekf_localization_trigger_ = std::make_unique<EkfLocalizationTriggerModule>(this);
  }
  if (declare_parameter<bool>("gnss_enabled")) {
    gnss_ = std::make_unique<GnssModule>(this);
  }
  if (declare_parameter<bool>("ndt_enabled")) {
    ndt_ = std::make_unique<NdtModule>(this);
    ndt_localization_trigger_ = std::make_unique<NdtLocalizationTriggerModule>(this);
  }
  if (declare_parameter<bool>("stop_check_enabled")) {
    // 为扭转缓冲增加1.0秒的余量。
    stop_check_duration_ = declare_parameter<double>("stop_check_duration");
    stop_check_ = std::make_unique<StopCheckModule>(this, stop_check_duration_ + 1.0);
  }
  // 初始化状态 ： 未初始 UNINITIALIZED
  change_state(State::Message::UNINITIALIZED);
}

PoseInitializer::~PoseInitializer()
{
  // to delete gnss module
}
// 发布车辆定位状态
void PoseInitializer::change_state(State::Message::_state_type state)
{
  state_.stamp = now();
  state_.state = state;
  pub_state_->publish(state_);
}

void PoseInitializer::on_initialize(
  const Initialize::Service::Request::SharedPtr req,
  const Initialize::Service::Response::SharedPtr res)
{
  // 注意:该函数互斥，初始化时不执行
  // 判断车辆是否在stop_check_duration_时间断里面处于停止状态，否则判断为不安全
  if (stop_check_ && !stop_check_->isVehicleStopped(stop_check_duration_)) {
    // 抛出异常，不安全
    throw ServiceException(
      Initialize::Service::Response::ERROR_UNSAFE, "The vehicle is not stopped.");
  }
  // 检测
  try {
    // 改变当前为 正在定位初始化
    change_state(State::Message::INITIALIZING);
    // 判断是不是空指针，不是空指针在执行下面的
    if (ekf_localization_trigger_) {
      // 关闭 ekf_localization 节点 ， Deactivation
      ekf_localization_trigger_->send_request(false);
    }
    if (ndt_localization_trigger_) {
      // 关闭 ndt_localization 节点 ， Deactivation
      ndt_localization_trigger_->send_request(false);
    }
    // req->pose.empty() = true , pose = get_gnss_pose() ;
    // else : pose = req->pose.front()
    // 如果 client 端未发布pose信息，则获取 gnss pose ; 若发布了 pose 信息，则直接读取 req->pose
    auto pose = req->pose.empty() ? get_gnss_pose() : req->pose.front();
    if (ndt_) {
      // 根据 ndt request 大概位置信息请求ndt scan smatcher 匹配后返一个较为准确的位置信息
      pose = ndt_->align_pose(pose);
    }
    // ndt pose 计算的协方差由参数中的output_pose_covariance_协方差给定
    pose.pose.covariance = output_pose_covariance_;
    // 发布 pose_reset
    pub_reset_->publish(pose);
    // 激活 ekf_localization
    if (ekf_localization_trigger_) {
      ekf_localization_trigger_->send_request(true);
    }
    // 激活 ndt_localization
    if (ndt_localization_trigger_) {
      ndt_localization_trigger_->send_request(true);
    }
    res->status.success = true;
    change_state(State::Message::INITIALIZED);
  } catch (const ServiceException & error) {
    res->status = error.status();
    change_state(State::Message::UNINITIALIZED);
  }
}

// 获取 gnss 数据
geometry_msgs::msg::PoseWithCovarianceStamped PoseInitializer::get_gnss_pose()
{
  // 检测 gnss_ 是不是空指针
  if (gnss_) {
    // 获取 gnss 数据
    PoseWithCovarianceStamped pose = gnss_->get_pose();
    // gnss 协方差由参数传递
    pose.pose.covariance = gnss_particle_covariance_;
    return pose;
  }
  // 否则抛出异常
  throw ServiceException(
    Initialize::Service::Response::ERROR_GNSS_SUPPORT, "GNSS is not supported.");
}
