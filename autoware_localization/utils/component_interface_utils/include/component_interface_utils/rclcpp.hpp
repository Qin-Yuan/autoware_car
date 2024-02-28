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

#ifndef COMPONENT_INTERFACE_UTILS__RCLCPP_HPP_
#define COMPONENT_INTERFACE_UTILS__RCLCPP_HPP_

#include <component_interface_utils/rclcpp/create_interface.hpp>
#include <component_interface_utils/rclcpp/interface.hpp>
#include <component_interface_utils/rclcpp/service_client.hpp>
#include <component_interface_utils/rclcpp/service_server.hpp>
#include <component_interface_utils/rclcpp/topic_publisher.hpp>
#include <component_interface_utils/rclcpp/topic_subscription.hpp>

#include <memory>
#include <optional>
#include <utility>

namespace component_interface_utils
{

class NodeAdaptor
{
private:
  using CallbackGroup = rclcpp::CallbackGroup::SharedPtr;

  template <class SharedPtrT, class InstanceT>
  using MessageCallback =
    void (InstanceT::*)(const typename SharedPtrT::element_type::SpecType::Message::ConstSharedPtr);

  template <class SharedPtrT, class InstanceT>
  using ServiceCallback = void (InstanceT::*)(
    const typename SharedPtrT::element_type::SpecType::Service::Request::SharedPtr,
    const typename SharedPtrT::element_type::SpecType::Service::Response::SharedPtr);

public:
  /// 构造函数.
  explicit NodeAdaptor(rclcpp::Node * node) { interface_ = std::make_shared<NodeInterface>(node); }

  /// 创建用于日志记录的客户机包装器
  template <class SharedPtrT>
  void init_cli(SharedPtrT & cli, CallbackGroup group = nullptr) const
  {
    using SpecT = typename SharedPtrT::element_type::SpecType;
    cli = create_client_impl<SpecT>(interface_, group);
  }

  /// 创建用于日志记录的服务包装器
  template <class SharedPtrT, class CallbackT>
  void init_srv(SharedPtrT & srv, CallbackT && callback, CallbackGroup group = nullptr) const
  {
    using SpecT = typename SharedPtrT::element_type::SpecType;
    srv = create_service_impl<SpecT>(interface_, std::forward<CallbackT>(callback), group);
  }

  /// 使用服务等特征创建发布者
  template <class SharedPtrT>
  void init_pub(SharedPtrT & pub) const
  {
    using SpecT = typename SharedPtrT::element_type::SpecType;
    pub = create_publisher_impl<SpecT>(interface_->node);
  }

  /// 使用服务等特征创建订阅
  template <class SharedPtrT, class CallbackT>
  void init_sub(SharedPtrT & sub, CallbackT && callback) const
  {
    using SpecT = typename SharedPtrT::element_type::SpecType;
    sub = create_subscription_impl<SpecT>(interface_->node, std::forward<CallbackT>(callback));
  }

  /// 传递消息
  template <class P, class S>
  void relay_message(P & pub, S & sub) const
  {
    using MsgT = typename P::element_type::SpecType::Message::ConstSharedPtr;
    init_pub(pub);
    init_sub(sub, [pub](MsgT msg) { pub->publish(*msg); });
  }

  /// 中继服务
  template <class C, class S>
  void relay_service(
    C & cli, S & srv, CallbackGroup group, std::optional<double> timeout = std::nullopt) const
  {
    init_cli(cli);
    init_srv(
      srv, [cli, timeout](auto req, auto res) { *res = *cli->call(req, timeout); }, group);
  }

  /// 创建订阅包装器
  template <class SharedPtrT, class InstanceT>
  void init_sub(
    SharedPtrT & sub, InstanceT * instance,
    MessageCallback<SharedPtrT, InstanceT> && callback) const
  {
    using std::placeholders::_1;
    init_sub(sub, std::bind(callback, instance, _1));
  }

  /// 创建用于日志记录的服务包装器
  template <class SharedPtrT, class InstanceT>
  void init_srv(
    SharedPtrT & srv, InstanceT * instance, ServiceCallback<SharedPtrT, InstanceT> && callback,
    CallbackGroup group = nullptr) const
  {
    using std::placeholders::_1;
    using std::placeholders::_2;
    init_srv(srv, std::bind(callback, instance, _1, _2), group);
  }

private:
  // 请使用节点指针，因为shared_from_this不能在构造函数中使用
  NodeInterface::SharedPtr interface_;
};

}  // namespace component_interface_utils

#endif  // COMPONENT_INTERFACE_UTILS__RCLCPP_HPP_
