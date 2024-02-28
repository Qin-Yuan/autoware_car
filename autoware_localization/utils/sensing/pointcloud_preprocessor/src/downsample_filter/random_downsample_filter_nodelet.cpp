// Copyright 2020 Tier IV, Inc.
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
/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "pointcloud_preprocessor/downsample_filter/random_downsample_filter_nodelet.hpp"

#include <vector>

namespace pointcloud_preprocessor
{
RandomDownsampleFilterComponent::RandomDownsampleFilterComponent(
  const rclcpp::NodeOptions & options)
: Filter("RandomDownsampleFilter", options)
{
  // 设置降采样的抽样指数 ， 默认 1500
  {
    sample_num_ = static_cast<size_t>(declare_parameter("sample_num", 1500));
  }

  using std::placeholders::_1;
  // 参数回调函数绑定，这里也是动态调整 降采样抽样指数参数 sample_num
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&RandomDownsampleFilterComponent::paramCallback, this, _1));
}

// 降采样的主要功能函数
void RandomDownsampleFilterComponent::filter(
  const PointCloud2ConstPtr & input, const IndicesPtr & /*indices*/, PointCloud2 & output)
{
  // 线程锁
  std::scoped_lock lock(mutex_);
  // 创建两个点云变量，可以看到这里只有XYZ属性
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_input(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_output(new pcl::PointCloud<pcl::PointXYZ>);
  // pcl_ros 库将 ros2 pointclouds 格式数据转化为 pcl 点云格式 ， input 赋值给 pcl_input
  pcl::fromROSMsg(*input, *pcl_input);
  // 设置 pcl_output 点云大小，根据 pcl_input 计算得到
  pcl_output->points.reserve(pcl_input->points.size());
  // pcl降采样类实例化 filter
  pcl::RandomSample<pcl::PointXYZ> filter;
  // 设置点云数据
  filter.setInputCloud(pcl_input);
  // filter.setSaveLeafLayout(true);
  // 设置抽样指数
  filter.setSample(sample_num_);
  // 执行降采样函数，并赋值给 
  filter.filter(*pcl_output);
  // 将其转为 ROS2 的话题信息
  pcl::toROSMsg(*pcl_output, output);
  // 保留原有的 header 信息
  output.header = input->header;
}

// 参数回调函数
rcl_interfaces::msg::SetParametersResult RandomDownsampleFilterComponent::paramCallback(
  const std::vector<rclcpp::Parameter> & p)
{
  std::scoped_lock lock(mutex_);
  // 获取最新的sample_num参数
  if (get_param(p, "sample_num", sample_num_)) {
    RCLCPP_DEBUG(get_logger(), "Setting new sample num to: %zu.", sample_num_);
  }
  // 参数服务返回结果
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

}  // namespace pointcloud_preprocessor
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_preprocessor::RandomDownsampleFilterComponent)
