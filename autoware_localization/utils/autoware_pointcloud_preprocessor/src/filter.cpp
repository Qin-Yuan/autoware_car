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
 * $Id: filter.cpp 35876 2011-02-09 01:04:36Z rusu $
 *
 */

#include "autoware_pointcloud_preprocessor/filter.hpp"

#include <pcl_ros/transforms.hpp>

#include <pcl/io/io.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// autoware_pointcloud_preprocessor 程序入口 ,在继承Filter类时就会依次执行下面的函数
autoware_pointcloud_preprocessor::Filter::Filter(
  const std::string & filter_name, const rclcpp::NodeOptions & options)
: Node(filter_name, options), filter_field_name_(filter_name)
{
  // Set parameters (moved from NodeletLazy onInit)
  {
    tf_input_frame_ = static_cast<std::string>(declare_parameter("input_frame", ""));     // 输入的 frame_id
    tf_output_frame_ = static_cast<std::string>(declare_parameter("output_frame", ""));   // 输出的 frame_id
    max_queue_size_ = static_cast<std::size_t>(declare_parameter("max_queue_size", 5));   // 订阅、发布 qos 队列大小

    // ---[ Optional parameters
    use_indices_ = static_cast<bool>(declare_parameter("use_indices", false));
    latched_indices_ = static_cast<bool>(declare_parameter("latched_indices", false));
    approximate_sync_ = static_cast<bool>(declare_parameter("approximate_sync", false));

    RCLCPP_INFO_STREAM(
      this->get_logger(),
      "Filter (as Component) successfully created with the following parameters:"
        << std::endl
        << " - approximate_sync : " << (approximate_sync_ ? "true" : "false") << std::endl
        << " - use_indices      : " << (use_indices_ ? "true" : "false") << std::endl
        << " - latched_indices  : " << (latched_indices_ ? "true" : "false") << std::endl
        << " - max_queue_size   : " << max_queue_size_);
  }

  // Set publisher
  // 滤波后点云的发布者
  {
    pub_output_ = this->create_publisher<PointCloud2>(
      "output", rclcpp::SensorDataQoS().keep_last(max_queue_size_));
  }
  // 跳转初始化点云订阅 ， filter_name 用于选择滤波函数 , 调用含有参数传递的 subscribe 函数
  subscribe(filter_name);

  // 设置tf_listener, tf_buffer
  setupTF();

  // 参数回调函数，如果有更改参数就会触发回调
  set_param_res_filter_ = this->add_on_set_parameters_callback(
    std::bind(&Filter::filterParamCallback, this, std::placeholders::_1));

  RCLCPP_DEBUG(this->get_logger(), "[Filter Constructor] successfully created.");
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void autoware_pointcloud_preprocessor::Filter::setupTF()
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// subscribe() 函数是一个复用的，区别在于有没有传递参数
// 其实没有传递参数最后也会进入带有参数的 subscribe() 函数中，使用的默认 filter 方法
void autoware_pointcloud_preprocessor::Filter::subscribe()
{
  std::string filter_name = "";
  subscribe(filter_name);
}

// 调用此函数初始化话题订阅
void autoware_pointcloud_preprocessor::Filter::subscribe(const std::string & filter_name)
{
  // TODO(sykwer): Change the corresponding node to subscribe to `faster_input_indices_callback`
  // each time a child class supports the faster version.
  // When all the child classes support the faster version, this workaround is deleted.
  // TODO(sykwer):将对应的节点更改为订阅' faster_input_indices_callback '
  // 每次子类支持更快的版本。
  // 当所有子类都支持更快的版本时，此解决方法将被删除。
  // 当 filter_name 等于 “CropBoxFilter” 滤波器时选择 faster_input_indices_callback 作为回调函数，
  // 否则选择 input_indices_callback 作为回调函数
  auto callback = filter_name == "CropBoxFilter" ? &Filter::faster_input_indices_callback
                                                : &Filter::input_indices_callback;
  // 判断是否用了 indices 索引; 默认 为 false
  if (use_indices_) {
    // 使用过滤器订阅输入
    sub_input_filter_.subscribe(
      this, "input", rclcpp::SensorDataQoS().keep_last(max_queue_size_).get_rmw_qos_profile());
    sub_indices_filter_.subscribe(
      this, "indices", rclcpp::SensorDataQoS().keep_last(max_queue_size_).get_rmw_qos_profile());

    if (approximate_sync_) {
      sync_input_indices_a_ = std::make_shared<ApproximateTimeSyncPolicy>(max_queue_size_);
      sync_input_indices_a_->connectInput(sub_input_filter_, sub_indices_filter_);
      sync_input_indices_a_->registerCallback(
        std::bind(callback, this, std::placeholders::_1, std::placeholders::_2));
    } else {
      sync_input_indices_e_ = std::make_shared<ExactTimeSyncPolicy>(max_queue_size_);
      sync_input_indices_e_->connectInput(sub_input_filter_, sub_indices_filter_);
      sync_input_indices_e_->registerCallback(
        std::bind(callback, this, std::placeholders::_1, std::placeholders::_2));
    }
    // 
  } else {
    // 以老式方式订阅仅输入(没有过滤器)
    // 这里不能使用自动输入。
    std::function<void(const PointCloud2ConstPtr msg)> cb =
      std::bind(callback, this, std::placeholders::_1, PointIndicesConstPtr());
    // 创建 点云 订阅者 ，上面 cb 作为回调函数 ， 即 Filter::input_indices_callback
    sub_input_ = create_subscription<PointCloud2>(
      "input", rclcpp::SensorDataQoS().keep_last(max_queue_size_), cb);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 没用到？
void autoware_pointcloud_preprocessor::Filter::unsubscribe()
{
  if (use_indices_) {
    sub_input_filter_.unsubscribe();
    sub_indices_filter_.unsubscribe();
    if (approximate_sync_) {
      sync_input_indices_a_.reset();
    } else {
      sync_input_indices_e_.reset();
    }
  } else {
    sub_input_.reset();
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// TODO(sykwer): Temporary Implementation: Delete this function definition when all the filter nodes
// conform to new API.
void autoware_pointcloud_preprocessor::Filter::computePublish(
  const PointCloud2ConstPtr & input, const IndicesPtr & indices)
{
  // 用来存储处理后的点云数据
  auto output = std::make_unique<PointCloud2>();

  // 在子进程中调用虚方法，这里只声明了filter虚函数，在子类继承该class后重写函数实现具体的功能即可:
  // 例如 random_downsample_filter_nodelet 中的点云降采样，重写 filter 函数实现
  filter(input, indices, *output);
  // 对点云 output 的 frame_id 进行校验和必要变换
  if (!convert_output_costly(output)) return;

  // 复制输入点云的时间戳，并在 output 中使用它
  output->header.stamp = input->header.stamp;

  // 发布处理后的点云数据
  pub_output_->publish(std::move(output));
}

//////////////////////////////////////////////////////////////////////////////////////////////
// 参数修改触发回调函数
rcl_interfaces::msg::SetParametersResult autoware_pointcloud_preprocessor::Filter::filterParamCallback(
  const std::vector<rclcpp::Parameter> & p)
{
  // 调用线程将锁住该互斥量
  // scoped_lock特别之处是，可以指定多个互斥量，同时锁定多个互斥量而不死锁
  std::scoped_lock lock(mutex_);
  // 修改 点云 相关的 frame_id 参数
  if (get_param(p, "input_frame", tf_input_frame_)) {
    RCLCPP_DEBUG(get_logger(), "Setting the input TF frame to: %s.", tf_input_frame_.c_str());
  }
  if (get_param(p, "output_frame", tf_output_frame_)) {
    RCLCPP_DEBUG(get_logger(), "Setting the output TF frame to: %s.", tf_output_frame_.c_str());
  }
  // 参数请求返回结果
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

//////////////////////////////////////////////////////////////////////////////////////////////
// TODO(sykwer): Temporary Implementation:删除这个函数定义时，所有的过滤节点
// 遵从新的API, cloud 为回调函数输入的原始点云数据
void autoware_pointcloud_preprocessor::Filter::input_indices_callback(
  const PointCloud2ConstPtr cloud, const PointIndicesConstPtr indices)
{
  // 检查点云是否有效 ，主要是判断data大小，是否有丢失，正确返回True
  // cloud->width * cloud->height * cloud->point_step != cloud->data.size() 
  if (!isValid(cloud)) {
    RCLCPP_ERROR(this->get_logger(), "[input_indices_callback] Invalid input!");
    return;
  }
  // If indices are given, check if they are valid
  if (indices && !isValid(indices)) {
    RCLCPP_ERROR(this->get_logger(), "[input_indices_callback] Invalid indices!");
    return;
  }

  /// DEBUG
  if (indices) {
    RCLCPP_DEBUG(
      this->get_logger(),
      "[input_indices_callback]\n"
      "   - PointCloud with %d data points (%s), stamp %f, and frame %s on input topic received.\n"
      "   - PointIndices with %zu values, stamp %f, and frame %s on indices topic received.",
      cloud->width * cloud->height, pcl::getFieldsList(*cloud).c_str(),
      rclcpp::Time(cloud->header.stamp).seconds(), cloud->header.frame_id.c_str(),
      indices->indices.size(), rclcpp::Time(indices->header.stamp).seconds(),
      indices->header.frame_id.c_str());
  } 
  // 只关注点云是否有效时
  else {
    RCLCPP_DEBUG(
      this->get_logger(),
      "[input_indices_callback] PointCloud with %d data points and frame %s on input topic "
      "received.",
      cloud->width * cloud->height, cloud->header.frame_id.c_str());
  }
  ///

  // 检查用户是否给出了不同的输入TF帧
  // 点云的原始frame_id
  tf_input_orig_frame_ = cloud->header.frame_id;
  PointCloud2ConstPtr cloud_tf;
  // 检测是否用户输入了 tf_input_frame ，并且 点云 output 和 tf_output_frame_ 不一致 ，进行点云转换 
  if (!tf_input_frame_.empty() && cloud->header.frame_id != tf_input_frame_) {
    RCLCPP_DEBUG(
      this->get_logger(), "[input_indices_callback] Transforming input dataset from %s to %s.",
      cloud->header.frame_id.c_str(), tf_input_frame_.c_str());
    // 保存原始帧ID
    // 将点云转换为不同的框架
    PointCloud2 cloud_transformed;
    // 检查点云的 frame_id 和 用户输入的 tf_input_frame_ 是否变换超时，这里timeout设置为 1s 
    // 超时直接返回，不做后续的处理
    if (!tf_buffer_->canTransform(
          tf_input_frame_, cloud->header.frame_id, this->now(),
          rclcpp::Duration::from_seconds(1.0))) {
      RCLCPP_ERROR_STREAM(
        this->get_logger(), "[input_indices_callback] timeout tf: " << cloud->header.frame_id
                                                                    << "->" << tf_input_frame_);
      return;
    }
    // transformPointCloud 函数的作用是通过转换矩阵transform将source_cloud转换后存到target_cloud中保存
    // tf_input_frame_ 输入点云的frame , cloud 原始点云，cloud_transformed 目标点云中存储 ，tf_buffer_ 转换矩阵
    if (!pcl_ros::transformPointCloud(tf_input_frame_, *cloud, cloud_transformed, *tf_buffer_)) {
      // 如果点云转换失败，打印 日志 并返回退出
      RCLCPP_ERROR(
        this->get_logger(),
        "[input_indices_callback] Error converting input dataset from %s to %s.",
        cloud->header.frame_id.c_str(), tf_input_frame_.c_str());
      return;
    }
    // 转换后的点云
    cloud_tf = std::make_shared<PointCloud2>(cloud_transformed);
  } 
  // 否则 没有输出 tf_input_frame ，直接赋值给 cloud_tf
  else {
    cloud_tf = cloud;
  }
  // 这里需要setInputCloud()，因为我们必须提取x/y/z
  IndicesPtr vindices;
  // 是否利用pcl索引进行点云提取
  if (indices) {
    vindices.reset(new std::vector<int>(indices->indices));
  }
  // 前面都是对输入点云做校验和必要变换准备， computePublish 才是对点云进行滤波得到output点云，然后发布出来 ：
  computePublish(cloud_tf, vindices);
}

// For performance reason, we get only a transformation matrix here.
// The implementation is based on the one shown in the URL below.
// https://github.com/ros-perception/perception_pcl/blob/628aaec1dc73ef4adea01e9d28f11eb417b948fd/pcl_ros/src/transforms.cpp#L61-L94
bool autoware_pointcloud_preprocessor::Filter::_calculate_transform_matrix(
  const std::string & target_frame, const sensor_msgs::msg::PointCloud2 & from,
  const tf2_ros::Buffer & tf_buffer, Eigen::Matrix4f & eigen_transform /*output*/)
{
  if (from.header.frame_id == target_frame) {
    eigen_transform = Eigen::Matrix4f::Identity(4, 4);
    return true;
  }

  geometry_msgs::msg::TransformStamped transform;

  try {
    transform = tf_buffer.lookupTransform(
      target_frame, from.header.frame_id, tf2_ros::fromMsg(from.header.stamp));
  } catch (tf2::LookupException & e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    return false;
  } catch (tf2::ExtrapolationException & e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    return false;
  }

  pcl_ros::transformAsMatrix(transform, eigen_transform);
  return true;
}

// Returns false in error cases
bool autoware_pointcloud_preprocessor::Filter::calculate_transform_matrix(
  const std::string & target_frame, const sensor_msgs::msg::PointCloud2 & from,
  TransformInfo & transform_info /*output*/)
{
  transform_info.need_transform = false;

  if (target_frame.empty() || from.header.frame_id == target_frame) return true;

  RCLCPP_DEBUG(
    this->get_logger(), "[get_transform_matrix] Transforming input dataset from %s to %s.",
    from.header.frame_id.c_str(), target_frame.c_str());

  if (!tf_buffer_->canTransform(
        target_frame, from.header.frame_id, this->now(), rclcpp::Duration::from_seconds(1.0))) {
    RCLCPP_ERROR_STREAM(
      this->get_logger(),
      "[get_transform_matrix] timeout tf: " << from.header.frame_id << "->" << target_frame);
    return false;
  }

  if (!_calculate_transform_matrix(
        target_frame, from, *tf_buffer_, transform_info.eigen_transform /*output*/)) {
    RCLCPP_ERROR(
      this->get_logger(),
      "[calculate_transform_matrix] Error converting input dataset from %s to %s.",
      from.header.frame_id.c_str(), target_frame.c_str());
    return false;
  }

  transform_info.need_transform = true;
  return true;
}

// Returns false in error cases
bool autoware_pointcloud_preprocessor::Filter::convert_output_costly(std::unique_ptr<PointCloud2> & output)
{
  // 在性能方面，我们应该避免使用pcl_ros库函数，
  // 但是在Autoware的主要用例中没有达到这个代码路径，所以现在就保留它。
  // 如果输入了 tf_output_frame_ 并且 点云 output 和 tf_output_frame_ 不一致 ，进行点云转换 
  if (!tf_output_frame_.empty() && output->header.frame_id != tf_output_frame_) {
    // 日志打印 ： 将 output frame_id 点云转到 tf_output_frame_
    RCLCPP_DEBUG(
      this->get_logger(), "[convert_output_costly] Transforming output dataset from %s to %s.",
      output->header.frame_id.c_str(), tf_output_frame_.c_str());

    // 将云转换为不同的框架
    auto cloud_transformed = std::make_unique<PointCloud2>();
    // 使用 pcl_ros 库转换 点云
    if (!pcl_ros::transformPointCloud(tf_output_frame_, *output, *cloud_transformed, *tf_buffer_)) {
      // 转换失败
      RCLCPP_ERROR(
        this->get_logger(),
        "[convert_output_costly] Error converting output dataset from %s to %s.",
        output->header.frame_id.c_str(), tf_output_frame_.c_str());
      return false;
    }
    // 将对象的状态或者所有权从一个对象转移到另一个对象，只是转移，没有内存的搬迁或者内存拷贝
    // 将转换的点云 cloud_transformed 赋给 output
    output = std::move(cloud_transformed);
  }

  // 没有指定 tf_output_frame_ 
  if (tf_output_frame_.empty() && output->header.frame_id != tf_input_orig_frame_) {
    // 没有指定tf_output_frame，将数据集转换为其原始帧
    RCLCPP_DEBUG(
      this->get_logger(), "[convert_output_costly] Transforming output dataset from %s back to %s.",
      output->header.frame_id.c_str(), tf_input_orig_frame_.c_str());
    // 将 output 点云转换到 原始点云frame_id 
    auto cloud_transformed = std::make_unique<PointCloud2>();
    if (!pcl_ros::transformPointCloud(
          tf_input_orig_frame_, *output, *cloud_transformed, *tf_buffer_)) {
      // 转换失败
      RCLCPP_ERROR(
        this->get_logger(),
        "[convert_output_costly] Error converting output dataset from %s back to %s.",
        output->header.frame_id.c_str(), tf_input_orig_frame_.c_str());
      return false;
    }
    // 同上
    output = std::move(cloud_transformed);
  }

  return true;
}

// TODO(sykwer): Temporary Implementation: Rename this function to `input_indices_callback()` when
// all the filter nodes conform to new API. Then delete the old `input_indices_callback()` defined
// above.
// TODO(sykwer): Temporary Implementation:将此函数重命名为input_indices_callback()
// 所有的过滤器节点都符合新的API。然后删除旧的' input_indices_callback() '定义
// 以上。
void autoware_pointcloud_preprocessor::Filter::faster_input_indices_callback(
  const PointCloud2ConstPtr cloud, const PointIndicesConstPtr indices)
{
  if (!isValid(cloud)) {
    RCLCPP_ERROR(this->get_logger(), "[input_indices_callback] Invalid input!");
    return;
  }

  if (indices && !isValid(indices)) {
    RCLCPP_ERROR(this->get_logger(), "[input_indices_callback] Invalid indices!");
    return;
  }

  if (indices) {
    RCLCPP_DEBUG(
      this->get_logger(),
      "[input_indices_callback]\n"
      "   - PointCloud with %d data points (%s), stamp %f, and frame %s on input topic received.\n"
      "   - PointIndices with %zu values, stamp %f, and frame %s on indices topic received.",
      cloud->width * cloud->height, pcl::getFieldsList(*cloud).c_str(),
      rclcpp::Time(cloud->header.stamp).seconds(), cloud->header.frame_id.c_str(),
      indices->indices.size(), rclcpp::Time(indices->header.stamp).seconds(),
      indices->header.frame_id.c_str());
  } else {
    RCLCPP_DEBUG(
      this->get_logger(),
      "[input_indices_callback] PointCloud with %d data points and frame %s on input topic "
      "received.",
      cloud->width * cloud->height, cloud->header.frame_id.c_str());
  }

  tf_input_orig_frame_ = cloud->header.frame_id;

  // For performance reason, defer the transform computation.
  // Do not use pcl_ros::transformPointCloud(). It's too slow due to the unnecessary copy.
  TransformInfo transform_info;
  if (!calculate_transform_matrix(tf_input_frame_, *cloud, transform_info)) return;

  // Need setInputCloud() here because we have to extract x/y/z
  IndicesPtr vindices;
  if (indices) {
    vindices.reset(new std::vector<int>(indices->indices));
  }

  auto output = std::make_unique<PointCloud2>();

  // TODO(sykwer): Change to `filter()` call after when the filter nodes conform to new API.
  faster_filter(cloud, vindices, *output, transform_info);

  if (!convert_output_costly(output)) return;

  output->header.stamp = cloud->header.stamp;
  pub_output_->publish(std::move(output));
}

// TODO(sykwer): Temporary Implementation: Remove this interface when all the filter nodes conform
// to new API. It's not a pure virtual function so that a child class does not have to implement
// this function.
void autoware_pointcloud_preprocessor::Filter::faster_filter(
  const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output,
  const TransformInfo & transform_info)
{
  (void)input;
  (void)indices;
  (void)output;
  (void)transform_info;
}
