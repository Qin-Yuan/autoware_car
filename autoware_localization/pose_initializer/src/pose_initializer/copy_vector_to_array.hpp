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

#ifndef POSE_INITIALIZER__COPY_VECTOR_TO_ARRAY_HPP_
#define POSE_INITIALIZER__COPY_VECTOR_TO_ARRAY_HPP_

#include <fmt/core.h>

#include <algorithm>
#include <array>
#include <string>
#include <vector>

template <typename T, size_t N>
// 传入 vector 和 array 两个变量 ，函数将vector复制给array
void copy_vector_to_array(const std::vector<T> & vector, std::array<T, N> & array)
{
  // vector 的计算大小和 传入 N大小不一致，抛出异常
  if (N != vector.size()) {
    // 抛出错误以防止引起匿名错误
    // 例如，只初始化部分数组
    const auto v = std::to_string(vector.size());
    const auto n = std::to_string(N);
    throw std::invalid_argument(
      "Vector size (which is " + v + ") is different from the copy size (which is " + n + ")");
  }
  // 从 vector.begin() ，到第N个元素，复制到以 array.begin() 开始的数组
  std::copy_n(vector.begin(), N, array.begin());
}

// 函数模板，返回一个double型的36长度数组 ，传入的class NodeT不固定
template <class NodeT>
std::array<double, 36> get_covariance_parameter(NodeT * node, const std::string & name)
{
  // 定义一个类型为 std::vector<double> ，名称为 name 的 ROS2 参数 ，但会的 parameter 为地址
  const auto parameter = node->template declare_parameter<std::vector<double>>(name);
  // 创建一个数组
  std::array<double, 36> covariance;
  // 将 parameter 参数传给 数组 covariance , 然后返回 数组
  copy_vector_to_array(parameter, covariance);
  return covariance;
}

#endif  // POSE_INITIALIZER__COPY_VECTOR_TO_ARRAY_HPP_
