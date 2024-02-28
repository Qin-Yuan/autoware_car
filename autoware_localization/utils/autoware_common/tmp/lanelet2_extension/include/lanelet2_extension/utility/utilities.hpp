// Copyright 2015-2023 Autoware Foundation. All rights reserved.
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
//
// Authors: Kenji Miyake, Ryohsuke Mitsudome

#ifndef LANELET2_EXTENSION__UTILITY__UTILITIES_HPP_
#define LANELET2_EXTENSION__UTILITY__UTILITIES_HPP_

// NOLINTBEGIN(readability-identifier-naming)

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <map>

namespace lanelet::utils
{
lanelet::LineString3d generateFineCenterline(
  const lanelet::ConstLanelet & lanelet_obj, const double resolution = 5.0);
lanelet::ConstLineString3d getCenterlineWithOffset(
  const lanelet::ConstLanelet & lanelet_obj, const double offset, const double resolution = 5.0);
lanelet::ConstLineString3d getRightBoundWithOffset(
  const lanelet::ConstLanelet & lanelet_obj, const double offset, const double resolution = 5.0);
lanelet::ConstLineString3d getLeftBoundWithOffset(
  const lanelet::ConstLanelet & lanelet_obj, const double offset, const double resolution = 5.0);

lanelet::ConstLanelet getExpandedLanelet(
  const lanelet::ConstLanelet & lanelet_obj, const double left_offset, const double right_offset);

lanelet::ConstLanelets getExpandedLanelets(
  const lanelet::ConstLanelets & lanelet_obj, const double left_offset, const double right_offset);

/// @brief copy the z values between 2 containers based on the 2D arc lengths
/// @tparam T1 a container of 3D points
/// @tparam T2 a container of 3D points
/// @param from points from which the z values will be copied
/// @param to points to which the z values will be copied
template <typename T1, typename T2>
void copyZ(const T1 & from, T2 & to)
{
  if (from.empty() || to.empty()) return;
  to.front().z() = from.front().z();
  if (from.size() < 2 || to.size() < 2) return;
  to.back().z() = from.back().z();
  auto i_from = 1lu;
  auto s_from = lanelet::geometry::distance2d(from[0], from[1]);
  auto s_to = 0.0;
  auto s_from_prev = 0.0;
  for (auto i_to = 1lu; i_to + 1 < to.size(); ++i_to) {
    s_to += lanelet::geometry::distance2d(to[i_to - 1], to[i_to]);
    for (; s_from < s_to && i_from + 1 < from.size(); ++i_from) {
      s_from_prev = s_from;
      s_from += lanelet::geometry::distance2d(from[i_from], from[i_from + 1]);
    }
    const auto ratio = (s_to - s_from_prev) / (s_from - s_from_prev);
    to[i_to].z() = from[i_from - 1].z() + ratio * (from[i_from].z() - from[i_from - 1].z());
  }
}

/**
 * @brief  Apply a patch for centerline because the original implementation
 * doesn't have enough quality
 */
void overwriteLaneletsCenterline(
  lanelet::LaneletMapPtr lanelet_map, const double resolution = 5.0,
  const bool force_overwrite = false);

lanelet::ConstLanelets getConflictingLanelets(
  const lanelet::routing::RoutingGraphConstPtr & graph, const lanelet::ConstLanelet & lanelet);

bool lineStringWithWidthToPolygon(
  const lanelet::ConstLineString3d & linestring, lanelet::ConstPolygon3d * polygon);

bool lineStringToPolygon(
  const lanelet::ConstLineString3d & linestring, lanelet::ConstPolygon3d * polygon);

double getLaneletLength2d(const lanelet::ConstLanelet & lanelet);
double getLaneletLength3d(const lanelet::ConstLanelet & lanelet);
double getLaneletLength2d(const lanelet::ConstLanelets & lanelet_sequence);
double getLaneletLength3d(const lanelet::ConstLanelets & lanelet_sequence);

lanelet::ArcCoordinates getArcCoordinates(
  const lanelet::ConstLanelets & lanelet_sequence, const geometry_msgs::msg::Pose & pose);

lanelet::ConstLineString3d getClosestSegment(
  const lanelet::BasicPoint2d & search_pt, const lanelet::ConstLineString3d & linestring);

lanelet::CompoundPolygon3d getPolygonFromArcLength(
  const lanelet::ConstLanelets & lanelets, const double s1, const double s2);
double getLaneletAngle(
  const lanelet::ConstLanelet & lanelet, const geometry_msgs::msg::Point & search_point);
bool isInLanelet(
  const geometry_msgs::msg::Pose & current_pose, const lanelet::ConstLanelet & lanelet,
  const double radius = 0.0);
geometry_msgs::msg::Pose getClosestCenterPose(
  const lanelet::ConstLanelet & lanelet, const geometry_msgs::msg::Point & search_point);

}  // namespace lanelet::utils

// NOLINTEND(readability-identifier-naming)

#endif  // LANELET2_EXTENSION__UTILITY__UTILITIES_HPP_
