// Copyright 2021 Tier IV, Inc.
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

#ifndef AUTOWARE__BEHAVIOR_PATH_START_PLANNER_MODULE__UTIL_HPP_
#define AUTOWARE__BEHAVIOR_PATH_START_PLANNER_MODULE__UTIL_HPP_

#include "autoware/behavior_path_planner_common/data_manager.hpp"
#include "autoware/behavior_path_planner_common/utils/drivable_area_expansion/static_drivable_area.hpp"
#include "autoware/behavior_path_planner_common/utils/path_safety_checker/path_safety_checker_parameters.hpp"
#include "autoware/behavior_path_planner_common/utils/path_safety_checker/safety_check.hpp"
#include "autoware/behavior_path_start_planner_module/pull_out_path.hpp"

#include <autoware/route_handler/route_handler.hpp>

#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_perception_msgs/msg/predicted_path.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <lanelet2_core/Forward.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::behavior_path_planner::start_planner_utils
{
using autoware::behavior_path_planner::utils::path_safety_checker::EgoPredictedPathParams;
using autoware::route_handler::RouteHandler;
using autoware_internal_planning_msgs::msg::PathWithLaneId;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_perception_msgs::msg::PredictedPath;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Twist;

PathWithLaneId getBackwardPath(
  const RouteHandler & route_handler, const lanelet::ConstLanelets & target_lanes,
  const Pose & current_pose, const Pose & backed_pose, const double velocity);
lanelet::ConstLanelets getPullOutLanes(
  const std::shared_ptr<const PlannerData> & planner_data, const double backward_length);
std::optional<PathWithLaneId> extractCollisionCheckSection(
  const PullOutPath & path, const double collision_check_distance_from_end);

std::vector<double> calcCurvatureFromTrajectory(
  const autoware_planning_msgs::msg::Trajectory & trajectory);

std::vector<double> calcCurvatureFromPoints(const std::vector<geometry_msgs::msg::Point> & points);

Pose findTargetPoseAlongPath(
  const PathWithLaneId & centerline_path, const Pose & start_pose,
  const double longitudinal_distance);

RelativePoseInfo calculateRelativePoseInVehicleCoordinate(
  const Pose & start_pose, const Pose & target_pose);

/**
 * @brief Get lane_ids for a given pose
 * @param pose Target pose
 * @param road_lanes Target lane group for search
 * @param previous_lane_ids Previous point's lane_ids (for inheritance, optional)
 * @return Retrieved lane_ids
 */
std::vector<int64_t> getLaneIdsFromPose(
  const geometry_msgs::msg::Pose & pose, const lanelet::ConstLanelets & road_lanes,
  const std::vector<int64_t> & previous_lane_ids);

/**
 * @brief Set lane_ids to PathPointWithLaneId
 * @param point Target PathPointWithLaneId to set
 * @param road_lanes Target lane group for search
 * @param previous_lane_ids Previous point's lane_ids (for inheritance, optional)
 */
void setLaneIdsToPathPoint(
  PathPointWithLaneId & point, const lanelet::ConstLanelets & road_lanes,
  const std::vector<int64_t> & previous_lane_ids);

/**
 * @brief Print detailed information of PathWithLaneId
 * @param path PathWithLaneId to print
 * @param path_name Name of the path for identification
 */
void printPathWithLaneIdDetails(const PathWithLaneId & path, const std::string & path_name);

}  // namespace autoware::behavior_path_planner::start_planner_utils

#endif  // AUTOWARE__BEHAVIOR_PATH_START_PLANNER_MODULE__UTIL_HPP_
