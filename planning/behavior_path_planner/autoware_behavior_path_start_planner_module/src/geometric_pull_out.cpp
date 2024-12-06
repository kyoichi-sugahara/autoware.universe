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

#include "autoware/behavior_path_start_planner_module/geometric_pull_out.hpp"

#include "autoware/behavior_path_planner_common/utils/path_safety_checker/objects_filtering.hpp"
#include "autoware/behavior_path_planner_common/utils/path_utils.hpp"
#include "autoware/behavior_path_planner_common/utils/utils.hpp"
#include "autoware/behavior_path_start_planner_module/util.hpp"
#include "autoware/universe_utils/geometry/boost_polygon_utils.hpp"

#include <autoware_lanelet2_extension/utility/utilities.hpp>

#include <iostream>
#include <limits>
#include <memory>
#include <utility>

using autoware::motion_utils::findNearestIndex;
using autoware::universe_utils::calcDistance2d;
using autoware::universe_utils::calcOffsetPose;
using lanelet::utils::getArcCoordinates;
namespace autoware::behavior_path_planner
{
using start_planner_utils::getPullOutLanes;

GeometricPullOut::GeometricPullOut(
  rclcpp::Node & node, const StartPlannerParameters & parameters,
  const std::shared_ptr<autoware::lane_departure_checker::LaneDepartureChecker>
    lane_departure_checker,
  std::shared_ptr<universe_utils::TimeKeeper> time_keeper)
: PullOutPlannerBase{node, parameters, time_keeper},
  parallel_parking_parameters_{parameters.parallel_parking_parameters},
  lane_departure_checker_(lane_departure_checker)
{
  planner_.setParameters(parallel_parking_parameters_);
}

std::optional<PullOutPath> GeometricPullOut::plan(
  const Pose & start_pose, const Pose & goal_pose, PlannerDebugData & planner_debug_data)
{
  std::cerr << "[GeometricPullOut::plan] Start planning..." << std::endl;
  PullOutPath output;

  // combine road lane and pull out lane
  const double backward_path_length =
    planner_data_->parameters.backward_path_length + parameters_.max_back_distance;
  const auto road_lanes = utils::getExtendedCurrentLanes(
    planner_data_, backward_path_length, std::numeric_limits<double>::max(),
    /*forward_only_in_route*/ true);
  const auto pull_out_lanes = getPullOutLanes(planner_data_, backward_path_length);

  if (road_lanes.empty()) {
    std::cerr << "[GeometricPullOut::plan] Failed: road_lanes is empty" << std::endl;
    return {};
  }

  if (pull_out_lanes.empty()) {
    std::cerr << "[GeometricPullOut::plan] Failed: pull_out_lanes is empty" << std::endl;
    return {};
  }

  // check if the ego is at left or right side of road lane center
  const bool left_side_start = 0 < getArcCoordinates(road_lanes, start_pose).distance;
  std::cerr << "[GeometricPullOut::plan] Left side start: " << (left_side_start ? "true" : "false")
            << std::endl;

  planner_.setTurningRadius(
    planner_data_->parameters, parallel_parking_parameters_.pull_out_max_steer_angle);
  planner_.setPlannerData(planner_data_);
  const bool found_valid_path = planner_.planPullOut(
    start_pose, goal_pose, road_lanes, pull_out_lanes, left_side_start, lane_departure_checker_);

  if (!found_valid_path) {
    std::cerr << "[GeometricPullOut::plan] Failed: No valid path found in planPullOut" << std::endl;
    planner_debug_data.conditions_evaluation.emplace_back("no path found");
    return {};
  }
  std::cerr << "[GeometricPullOut::plan] Valid path found" << std::endl;

  // collision check with stop objects in pull out lanes
  const auto arc_path = planner_.getArcPath();
  const double velocity = parallel_parking_parameters_.forward_parking_velocity;

  if (parameters_.divide_pull_out_path) {
    std::cerr << "[GeometricPullOut::plan] Processing divided pull-out path" << std::endl;
    output.partial_paths = planner_.getPaths();

    if (output.partial_paths.empty() || output.partial_paths.front().points.empty()) {
      std::cerr << "[GeometricPullOut::plan] Failed: Empty partial paths or points" << std::endl;
      return {};
    }

    output.partial_paths.front().points.back().point.longitudinal_velocity_mps = 0.0;
    const double arc_length_on_first_arc_path =
      autoware::motion_utils::calcArcLength(output.partial_paths.front().points);
    std::cerr << "[GeometricPullOut::plan] First arc path length: " << arc_length_on_first_arc_path
              << std::endl;

    const double time_to_center = arc_length_on_first_arc_path / (2 * velocity);
    const double average_velocity = arc_length_on_first_arc_path / (time_to_center * 2);
    const double average_acceleration = average_velocity / (time_to_center * 2);
    output.pairs_terminal_velocity_and_accel.push_back(
      std::make_pair(average_velocity, average_acceleration));

    const double arc_length_on_second_arc_path =
      autoware::motion_utils::calcArcLength(planner_.getArcPaths().at(1).points);
    std::cerr << "[GeometricPullOut::plan] Second arc path length: "
              << arc_length_on_second_arc_path << std::endl;

    output.pairs_terminal_velocity_and_accel.push_back(
      std::make_pair(velocity, velocity * velocity / (2 * arc_length_on_second_arc_path)));
  } else {
    std::cerr << "[GeometricPullOut::plan] Processing combined pull-out path" << std::endl;
    const auto partial_paths = planner_.getPaths();

    if (partial_paths.size() < 2) {
      std::cerr << "[GeometricPullOut::plan] Failed: Insufficient partial paths" << std::endl;
      return {};
    }

    const auto combined_path = utils::combinePath(partial_paths.at(0), partial_paths.at(1));
    output.partial_paths.push_back(combined_path);

    const double arc_length_on_path = autoware::motion_utils::calcArcLength(combined_path.points);
    std::cerr << "[GeometricPullOut::plan] Combined path length: " << arc_length_on_path
              << std::endl;

    output.pairs_terminal_velocity_and_accel.push_back(
      std::make_pair(velocity, velocity * velocity / 2 * arc_length_on_path));
  }

  const auto & arc_paths = planner_.getArcPaths();
  if (arc_paths.empty() || arc_paths.at(0).points.empty() || arc_paths.at(1).points.empty()) {
    std::cerr << "[GeometricPullOut::plan] Failed: Invalid arc paths" << std::endl;
    return {};
  }

  output.start_pose = arc_paths.at(0).points.front().point.pose;
  output.end_pose = arc_paths.at(1).points.back().point.pose;

  if (isPullOutPathCollided(output, parameters_.geometric_collision_check_distance_from_end)) {
    std::cerr << "[GeometricPullOut::plan] Failed: Collision detected in pull-out path"
              << std::endl;
    planner_debug_data.conditions_evaluation.emplace_back("collision");
    return {};
  }

  std::cerr << "[GeometricPullOut::plan] Successfully generated pull-out path" << std::endl;
  planner_debug_data.conditions_evaluation.emplace_back("success");
  return output;
}
}  // namespace autoware::behavior_path_planner
