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

#ifndef AUTOWARE__BEHAVIOR_PATH_START_PLANNER_MODULE__SHIFT_PULL_OUT_HPP_
#define AUTOWARE__BEHAVIOR_PATH_START_PLANNER_MODULE__SHIFT_PULL_OUT_HPP_

#include "autoware/behavior_path_start_planner_module/pull_out_path.hpp"
#include "autoware/behavior_path_start_planner_module/pull_out_planner_base.hpp"
#include "autoware_utils/system/time_keeper.hpp"

#include <autoware/boundary_departure_checker/boundary_departure_checker.hpp>

#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>

#include <memory>
#include <vector>

namespace autoware::behavior_path_planner
{
using autoware::boundary_departure_checker::BoundaryDepartureChecker;

class ShiftPullOut : public PullOutPlannerBase
{
public:
  explicit ShiftPullOut(
    rclcpp::Node & node, const StartPlannerParameters & parameters,
    std::shared_ptr<autoware_utils::TimeKeeper> time_keeper =
      std::make_shared<autoware_utils::TimeKeeper>());

  PlannerType getPlannerType() const override { return PlannerType::SHIFT; };
  std::optional<PullOutPath> plan(
    const Pose & start_pose, const Pose & goal_pose,
    const std::shared_ptr<const PlannerData> & planner_data,
    PlannerDebugData & planner_debug_data) override;

  /**
   * @brief Calculates possible pull-out paths from a parking spot to the road
   *
   * @details Generates multiple candidate paths for pulling out from a parked position to the road,
   *          considering various parameters and constraints. The function:
   *          1. Generates a reference path from the road lanes
   *          2. Creates multiple path candidates with different lateral accelerations
   *          3. Evaluates path feasibility based on curvature and distance constraints
   *          4. Applies velocity profiles to the generated paths
   *
   * @param[in] route_handler    Handler containing route information
   * @param[in] road_lanes      Target road lanelets for pull-out
   * @param[in] start_pose      Initial pose of the vehicle (parked position)
   * @param[in] goal_pose       Target goal pose on the road
   *
   * @return Vector of PullOutPath containing all viable pull-out path candidates
   *
   * @note Key parameters considered include:
   *       - Forward/backward path lengths
   *       - Lateral jerk and acceleration limits
   *       - Maximum curvature constraints
   *       - Minimum pull-out distance
   *       - Path interval parameters
   *
   * @note Special cases:
   *       - Returns non-shifted path if shift length is very small (< 0.01)
   *       - Handles cases where end pose is on a curve
   *       - Adjusts velocity profile based on path characteristics
   *
   */
  std::vector<PullOutPath> calcPullOutPaths(
    const RouteHandler & route_handler, const lanelet::ConstLanelets & road_lanes,
    const Pose & start_pose, const Pose & goal_pose,
    const BehaviorPathPlannerParameters & behavior_path_parameters);

  double calcBeforeShiftedArcLength(
    const PathWithLaneId & path, const double target_after_arc_length, const double dr);

  /**
   * @brief Iteratively refines a shifted path to match the start pose
   *
   * @details Iteratively improves a shifted path based on given start and end poses.
   *          In each iteration, the function:
   *          1. Calculates the lateral offset at the start pose for the current path
   *          2. Generates a new shift line and processes it with the path shifter
   *          3. Continues until the lateral offset converges within tolerance
   *
   * @param[in,out] shifted_path      The shifted path to be refined
   * @param[in]     start_pose        The target start pose
   * @param[in]     end_pose          The target end pose
   * @param[in]     longitudinal_acc  Longitudinal acceleration limit
   * @param[in]     lateral_acc       Lateral acceleration limit
   * @return true if successful, false if refinement fails
   */
  bool refineShiftedPathToStartPose(
    ShiftedPath & shifted_path, const Pose & start_pose, const Pose & end_pose,
    const double longitudinal_acc, const double lateral_acc);

  std::shared_ptr<BoundaryDepartureChecker> boundary_departure_checker_;

  friend class TestShiftPullOut;

private:
  /**
   * @brief calculate longitudinal distance based on the acceleration limit, curvature limit, and
   * the minimum distance requirement.
   * @param[in] lon_acc Longitudinal acceleration
   * @param[in] shift_time Time required for pull out
   * @param[in] shift_length pull out distance
   * @param[in] max_curvature Maximum curvature
   * @param[in] min_distance Minimum distance
   * @return The minimum longitudinal distance during pull out
   */
  double calcPullOutLongitudinalDistance(
    const double lon_acc, const double shift_time, const double shift_length,
    const double max_curvature, const double min_distance) const;
};
}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_START_PLANNER_MODULE__SHIFT_PULL_OUT_HPP_
