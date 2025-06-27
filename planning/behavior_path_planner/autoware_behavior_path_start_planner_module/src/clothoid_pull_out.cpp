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

#include "autoware/behavior_path_start_planner_module/clothoid_pull_out.hpp"

#include "autoware/behavior_path_planner_common/utils/parking_departure/utils.hpp"
#include "autoware/behavior_path_planner_common/utils/path_safety_checker/objects_filtering.hpp"
#include "autoware/behavior_path_planner_common/utils/path_utils.hpp"
#include "autoware/behavior_path_planner_common/utils/utils.hpp"
#include "autoware/behavior_path_start_planner_module/util.hpp"
#include "autoware/motion_utils/trajectory/path_with_lane_id.hpp"
#include "autoware_utils/geometry/boost_polygon_utils.hpp"

#include <autoware/motion_utils/trajectory/path_shift.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <iostream>
#include <limits>
#include <memory>
#include <numeric>
#include <utility>
#include <vector>

using autoware::motion_utils::findNearestIndex;
using autoware_utils::calc_distance2d;
using autoware_utils::calc_offset_pose;
using lanelet::utils::getArcCoordinates;
namespace autoware::behavior_path_planner
{
using start_planner_utils::getPullOutLanes;

ClothoidPullOut::ClothoidPullOut(
  rclcpp::Node & node, const StartPlannerParameters & parameters,
  std::shared_ptr<autoware_utils::TimeKeeper> time_keeper)
: PullOutPlannerBase{node, parameters, time_keeper}
{
  autoware::boundary_departure_checker::Param boundary_departure_checker_params;
  boundary_departure_checker_params.footprint_extra_margin =
    parameters.lane_departure_check_expansion_margin;
  boundary_departure_checker_ =
    std::make_shared<autoware::boundary_departure_checker::BoundaryDepartureChecker>(
      boundary_departure_checker_params, vehicle_info_, time_keeper_);
}

std::optional<PullOutPath> ClothoidPullOut::plan(
  const Pose & start_pose, const Pose & /*goal_pose*/,
  const std::shared_ptr<const PlannerData> & planner_data,
  PlannerDebugData & /*planner_debug_data*/)
{
  const auto & route_handler = planner_data->route_handler;
  const auto & common_parameters = planner_data->parameters;

  const double backward_path_length =
    planner_data->parameters.backward_path_length + parameters_.max_back_distance;
  const auto road_lanes = utils::getExtendedCurrentLanes(
    planner_data, backward_path_length, std::numeric_limits<double>::max(),
    /*forward_only_in_route*/ true);

  // Generate centerline path from road_lanes
  const auto centerline_path = utils::getCenterLinePath(
    *route_handler, road_lanes, start_pose, backward_path_length,
    std::numeric_limits<double>::max(), common_parameters);

  // Calculate lateral offset only if we have centerline points
  const double lateral_offset =
    centerline_path.points.empty()
      ? 0.0
      : autoware::motion_utils::calcLateralOffset(centerline_path.points, start_pose.position);
  std::cerr << "Lateral offset: " << lateral_offset << std::endl;

  // Temporary variables to avoid compilation errors
  const double minimum_radius = 13.46;

  // longitudinal necessary distance for pull out
  // minus lateral offset should be fixed later
  const double longitudinal_distance =
    start_planner_utils::calc_necessary_longitudinal_distance(-lateral_offset, minimum_radius);
  std::cerr << "Longitudinal distance: " << longitudinal_distance << std::endl;
  // target pose on the target lane
  // Get target pose from centerline path at longitudinal_distance ahead
  Pose target_pose = start_pose;
  if (!centerline_path.points.empty()) {
    // Find the point on centerline path that is longitudinal_distance ahead
    const auto start_idx =
      autoware::motion_utils::findNearestIndex(centerline_path.points, start_pose.position);
    double accumulated_distance = 0.0;
    size_t target_idx = start_idx;

    for (size_t i = start_idx; i < centerline_path.points.size() - 1; ++i) {
      const double segment_distance = autoware_utils::calc_distance2d(
        centerline_path.points[i].point.pose.position,
        centerline_path.points[i + 1].point.pose.position);
      accumulated_distance += segment_distance;

      if (accumulated_distance >= longitudinal_distance) {
        target_idx = i + 1;
        break;
      }
    }

    if (target_idx < centerline_path.points.size()) {
      target_pose = centerline_path.points[target_idx].point.pose;
    }
  }
  std::cerr << "start_pose: " << start_pose.position.x << ", " << start_pose.position.y << ", "
            << tf2::getYaw(start_pose.orientation) << std::endl;
  std::cerr << "target_pose: " << target_pose.position.x << ", " << target_pose.position.y << ", "
            << tf2::getYaw(target_pose.orientation) << std::endl;

  // Calculate relative position in vehicle coordinate system
  const double dx = target_pose.position.x - start_pose.position.x;
  const double dy = target_pose.position.y - start_pose.position.y;
  const double start_yaw = tf2::getYaw(start_pose.orientation);
  const double target_yaw = tf2::getYaw(target_pose.orientation);

  // Transform to vehicle coordinate system (x: forward, y: left)
  const double longitudinal_distance_vehicle = dx * std::cos(start_yaw) + dy * std::sin(start_yaw);
  const double lateral_distance_vehicle = -dx * std::sin(start_yaw) + dy * std::cos(start_yaw);

  // Calculate angle difference
  double angle_diff = target_yaw - start_yaw;
  // Normalize angle to [-pi, pi]
  while (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;
  while (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;

  std::cerr << "Vehicle coordinate relative position:" << std::endl;
  std::cerr << "  Longitudinal (forward): " << longitudinal_distance_vehicle << " m" << std::endl;
  std::cerr << "  Lateral (left): " << lateral_distance_vehicle << " m" << std::endl;
  std::cerr << "  Angle difference: " << angle_diff << " rad (" << angle_diff * 180.0 / M_PI
            << " deg)" << std::endl;

  const auto circular_path = start_planner_utils::calc_circular_path(
    start_pose, longitudinal_distance_vehicle, lateral_distance_vehicle, angle_diff,
    minimum_radius);
  // const auto corrected_path = autoware::motion_utils::correctPathWithLaneId(
  //   circular_path, common_parameters.ego_nearest_dist_threshold,
  //   common_parameters.ego_nearest_yaw_threshold);

  // const auto check_constraints = autoware::motion_utils::checkPathConstraints(
  //   corrected_path, common_parameters.ego_nearest_dist_threshold,
  //   common_parameters.ego_nearest_yaw_threshold, parameters_.maximum_curvature);

  // if (pull_out_paths.empty()) {
  //   planner_debug_data.conditions_evaluation.emplace_back("no path found");
  //   return std::nullopt;
  // }

  // const auto lanelet_map_ptr = planner_data->route_handler->getLaneletMapPtr();

  // std::vector<lanelet::Id> fused_id_start_to_end{};
  // std::optional<autoware_utils::Polygon2d> fused_polygon_start_to_end = std::nullopt;

  // std::vector<lanelet::Id> fused_id_crop_points{};
  // std::optional<autoware_utils::Polygon2d> fused_polygon_crop_points = std::nullopt;
  // // get safe path
  // for (auto & pull_out_path : pull_out_paths) {
  //   autoware_utils::ScopedTimeTrack st("get safe path", *time_keeper_);

  //   // shift path is not separate but only one.
  //   auto & shift_path = pull_out_path.partial_paths.front();
  //   // check lane_departure with path between pull_out_start to pull_out_end
  //   PathWithLaneId path_shift_start_to_end{};
  //   {
  //     const size_t pull_out_start_idx = findNearestIndex(shift_path.points, start_pose.position);
  //     const size_t pull_out_end_idx =
  //       findNearestIndex(shift_path.points, pull_out_path.end_pose.position);

  //     path_shift_start_to_end.points.insert(
  //       path_shift_start_to_end.points.begin(), shift_path.points.begin() + pull_out_start_idx,
  //       shift_path.points.begin() + pull_out_end_idx + 1);
  //   }

  //   // if lane departure check override is true, and if the initial pose is not fully within a
  //   lane,
  //   // cancel lane departure check
  //   const bool is_lane_departure_check_required = std::invoke([&]() -> bool {
  //     if (!parameters_.allow_check_shift_path_lane_departure_override)
  //       return parameters_.check_shift_path_lane_departure;

  //     PathWithLaneId path_with_only_first_pose{};
  //     path_with_only_first_pose.points.push_back(path_shift_start_to_end.points.front());
  //     return !boundary_departure_checker_->checkPathWillLeaveLane(
  //       lanelet_map_ptr, path_with_only_first_pose);
  //   });

  //   // check lane departure
  //   // The method for lane departure checking verifies if the footprint of each point on the path
  //   // is contained within a lanelet using `boost::geometry::within`, which incurs a high
  //   // computational cost.

  //   if (
  //     is_lane_departure_check_required && boundary_departure_checker_->checkPathWillLeaveLane(
  //                                           lanelet_map_ptr, path_shift_start_to_end,
  //                                           fused_id_start_to_end, fused_polygon_start_to_end)) {
  //     planner_debug_data.conditions_evaluation.emplace_back("lane departure");
  //     continue;
  //   }

  //   // crop backward path
  //   // removes points which are out of lanes up to the start pose.
  //   // this ensures that the backward_path stays within the drivable area when starting from a
  //   // narrow place.

  //   const size_t start_segment_idx =
  //     autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(
  //       shift_path.points, start_pose, common_parameters.ego_nearest_dist_threshold,
  //       common_parameters.ego_nearest_yaw_threshold);

  //   const auto cropped_path = boundary_departure_checker_->cropPointsOutsideOfLanes(
  //     lanelet_map_ptr, shift_path, start_segment_idx, fused_id_crop_points,
  //     fused_polygon_crop_points);
  //   if (cropped_path.points.empty()) {
  //     planner_debug_data.conditions_evaluation.emplace_back("cropped path is empty");
  //     continue;
  //   }

  //   // check that the path is not cropped in excess and there is not excessive longitudinal
  //   // deviation between the first 2 points
  //   auto validate_cropped_path = [&](const auto & cropped_path) -> bool {
  //     if (cropped_path.points.size() < 2) return false;
  //     const double max_long_offset = parameters_.maximum_longitudinal_deviation;
  //     const size_t start_segment_idx_after_crop =
  //       autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(
  //         cropped_path.points, start_pose);

  //     // if the start segment id after crop is not 0, then the cropping is not excessive
  //     if (start_segment_idx_after_crop != 0) return true;

  //     const auto long_offset_to_closest_point =
  //       autoware::motion_utils::calcLongitudinalOffsetToSegment(
  //         cropped_path.points, start_segment_idx_after_crop, start_pose.position);
  //     const auto long_offset_to_next_point =
  //       autoware::motion_utils::calcLongitudinalOffsetToSegment(
  //         cropped_path.points, start_segment_idx_after_crop + 1, start_pose.position);
  //     return std::abs(long_offset_to_closest_point - long_offset_to_next_point) <
  //     max_long_offset;
  //   };

  //   if (!validate_cropped_path(cropped_path)) {
  //     planner_debug_data.conditions_evaluation.emplace_back("cropped path is invalid");
  //     continue;
  //   }
  //   shift_path.points = cropped_path.points;
  //   shift_path.header = planner_data->route_handler->getRouteHeader();

  //   if (isPullOutPathCollided(
  //         pull_out_path, planner_data, parameters_.shift_collision_check_distance_from_end)) {
  //     planner_debug_data.conditions_evaluation.emplace_back("collision");
  //     continue;
  //   }

  //   planner_debug_data.conditions_evaluation.emplace_back("success");
  // return pull_out_path;

  // Implement actual clothoid pull out path generation
  // For now, return nullopt to indicate no path found
  // circular_pathが空の場合は処理を終了
  if (circular_path.empty()) {
    return std::nullopt;
  }

  // circular_pathをTrajectoryに変換して曲率を計算
  const auto trajectory = start_planner_utils::convertCircularPathToTrajectory(circular_path);
  const auto curvatures = start_planner_utils::calcCurvatureFromTrajectory(trajectory);

  // 曲率情報を端末に出力
  std::cerr << "=== Circular Path Curvature Information ===" << std::endl;
  std::cerr << "Number of points: " << circular_path.size() << std::endl;
  std::cerr << "Number of curvatures: " << curvatures.size() << std::endl;

  if (!curvatures.empty()) {
    // 統計情報を計算
    double max_curvature = *std::max_element(curvatures.begin(), curvatures.end());
    double min_curvature = *std::min_element(curvatures.begin(), curvatures.end());
    double sum_curvature = std::accumulate(curvatures.begin(), curvatures.end(), 0.0);
    double avg_curvature = sum_curvature / curvatures.size();

    std::cerr << "Curvature statistics:" << std::endl;
    std::cerr << "  Maximum: " << max_curvature << " [1/m]" << std::endl;
    std::cerr << "  Minimum: " << min_curvature << " [1/m]" << std::endl;
    std::cerr << "  Average: " << avg_curvature << " [1/m]" << std::endl;

    // 各点の曲率を出力（すべての点）
    std::cerr << "Curvature at each point:" << std::endl;
    for (size_t i = 0; i < curvatures.size(); ++i) {
      std::cerr << "  Point " << i << ": " << curvatures[i] << " [1/m]" << std::endl;
    }
  }
  std::cerr << "=========================================" << std::endl;

  // circular_pathからPathWithLaneIdを作成
  PathWithLaneId path_with_lane_id;
  path_with_lane_id.header = planner_data->route_handler->getRouteHeader();

  // 各座標点をPathPointWithLaneIdに変換
  for (size_t i = 0; i < circular_path.size(); ++i) {
    PathPointWithLaneId path_point;

    // 座標設定
    path_point.point.pose.position.x = circular_path[i].first;
    path_point.point.pose.position.y = circular_path[i].second;
    path_point.point.pose.position.z = start_pose.position.z;  // 高さは開始点と同じ

    // 向きを計算（次の点への方向）
    if (i < circular_path.size() - 1) {
      const double dx = circular_path[i + 1].first - circular_path[i].first;
      const double dy = circular_path[i + 1].second - circular_path[i].second;
      const double yaw = std::atan2(dy, dx);

      // quaternionを直接設定
      path_point.point.pose.orientation.x = 0.0;
      path_point.point.pose.orientation.y = 0.0;
      path_point.point.pose.orientation.z = std::sin(yaw / 2.0);
      path_point.point.pose.orientation.w = std::cos(yaw / 2.0);
    } else {
      // 最後の点は目標姿勢と同じ向き
      path_point.point.pose.orientation = target_pose.orientation;
    }

    // 速度設定（一定速度）
    path_point.point.longitudinal_velocity_mps = 5.0;  // 5 m/s
    path_point.point.lateral_velocity_mps = 0.0;
    path_point.point.heading_rate_rps = 0.0;
    path_point.point.is_final = (i == circular_path.size() - 1);

    // レーンID設定（現在のレーンのIDを使用）
    if (!road_lanes.empty()) {
      path_point.lane_ids.push_back(road_lanes[0].id());
    }

    path_with_lane_id.points.push_back(path_point);
  }

  // PullOutPathを作成
  PullOutPath pull_out_path;
  pull_out_path.partial_paths.push_back(path_with_lane_id);
  pull_out_path.start_pose = start_pose;
  pull_out_path.end_pose = target_pose;

  // 速度と加速度のペア設定
  pull_out_path.pairs_terminal_velocity_and_accel.push_back(std::make_pair(5.0, 1.0));

  return pull_out_path;
}

}  // namespace autoware::behavior_path_planner
