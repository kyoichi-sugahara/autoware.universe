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

#include <autoware/behavior_path_planner_common/utils/parking_departure/geometric_parallel_parking.hpp>
#include <autoware/behavior_path_planner_common/utils/parking_departure/utils.hpp>
#include <autoware/behavior_path_planner_common/utils/path_utils.hpp>
#include <autoware/behavior_path_planner_common/utils/utils.hpp>
#include <autoware/interpolation/spline_interpolation.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware/universe_utils/math/unit_conversion.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>

#include <boost/geometry/algorithms/within.hpp>

#include <lanelet2_core/geometry/Polygon.h>

#include <algorithm>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

using autoware::universe_utils::calcDistance2d;
using autoware::universe_utils::calcOffsetPose;
using autoware::universe_utils::inverseTransformPoint;
using autoware::universe_utils::normalizeRadian;
using autoware::universe_utils::transformPose;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;
using lanelet::utils::getArcCoordinates;
using tier4_planning_msgs::msg::PathWithLaneId;

namespace autoware::behavior_path_planner
{
void GeometricParallelParking::incrementPathIndex()
{
  current_path_idx_ = std::min(current_path_idx_ + 1, paths_.size() - 1);
}

PathWithLaneId GeometricParallelParking::getPathByIdx(size_t const idx) const
{
  if (paths_.empty() || paths_.size() <= idx) {
    return PathWithLaneId{};
  }

  return paths_.at(idx);
}

PathWithLaneId GeometricParallelParking::getCurrentPath() const
{
  return paths_.at(current_path_idx_);
}

PathWithLaneId GeometricParallelParking::getFullPath() const
{
  PathWithLaneId path{};
  for (const auto & partial_path : paths_) {
    path.points.insert(path.points.end(), partial_path.points.begin(), partial_path.points.end());
  }

  PathWithLaneId filtered_path = path;
  filtered_path.points = autoware::motion_utils::removeOverlapPoints(filtered_path.points);
  return filtered_path;
}

PathWithLaneId GeometricParallelParking::getArcPath() const
{
  PathWithLaneId path{};
  for (const auto & arc_path : arc_paths_) {
    path.points.insert(path.points.end(), arc_path.points.begin(), arc_path.points.end());
  }
  return path;
}

bool GeometricParallelParking::isParking() const
{
  return current_path_idx_ > 0;
}

void GeometricParallelParking::setVelocityToArcPaths(
  std::vector<PathWithLaneId> & arc_paths, const double velocity, const bool set_stop_end)
{
  for (auto & path : arc_paths) {
    for (size_t i = 0; i < path.points.size(); i++) {
      if (i == path.points.size() - 1 && set_stop_end) {
        // stop point at the end of the path
        path.points.at(i).point.longitudinal_velocity_mps = 0.0;
      } else {
        path.points.at(i).point.longitudinal_velocity_mps = velocity;
      }
    }
  }
}

std::vector<PathWithLaneId> GeometricParallelParking::generatePullOverPaths(
  const Pose & start_pose, const Pose & goal_pose, const double R_E_far,
  const lanelet::ConstLanelets & road_lanes, const lanelet::ConstLanelets & pull_over_lanes,
  const bool is_forward, const bool left_side_parking, const double end_pose_offset,
  const double velocity)
{
  const double lane_departure_margin = is_forward
                                         ? parameters_.forward_parking_lane_departure_margin
                                         : parameters_.backward_parking_lane_departure_margin;
  const double arc_path_interval = is_forward ? parameters_.forward_parking_path_interval
                                              : parameters_.backward_parking_path_interval;
  auto arc_paths = planOneTrial(
    start_pose, goal_pose, R_E_far, road_lanes, pull_over_lanes, is_forward, left_side_parking,
    end_pose_offset, lane_departure_margin, arc_path_interval, {});
  if (arc_paths.empty()) {
    return std::vector<PathWithLaneId>{};
  }
  arc_paths_ = arc_paths;

  // set parking velocity and stop velocity at the end of the path
  constexpr bool set_stop_end = true;
  setVelocityToArcPaths(arc_paths, velocity, set_stop_end);

  // straight path from current to parking start
  const auto straight_path = generateStraightPath(start_pose, road_lanes);

  // check the continuity of straight path and arc path
  const Pose & road_path_last_pose = straight_path.points.back().point.pose;
  const Pose & arc_path_first_pose = arc_paths.front().points.front().point.pose;
  const double yaw_diff = std::abs(autoware::universe_utils::normalizeRadian(
    tf2::getYaw(road_path_last_pose.orientation) - tf2::getYaw(arc_path_first_pose.orientation)));
  const double distance = calcDistance2d(road_path_last_pose, arc_path_first_pose);
  if (yaw_diff > autoware::universe_utils::deg2rad(5.0) || distance > 0.1) {
    return std::vector<PathWithLaneId>{};
  }

  // combine straight_path -> arc_path*2
  auto paths = arc_paths;
  paths.insert(paths.begin(), straight_path);

  return paths;
}

void GeometricParallelParking::clearPaths()
{
  current_path_idx_ = 0;
  arc_paths_.clear();
  paths_.clear();
  pairs_terminal_velocity_and_accel_.clear();
}

bool GeometricParallelParking::planPullOver(
  const Pose & goal_pose, const lanelet::ConstLanelets & road_lanes,
  const lanelet::ConstLanelets & pull_over_lanes, const bool is_forward,
  const bool left_side_parking)
{
  const auto & common_params = planner_data_->parameters;
  const double end_pose_offset = is_forward ? -parameters_.after_forward_parking_straight_distance
                                            : parameters_.after_backward_parking_straight_distance;
  const Pose arc_end_pose = calcOffsetPose(goal_pose, end_pose_offset, 0, 0);

  // not plan after parking has started
  if (isParking()) {
    return false;
  }

  if (is_forward) {
    // When turning forward to the right, the front left goes out,
    // so reduce the steer angle at that time for seach no lane departure path.
    // TODO(Sugahara): define in the config
    constexpr double start_pose_offset = 0.0;
    constexpr double min_steer_rad = 0.05;
    constexpr double steer_interval = 0.1;
    for (double steer = parameters_.forward_parking_max_steer_angle; steer > min_steer_rad;
         steer -= steer_interval) {
      const double R_E_far = common_params.wheel_base / std::tan(steer);
      const auto start_pose = calcStartPose(
        arc_end_pose, road_lanes, start_pose_offset, R_E_far, is_forward, left_side_parking);
      if (!start_pose) {
        continue;
      }

      const auto paths = generatePullOverPaths(
        *start_pose, goal_pose, R_E_far, road_lanes, pull_over_lanes, is_forward, left_side_parking,
        end_pose_offset, parameters_.forward_parking_velocity);
      if (!paths.empty()) {
        paths_ = paths;
        return true;
      }
    }
  } else {  // backward
    // When turning backward to the left, the front right goes out,
    // so make the parking start point in front for seach no lane departure path
    // (same to reducing the steer angle)
    constexpr double max_offset = 5.0;
    constexpr double offset_interval = 1.0;
    for (double start_pose_offset = 0; start_pose_offset < max_offset;
         start_pose_offset += offset_interval) {
      const auto start_pose = calcStartPose(
        arc_end_pose, road_lanes, start_pose_offset, R_E_min_, is_forward, left_side_parking);
      if (!start_pose) {
        continue;
      }

      const auto paths = generatePullOverPaths(
        *start_pose, goal_pose, R_E_min_, road_lanes, pull_over_lanes, is_forward,
        left_side_parking, end_pose_offset, parameters_.backward_parking_velocity);
      if (!paths.empty()) {
        paths_ = paths;
        return true;
      }
    }
  }

  return false;
}

bool GeometricParallelParking::planPullOut(
  const Pose & start_pose, const Pose & goal_pose, const lanelet::ConstLanelets & road_lanes,
  const lanelet::ConstLanelets & pull_over_lanes, const bool left_side_start,
  const std::shared_ptr<autoware::lane_departure_checker::LaneDepartureChecker>
    lane_departure_checker)
{
  std::cerr << "[planPullOut] Start planning pull out..." << std::endl;
  std::cerr << "[planPullOut] Left side start: " << (left_side_start ? "true" : "false")
            << std::endl;
  std::cerr << "[planPullOut] R_E_min_: " << R_E_min_ << std::endl;

  if (road_lanes.empty()) {
    std::cerr << "[planPullOut] Failed: road_lanes is empty" << std::endl;
    return false;
  }

  if (pull_over_lanes.empty()) {
    std::cerr << "[planPullOut] Failed: pull_over_lanes is empty" << std::endl;
    return false;
  }

  constexpr bool is_forward = false;         // parking backward means pull_out forward
  constexpr double start_pose_offset = 0.0;  // start_pose is current_pose
  constexpr double max_offset = 10.0;
  constexpr double offset_interval = 1.0;

  std::cerr << "[planPullOut] Starting end pose offset loop (max_offset: " << max_offset
            << ", interval: " << offset_interval << ")" << std::endl;

  for (double end_pose_offset = 0; end_pose_offset < max_offset;
       end_pose_offset += offset_interval) {
    std::cerr << "\n[planPullOut] Trying with end_pose_offset: " << end_pose_offset << std::endl;

    // pull_out end pose which is the second arc path end
    const auto end_pose =
      calcStartPose(start_pose, road_lanes, end_pose_offset, R_E_min_, is_forward, left_side_start);
    if (!end_pose) {
      std::cerr << "[planPullOut] Failed to calculate end pose for offset " << end_pose_offset
                << std::endl;
      continue;
    }
    std::cerr << "[planPullOut] Successfully calculated end pose" << std::endl;

    // plan reverse path of parking. end_pose <-> start_pose
    auto arc_paths = planOneTrial(
      *end_pose, start_pose, R_E_min_, road_lanes, pull_over_lanes, is_forward, left_side_start,
      start_pose_offset, parameters_.pull_out_lane_departure_margin,
      parameters_.pull_out_arc_path_interval, lane_departure_checker);
    if (arc_paths.empty()) {
      std::cerr << "[planPullOut] No valid arc paths found in planOneTrial" << std::endl;
      continue;
    }
    std::cerr << "[planPullOut] Found valid arc paths, count: " << arc_paths.size() << std::endl;

    // reverse to turn_right -> turn_left
    std::reverse(arc_paths.begin(), arc_paths.end());

    // reverse path points order and lane_ids
    for (auto & path : arc_paths) {
      std::reverse(path.points.begin(), path.points.end());
    }

    // reverse lane_ids. shoulder, lane order
    for (auto & path : arc_paths) {
      for (auto & p : path.points) {
        std::reverse(p.lane_ids.begin(), p.lane_ids.end());
      }
    }

    // get road center line path
    const double s_start = getArcCoordinates(road_lanes, *end_pose).length;
    const auto path_end_info = utils::parking_departure::calcEndArcLength(
      s_start, planner_data_->parameters.forward_path_length, road_lanes, goal_pose);
    const double s_end = path_end_info.first;
    const bool path_terminal_is_goal = path_end_info.second;

    std::cerr << "[planPullOut] Calculating road center line path (s_start: " << s_start
              << ", s_end: " << s_end << ")" << std::endl;

    const PathWithLaneId road_center_line_path = utils::resamplePathWithSpline(
      planner_data_->route_handler->getCenterLinePath(road_lanes, s_start, s_end, true),
      parameters_.center_line_path_interval);

    if (road_center_line_path.points.empty()) {
      std::cerr << "[planPullOut] Failed: Road center line path is empty" << std::endl;
      continue;
    }

    // check path continuity
    const Pose & road_path_first_pose = road_center_line_path.points.front().point.pose;
    const Pose & arc_path_last_pose = arc_paths.back().points.back().point.pose;
    const double yaw_diff = std::abs(autoware::universe_utils::normalizeRadian(
      tf2::getYaw(road_path_first_pose.orientation) - tf2::getYaw(arc_path_last_pose.orientation)));
    const double distance = calcDistance2d(road_path_first_pose, arc_path_last_pose);

    std::cerr << "[planPullOut] Path continuity check - yaw_diff: " << yaw_diff << " rad ("
              << autoware::universe_utils::rad2deg(yaw_diff) << " deg)"
              << ", distance: " << distance << " m" << std::endl;

    if (yaw_diff > autoware::universe_utils::deg2rad(5.0) || distance > 0.1) {
      std::cerr << "[planPullOut] Failed: Path continuity check failed" << std::endl;
      continue;
    }

    // set velocity and combine paths
    constexpr bool set_stop_end = false;
    setVelocityToArcPaths(arc_paths, parameters_.pull_out_velocity, set_stop_end);

    auto paths = arc_paths;
    paths.back().points.insert(
      paths.back().points.end(),
      road_center_line_path.points.begin() + 1,  // to avoid overlapped point
      road_center_line_path.points.end());
    paths.back().points = autoware::motion_utils::removeOverlapPoints(paths.back().points);

    if (path_terminal_is_goal) {
      paths.back().points.back().point.longitudinal_velocity_mps = 0.0;
      std::cerr << "[planPullOut] Set zero velocity to goal point" << std::endl;
    }

    arc_paths_ = arc_paths;
    paths_ = paths;

    std::cerr << "[planPullOut] Successfully generated pull out path" << std::endl;
    return true;
  }

  std::cerr << "[planPullOut] Failed: Could not find valid path with any offset" << std::endl;
  return false;
}

std::optional<Pose> GeometricParallelParking::calcStartPose(
  const Pose & goal_pose, const lanelet::ConstLanelets & road_lanes, const double start_pose_offset,
  const double R_E_far, const bool is_forward, const bool left_side_parking)
{
  std::cerr << "\n[calcStartPose] Starting calculation with parameters:"
            << "\n  start_pose_offset: " << start_pose_offset << "\n  R_E_far: " << R_E_far
            << "\n  is_forward: " << (is_forward ? "true" : "false")
            << "\n  left_side_parking: " << (left_side_parking ? "true" : "false") << std::endl;

  const auto arc_coordinates = lanelet::utils::getArcCoordinates(road_lanes, goal_pose);
  std::cerr << "[calcStartPose] Arc coordinates:"
            << "\n  distance: " << arc_coordinates.distance
            << "\n  length: " << arc_coordinates.length << std::endl;

  // Calculate squared_distance_to_arc_connect
  const double term_inside_sqrt = left_side_parking ? -arc_coordinates.distance / 2 + R_E_far
                                                    : arc_coordinates.distance / 2 + R_E_far;

  std::cerr << "[calcStartPose] Calculation details:"
            << "\n  term_inside_sqrt: " << term_inside_sqrt
            << "\n  R_E_far^2: " << std::pow(R_E_far, 2)
            << "\n  term_inside_sqrt^2: " << std::pow(term_inside_sqrt, 2) << std::endl;

  const double squared_distance_to_arc_connect =
    left_side_parking ? std::pow(R_E_far, 2) - std::pow(-arc_coordinates.distance / 2 + R_E_far, 2)
                      : std::pow(R_E_far, 2) - std::pow(arc_coordinates.distance / 2 + R_E_far, 2);

  std::cerr << "[calcStartPose] squared_distance_to_arc_connect: "
            << squared_distance_to_arc_connect << std::endl;

  if (squared_distance_to_arc_connect < 0) {
    std::cerr << "[calcStartPose] Failed: squared_distance_to_arc_connect is negative"
              << " (may be current_pose is behind the lane)" << std::endl;
    return std::nullopt;
  }

  const double dx_sign = is_forward ? -1 : 1;
  const double dx = 2 * std::sqrt(squared_distance_to_arc_connect) * dx_sign;

  std::cerr << "[calcStartPose] Offset calculations:"
            << "\n  dx_sign: " << dx_sign << "\n  dx: " << dx << std::endl;

  // Calculate approximate start pose
  const Pose approximate_start_pose = calcOffsetPose(goal_pose, dx, -arc_coordinates.distance, 0);
  std::cerr << "[calcStartPose] Approximate start pose:"
            << "\n  x: " << approximate_start_pose.position.x
            << "\n  y: " << approximate_start_pose.position.y
            << "\n  yaw: " << tf2::getYaw(approximate_start_pose.orientation) << std::endl;

  // Get closest road lane
  lanelet::ConstLanelet closest_road_lane{};
  lanelet::utils::query::getClosestLanelet(road_lanes, approximate_start_pose, &closest_road_lane);

  if (!closest_road_lane.id()) {
    std::cerr << "[calcStartPose] Failed: Could not find closest road lane" << std::endl;
    return std::nullopt;
  }
  std::cerr << "[calcStartPose] Found closest road lane ID: " << closest_road_lane.id()
            << std::endl;

  // Calculate start pose on centerline
  const Pose start_pose_no_offset =
    lanelet::utils::getClosestCenterPose(closest_road_lane, approximate_start_pose.position);

  std::cerr << "[calcStartPose] Start pose (before offset):"
            << "\n  x: " << start_pose_no_offset.position.x
            << "\n  y: " << start_pose_no_offset.position.y
            << "\n  yaw: " << tf2::getYaw(start_pose_no_offset.orientation) << std::endl;

  // Get road lane path and calculate final start pose
  const auto road_lane_path = planner_data_->route_handler->getCenterLinePath(
    road_lanes, 0.0, std::numeric_limits<double>::max());

  if (road_lane_path.points.empty()) {
    std::cerr << "[calcStartPose] Failed: Road lane path is empty" << std::endl;
    return std::nullopt;
  }

  const auto start_pose = autoware::motion_utils::calcLongitudinalOffsetPose(
    road_lane_path.points, start_pose_no_offset.position, start_pose_offset);

  if (!start_pose) {
    std::cerr << "[calcStartPose] Failed: Could not calculate final start pose" << std::endl;
    return std::nullopt;
  }

  std::cerr << "[calcStartPose] Final start pose:"
            << "\n  x: " << start_pose->position.x << "\n  y: " << start_pose->position.y
            << "\n  yaw: " << tf2::getYaw(start_pose->orientation) << std::endl;

  return start_pose;
}

PathWithLaneId GeometricParallelParking::generateStraightPath(
  const Pose & start_pose, const lanelet::ConstLanelets & road_lanes)
{
  // get straight path before parking.
  const auto start_arc_position = lanelet::utils::getArcCoordinates(road_lanes, start_pose);

  const Pose current_pose = planner_data_->self_odometry->pose.pose;
  const auto current_arc_position = lanelet::utils::getArcCoordinates(road_lanes, current_pose);

  auto path = utils::resamplePathWithSpline(
    planner_data_->route_handler->getCenterLinePath(
      road_lanes, current_arc_position.length, start_arc_position.length, true),
    parameters_.center_line_path_interval);
  path.header = planner_data_->route_handler->getRouteHeader();
  if (!path.points.empty()) {
    path.points.back().point.longitudinal_velocity_mps = 0;
  }

  return path;
}

std::vector<PathWithLaneId> GeometricParallelParking::planOneTrial(
  const Pose & start_pose, const Pose & goal_pose, const double R_E_far,
  const lanelet::ConstLanelets & road_lanes, const lanelet::ConstLanelets & pull_over_lanes,
  const bool is_forward, const bool left_side_parking, const double end_pose_offset,
  const double lane_departure_margin, const double arc_path_interval,
  const std::shared_ptr<autoware::lane_departure_checker::LaneDepartureChecker>
    lane_departure_checker)
{
  std::cerr << "\n[planOneTrial] Starting with parameters:"
            << "\n  R_E_far: " << R_E_far << "\n  is_forward: " << (is_forward ? "true" : "false")
            << "\n  left_side_parking: " << (left_side_parking ? "true" : "false")
            << "\n  end_pose_offset: " << end_pose_offset
            << "\n  lane_departure_margin: " << lane_departure_margin
            << "\n  arc_path_interval: " << arc_path_interval << std::endl;

  clearPaths();
  const auto & common_params = planner_data_->parameters;
  const auto & route_handler = planner_data_->route_handler;

  const Pose arc_end_pose = calcOffsetPose(goal_pose, end_pose_offset, 0, 0);
  const double self_yaw = tf2::getYaw(start_pose.orientation);
  const double goal_yaw = tf2::getYaw(arc_end_pose.orientation);
  const double psi = normalizeRadian(self_yaw - goal_yaw);

  std::cerr << "[planOneTrial] Pose calculations:"
            << "\n  self_yaw: " << self_yaw << "\n  goal_yaw: " << goal_yaw << "\n  psi: " << psi
            << std::endl;

  const Pose C_far = left_side_parking ? calcOffsetPose(arc_end_pose, 0, -R_E_far, 0)
                                       : calcOffsetPose(arc_end_pose, 0, R_E_far, 0);
  const double d_C_far_Einit = calcDistance2d(C_far, start_pose);

  const Point C_far_goal_coords = inverseTransformPoint(C_far.position, arc_end_pose);
  const Point self_point_goal_coords = inverseTransformPoint(start_pose.position, arc_end_pose);

  const double alpha =
    left_side_parking
      ? M_PI_2 - psi + std::asin((self_point_goal_coords.y - C_far_goal_coords.y) / d_C_far_Einit)
      : M_PI_2 + psi - std::asin((self_point_goal_coords.y - C_far_goal_coords.y) / d_C_far_Einit);

  std::cerr << "[planOneTrial] Path geometry calculations:"
            << "\n  d_C_far_Einit: " << d_C_far_Einit << "\n  alpha: " << alpha << std::endl;

  const double R_E_near = (std::pow(d_C_far_Einit, 2) - std::pow(R_E_far, 2)) /
                          (2 * (R_E_far + d_C_far_Einit * std::cos(alpha)));

  std::cerr << "[planOneTrial] Calculated R_E_near: " << R_E_near << std::endl;

  if (R_E_near <= 0) {
    std::cerr << "[planOneTrial] Failed: R_E_near is non-positive" << std::endl;
    return std::vector<PathWithLaneId>{};
  }

  lanelet::ConstLanelets lanes{};
  autoware::universe_utils::Point2d start_point2d(start_pose.position.x, start_pose.position.y);

  std::cerr << "[planOneTrial] Finding containing lane for start point: (" << start_point2d.x()
            << ", " << start_point2d.y() << ")" << std::endl;

  for (const auto & lane : road_lanes) {
    if (boost::geometry::within(start_point2d, lane.polygon2d().basicPolygon())) {
      std::cerr << "[planOneTrial] Found containing lane ID: " << lane.id() << std::endl;
      lanes.push_back(lane);
      break;
    }
    lanes.push_back(lane);
  }
  lanes.insert(lanes.end(), pull_over_lanes.begin(), pull_over_lanes.end());
  std::cerr << "[planOneTrial] Total combined lanes: " << lanes.size() << std::endl;

  // If start_pose is parallel to goal_pose, we can know lateral deviation of edges of vehicle,
  // and detect lane departure.
  if (is_forward) {  // Check near bound
    const double R_front_near =
      std::hypot(R_E_far + common_params.vehicle_width / 2, common_params.base_link2front);
    const double distance_to_near_bound =
      utils::getSignedDistanceFromBoundary(pull_over_lanes, arc_end_pose, left_side_parking);
    const double near_deviation = R_front_near - R_E_far;

    std::cerr << "[planOneTrial] Forward check parameters:"
              << "\n  R_front_near: " << R_front_near
              << "\n  distance_to_near_bound: " << distance_to_near_bound
              << "\n  near_deviation: " << near_deviation
              << "\n  margin check: " << (std::abs(distance_to_near_bound) - near_deviation)
              << " vs " << lane_departure_margin << std::endl;

    if (std::abs(distance_to_near_bound) - near_deviation < lane_departure_margin) {
      std::cerr << "[planOneTrial] Failed: Forward check margin violation" << std::endl;
      return std::vector<PathWithLaneId>{};
    }
  } else {
    const double R_front_far =
      std::hypot(R_E_near + common_params.vehicle_width / 2, common_params.base_link2front);
    const double far_deviation = R_front_far - R_E_near;
    const double distance_to_far_bound =
      utils::getSignedDistanceFromBoundary(lanes, start_pose, !left_side_parking);

    std::cerr << "[planOneTrial] Backward check parameters:"
              << "\n  R_front_far: " << R_front_far
              << "\n  distance_to_far_bound: " << distance_to_far_bound
              << "\n  far_deviation: " << far_deviation
              << "\n  margin check: " << (std::abs(distance_to_far_bound) - far_deviation) << " vs "
              << lane_departure_margin << std::endl;

    if (std::abs(distance_to_far_bound) - far_deviation < lane_departure_margin) {
      std::cerr << "[planOneTrial] Failed: Backward check margin violation" << std::endl;
      return std::vector<PathWithLaneId>{};
    }
  }

  std::cerr << "[planOneTrial] Generating arc paths..." << std::endl;

  const Pose C_near = left_side_parking ? calcOffsetPose(start_pose, 0, R_E_near, 0)
                                        : calcOffsetPose(start_pose, 0, -R_E_near, 0);
  const double theta_near =
    std::acos(
      (std::pow(R_E_near, 2) + std::pow(R_E_near + R_E_far, 2) - std::pow(d_C_far_Einit, 2)) /
      (2 * R_E_near * (R_E_near + R_E_far))) *
    (is_forward == left_side_parking ? 1 : -1);

  std::cerr << "[planOneTrial] Arc path parameters:"
            << "\n  theta_near: " << theta_near << "\n  arc_path_interval: " << arc_path_interval
            << std::endl;

  // Check if arc_path_interval is valid
  if (arc_path_interval <= 0.0) {
    std::cerr << "[planOneTrial] Error: Invalid arc_path_interval: " << arc_path_interval
              << std::endl;
    return std::vector<PathWithLaneId>{};
  }

  const auto generateArcPathWithHeader =
    [&](
      const auto & C, const auto & R_E, const auto & start_angle, const auto & end_angle,
      bool is_forward_first, bool is_forward_second) -> PathWithLaneId {
    std::cerr << "[generateArcPathWithHeader] Generating arc path with:"
              << "\n  C: (" << C.position.x << ", " << C.position.y << ")"
              << "\n  R_E: " << R_E << "\n  start_angle: " << start_angle << " rad ("
              << start_angle * 180.0 / M_PI << " deg)"
              << "\n  end_angle: " << end_angle << " rad (" << end_angle * 180.0 / M_PI << " deg)"
              << "\n  is_forward_first: " << (is_forward_first ? "true" : "false")
              << "\n  is_forward_second: " << (is_forward_second ? "true" : "false") << std::endl;

    auto path = generateArcPath(
      C, R_E, start_angle, end_angle, arc_path_interval, is_forward_first, is_forward_second);

    std::cerr << "[generateArcPathWithHeader] Generated path with " << path.points.size()
              << " points" << std::endl;

    path.header = route_handler->getRouteHeader();
    return path;
  };

  std::cerr << "[planOneTrial] Calculating path_turn_first parameters for "
            << (left_side_parking ? "left side parking" : "right side parking") << std::endl;

  double first_start_angle = left_side_parking ? -M_PI_2 : M_PI_2;
  double first_end_angle = left_side_parking ? normalizeRadian(-M_PI_2 + theta_near)
                                             : normalizeRadian(M_PI_2 + theta_near);

  std::cerr << "[planOneTrial] First turn angles:"
            << "\n  start: " << first_start_angle << " rad (" << first_start_angle * 180.0 / M_PI
            << " deg)"
            << "\n  end: " << first_end_angle << " rad (" << first_end_angle * 180.0 / M_PI
            << " deg)" << std::endl;

  PathWithLaneId path_turn_first =
    left_side_parking
      ? generateArcPathWithHeader(
          C_near, R_E_near, first_start_angle, first_end_angle, is_forward, is_forward)
      : generateArcPathWithHeader(
          C_near, R_E_near, first_start_angle, first_end_angle, !is_forward, is_forward);

  std::cerr << "[planOneTrial] Generated first turn path with " << path_turn_first.points.size()
            << " points" << std::endl;

  PathWithLaneId path_turn_second =
    left_side_parking ? generateArcPathWithHeader(
                          C_far, R_E_far, normalizeRadian(psi + M_PI_2 + theta_near), M_PI_2,
                          !is_forward, is_forward)
                      : generateArcPathWithHeader(
                          C_far, R_E_far, normalizeRadian(psi - M_PI_2 + theta_near), -M_PI_2,
                          is_forward, is_forward);

  std::cerr << "[planOneTrial] Generated second turn path with " << path_turn_second.points.size()
            << " points" << std::endl;

  if (std::abs(end_pose_offset) > 0) {
    PathPointWithLaneId straight_point{};
    straight_point.point.pose = goal_pose;
    path_turn_second.points.push_back(straight_point);
    std::cerr << "[planOneTrial] Added straight point to second turn path" << std::endl;
  }

  // Populate lane ids for a given path.
  // It checks if each point in the path is within a lane
  // and if its ID hasn't been added yet, it appends the ID to the container.
  std::vector<lanelet::Id> path_lane_ids;
  std::cerr << "[planOneTrial] Starting lane ID population..." << std::endl;

  for (const auto & p : path_turn_first.points) {
    for (const auto & lane : lanes) {
      if (
        lanelet::utils::isInLanelet(p.point.pose, lane) &&
        std::find(path_lane_ids.begin(), path_lane_ids.end(), lane.id()) == path_lane_ids.end()) {
        path_lane_ids.push_back(lane.id());
        std::cerr << "[planOneTrial] Added lane ID: " << lane.id() << " for first turn"
                  << std::endl;
      }
    }
  }

  for (const auto & p : path_turn_second.points) {
    for (const auto & lane : lanes) {
      if (
        lanelet::utils::isInLanelet(p.point.pose, lane) &&
        std::find(path_lane_ids.begin(), path_lane_ids.end(), lane.id()) == path_lane_ids.end()) {
        path_lane_ids.push_back(lane.id());
        std::cerr << "[planOneTrial] Added lane ID: " << lane.id() << " for second turn"
                  << std::endl;
      }
    }
  }

  for (auto & p : path_turn_first.points) {
    p.lane_ids = path_lane_ids;
  }
  for (auto & p : path_turn_second.points) {
    p.lane_ids = path_lane_ids;
  }

  std::cerr << "[planOneTrial] Checking lane departure..." << std::endl;

  if (lane_departure_checker) {
    const auto lanelet_map_ptr = planner_data_->route_handler->getLaneletMapPtr();

    const bool is_path_turn_first_outside_lanes =
      lane_departure_checker->checkPathWillLeaveLane(lanelet_map_ptr, path_turn_first);

    if (is_path_turn_first_outside_lanes) {
      std::cerr << "[planOneTrial] Failed: First turn path will leave lane" << std::endl;
      return std::vector<PathWithLaneId>{};
    }

    const bool is_path_turn_second_outside_lanes =
      lane_departure_checker->checkPathWillLeaveLane(lanelet_map_ptr, path_turn_second);

    if (is_path_turn_second_outside_lanes) {
      std::cerr << "[planOneTrial] Failed: Second turn path will leave lane" << std::endl;
      return std::vector<PathWithLaneId>{};
    }
  }

  // generate arc path vector
  paths_.push_back(path_turn_first);
  paths_.push_back(path_turn_second);

  // set terminal velocity and acceleration(temporary implementation)
  if (is_forward) {
    pairs_terminal_velocity_and_accel_.emplace_back(parameters_.forward_parking_velocity, 0.0);
    pairs_terminal_velocity_and_accel_.emplace_back(parameters_.forward_parking_velocity, 0.0);
  } else {
    pairs_terminal_velocity_and_accel_.emplace_back(parameters_.backward_parking_velocity, 0.0);
    pairs_terminal_velocity_and_accel_.emplace_back(parameters_.backward_parking_velocity, 0.0);
  }

  // set pull_over start and end pose
  // todo: make start and end pose for pull_out
  start_pose_ = start_pose;
  arc_end_pose_ = arc_end_pose;

  // debug
  Cr_ = left_side_parking ? C_far : C_near;
  Cl_ = left_side_parking ? C_near : C_far;

  std::cerr << "[planOneTrial] Successfully generated pull-out path" << std::endl;
  return paths_;
}

PathWithLaneId GeometricParallelParking::generateArcPath(
  const Pose & center, const double radius, const double start_yaw, double end_yaw,
  const double arc_path_interval,
  const bool is_left_turn,  // is_left_turn means clockwise around center.
  const bool is_forward)
{
  PathWithLaneId path;
  const double yaw_interval = arc_path_interval / radius;
  double yaw = start_yaw;
  if (is_left_turn) {
    if (end_yaw < start_yaw) end_yaw += M_PI_2;
    while (yaw < end_yaw) {
      const auto p = generateArcPathPoint(center, radius, yaw, is_left_turn, is_forward);
      path.points.push_back(p);
      yaw += yaw_interval;
    }
  } else {  // right_turn
    if (end_yaw > start_yaw) end_yaw -= M_PI_2;
    while (yaw > end_yaw) {
      const auto p = generateArcPathPoint(center, radius, yaw, is_left_turn, is_forward);
      path.points.push_back(p);
      yaw -= yaw_interval;
    }
  }

  // insert the last point exactly
  const auto p = generateArcPathPoint(center, radius, end_yaw, is_left_turn, is_forward);
  constexpr double min_dist = 0.01;
  if (path.points.empty() || calcDistance2d(path.points.back(), p) > min_dist) {
    path.points.push_back(p);
  }

  return path;
}

PathPointWithLaneId GeometricParallelParking::generateArcPathPoint(
  const Pose & center, const double radius, const double yaw, const bool is_left_turn,
  const bool is_forward)
{
  // get pose in center_pose coords
  Pose pose_center_coords;
  pose_center_coords.position.x = radius * std::cos(yaw);
  pose_center_coords.position.y = radius * std::sin(yaw);

  // set orientation
  tf2::Quaternion quat;
  if ((is_left_turn && !is_forward) || (!is_left_turn && is_forward)) {
    quat.setRPY(0, 0, normalizeRadian(yaw - M_PI_2));
  } else {
    quat.setRPY(0, 0, normalizeRadian(yaw + M_PI_2));
  }
  pose_center_coords.orientation = tf2::toMsg(quat);

  // get pose in map coords
  PathPointWithLaneId p{};
  p.point.pose = transformPose(pose_center_coords, center);

  return p;
}

void GeometricParallelParking::setTurningRadius(
  const BehaviorPathPlannerParameters & common_params, const double max_steer_angle)
{
  std::cerr << "[setTurningRadius] Setting turning radius with parameters:"
            << "\n  wheel_base: " << common_params.wheel_base
            << "\n  wheel_tread: " << common_params.wheel_tread
            << "\n  left_over_hang: " << common_params.left_over_hang
            << "\n  front_overhang: " << common_params.front_overhang
            << "\n  max_steer_angle: " << max_steer_angle << std::endl;
  R_E_min_ = common_params.wheel_base / std::tan(max_steer_angle);
  R_Bl_min_ = std::hypot(
    R_E_min_ + common_params.wheel_tread / 2 + common_params.left_over_hang,
    common_params.wheel_base + common_params.front_overhang);
}

}  // namespace autoware::behavior_path_planner
