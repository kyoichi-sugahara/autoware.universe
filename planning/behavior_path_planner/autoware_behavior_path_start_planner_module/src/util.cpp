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

#include "autoware/behavior_path_start_planner_module/util.hpp"

#include "autoware/behavior_path_planner_common/utils/path_shifter/path_shifter.hpp"
#include "autoware/behavior_path_planner_common/utils/path_utils.hpp"
#include "autoware/behavior_path_planner_common/utils/utils.hpp"

#include <autoware/motion_utils/trajectory/path_with_lane_id.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_utils/geometry/boost_geometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <boost/geometry/algorithms/dispatch/distance.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::behavior_path_planner::start_planner_utils
{
PathWithLaneId getBackwardPath(
  const RouteHandler & route_handler, const lanelet::ConstLanelets & shoulder_lanes,
  const Pose & current_pose, const Pose & backed_pose, const double velocity)
{
  const auto current_pose_arc_coords = lanelet::utils::getArcCoordinatesOnEgoCenterline(
    shoulder_lanes, current_pose, route_handler.getLaneletMapPtr());
  const auto backed_pose_arc_coords = lanelet::utils::getArcCoordinatesOnEgoCenterline(
    shoulder_lanes, backed_pose, route_handler.getLaneletMapPtr());

  const double s_start = backed_pose_arc_coords.length;
  const double s_end = current_pose_arc_coords.length;

  PathWithLaneId backward_path;
  {
    // forward center line path
    backward_path = route_handler.getCenterLinePath(shoulder_lanes, s_start, s_end, true);

    // If the returned path is empty, return an empty path
    if (backward_path.points.empty()) {
      return backward_path;
    }

    // backward center line path
    std::reverse(backward_path.points.begin(), backward_path.points.end());
    for (auto & p : backward_path.points) {
      p.point.longitudinal_velocity_mps = velocity;
    }
    backward_path.points.back().point.longitudinal_velocity_mps = 0.0;

    // lateral shift to current_pose
    const double lateral_distance_to_shoulder_center = current_pose_arc_coords.distance;
    for (size_t i = 0; i < backward_path.points.size(); ++i) {
      auto & p = backward_path.points.at(i).point.pose;
      p = autoware_utils::calc_offset_pose(p, 0, lateral_distance_to_shoulder_center, 0);
    }
  }

  return backward_path;
}

lanelet::ConstLanelets getPullOutLanes(
  const std::shared_ptr<const PlannerData> & planner_data, const double backward_length)
{
  const double & vehicle_width = planner_data->parameters.vehicle_width;
  const auto & route_handler = planner_data->route_handler;
  const auto start_pose = planner_data->route_handler->getOriginalStartPose();

  const auto current_shoulder_lane = route_handler->getPullOutStartLane(start_pose, vehicle_width);
  if (current_shoulder_lane) {
    // pull out from shoulder lane
    return route_handler->getShoulderLaneletSequence(*current_shoulder_lane, start_pose);
  }

  // pull out from road lane
  return utils::getExtendedCurrentLanes(
    planner_data, backward_length,
    /*forward_length*/ std::numeric_limits<double>::max(),
    /*forward_only_in_route*/ true);
}

std::optional<PathWithLaneId> extractCollisionCheckSection(
  const PullOutPath & path, const double collision_check_distance_from_end)
{
  PathWithLaneId full_path;
  for (const auto & partial_path : path.partial_paths) {
    full_path.points.insert(
      full_path.points.end(), partial_path.points.begin(), partial_path.points.end());
  }

  if (full_path.points.empty()) return std::nullopt;
  // Find the start index for collision check section based on the shift start pose
  const auto shift_start_idx =
    autoware::motion_utils::findNearestIndex(full_path.points, path.start_pose.position);

  // Find the end index for collision check section based on the end pose and collision check
  // distance
  const auto collision_check_end_idx = [&]() -> size_t {
    const auto end_pose_offset = autoware::motion_utils::calcLongitudinalOffsetPose(
      full_path.points, path.end_pose.position, collision_check_distance_from_end);

    return end_pose_offset
             ? autoware::motion_utils::findNearestIndex(full_path.points, end_pose_offset->position)
             : full_path.points.size() - 1;  // Use the last point if offset pose is not calculable
  }();

  // Extract the collision check section from the full path
  PathWithLaneId collision_check_section;
  if (shift_start_idx < collision_check_end_idx) {
    collision_check_section.points.assign(
      full_path.points.begin() + shift_start_idx,
      full_path.points.begin() + collision_check_end_idx + 1);
  }

  return collision_check_section;
}

double calc_necessary_longitudinal_distance(
  const double lateral_offset, const double minimum_radius)
{
  // 試行する縦方向距離の候補
  std::vector<double> distances_to_try = {
    0.5 * minimum_radius, 0.75 * minimum_radius, 1.0 * minimum_radius, 1.5 * minimum_radius,
    2.0 * minimum_radius, 3.0 * minimum_radius,  4.0 * minimum_radius, 5.0 * minimum_radius,
    6.0 * minimum_radius, 8.0 * minimum_radius,  10.0 * minimum_radius};

  std::cout << "\n--- Arc-based Analysis ---" << std::endl;

  double best_distance = 0.0;
  double best_score = -1e9;  // Arc1の長さを優先するため最大スコアを探索
  bool found_valid = false;
  int valid_results_count = 0;

  // 開始姿勢を原点・0度と仮定
  double x_start = 0.0, y_start = 0.0, yaw_start = 0.0;

  for (double trial_distance : distances_to_try) {
    // 目標位置を計算（開始姿勢に対して縦方向に移動）
    double x_goal = x_start + trial_distance * std::cos(yaw_start);
    double y_goal = y_start + trial_distance * std::sin(yaw_start);

    // 横方向偏差を追加
    x_goal += lateral_offset * (-std::sin(yaw_start));
    y_goal += lateral_offset * std::cos(yaw_start);

    // 開始円弧の中心を計算（右回りを想定）
    double C_rx = x_start + minimum_radius * std::sin(yaw_start);
    double C_ry = y_start - minimum_radius * std::cos(yaw_start);

    // std::cerr << "Trial distance: " << std::fixed << std::setprecision(2) << trial_distance
    //           << " m, x_goal: " << std::fixed << std::setprecision(2) << x_goal
    //           << " m, y_goal: " << std::fixed << std::setprecision(2) << y_goal << " m"
    //           << std::endl;
    // std::cerr << "C_rx: " << std::fixed << std::setprecision(2) << C_rx
    //           << " m, C_ry: " << std::fixed << std::setprecision(2) << C_ry << " m" << std::endl;

    // 目標円弧の半径を計算
    double dx_goal = x_goal - C_rx;
    double dy_goal = y_goal - C_ry;
    double distance_to_goal = std::sqrt(dx_goal * dx_goal + dy_goal * dy_goal);

    double cos_term = (y_goal - C_ry) / distance_to_goal;
    cos_term = std::clamp(cos_term, -1.0, 1.0);  // For numerical stability

    // # Adjust angle for goal approach (π added for reverse direction)
    // alpha = (yaw_goal + np.pi) + np.arccos(cos_term)
    // double alpha = std::acos(cos_term);
    // double alpha = (yaw_goal + M_PI) + std::acos(cos_term);
    double alpha = M_PI + std::acos(cos_term);
    const double denominator = 2 * minimum_radius + 2 * distance_to_goal * std::cos(alpha);
    const double radius_goal =
      (distance_to_goal * distance_to_goal - minimum_radius * minimum_radius) / denominator;
    // const double center_goal_x = x_goal + radius_goal * std::sin(yaw_start);
    // const double center_goal_y = y_goal - radius_goal * std::cos(yaw_start);
    // std::cerr << "Radius goal: " << std::fixed << std::setprecision(2) << radius_goal
    //           << " m, Center goal: (" << std::fixed << std::setprecision(2) << center_goal_x <<
    //           ", "
    //           << std::fixed << std::setprecision(2) << center_goal_y << ")" << std::endl;
    // std::cerr << "alpha: " << std::fixed << std::setprecision(2) << alpha
    //           << " rad, cos_term: " << std::fixed << std::setprecision(2) << cos_term <<
    //           std::endl;

    // 目標円弧の半径は、目標位置から開始円弧中心までの距離
    // double R_goal = distance_to_goal;
    // std::cerr << "Trial distance: " << std::fixed << std::setprecision(2) << trial_distance
    //           << "dx_goal: " << std::fixed << std::setprecision(2) << dx_goal
    //           << " m, dy_goal: " << std::fixed << std::setprecision(2) << dy_goal
    //           << " m, Distance to goal: " << std::fixed << std::setprecision(2) <<
    //           distance_to_goal
    //           << " m, R_goal: " << std::setprecision(2) << R_goal
    //           << " m, Lateral offset: " << std::setprecision(2) << lateral_offset << " m"
    //           << std::endl;

    // 接続不可能な場合をスキップ
    if (radius_goal < 0) {
      std::cout << "Warning: Calculated radius is negative (distance_to_goal: " << std::fixed
                << std::setprecision(3) << distance_to_goal << ")" << std::endl;
      std::cout << "  Trial distance: " << trial_distance << " m - SKIPPED (connection impossible)"
                << std::endl;
      continue;
    }

    if (radius_goal < minimum_radius) {
      std::cout << "Warning: Calculated radius is smaller than minimum (distance_to_goal: "
                << std::fixed << std::setprecision(3) << distance_to_goal
                << " < R_min: " << minimum_radius << ")" << std::endl;
      std::cout << "  Trial distance: " << trial_distance << " m - SKIPPED (connection impossible)"
                << std::endl;
      continue;
    }

    // 目標円弧の中心を計算（左回りを想定）
    double C_lx = x_goal - radius_goal * std::sin(yaw_start);
    double C_ly = y_goal + radius_goal * std::cos(yaw_start);
    // std::cerr << "C_lx: " << std::fixed << std::setprecision(2) << C_lx
    //           << " m, C_ly: " << std::fixed << std::setprecision(2) << C_ly << " m" << std::endl;

    // 円弧同士の接続状態をチェック
    double dx_centers = C_lx - C_rx;
    double dy_centers = C_ly - C_ry;
    double distance_between_centers = std::sqrt(dx_centers * dx_centers + dy_centers * dy_centers);

    // 接続判定
    double external_tangent_distance = minimum_radius + radius_goal;
    double internal_tangent_distance = std::abs(minimum_radius - radius_goal);
    double tolerance = 0.1;

    bool connection_valid = false;
    if (
      std::abs(distance_between_centers - external_tangent_distance) <= tolerance ||
      std::abs(distance_between_centers - internal_tangent_distance) <= tolerance ||
      (distance_between_centers > external_tangent_distance + tolerance &&
       distance_between_centers - external_tangent_distance <= 2.0) ||
      (distance_between_centers < internal_tangent_distance - tolerance &&
       internal_tangent_distance - distance_between_centers <=
         std::min(minimum_radius, radius_goal) * 0.8)) {
      connection_valid = true;
    }

    if (!connection_valid) {
      std::cout << "  Trial distance: " << std::fixed << std::setprecision(2) << trial_distance
                << " m - SKIPPED (arc connection invalid)" << std::endl;
      continue;
    }

    // 接線点を計算
    double tangent_x, tangent_y;
    if (std::abs(distance_between_centers - external_tangent_distance) <= tolerance) {
      // 外接の場合
      double ratio = minimum_radius / (minimum_radius + radius_goal);
      tangent_x = C_rx + ratio * dx_centers;
      tangent_y = C_ry + ratio * dy_centers;
    } else {
      // その他の場合の近似計算
      double ratio = 0.5;
      tangent_x = C_rx + ratio * dx_centers;
      tangent_y = C_ry + ratio * dy_centers;
    }

    // 実際の横方向偏差を計算
    double dx_actual = x_goal - x_start;
    double dy_actual = y_goal - y_start;
    double lateral_x = -std::sin(yaw_start);
    double lateral_y = std::cos(yaw_start);
    double actual_lateral_offset = dx_actual * lateral_x + dy_actual * lateral_y;

    double error = std::abs(actual_lateral_offset - lateral_offset);

    // Arc1の長さを計算
    double start_angle = std::atan2(y_start - C_ry, x_start - C_rx);
    double tangent_angle = std::atan2(tangent_y - C_ry, tangent_x - C_rx);
    double angle_diff = tangent_angle - start_angle;

    // 時計回りの角度調整
    if (angle_diff > 0) {
      angle_diff -= 2 * M_PI;
    }

    double arc1_length = minimum_radius * std::abs(angle_diff);

    // デバッグ出力
    // std::cout << "  Trial distance: " << std::fixed << std::setprecision(2) << trial_distance
    //           << " m, radius_goal: " << std::setprecision(2) << radius_goal
    //           << " m, Arc1 length: " << std::setprecision(3) << arc1_length
    //           << " m, Actual offset: " << std::setprecision(3) << actual_lateral_offset
    //           << " m, Error: " << std::setprecision(3) << error << " m" << std::endl;

    valid_results_count++;

    // 誤差が許容範囲内で、Arc1の長さが最大のものを選択
    double error_threshold = 0.5;
    if (error <= error_threshold) {
      if (arc1_length > best_score) {
        best_score = arc1_length;
        best_distance = trial_distance;
        found_valid = true;
      }
    } else if (!found_valid && arc1_length > best_score) {
      // 許容範囲内の解がない場合、最も良いものを選択
      best_score = arc1_length;
      best_distance = trial_distance;
    }
  }

  // 選択結果の出力
  std::cout << "\n--- Selection Results ---" << std::endl;
  std::cout << "Valid results: " << valid_results_count << std::endl;

  double error_threshold = 0.5;
  if (found_valid) {
    std::cout << "Acceptable results (error <= " << std::fixed << std::setprecision(1)
              << error_threshold << "m): found" << std::endl;
    std::cout << "Selected result: Arc1 length = " << std::setprecision(3) << best_score
              << " m, Distance = " << std::setprecision(3) << best_distance << " m" << std::endl;
  } else {
    std::cout << "No acceptable results found" << std::endl;
  }

  // 有効な解が見つからない場合のフォールバック
  if (!found_valid && best_distance == 0.0) {
    best_distance = std::max(4.0 * minimum_radius, std::abs(lateral_offset) * 2.0);
    std::cout << "Using geometric estimation: " << std::setprecision(3) << best_distance << " m"
              << std::endl;
  }

  return best_distance;
}

CompositeArcPath calc_circular_path(
  const Pose & start_pose, const double longitudinal_distance, const double lateral_distance,
  const double angle_diff, const double minimum_radius)
{
  const double PI = M_PI;

  std::cout << "\n=== Circular Path Planning (Relative Direct) ===" << std::endl;
  std::cout << std::fixed << std::setprecision(2);
  std::cout << "Start: (" << start_pose.position.x << ", " << start_pose.position.y
            << "), yaw=" << tf2::getYaw(start_pose.orientation) * 180.0 / PI << "°" << std::endl;
  std::cout << "Relative target: longitudinal=" << longitudinal_distance
            << "m, lateral=" << lateral_distance << "m, angle_diff=" << angle_diff * 180.0 / PI
            << "°" << std::endl;

  // 相対座標系で計算（原点を開始点、X軸を進行方向とする）
  // 開始点: (0, 0, 0)
  // 目標点: (longitudinal_distance, lateral_distance, angle_diff)

  const double x_start_rel = 0.0;
  const double y_start_rel = 0.0;
  const double yaw_start_rel = 0.0;

  const double x_goal_rel = longitudinal_distance;
  const double y_goal_rel = lateral_distance;
  const double yaw_goal_rel = angle_diff;

  std::cout << "Relative coordinates - Start: (" << x_start_rel << ", " << y_start_rel
            << "), Goal: (" << x_goal_rel << ", " << y_goal_rel << ")" << std::endl;

  // 開始円弧の中心を計算（右回りを想定）
  double C_rx_rel = x_start_rel + minimum_radius * std::sin(yaw_start_rel);
  double C_ry_rel = y_start_rel - minimum_radius * std::cos(yaw_start_rel);

  // 目標点から開始円弧中心までの距離
  double dx_goal_rel = x_goal_rel - C_rx_rel;
  double dy_goal_rel = y_goal_rel - C_ry_rel;
  double d_goal_Cr_rel = std::sqrt(dx_goal_rel * dx_goal_rel + dy_goal_rel * dy_goal_rel);

  if (d_goal_Cr_rel < 1e-6) {
    std::cout << "Warning: Goal is too close to start arc center (distance: "
              << std::setprecision(6) << d_goal_Cr_rel << ")" << std::endl;
    return CompositeArcPath();
  }

  // Al-Kashi定理を使用した半径計算
  double cos_term = dy_goal_rel / d_goal_Cr_rel;
  cos_term = std::max(-1.0, std::min(1.0, cos_term));

  // 目標への進入角度調整（逆方向なのでπを加算）
  double alpha = (yaw_goal_rel + PI) + std::acos(cos_term);

  double denominator = 2 * minimum_radius + 2 * d_goal_Cr_rel * std::cos(alpha);

  if (std::abs(denominator) < 1e-6) {
    std::cout << "Warning: Denominator too small (denominator: " << std::setprecision(6)
              << denominator << ")" << std::endl;
    return CompositeArcPath();
  }

  double R_goal = (d_goal_Cr_rel * d_goal_Cr_rel - minimum_radius * minimum_radius) / denominator;

  // 物理的に接続可能かチェック
  if (R_goal < 0) {
    std::cout << "Warning: Calculated radius is negative (R_goal: " << std::setprecision(3)
              << R_goal << ")" << std::endl;
    return CompositeArcPath();
  }

  if (R_goal < minimum_radius) {
    std::cout << "Warning: Calculated radius is smaller than minimum (R_goal: "
              << std::setprecision(3) << R_goal << " < R_min: " << minimum_radius << ")"
              << std::endl;
    return CompositeArcPath();
  }

  // 目標円弧の中心を計算（左回りを想定）
  double C_lx_rel = x_goal_rel - R_goal * std::sin(yaw_goal_rel);
  double C_ly_rel = y_goal_rel + R_goal * std::cos(yaw_goal_rel);

  // 接線点の計算
  double dx_centers = C_lx_rel - C_rx_rel;
  double dy_centers = C_ly_rel - C_ry_rel;
  double distance_centers = std::sqrt(dx_centers * dx_centers + dy_centers * dy_centers);

  double tangent_x_rel, tangent_y_rel;

  // 外接円の場合の接線点計算
  double external_tangent_distance = minimum_radius + R_goal;
  double tolerance = 0.01;

  if (distance_centers < 1e-6) {
    tangent_x_rel = (C_rx_rel + C_lx_rel) / 2.0;
    tangent_y_rel = (C_ry_rel + C_ly_rel) / 2.0;
  } else if (std::abs(distance_centers - external_tangent_distance) <= tolerance) {
    double ratio = minimum_radius / distance_centers;
    tangent_x_rel = C_rx_rel + ratio * dx_centers;
    tangent_y_rel = C_ry_rel + ratio * dy_centers;
  } else {
    double ratio = minimum_radius / distance_centers;
    tangent_x_rel = C_rx_rel + ratio * dx_centers;
    tangent_y_rel = C_ry_rel + ratio * dy_centers;
  }

  std::cout << "Relative arc centers: C_r=(" << C_rx_rel << ", " << C_ry_rel << "), C_l=("
            << C_lx_rel << ", " << C_ly_rel << ")" << std::endl;
  std::cout << "Radii: R_start=" << minimum_radius << " m, R_goal=" << R_goal << " m" << std::endl;
  std::cout << "Relative tangent point: (" << tangent_x_rel << ", " << tangent_y_rel << ")"
            << std::endl;

  // 第1円弧（開始点から接線点まで、時計回り）
  double start_angle1 = std::atan2(y_start_rel - C_ry_rel, x_start_rel - C_rx_rel);
  double end_angle1 = std::atan2(tangent_y_rel - C_ry_rel, tangent_x_rel - C_rx_rel);
  double angle_diff1 = end_angle1 - start_angle1;

  // 時計回りの角度調整
  if (angle_diff1 > 0) {
    angle_diff1 -= 2 * PI;
  }

  double arc1_length = minimum_radius * std::abs(angle_diff1);

  std::cout << "\n=== Arc Length Analysis ===" << std::endl;
  std::cout << std::setprecision(3);
  std::cout << "Arc 1 length: " << arc1_length << " m" << std::endl;

  // 第2円弧（接線点から目標点まで、反時計回り）
  double start_angle2 = std::atan2(tangent_y_rel - C_ly_rel, tangent_x_rel - C_lx_rel);
  double end_angle2 = std::atan2(y_goal_rel - C_ly_rel, x_goal_rel - C_lx_rel);
  double angle_diff2 = end_angle2 - start_angle2;

  // 反時計回りの角度調整
  if (angle_diff2 < 0) {
    angle_diff2 += 2 * PI;
  }

  double arc2_length = R_goal * std::abs(angle_diff2);
  std::cout << "Arc 2 length: " << arc2_length << " m" << std::endl;
  std::cout << "Total path length: " << arc1_length + arc2_length << " m" << std::endl;

  // グローバル座標系への変換のための準備
  const double start_yaw = tf2::getYaw(start_pose.orientation);
  const double cos_yaw = std::cos(start_yaw);
  const double sin_yaw = std::sin(start_yaw);

  // CompositeArcPathを作成
  CompositeArcPath composite_path;

  // 第1円弧セグメントを作成
  ArcSegment arc1;
  arc1.radius = minimum_radius;
  arc1.is_clockwise = true;

  // 相対座標系の中心をグローバル座標系に変換
  arc1.center.x = start_pose.position.x + C_rx_rel * cos_yaw - C_ry_rel * sin_yaw;
  arc1.center.y = start_pose.position.y + C_rx_rel * sin_yaw + C_ry_rel * cos_yaw;
  arc1.center.z = start_pose.position.z;

  // 開始姿勢と終了姿勢を設定
  arc1.start_pose = start_pose;

  // 接線点での姿勢を計算（グローバル座標系）
  geometry_msgs::msg::Pose tangent_pose;
  tangent_pose.position.x =
    start_pose.position.x + tangent_x_rel * cos_yaw - tangent_y_rel * sin_yaw;
  tangent_pose.position.y =
    start_pose.position.y + tangent_x_rel * sin_yaw + tangent_y_rel * cos_yaw;
  tangent_pose.position.z = start_pose.position.z;

  // 接線点での向きを計算（円弧の接線方向）
  double tangent_angle_global = end_angle1 + (arc1.is_clockwise ? -PI / 2 : PI / 2) + start_yaw;
  tangent_pose.orientation.x = 0.0;
  tangent_pose.orientation.y = 0.0;
  tangent_pose.orientation.z = std::sin(tangent_angle_global / 2.0);
  tangent_pose.orientation.w = std::cos(tangent_angle_global / 2.0);

  arc1.end_pose = tangent_pose;

  // 第2円弧セグメントを作成
  ArcSegment arc2;
  arc2.radius = R_goal;
  arc2.is_clockwise = false;

  // 相対座標系の中心をグローバル座標系に変換
  arc2.center.x = start_pose.position.x + C_lx_rel * cos_yaw - C_ly_rel * sin_yaw;
  arc2.center.y = start_pose.position.y + C_lx_rel * sin_yaw + C_ly_rel * cos_yaw;
  arc2.center.z = start_pose.position.z;

  // 開始姿勢（接線点）と終了姿勢（目標点）を設定
  arc2.start_pose = tangent_pose;

  // 目標姿勢を計算（グローバル座標系）
  geometry_msgs::msg::Pose goal_pose;
  goal_pose.position.x = start_pose.position.x + x_goal_rel * cos_yaw - y_goal_rel * sin_yaw;
  goal_pose.position.y = start_pose.position.y + x_goal_rel * sin_yaw + y_goal_rel * cos_yaw;
  goal_pose.position.z = start_pose.position.z;

  // 目標点での向きを計算
  double goal_yaw_global = start_yaw + yaw_goal_rel;
  goal_pose.orientation.x = 0.0;
  goal_pose.orientation.y = 0.0;
  goal_pose.orientation.z = std::sin(goal_yaw_global / 2.0);
  goal_pose.orientation.w = std::cos(goal_yaw_global / 2.0);

  arc2.end_pose = goal_pose;

  // セグメントを追加
  composite_path.segments.push_back(arc1);
  composite_path.segments.push_back(arc2);

  return composite_path;
}

autoware_planning_msgs::msg::Trajectory convertCircularPathToTrajectory(
  const CompositeArcPath & composite_arc_path, const double velocity, const double z)
{
  using autoware_planning_msgs::msg::Trajectory;
  using autoware_planning_msgs::msg::TrajectoryPoint;

  Trajectory trajectory;
  trajectory.header.stamp = rclcpp::Clock{RCL_ROS_TIME}.now();
  trajectory.header.frame_id = "map";

  if (composite_arc_path.segments.empty()) {
    return trajectory;
  }

  // CompositeArcPathから点群を生成
  std::vector<std::pair<double, double>> path_points;
  const int points_per_segment = 50;

  for (const auto & segment : composite_arc_path.segments) {
    // 各セグメントから点を生成
    for (int i = 0; i < points_per_segment; ++i) {
      // 最初のセグメント以外は最初の点をスキップ（重複回避）
      if (!path_points.empty() && i == 0) {
        continue;
      }

      double progress = static_cast<double>(i) / (points_per_segment - 1);

      // 開始角度と終了角度を計算
      double start_angle = segment.getStartAngle();
      double end_angle = segment.getEndAngle();
      double current_angle;

      if (segment.is_clockwise) {
        // 時計回りの場合の角度調整
        double angle_diff = end_angle - start_angle;
        if (angle_diff > 0) {
          angle_diff -= 2 * M_PI;
        }
        current_angle = start_angle + angle_diff * progress;
      } else {
        // 反時計回りの場合の角度調整
        double angle_diff = end_angle - start_angle;
        if (angle_diff < 0) {
          angle_diff += 2 * M_PI;
        }
        current_angle = start_angle + angle_diff * progress;
      }

      auto point = segment.getPointAtAngle(current_angle);
      path_points.push_back(std::make_pair(point.x, point.y));
    }
  }

  if (path_points.empty()) {
    return trajectory;
  }

  trajectory.points.reserve(path_points.size());

  for (size_t i = 0; i < path_points.size(); ++i) {
    TrajectoryPoint point;

    // 位置設定
    point.pose.position.x = path_points[i].first;
    point.pose.position.y = path_points[i].second;
    point.pose.position.z = z;

    // 方向設定（次の点への方向）
    if (i < path_points.size() - 1) {
      const double dx = path_points[i + 1].first - path_points[i].first;
      const double dy = path_points[i + 1].second - path_points[i].second;
      const double yaw = std::atan2(dy, dx);
      point.pose.orientation = autoware_utils::create_quaternion_from_yaw(yaw);
    } else {
      // 最後の点は前の点と同じ方向
      if (i > 0) {
        const double dx = path_points[i].first - path_points[i - 1].first;
        const double dy = path_points[i].second - path_points[i - 1].second;
        const double yaw = std::atan2(dy, dx);
        point.pose.orientation = autoware_utils::create_quaternion_from_yaw(yaw);
      } else {
        point.pose.orientation = autoware_utils::create_quaternion_from_yaw(0.0);
      }
    }

    // 速度設定
    point.longitudinal_velocity_mps = velocity;
    point.lateral_velocity_mps = 0.0;
    point.acceleration_mps2 = 0.0;
    point.heading_rate_rps = 0.0;
    point.front_wheel_angle_rad = 0.0;
    point.rear_wheel_angle_rad = 0.0;

    // 時間設定
    if (i == 0) {
      point.time_from_start.sec = 0;
      point.time_from_start.nanosec = 0;
    } else {
      const double distance = std::sqrt(
        std::pow(path_points[i].first - path_points[i - 1].first, 2) +
        std::pow(path_points[i].second - path_points[i - 1].second, 2));
      const double time_diff = distance / velocity;

      // builtin_interfaces::msg::Durationを使用
      const auto prev_time = trajectory.points[i - 1].time_from_start;
      const auto time_diff_sec = static_cast<int32_t>(time_diff);
      const auto time_diff_nanosec = static_cast<uint32_t>((time_diff - time_diff_sec) * 1e9);

      point.time_from_start.sec = prev_time.sec + time_diff_sec;
      point.time_from_start.nanosec = prev_time.nanosec + time_diff_nanosec;

      // ナノ秒のオーバーフロー処理
      if (point.time_from_start.nanosec >= 1000000000) {
        point.time_from_start.sec += 1;
        point.time_from_start.nanosec -= 1000000000;
      }
    }

    trajectory.points.push_back(point);
  }

  return trajectory;
}

std::vector<double> calcCurvatureFromTrajectory(
  const autoware_planning_msgs::msg::Trajectory & trajectory)
{
  using autoware_utils::calc_curvature;

  std::vector<double> curvatures;

  if (trajectory.points.size() < 3) {
    // 点が3つ未満の場合は曲率を計算できない
    curvatures.resize(trajectory.points.size(), 0.0);
    return curvatures;
  }

  curvatures.reserve(trajectory.points.size());

  for (size_t i = 0; i < trajectory.points.size(); ++i) {
    if (i == 0) {
      // 最初の点：次の2点を使用
      const auto & p1 = trajectory.points[0].pose.position;
      const auto & p2 = trajectory.points[1].pose.position;
      const auto & p3 = trajectory.points[2].pose.position;
      curvatures.push_back(calc_curvature(p1, p2, p3));
    } else if (i == trajectory.points.size() - 1) {
      // 最後の点：前の2点を使用
      const auto & p1 = trajectory.points[i - 2].pose.position;
      const auto & p2 = trajectory.points[i - 1].pose.position;
      const auto & p3 = trajectory.points[i].pose.position;
      curvatures.push_back(calc_curvature(p1, p2, p3));
    } else {
      // 中間の点：前後の点を使用
      const auto & p1 = trajectory.points[i - 1].pose.position;
      const auto & p2 = trajectory.points[i].pose.position;
      const auto & p3 = trajectory.points[i + 1].pose.position;
      curvatures.push_back(calc_curvature(p1, p2, p3));
    }
  }

  return curvatures;
}

std::vector<double> calcCurvatureFromPoints(const std::vector<geometry_msgs::msg::Point> & points)
{
  using autoware_utils::calc_curvature;

  std::vector<double> curvatures;

  if (points.size() < 3) {
    // 点が3つ未満の場合は曲率を計算できない
    curvatures.resize(points.size(), 0.0);
    return curvatures;
  }

  curvatures.reserve(points.size());

  for (size_t i = 0; i < points.size(); ++i) {
    try {
      if (i == 0) {
        // 最初の点：次の2点を使用
        const auto & p1 = points[0];
        const auto & p2 = points[1];
        const auto & p3 = points[2];
        curvatures.push_back(calc_curvature(p1, p2, p3));
      } else if (i == points.size() - 1) {
        // 最後の点：前の2点を使用
        const auto & p1 = points[i - 2];
        const auto & p2 = points[i - 1];
        const auto & p3 = points[i];
        curvatures.push_back(calc_curvature(p1, p2, p3));
      } else {
        // 中間の点：前後の点を使用
        const auto & p1 = points[i - 1];
        const auto & p2 = points[i];
        const auto & p3 = points[i + 1];
        curvatures.push_back(calc_curvature(p1, p2, p3));
      }
    } catch (const std::runtime_error & e) {
      // 点が近すぎる場合は曲率を0とする
      curvatures.push_back(0.0);
    }
  }

  return curvatures;
}

Pose findTargetPoseAlongPath(
  const PathWithLaneId & centerline_path, const Pose & start_pose,
  const double longitudinal_distance)
{
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

  return target_pose;
}

RelativePoseInfo calculateRelativePoseInVehicleCoordinate(
  const Pose & start_pose, const Pose & target_pose)
{
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

  return {longitudinal_distance_vehicle, lateral_distance_vehicle, angle_diff};
}

}  // namespace autoware::behavior_path_planner::start_planner_utils
