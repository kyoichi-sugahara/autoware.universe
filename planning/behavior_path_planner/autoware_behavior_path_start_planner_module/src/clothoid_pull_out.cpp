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
#include "autoware/behavior_path_start_planner_module/pull_out_path.hpp"
#include "autoware/behavior_path_start_planner_module/util.hpp"
#include "autoware/motion_utils/trajectory/path_with_lane_id.hpp"
#include "autoware_utils/geometry/boost_polygon_utils.hpp"

#include <autoware/motion_utils/trajectory/path_shift.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <numeric>
#include <optional>
#include <string>
#include <utility>
#include <vector>

using autoware::motion_utils::findNearestIndex;
using autoware_utils::calc_distance2d;
using autoware_utils::calc_offset_pose;
using lanelet::utils::getArcCoordinates;
namespace autoware::behavior_path_planner
{
using start_planner_utils::getPullOutLanes;

/**
 * @brief 剛体変換（回転・平行移動・スケーリング）のみでクロソイドを補正
 * @param clothoid_points クロソイド変換後の点列
 * @param original_segment 元の円弧セグメント
 * @param start_pose セグメントの開始姿勢
 * @return 補正後の点列
 */
std::vector<geometry_msgs::msg::Point> correctClothoidByRigidTransform(
  const std::vector<geometry_msgs::msg::Point> & clothoid_points,
  const ArcSegment & original_segment, const geometry_msgs::msg::Pose & start_pose)
{
  if (clothoid_points.size() < 2) {
    return clothoid_points;
  }

  auto clothoid_start = clothoid_points.front();
  auto clothoid_end = clothoid_points.back();

  // 目標の開始・終了位置を取得
  auto target_start = start_pose.position;
  auto target_end = original_segment.getPointAtAngle(original_segment.getEndAngle());

  // 2. 方向ベクトルを計算
  double clothoid_dx = clothoid_end.x - clothoid_start.x;
  double clothoid_dy = clothoid_end.y - clothoid_start.y;
  double clothoid_length = std::sqrt(clothoid_dx * clothoid_dx + clothoid_dy * clothoid_dy);

  double target_dx = target_end.x - target_start.x;
  double target_dy = target_end.y - target_start.y;
  double target_length = std::sqrt(target_dx * target_dx + target_dy * target_dy);

  // 3. スケーリング係数を計算
  double scale_factor = (clothoid_length > 1e-10) ? target_length / clothoid_length : 1.0;

  // 4. 回転角度を計算
  double clothoid_angle = std::atan2(clothoid_dy, clothoid_dx);
  double target_angle = std::atan2(target_dy, target_dx);
  double rotation_angle = target_angle - clothoid_angle;

  // 角度を [-π, π] の範囲に正規化
  while (rotation_angle > M_PI) rotation_angle -= 2 * M_PI;
  while (rotation_angle < -M_PI) rotation_angle += 2 * M_PI;

  // 5. 変換行列の要素を計算
  double cos_theta = std::cos(rotation_angle);
  double sin_theta = std::sin(rotation_angle);

  // 6. 剛体変換を適用
  std::vector<geometry_msgs::msg::Point> corrected_points;
  corrected_points.reserve(clothoid_points.size());

  for (size_t i = 0; i < clothoid_points.size(); ++i) {
    geometry_msgs::msg::Point corrected_point;

    // 始点を原点に移動
    double rel_x = clothoid_points[i].x - clothoid_start.x;
    double rel_y = clothoid_points[i].y - clothoid_start.y;

    // スケーリング
    rel_x *= scale_factor;
    rel_y *= scale_factor;

    // 回転
    double rotated_x = cos_theta * rel_x - sin_theta * rel_y;
    double rotated_y = sin_theta * rel_x + cos_theta * rel_y;

    // 目標始点に平行移動
    corrected_point.x = rotated_x + target_start.x;
    corrected_point.y = rotated_y + target_start.y;
    corrected_point.z = clothoid_points[i].z;  // Z座標はそのまま

    corrected_points.push_back(corrected_point);
  }

  return corrected_points;
}

/**
 * @brief エントリクロソイドセグメントを生成（数値積分版）
 */
std::pair<std::vector<geometry_msgs::msg::Point>, geometry_msgs::msg::Pose> generateClothoidEntry(
  const ClothoidSegment & segment, const geometry_msgs::msg::Pose & start_pose, int num_points)
{
  double A = segment.A;
  double L = segment.L;
  double direction_factor = segment.is_clockwise ? -1.0 : 1.0;
  double start_yaw = tf2::getYaw(start_pose.orientation);

  std::vector<geometry_msgs::msg::Point> points;

  // Entry Clothoid: 曲率を0から目標曲率まで線形に増加させる
  double target_curvature = (L / (A * A)) * direction_factor;
  double start_curvature = 0.0;

  // 数値積分による正確な計算
  double current_x = start_pose.position.x;
  double current_y = start_pose.position.y;
  double current_psi = start_yaw;

  for (int i = 0; i < num_points; ++i) {
    geometry_msgs::msg::Point point;
    point.x = current_x;
    point.y = current_y;
    point.z = 0.0;
    points.push_back(point);

    double progress = static_cast<double>(i) / (num_points - 1);
    // Entry Clothoid: 曲率を線形に0から目標曲率まで増加させる
    double current_curvature = start_curvature + (target_curvature - start_curvature) * progress;

    if (i < num_points - 1) {
      double ds = L / (num_points - 1);  // 微小区間

      // 数値積分による座標更新
      current_x += std::cos(current_psi) * ds;
      current_y += std::sin(current_psi) * ds;
      current_psi += current_curvature * ds;
    }
  }

  // 終端状態
  double final_psi = current_psi;

  geometry_msgs::msg::Pose end_pose;
  end_pose.position = points.back();
  end_pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), final_psi));

  return {points, end_pose};
}

/**
 * @brief 円弧セグメントを生成
 */
std::pair<std::vector<geometry_msgs::msg::Point>, geometry_msgs::msg::Pose> generateCircularSegment(
  const ClothoidSegment & segment, const geometry_msgs::msg::Pose & start_pose, int num_points)
{
  double radius = segment.radius;
  double angle = segment.angle;
  double direction_factor = segment.is_clockwise ? -1.0 : 1.0;
  double start_yaw = tf2::getYaw(start_pose.orientation);

  std::vector<geometry_msgs::msg::Point> points;

  // 円弧中心計算
  double center_x = start_pose.position.x - radius * std::sin(start_yaw) * direction_factor;
  double center_y = start_pose.position.y + radius * std::cos(start_yaw) * direction_factor;

  for (int i = 0; i < num_points; ++i) {
    double progress = static_cast<double>(i) / (num_points - 1);
    double angle_progress = angle * progress * direction_factor;
    double current_psi = start_yaw + angle_progress;
    double angle_from_center = current_psi - M_PI / 2.0 * direction_factor;

    geometry_msgs::msg::Point point;
    point.x = center_x + radius * std::cos(angle_from_center);
    point.y = center_y + radius * std::sin(angle_from_center);
    point.z = 0.0;

    points.push_back(point);
  }

  // 終端状態
  double final_psi = start_yaw + angle * direction_factor;

  geometry_msgs::msg::Pose end_pose;
  end_pose.position = points.back();
  end_pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), final_psi));

  return {points, end_pose};
}

/**
 * @brief エグジットクロソイドセグメントを生成
 */
std::pair<std::vector<geometry_msgs::msg::Point>, geometry_msgs::msg::Pose> generateClothoidExit(
  const ClothoidSegment & segment, const geometry_msgs::msg::Pose & start_pose, int num_points)
{
  double L = segment.L;
  double start_yaw = tf2::getYaw(start_pose.orientation);
  double direction_factor = segment.is_clockwise ? -1.0 : 1.0;

  std::vector<geometry_msgs::msg::Point> points;

  // 前のセグメント（円弧）の曲率を計算（回転方向を考慮）
  double start_curvature = (1.0 / segment.radius) * direction_factor;

  // 数値積分による正確な計算
  double current_x = start_pose.position.x;
  double current_y = start_pose.position.y;
  double current_psi = start_yaw;

  for (int i = 0; i < num_points; ++i) {
    geometry_msgs::msg::Point point;
    point.x = current_x;
    point.y = current_y;
    point.z = 0.0;
    points.push_back(point);

    double progress = static_cast<double>(i) / (num_points - 1);
    // Exit Clothoid: 曲率を線形に0まで減少させる
    double current_curvature = start_curvature * (1.0 - progress);

    if (i < num_points - 1) {
      double ds = L / (num_points - 1);  // 微小区間

      // 数値積分による座標更新
      current_x += std::cos(current_psi) * ds;
      current_y += std::sin(current_psi) * ds;
      current_psi += current_curvature * ds;
    }
  }

  // 終端状態
  geometry_msgs::msg::Pose end_pose;
  end_pose.position.x = current_x;
  end_pose.position.y = current_y;
  end_pose.position.z = start_pose.position.z;
  end_pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), current_psi));

  return {points, end_pose};
}

/**
 * @brief クロソイド経路の座標点列を生成
 */
std::vector<geometry_msgs::msg::Point> generateClothoidPath(
  const std::vector<ClothoidSegment> & segments, int num_points_per_segment,
  const geometry_msgs::msg::Pose & start_pose)
{
  // 各セグメントの理論弧長を計算
  std::vector<double> theoretical_lengths;
  for (const auto & segment : segments) {
    if (
      segment.type == ClothoidSegment::CLOTHOID_ENTRY ||
      segment.type == ClothoidSegment::CLOTHOID_EXIT) {
      theoretical_lengths.push_back(segment.L);
    } else if (segment.type == ClothoidSegment::CIRCULAR_ARC) {
      theoretical_lengths.push_back(segment.radius * segment.angle);
    }
  }

  double total_theoretical_length = 0.0;
  for (double length : theoretical_lengths) {
    total_theoretical_length += length;
  }

  // セグメント毎の点数を弧長に比例して調整
  int total_points = num_points_per_segment * segments.size();
  std::vector<int> adjusted_points;
  for (double length : theoretical_lengths) {
    int points = std::max(10, static_cast<int>(total_points * length / total_theoretical_length));
    adjusted_points.push_back(points);
  }

  // 初期状態の設定
  geometry_msgs::msg::Pose current_pose = start_pose;

  std::vector<geometry_msgs::msg::Point> all_points;

  for (size_t i = 0; i < segments.size(); ++i) {
    std::vector<geometry_msgs::msg::Point> segment_points;
    geometry_msgs::msg::Pose end_pose;

    int num_points = adjusted_points[i];

    if (segments[i].type == ClothoidSegment::CLOTHOID_ENTRY) {
      auto result = generateClothoidEntry(segments[i], current_pose, num_points);
      segment_points = result.first;
      end_pose = result.second;
    } else if (segments[i].type == ClothoidSegment::CIRCULAR_ARC) {
      auto result = generateCircularSegment(segments[i], current_pose, num_points);
      segment_points = result.first;
      end_pose = result.second;
    } else if (segments[i].type == ClothoidSegment::CLOTHOID_EXIT) {
      auto result = generateClothoidExit(segments[i], current_pose, num_points);
      segment_points = result.first;
      end_pose = result.second;
    }

    // 重複点を避けて結合
    size_t start_idx = (all_points.empty()) ? 0 : 1;
    for (size_t j = start_idx; j < segment_points.size(); ++j) {
      all_points.push_back(segment_points[j]);
    }

    current_pose = end_pose;
  }

  return all_points;
}

/**
 * @brief ArcSegmentをクロソイド曲線に変換する
 */
std::vector<geometry_msgs::msg::Point> convertArcToClothoid(
  const ArcSegment & arc_segment, const geometry_msgs::msg::Pose & start_pose, double A_min,
  double L_min, int num_points_per_segment)
{
  // 円弧情報の抽出
  double start_angle = arc_segment.getStartAngle();
  double end_angle = arc_segment.getEndAngle();

  double total_angle = std::abs(end_angle - start_angle);

  if (total_angle > M_PI) {
    total_angle = 2.0 * M_PI - total_angle;
  }

  double radius = arc_segment.radius;
  bool is_clockwise = arc_segment.is_clockwise;

  // クロソイドパラメータ
  double A = A_min;
  double L = L_min;
  double alpha_clothoid = (L * L) / (2.0 * A * A);  // 単一クロソイドの角度変化

  std::vector<ClothoidSegment> segments;

  if (total_angle >= 2.0 * alpha_clothoid) {
    // Case A: CAC(A, L, θ)
    double theta_arc = total_angle - 2.0 * alpha_clothoid;

    // エントリクロソイド
    ClothoidSegment entry(ClothoidSegment::CLOTHOID_ENTRY, A, L);
    entry.radius = radius;
    entry.is_clockwise = is_clockwise;
    entry.description = "Entry clothoid (κ: 0 → 1/R)";
    segments.push_back(entry);

    // 円弧セグメント
    ClothoidSegment circular(ClothoidSegment::CIRCULAR_ARC);
    circular.radius = radius;
    circular.angle = theta_arc;
    circular.is_clockwise = is_clockwise;
    circular.description = "Circular arc (κ = 1/R = " + std::to_string(1.0 / radius) + ")";
    segments.push_back(circular);

    // エグジットクロソイド
    ClothoidSegment exit(ClothoidSegment::CLOTHOID_EXIT, A, L);
    exit.radius = radius;
    exit.is_clockwise = is_clockwise;
    exit.description = "Exit clothoid (κ: 1/R → 0)";
    segments.push_back(exit);
  } else {
    std::cerr << "Case B is not implemented. Please use Case A conditions." << std::endl;
    return {};
  }

  // クロソイド経路生成
  std::vector<geometry_msgs::msg::Point> clothoid_path =
    generateClothoidPath(segments, num_points_per_segment, start_pose);

  return clothoid_path;
}

/**
 * @brief 改良版のクロソイド変換関数（終点補正付き）
 */
std::vector<geometry_msgs::msg::Point> convertArcToClothoidWithCorrection(
  const ArcSegment & arc_segment, const geometry_msgs::msg::Pose & start_pose, double A_min,
  double L_min, int num_points_per_segment)
{
  // 元のクロソイド変換を実行
  auto clothoid_points =
    convertArcToClothoid(arc_segment, start_pose, A_min, L_min, num_points_per_segment);

  if (clothoid_points.empty()) {
    std::cerr << "Clothoid conversion failed!" << std::endl;
    return clothoid_points;
  }

  // 終点補正を適用
  auto corrected_points = correctClothoidByRigidTransform(clothoid_points, arc_segment, start_pose);

  return corrected_points;
}

/**
 * @brief クロソイドパスからPathWithLaneIdを生成する関数
 * @param clothoid_paths クロソイドパスの配列
 * @param target_pose 目標姿勢
 * @param velocity 速度
 * @param road_lanes 道路レーン情報
 * @param route_handler ルートハンドラー
 * @return PathWithLaneId
 */
PathWithLaneId createPathWithLaneIdFromClothoidPaths(
  const std::vector<std::vector<geometry_msgs::msg::Point>> & clothoid_paths,
  const geometry_msgs::msg::Pose & target_pose, double velocity,
  const lanelet::ConstLanelets & road_lanes,
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler)
{
  // 全てのクロソイドパスを結合
  std::vector<geometry_msgs::msg::Point> all_clothoid_points;
  for (const auto & path : clothoid_paths) {
    // 最初のパス以外は最初の点をスキップ（重複回避）
    size_t start_idx = (all_clothoid_points.empty()) ? 0 : 1;
    for (size_t j = start_idx; j < path.size(); ++j) {
      all_clothoid_points.push_back(path[j]);
    }
  }

  // PathWithLaneIdを作成
  PathWithLaneId path_with_lane_id;
  path_with_lane_id.header = route_handler->getRouteHeader();

  // 各座標点をPathPointWithLaneIdに変換
  for (size_t i = 0; i < all_clothoid_points.size(); ++i) {
    PathPointWithLaneId path_point;

    // 座標設定
    path_point.point.pose.position = all_clothoid_points[i];

    // 向きを計算（次の点への方向）
    if (i < all_clothoid_points.size() - 1) {
      const double dx = all_clothoid_points[i + 1].x - all_clothoid_points[i].x;
      const double dy = all_clothoid_points[i + 1].y - all_clothoid_points[i].y;
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
    path_point.point.longitudinal_velocity_mps = velocity;
    path_point.point.lateral_velocity_mps = 0.0;
    path_point.point.heading_rate_rps = 0.0;
    path_point.point.is_final = (i == all_clothoid_points.size() - 1);

    // レーンIDの設定
    lanelet::Lanelet closest_lanelet{};
    bool found_containing_lane = false;

    for (const auto & lane : road_lanes) {
      if (lanelet::utils::isInLanelet(path_point.point.pose, lane)) {
        path_point.lane_ids.push_back(lane.id());
        found_containing_lane = true;
      }
    }

    if (!found_containing_lane) {
      if (lanelet::utils::query::getClosestLanelet(
            road_lanes, path_point.point.pose, &closest_lanelet)) {
        path_point.lane_ids = {closest_lanelet.id()};
      } else if (i > 0) {
        // 前の点のlane_idsを継承
        path_point.lane_ids = path_with_lane_id.points[i - 1].lane_ids;
      } else if (!road_lanes.empty()) {
        // 最後のフォールバック
        path_point.lane_ids.push_back(road_lanes[0].id());
      }
    }

    path_with_lane_id.points.push_back(path_point);
  }

  return path_with_lane_id;
}

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
  // TODO(Sugahara): define as parameter
  const std::vector<double> max_steer_angle_degs = {5.0, 10.0, 20.0};
  const std::vector<double> max_steer_angle = {
    max_steer_angle_degs[0] * M_PI / 180.0, max_steer_angle_degs[1] * M_PI / 180.0};
  // const std::vector<double> max_steer_angle_degs = {20.0, 30.0, 40.0, 50.0, 60.0};
  // const std::vector<double> max_steer_angle = {
  //   max_steer_angle_degs[0] * M_PI / 180.0, max_steer_angle_degs[1] * M_PI / 180.0,
  //   max_steer_angle_degs[2] * M_PI / 180.0, max_steer_angle_degs[3] * M_PI / 180.0,
  //   max_steer_angle_degs[4] * M_PI / 180.0};

  const double max_steer_angle_rate_deg_per_sec = 10.0;  // Assume a constant rate for simplicity
  const double max_steer_angle_rate = max_steer_angle_rate_deg_per_sec * M_PI / 180.0;
  // TODO(Sugahara): define as parameter
  const double velocity = 1.0;  // Assume a constant velocity for the pull-out maneuver
  const double wheel_base = planner_data->parameters.vehicle_info.wheel_base_m;

  for (const auto & steer_angle : max_steer_angle) {
    // Calculate minimum radius based on the maximum steer angle
    const double minimum_radius = wheel_base / std::tan(steer_angle);

    const double longitudinal_distance =
      start_planner_utils::calc_necessary_longitudinal_distance(-lateral_offset, minimum_radius);

    const Pose target_pose = start_planner_utils::findTargetPoseAlongPath(
      centerline_path, start_pose, longitudinal_distance);

    const auto relative_pose_info =
      start_planner_utils::calculateRelativePoseInVehicleCoordinate(start_pose, target_pose);

    const auto circular_path = start_planner_utils::calc_circular_path(
      start_pose, relative_pose_info.longitudinal_distance_vehicle,
      relative_pose_info.lateral_distance_vehicle, relative_pose_info.angle_diff, minimum_radius);

    if (circular_path.segments.empty()) {
      std::cerr << "No circular path segments found for steer angle " << steer_angle * 180.0 / M_PI
                << " deg." << std::endl;
      continue;
    }

    // 車両パラメータから最適なクロソイドパラメータを計算
    const double circular_steer_angle = std::atan(wheel_base / minimum_radius);
    const double minimum_steer_time = circular_steer_angle / max_steer_angle_rate;
    const double L_min = velocity * minimum_steer_time;
    const double A_min = std::sqrt(minimum_radius * L_min);

    // セグメント間の連続性を保つための姿勢管理
    geometry_msgs::msg::Pose current_segment_pose = start_pose;
    std::vector<std::vector<geometry_msgs::msg::Point>> clothoid_paths;

    for (size_t i = 0; i < circular_path.segments.size(); ++i) {
      const auto & segment = circular_path.segments[i];

      // クロソイド変換を実行
      auto clothoid_points =
        convertArcToClothoidWithCorrection(segment, current_segment_pose, A_min, L_min, 50);

      if (!clothoid_points.empty()) {
        clothoid_paths.push_back(clothoid_points);

        // 次のセグメントのために終点姿勢を更新
        if (i < circular_path.segments.size() - 1) {
          const auto & last_point = clothoid_points.back();

          // 終点での進行方向を計算（最後の2点から）
          if (clothoid_points.size() >= 2) {
            const auto & second_last = clothoid_points[clothoid_points.size() - 2];
            double dx = last_point.x - second_last.x;
            double dy = last_point.y - second_last.y;
            double heading = std::atan2(dy, dx);

            current_segment_pose.position = last_point;
            current_segment_pose.orientation =
              tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), heading));
          }
        }
      } else {
        std::cerr << "Failed to convert segment " << (i + 1) << " to clothoid" << std::endl;
      }
    }

    // クロソイドパスが生成された場合、PathWithLaneIdを作成
    if (!clothoid_paths.empty()) {
      // PathWithLaneIdを作成
      PathWithLaneId path_with_lane_id = createPathWithLaneIdFromClothoidPaths(
        clothoid_paths, target_pose, velocity, road_lanes, route_handler);

      // PullOutPathを作成
      PullOutPath pull_out_path;
      pull_out_path.partial_paths.push_back(path_with_lane_id);
      pull_out_path.start_pose = start_pose;
      pull_out_path.end_pose = target_pose;

      // 速度と加速度のペア設定
      // TODO(Sugahara): set parameter properly
      pull_out_path.pairs_terminal_velocity_and_accel.push_back(std::make_pair(velocity, 1.0));

      // target_poseからcenter lineのpathに接続する処理を追加
      if (!centerline_path.points.empty()) {
        // target_poseの位置でcenterline_pathから接続点を見つける
        const auto target_idx =
          autoware::motion_utils::findNearestIndex(centerline_path.points, target_pose.position);

        // target_poseから先のcenterline pathを取得
        if (target_idx < centerline_path.points.size()) {
          PathWithLaneId centerline_extension;
          centerline_extension.header = centerline_path.header;

          // target_poseから先の点をcenterline_extensionに追加
          for (size_t i = target_idx; i < centerline_path.points.size(); ++i) {
            centerline_extension.points.push_back(centerline_path.points[i]);
          }

          // centerline extensionが存在する場合、既存のpathと結合
          if (!centerline_extension.points.empty()) {
            // 重複点を避けて結合
            auto combined_path = utils::combinePath(path_with_lane_id, centerline_extension);

            // 結合されたpathでPullOutPathを更新
            pull_out_path.partial_paths.clear();
            pull_out_path.partial_paths.push_back(combined_path);
          }
        }
      }

      // TODO(Sugahara): check lane departure
      return pull_out_path;
    }
  }

  // 経路が生成できなかった場合
  return std::nullopt;
}

}  // namespace autoware::behavior_path_planner
