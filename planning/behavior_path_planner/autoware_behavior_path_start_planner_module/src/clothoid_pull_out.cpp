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

#include <autoware/interpolation/linear_interpolation.hpp>
#include <autoware/motion_utils/trajectory/path_shift.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_utils/geometry/geometry.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <angles/angles.h>
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
 * @param velocity 初期速度
 * @param target_velocity 目標速度
 * @param road_lanes 道路レーン情報
 * @param route_handler ルートハンドラー
 * @param parameters パラメータ
 * @return PathWithLaneId
 */
PathWithLaneId createPathWithLaneIdFromClothoidPaths(
  const std::vector<std::vector<geometry_msgs::msg::Point>> & clothoid_paths,
  const geometry_msgs::msg::Pose & target_pose, double velocity, double target_velocity,
  const lanelet::ConstLanelets & road_lanes,
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler)
{
  (void)target_pose;  // unused parameter警告抑制
  // クロソイドパスが空の場合は空のPathWithLaneIdを返す
  if (clothoid_paths.empty()) {
    PathWithLaneId empty_path;
    empty_path.header = route_handler->getRouteHeader();
    return empty_path;
  }

  // 全てのクロソイドパスを結合
  std::vector<geometry_msgs::msg::Point> all_clothoid_points;
  for (const auto & path : clothoid_paths) {
    // パスが空でない場合のみ処理
    if (path.empty()) {
      continue;
    }

    // 最初のパス以外は最初の点をスキップ（重複回避）
    size_t start_idx = (all_clothoid_points.empty()) ? 0 : 1;
    for (size_t j = start_idx; j < path.size(); ++j) {
      all_clothoid_points.push_back(path[j]);
    }
  }

  // 結合後も空の場合は空のPathWithLaneIdを返す
  if (all_clothoid_points.empty()) {
    PathWithLaneId empty_path;
    empty_path.header = route_handler->getRouteHeader();
    return empty_path;
  }

  // 一定加速度の設定（パラメータとして設定可能）
  const double acceleration = 3.0;  // [m/s^2] - パラメータ化することを推奨

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
      // 最後の点も同様に、前の点との方向から計算
      if (all_clothoid_points.size() >= 2) {
        const double dx = all_clothoid_points[i].x - all_clothoid_points[i - 1].x;
        const double dy = all_clothoid_points[i].y - all_clothoid_points[i - 1].y;
        const double yaw = std::atan2(dy, dx);
        path_point.point.pose.orientation.x = 0.0;
        path_point.point.pose.orientation.y = 0.0;
        path_point.point.pose.orientation.z = std::sin(yaw / 2.0);
        path_point.point.pose.orientation.w = std::cos(yaw / 2.0);
      } else {
        // 1点しかない場合は0
        path_point.point.pose.orientation.x = 0.0;
        path_point.point.pose.orientation.y = 0.0;
        path_point.point.pose.orientation.z = 0.0;
        path_point.point.pose.orientation.w = 1.0;
      }
    }

    // 速度プロファイルの計算（一定加速度で加速）
    double current_velocity;
    if (i == 0) {
      // 最初の点は初期速度
      current_velocity = velocity;
    } else {
      // 累積距離を計算
      double accumulated_distance = 0.0;
      for (size_t j = 0; j < i; ++j) {
        const double dx = all_clothoid_points[j + 1].x - all_clothoid_points[j].x;
        const double dy = all_clothoid_points[j + 1].y - all_clothoid_points[j].y;
        accumulated_distance += std::sqrt(dx * dx + dy * dy);
      }

      // 等加速度運動の公式: v^2 = v0^2 + 2*a*s
      // ただし、目標速度を超えないように制限
      double calculated_velocity =
        std::sqrt(velocity * velocity + 2.0 * acceleration * accumulated_distance);
      current_velocity = std::min(calculated_velocity, target_velocity);
    }

    // 速度設定
    path_point.point.longitudinal_velocity_mps = current_velocity;
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

  // 点間隔を揃えるためにリサンプリング
  return autoware::behavior_path_planner::utils::resamplePathWithSpline(path_with_lane_id, 1.0);
}

/**
 * @brief センターラインパスとクロソイドパスを結合する関数
 */
PathWithLaneId combinePathWithCenterline(
  const PathWithLaneId & clothoid_path, const PathWithLaneId & centerline_path,
  const geometry_msgs::msg::Pose & target_pose)
{
  // センターラインパスが空の場合はクロソイドパスをそのまま返す
  if (centerline_path.points.empty()) {
    return clothoid_path;
  }

  // target_poseの位置でcenterline_pathから接続点を見つける
  const auto target_idx =
    autoware::motion_utils::findNearestIndex(centerline_path.points, target_pose.position);

  // target_poseから先のcenterline pathを取得
  if (target_idx < centerline_path.points.size()) {
    PathWithLaneId centerline_extension;
    centerline_extension.header = centerline_path.header;
    std::cerr << "target_pose: " << target_pose.position.x << ", " << target_pose.position.y
              << std::endl;
    // target_poseから先の点をcenterline_extensionに追加
    for (size_t i = target_idx; i < centerline_path.points.size(); ++i) {
      std::cerr << "Adding point to centerline extension: "
                << centerline_path.points[i].point.pose.position.x << ", "
                << centerline_path.points[i].point.pose.position.y << std::endl;
      centerline_extension.points.push_back(centerline_path.points[i]);
    }

    // centerline extensionが存在する場合、既存のpathと結合
    if (!centerline_extension.points.empty()) {
      // 重複点を避けて結合
      return utils::combinePath(clothoid_path, centerline_extension);
    }
  }

  // 結合できない場合はクロソイドパスをそのまま返す
  return clothoid_path;
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
  const double initial_velocity = 1.0;
  const auto & route_handler = planner_data->route_handler;
  const auto & common_parameters = planner_data->parameters;

  const double backward_path_length =
    planner_data->parameters.backward_path_length + parameters_.max_back_distance;
  const auto road_lanes = utils::getExtendedCurrentLanes(
    planner_data, backward_path_length, std::numeric_limits<double>::max(),
    /*forward_only_in_route*/ true);

  // Generate centerline path from road_lanes
  const auto row_centerline_path = utils::getCenterLinePath(
    *route_handler, road_lanes, start_pose, backward_path_length,
    std::numeric_limits<double>::max(), common_parameters);

  PathWithLaneId centerline_path =
    utils::resamplePathWithSpline(row_centerline_path, parameters_.center_line_path_interval);

  // =====================================================================
  // 追加: 初期直進距離を設定し，その区間の終端点を start_pose として使用できるよう
  //       直進区間の PathPoint 群を生成しておく．
  //       現状はパラメータ化せず固定長さとする（TODO: パラメータ化）。
  // =====================================================================

  constexpr double initial_forward_straight_distance = 3.0;  // [m] 直進区間長さ（仮）

  // 現在車両の直進方向に直進距離分進んだ位置を計算
  Pose straight_end_pose = start_pose;
  const double start_yaw = tf2::getYaw(start_pose.orientation);
  straight_end_pose.position.x =
    start_pose.position.x + initial_forward_straight_distance * std::cos(start_yaw);
  straight_end_pose.position.y =
    start_pose.position.y + initial_forward_straight_distance * std::sin(start_yaw);
  // 姿勢（yaw）は start_pose と同じ
  straight_end_pose.orientation = start_pose.orientation;

  // 直進区間の PathPoint 群を手動生成（車両の向きに沿って）
  std::vector<PathPointWithLaneId> straight_forward_points;
  const double point_interval = parameters_.center_line_path_interval;
  const int num_points = static_cast<int>(initial_forward_straight_distance / point_interval);

  for (int i = 1; i <= num_points; ++i) {
    PathPointWithLaneId pt;
    const double distance = i * point_interval;
    pt.point.pose.position.x = start_pose.position.x + distance * std::cos(start_yaw);
    pt.point.pose.position.y = start_pose.position.y + distance * std::sin(start_yaw);
    pt.point.pose.position.z = start_pose.position.z;
    pt.point.pose.orientation = start_pose.orientation;
    pt.point.longitudinal_velocity_mps = initial_velocity;  // 適切な速度を設定
    pt.point.is_final = false;
    // pt.lane_ids = default_lane_ids;
    straight_forward_points.push_back(pt);
  }

  const double lateral_offset = centerline_path.points.empty()
                                  ? 0.0
                                  : autoware::motion_utils::calcLateralOffset(
                                      centerline_path.points, straight_end_pose.position);
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
  const double wheel_base = common_parameters.vehicle_info.wheel_base_m;

  for (const auto & steer_angle : max_steer_angle) {
    // Calculate minimum radius based on the maximum steer angle
    const double minimum_radius = wheel_base / std::tan(steer_angle);

    const double longitudinal_distance =
      start_planner_utils::calc_necessary_longitudinal_distance(-lateral_offset, minimum_radius);

    const Pose target_pose = start_planner_utils::findTargetPoseAlongPath(
      centerline_path, straight_end_pose, longitudinal_distance);

    // TODO(Sugahara): ここでlateral_offset がプラスな場合は直進経路でよい。
    const auto relative_pose_info =
      start_planner_utils::calculateRelativePoseInVehicleCoordinate(straight_end_pose, target_pose);
    std::cerr << "target_pose: x=" << target_pose.position.x << ", y=" << target_pose.position.y
              << ", yaw=" << tf2::getYaw(target_pose.orientation) * 180.0 / M_PI << " deg"
              << std::endl;

    const auto circular_path = start_planner_utils::calc_circular_path(
      straight_end_pose, relative_pose_info.longitudinal_distance_vehicle,
      relative_pose_info.lateral_distance_vehicle, relative_pose_info.angle_diff, minimum_radius);

    if (circular_path.segments.empty()) {
      // TODO(Sugahara): steer_angle, 縦距離、横距離、角度差、最小半径をデバッグ出力
      std::cerr << "No circular path segments found for steer angle " << steer_angle * 180.0 / M_PI
                << " deg." << std::endl;
      continue;
    }

    geometry_msgs::msg::Pose current_segment_pose = straight_end_pose;
    std::vector<std::vector<geometry_msgs::msg::Point>> clothoid_paths;

    for (size_t i = 0; i < circular_path.segments.size(); ++i) {
      const auto & segment = circular_path.segments[i];
      // 車両パラメータから最適なクロソイドパラメータを計算
      const double circular_steer_angle = std::atan(wheel_base / minimum_radius);
      const double minimum_steer_time = circular_steer_angle / max_steer_angle_rate;
      const double L_min = initial_velocity * minimum_steer_time;
      const double A_min = std::sqrt(minimum_radius * L_min);

      // クロソイド変換を実行
      auto clothoid_points =
        convertArcToClothoidWithCorrection(segment, current_segment_pose, A_min, L_min, 20);

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

    // 目標速度を取得（centerline_pathからtarget_poseに最も近い点の速度を使用）
    double target_velocity = initial_velocity;  // デフォルト値
    if (!centerline_path.points.empty()) {
      const auto target_idx =
        autoware::motion_utils::findNearestIndex(centerline_path.points, target_pose.position);
      if (target_idx < centerline_path.points.size()) {
        target_velocity = centerline_path.points[target_idx].point.longitudinal_velocity_mps;
      }
    }

    PathWithLaneId path_with_lane_id = createPathWithLaneIdFromClothoidPaths(
      clothoid_paths, target_pose, initial_velocity, target_velocity, road_lanes, route_handler);

    if (path_with_lane_id.points.empty()) {
      std::cerr << "No clothoid path found for steer angle " << steer_angle * 180.0 / M_PI
                << " deg." << std::endl;
      continue;
    }

    // PullOutPathを作成
    PullOutPath pull_out_path;
    pull_out_path.start_pose = start_pose;
    pull_out_path.end_pose = target_pose;

    // 速度と加速度のペア設定
    // TODO(Sugahara): set parameter properly
    pull_out_path.pairs_terminal_velocity_and_accel.push_back(
      std::make_pair(initial_velocity, 1.0));

    // センターラインパスとの結合（空チェックは関数内で実行）
    auto combined_path = combinePathWithCenterline(path_with_lane_id, centerline_path, target_pose);

    // --- yaw不連続デバッグ出力ここまで ---

    // autoware::interpolation::lerpによる等間隔リサンプリング
    PathWithLaneId resampled_combined_path = combined_path;
    if (combined_path.points.size() >= 2) {
      // 1. arclength配列
      std::vector<double> arclengths(combined_path.points.size(), 0.0);
      for (size_t i = 1; i < combined_path.points.size(); ++i) {
        const auto & p0 = combined_path.points[i - 1].point.pose.position;
        const auto & p1 = combined_path.points[i].point.pose.position;
        arclengths[i] = arclengths[i - 1] + std::hypot(p1.x - p0.x, p1.y - p0.y);
      }
      // 2. 新しいarclength配列
      std::vector<double> query_s;
      for (double s = 0.0; s < arclengths.back(); s += parameters_.center_line_path_interval)
        query_s.push_back(s);
      if (query_s.empty() || query_s.back() < arclengths.back())
        query_s.push_back(arclengths.back());
      // 3. 各値をlerp補間
      std::vector<double> xs, ys, zs, yaws, vels;
      for (const auto & pt : combined_path.points) {
        xs.push_back(pt.point.pose.position.x);
        ys.push_back(pt.point.pose.position.y);
        zs.push_back(pt.point.pose.position.z);
        vels.push_back(pt.point.longitudinal_velocity_mps);
        yaws.push_back(tf2::getYaw(pt.point.pose.orientation));
      }
      auto lerp_x = autoware::interpolation::lerp(arclengths, xs, query_s);
      auto lerp_y = autoware::interpolation::lerp(arclengths, ys, query_s);
      auto lerp_z = autoware::interpolation::lerp(arclengths, zs, query_s);
      auto lerp_yaw = autoware::interpolation::lerp(arclengths, yaws, query_s);
      auto lerp_vel = autoware::interpolation::lerp(arclengths, vels, query_s);
      // 4. PathWithLaneId生成
      resampled_combined_path = combined_path;
      resampled_combined_path.points.clear();
      for (size_t i = 0; i < query_s.size(); ++i) {
        PathPointWithLaneId pt;
        pt.point.pose.position.x = lerp_x[i];
        pt.point.pose.position.y = lerp_y[i];
        pt.point.pose.position.z = lerp_z[i];
        pt.point.pose.orientation = autoware_utils::create_quaternion_from_yaw(lerp_yaw[i]);
        pt.point.longitudinal_velocity_mps = lerp_vel[i];
        pt.point.is_final = false;
        resampled_combined_path.points.push_back(pt);
      }
    }

    // 1. 後退パス生成
    const double backward_distance = 6.0;  // 例: 6m後退
    const double interval = 1.0;           // 1.0m間隔
    // lane_idの決定
    std::vector<int64_t> default_lane_ids;
    if (!resampled_combined_path.points.empty()) {
      default_lane_ids = resampled_combined_path.points.front().lane_ids;
    } else if (!centerline_path.points.empty()) {
      default_lane_ids = centerline_path.points.front().lane_ids;
    } else if (!road_lanes.empty()) {
      default_lane_ids.push_back(road_lanes.front().id());
    }

    std::vector<PathPointWithLaneId> backward_points;
    for (double d = interval; d <= backward_distance + 1e-3; d += interval) {
      PathPointWithLaneId pt;
      double yaw = tf2::getYaw(start_pose.orientation);
      pt.point.pose.position.x = start_pose.position.x - d * std::cos(yaw);
      pt.point.pose.position.y = start_pose.position.y - d * std::sin(yaw);
      pt.point.pose.position.z = start_pose.position.z;
      pt.point.pose.orientation = start_pose.orientation;  // yawはそのまま
      pt.point.longitudinal_velocity_mps = 1.0;            // 後退速度
      pt.point.is_final = false;
      pt.lane_ids = default_lane_ids;  // lane_idを設定
      backward_points.push_back(pt);
    }
    // 生成した後退点列を逆順にする
    std::reverse(backward_points.begin(), backward_points.end());
    // 2. 既存resampled_combined_pathの先頭に挿入
    const size_t num_backward = backward_points.size();
    resampled_combined_path.points.insert(
      resampled_combined_path.points.begin(), backward_points.begin(), backward_points.end());

    // -----------------------------------------------------------------
    // 追加: 直進区間の PathPoint 群を後退区間と start_pose の間に挿入
    //       挿入位置は [後退点列サイズ] + 1 (start_pose の直後)
    // -----------------------------------------------------------------
    if (!straight_forward_points.empty()) {
      const size_t insertion_index = num_backward + 1;
      resampled_combined_path.points.insert(
        resampled_combined_path.points.begin() + insertion_index, straight_forward_points.begin(),
        straight_forward_points.end());
    }

    // 3. そのままpartial_pathsにpush
    // 追加: 先頭10点のデバッグ出力（既存フォーマットに合わせる）
    {
      size_t print_num = std::min(size_t(10), resampled_combined_path.points.size());
      std::cerr << "[Debug] resampled_combined_path 先頭10点 (idx=0～" << (print_num - 1) << ")"
                << std::endl;
      for (size_t i = 0; i < print_num; ++i) {
        const auto & p = resampled_combined_path.points[i].point.pose.position;
        double yaw = tf2::getYaw(resampled_combined_path.points[i].point.pose.orientation);
        double dist = 0.0, dx = 0.0, dy = 0.0, dyaw = 0.0, dyaw_deg = 0.0;
        if (i > 0) {
          const auto & p_prev = resampled_combined_path.points[i - 1].point.pose.position;
          dx = p.x - p_prev.x;
          dy = p.y - p_prev.y;
          dist = std::hypot(dx, dy);
          double prev_yaw =
            tf2::getYaw(resampled_combined_path.points[i - 1].point.pose.orientation);
          dyaw = angles::shortest_angular_distance(prev_yaw, yaw);
          dyaw_deg = dyaw * 180.0 / M_PI;
        }
        std::cerr << "  idx=" << i << ": x=" << p.x << ", y=" << p.y << ", yaw=" << yaw << " rad"
                  << ", dist_from_prev=" << dist << ", dx=" << dx << ", dy=" << dy
                  << ", dyaw=" << dyaw << " rad (" << dyaw_deg << " deg)" << std::endl;
      }
    }
    pull_out_path.partial_paths.push_back(resampled_combined_path);

    // TODO(Sugahara): check lane departure
    return pull_out_path;
  }

  // 経路が生成できなかった場合
  return std::nullopt;
}

}  // namespace autoware::behavior_path_planner
