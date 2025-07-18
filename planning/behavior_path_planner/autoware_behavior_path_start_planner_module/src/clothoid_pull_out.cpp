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
#include "autoware/universe_utils/geometry/geometry.hpp"
#include "autoware_utils/geometry/boost_polygon_utils.hpp"

#include <autoware/interpolation/linear_interpolation.hpp>
#include <autoware/motion_utils/trajectory/path_shift.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_utils/geometry/geometry.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iomanip>
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
using start_planner_utils::getLaneIdsFromPose;
using start_planner_utils::getPullOutLanes;
using start_planner_utils::printPathWithLaneIdDetails;
using start_planner_utils::setLaneIdsToPathPoint;

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

  // 180度以上回転する場合は、反対方向の短い回転を選択
  if (std::abs(rotation_angle) > M_PI) {
    rotation_angle = (rotation_angle > 0) ? rotation_angle - 2 * M_PI : rotation_angle + 2 * M_PI;
  }

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
 * @brief エントリクロソイドセグメントを生成（各点にyaw角も含む）
 */
std::pair<std::vector<geometry_msgs::msg::Pose>, geometry_msgs::msg::Pose>
generateClothoidEntryWithYaw(
  const ClothoidSegment & segment, const geometry_msgs::msg::Pose & start_pose, int num_points)
{
  double A = segment.A;
  double L = segment.L;
  double direction_factor = segment.is_clockwise ? -1.0 : 1.0;
  double start_yaw = tf2::getYaw(start_pose.orientation);

  std::vector<geometry_msgs::msg::Pose> poses;

  // Entry Clothoid: 曲率を0から目標曲率まで線形に増加させる
  double target_curvature = (L / (A * A)) * direction_factor;
  double start_curvature = 0.0;

  // 数値積分による正確な計算
  double current_x = start_pose.position.x;
  double current_y = start_pose.position.y;
  double current_psi = start_yaw;

  for (int i = 0; i < num_points; ++i) {
    // 現在の点を作成してposesに追加
    geometry_msgs::msg::Pose pose;
    pose.position.x = current_x;
    pose.position.y = current_y;
    pose.position.z = 0.0;
    pose.orientation = autoware::universe_utils::createQuaternionFromYaw(current_psi);
    poses.push_back(pose);

    // 最後の点でない場合、次の点への積分計算を実行
    if (i < num_points - 1) {
      // 次の点への微小区間長
      double ds = L / (num_points - 1);

      // 現在の位置における曲率を計算
      double progress = static_cast<double>(i) / (num_points - 1);
      double current_curvature = start_curvature + (target_curvature - start_curvature) * progress;

      // 数値積分による座標更新
      current_x += std::cos(current_psi) * ds;
      current_y += std::sin(current_psi) * ds;
      current_psi += current_curvature * ds;
    }
  }

  // 終端状態を作成
  geometry_msgs::msg::Pose end_pose;
  end_pose.position.x = current_x;
  end_pose.position.y = current_y;
  end_pose.position.z = 0.0;
  end_pose.orientation = autoware::universe_utils::createQuaternionFromYaw(current_psi);

  return {poses, end_pose};
}

/**
 * @brief 円弧セグメントを生成（各点にyaw角も含む）
 */
std::pair<std::vector<geometry_msgs::msg::Pose>, geometry_msgs::msg::Pose>
generateCircularSegmentWithYaw(
  const ClothoidSegment & segment, const geometry_msgs::msg::Pose & start_pose, int num_points)
{
  double radius = segment.radius;
  double angle = segment.angle;
  double direction_factor = segment.is_clockwise ? -1.0 : 1.0;
  double start_yaw = tf2::getYaw(start_pose.orientation);

  std::vector<geometry_msgs::msg::Pose> poses;

  // 円弧中心計算
  double center_x = start_pose.position.x - radius * std::sin(start_yaw) * direction_factor;
  double center_y = start_pose.position.y + radius * std::cos(start_yaw) * direction_factor;

  for (int i = 0; i < num_points; ++i) {
    double progress = static_cast<double>(i) / (num_points - 1);
    double angle_progress = angle * progress * direction_factor;
    double current_psi = start_yaw + angle_progress;

    // 円弧上の位置を計算（中心からの角度）
    double angle_from_center = current_psi - M_PI / 2.0 * direction_factor;

    geometry_msgs::msg::Pose pose;
    pose.position.x = center_x + radius * std::cos(angle_from_center);
    pose.position.y = center_y + radius * std::sin(angle_from_center);
    pose.position.z = 0.0;
    pose.orientation = autoware::universe_utils::createQuaternionFromYaw(current_psi);

    poses.push_back(pose);
  }

  // 終端状態（最終的なyaw角）
  double final_psi = start_yaw + angle * direction_factor;

  geometry_msgs::msg::Pose end_pose;
  end_pose.position = poses.back().position;
  end_pose.orientation = autoware::universe_utils::createQuaternionFromYaw(final_psi);

  return {poses, end_pose};
}

/**
 * @brief エグジットクロソイドセグメントを生成（各点にyaw角も含む）
 */
std::pair<std::vector<geometry_msgs::msg::Pose>, geometry_msgs::msg::Pose>
generateClothoidExitWithYaw(
  const ClothoidSegment & segment, const geometry_msgs::msg::Pose & start_pose, int num_points)
{
  double L = segment.L;
  double start_yaw = tf2::getYaw(start_pose.orientation);
  double direction_factor = segment.is_clockwise ? -1.0 : 1.0;

  std::vector<geometry_msgs::msg::Pose> poses;

  // 前のセグメント（円弧）の曲率を計算（回転方向を考慮）
  double start_curvature = (1.0 / segment.radius) * direction_factor;

  // 数値積分による正確な計算
  double current_x = start_pose.position.x;
  double current_y = start_pose.position.y;
  double current_psi = start_yaw;

  for (int i = 0; i < num_points; ++i) {
    // 現在の点を作成してposesに追加
    geometry_msgs::msg::Pose pose;
    pose.position.x = current_x;
    pose.position.y = current_y;
    pose.position.z = 0.0;
    pose.orientation = autoware::universe_utils::createQuaternionFromYaw(current_psi);
    poses.push_back(pose);

    // 最後の点でない場合、次の点への積分計算を実行
    if (i < num_points - 1) {
      // 次の点への微小区間長
      double ds = L / (num_points - 1);

      // 現在の位置における曲率を計算（Exit Clothoid: 開始曲率から0まで線形に減少）
      double progress = static_cast<double>(i) / (num_points - 1);
      double current_curvature = start_curvature * (1.0 - progress);

      // 数値積分による座標更新
      current_x += std::cos(current_psi) * ds;
      current_y += std::sin(current_psi) * ds;
      current_psi += current_curvature * ds;
    }
  }

  // 終端状態を作成
  geometry_msgs::msg::Pose end_pose;
  end_pose.position.x = current_x;
  end_pose.position.y = current_y;
  end_pose.position.z = start_pose.position.z;
  end_pose.orientation = autoware::universe_utils::createQuaternionFromYaw(current_psi);

  return {poses, end_pose};
}

/**
 * @brief クロソイド経路の座標点列を生成
 */
std::vector<geometry_msgs::msg::Point> generateClothoidPath(
  const std::vector<ClothoidSegment> & segments, double point_interval,
  const geometry_msgs::msg::Pose & start_pose)
{
  // セグメントが空の場合の早期リターン
  if (segments.empty()) {
    std::cerr << "No clothoid segments provided to generateClothoidPath" << std::endl;
    return {};
  }

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

  // セグメント毎の点数を弧長と指定間隔から計算
  std::vector<int> segment_points;
  for (double length : theoretical_lengths) {
    int points = std::max(2, static_cast<int>(std::ceil(length / point_interval)) + 1);
    segment_points.push_back(points);
  }

  // 初期状態の設定
  geometry_msgs::msg::Pose current_pose = start_pose;

  std::vector<geometry_msgs::msg::Point> all_points;

  for (size_t i = 0; i < segments.size(); ++i) {
    std::vector<geometry_msgs::msg::Pose> segment_poses_vec;
    geometry_msgs::msg::Pose end_pose;

    int num_points = segment_points[i];

    if (segments[i].type == ClothoidSegment::CLOTHOID_ENTRY) {
      auto result = generateClothoidEntryWithYaw(segments[i], current_pose, num_points);
      segment_poses_vec = result.first;
      end_pose = result.second;
    } else if (segments[i].type == ClothoidSegment::CIRCULAR_ARC) {
      auto result = generateCircularSegmentWithYaw(segments[i], current_pose, num_points);
      segment_poses_vec = result.first;
      end_pose = result.second;
    } else if (segments[i].type == ClothoidSegment::CLOTHOID_EXIT) {
      auto result = generateClothoidExitWithYaw(segments[i], current_pose, num_points);
      segment_poses_vec = result.first;
      end_pose = result.second;
    }

    // 重複点を避けて結合（Poseから座標のみを抽出）
    size_t start_idx = (all_points.empty()) ? 0 : 1;
    for (size_t j = start_idx; j < segment_poses_vec.size(); ++j) {
      all_points.push_back(segment_poses_vec[j].position);
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
  double L_min, double point_interval)
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
    std::cerr << "Current parameters: total_angle=" << total_angle * 180.0 / M_PI
              << "°, alpha_clothoid=" << alpha_clothoid * 180.0 / M_PI
              << "°, required: total_angle >= " << 2.0 * alpha_clothoid * 180.0 / M_PI << "°"
              << std::endl;
  }

  // クロソイド経路生成
  std::vector<geometry_msgs::msg::Point> clothoid_path =
    generateClothoidPath(segments, point_interval, start_pose);

  return clothoid_path;
}

/**
 * @brief 改良版のクロソイド変換関数（終点補正付き）
 */
std::vector<geometry_msgs::msg::Point> convertArcToClothoidWithCorrection(
  const ArcSegment & arc_segment, const geometry_msgs::msg::Pose & start_pose,
  double initial_velocity, double wheel_base, double max_steer_angle_rate, double point_interval)
{
  // 最小半径を計算（arc_segmentから）
  const double minimum_radius = arc_segment.radius;

  // 車両パラメータから最適なクロソイドパラメータを計算
  const double circular_steer_angle = std::atan(wheel_base / minimum_radius);
  const double minimum_steer_time = circular_steer_angle / max_steer_angle_rate;
  const double L_min = initial_velocity * minimum_steer_time;
  const double A_min = std::sqrt(minimum_radius * L_min);

  std::cerr << "Clothoid parameters: radius=" << minimum_radius << ", A_min=" << A_min
            << ", L_min=" << L_min << ", velocity=" << initial_velocity << std::endl;

  // 元のクロソイド変換を実行
  auto clothoid_points =
    convertArcToClothoid(arc_segment, start_pose, A_min, L_min, point_interval);

  if (clothoid_points.empty()) {
    std::cerr << "Clothoid conversion failed! Check parameters and arc segment validity."
              << std::endl;
    std::cerr << "Arc segment: radius=" << arc_segment.radius << ", center=("
              << arc_segment.center.x << ", " << arc_segment.center.y << ")"
              << ", is_clockwise=" << arc_segment.is_clockwise << std::endl;
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
 * @param acceleration 加速度
 * @param road_lanes 道路レーン情報
 * @param route_handler ルートハンドラー
 * @param parameters パラメータ
 * @return PathWithLaneId
 */
// std::optional<PathWithLaneId> でよさそう
PathWithLaneId createPathWithLaneIdFromClothoidPaths(
  const std::vector<std::vector<geometry_msgs::msg::Point>> & clothoid_paths,
  const geometry_msgs::msg::Pose & target_pose, double velocity, double target_velocity,
  double acceleration, const lanelet::ConstLanelets & road_lanes,
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
    // スキップしていいの？
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

  // PathWithLaneIdを作成
  PathWithLaneId path_with_lane_id;
  path_with_lane_id.header = route_handler->getRouteHeader();

  // 各座標点をPathPointWithLaneIdに変換
  for (size_t i = 0; i < all_clothoid_points.size(); ++i) {
    PathPointWithLaneId path_point;

    // 座標設定
    path_point.point.pose.position = all_clothoid_points[i];

    // // 向きを計算（次の点への方向）
    // if (i < all_clothoid_points.size() - 1) {
    //   const double dx = all_clothoid_points[i + 1].x - all_clothoid_points[i].x;
    //   const double dy = all_clothoid_points[i + 1].y - all_clothoid_points[i].y;
    //   const double yaw = std::atan2(dy, dx);
    //   path_point.point.pose.orientation =
    //   autoware::universe_utils::createQuaternionFromYaw(yaw);
    // } else {
    //   // 最後の点は前の点との方向から計算
    //   if (all_clothoid_points.size() >= 2) {
    //     const double dx = all_clothoid_points[i].x - all_clothoid_points[i - 1].x;
    //     const double dy = all_clothoid_points[i].y - all_clothoid_points[i - 1].y;
    //     const double yaw = std::atan2(dy, dx);
    //     path_point.point.pose.orientation =
    //     autoware::universe_utils::createQuaternionFromYaw(yaw);
    //   } else {
    //     // 1点しかない場合は単位クォータニオン
    //     path_point.point.pose.orientation.x = 0.0;
    //     path_point.point.pose.orientation.y = 0.0;
    //     path_point.point.pose.orientation.z = 0.0;
    //     path_point.point.pose.orientation.w = 1.0;
    //   }
    // }

    // z座標の設定: レーンレット情報から最も近い点のz値を取得
    if (!road_lanes.empty()) {
      // 最も近いレーンレットを見つける
      lanelet::Lanelet closest_lanelet;
      if (lanelet::utils::query::getClosestLanelet(
            road_lanes, path_point.point.pose, &closest_lanelet)) {
        // レーンレットのセンターラインから最も近い点のz値を取得
        const auto centerline = closest_lanelet.centerline();
        if (!centerline.empty()) {
          double min_distance = std::numeric_limits<double>::max();
          double closest_z = all_clothoid_points[i].z;  // デフォルトは元のz値

          for (const auto & point : centerline) {
            const double distance = std::hypot(
              point.x() - all_clothoid_points[i].x, point.y() - all_clothoid_points[i].y);
            if (distance < min_distance) {
              min_distance = distance;
              closest_z = point.z();
            }
          }
          path_point.point.pose.position.z = closest_z;
        }
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
    path_point.point.is_final = false;

    // レーンIDの設定
    std::vector<int64_t> previous_lane_ids;
    if (i > 0) {
      previous_lane_ids = path_with_lane_id.points[i - 1].lane_ids;
    }
    setLaneIdsToPathPoint(path_point, road_lanes, previous_lane_ids);

    path_with_lane_id.points.push_back(path_point);
  }

  return path_with_lane_id;
  // return
  // autoware::behavior_path_planner::utils::resamplePathWithSpline(path_with_lane_id, 1.0);
}

/**
 * @brief 円弧パスのセグメントをクロソイド曲線に変換する関数
 * @param circular_path 円弧パス（2つのセグメントを含む）
 * @param start_pose 開始姿勢
 * @param initial_velocity 初期速度
 * @param wheel_base ホイールベース
 * @param max_steer_angle_rate 最大ステア角速度
 * @param point_interval 点間隔
 * @return クロソイドパスの配列
 */
std::vector<std::vector<geometry_msgs::msg::Point>> convertCircularPathToClothoidPaths(
  const CompositeArcPath & circular_path, const geometry_msgs::msg::Pose & start_pose,
  double initial_velocity, double wheel_base, double max_steer_angle_rate, double point_interval)
{
  std::vector<std::vector<geometry_msgs::msg::Point>> clothoid_paths;

  if (circular_path.segments.size() < 2) {
    std::cerr << "Circular path must have at least 2 segments" << std::endl;
    return clothoid_paths;
  }

  geometry_msgs::msg::Pose current_segment_pose = start_pose;

  // 第1セグメント（開始セグメント）の処理
  const auto & first_segment = circular_path.segments[0];
  auto first_clothoid_points = convertArcToClothoidWithCorrection(
    first_segment, current_segment_pose, initial_velocity, wheel_base, max_steer_angle_rate,
    point_interval);

  // 第1セグメントの変換が失敗した場合の早期リターン
  if (first_clothoid_points.empty()) {
    std::cerr << "Failed to convert first segment to clothoid" << std::endl;
    return clothoid_paths;
  }

  clothoid_paths.push_back(first_clothoid_points);

  // 第1セグメント終了時の姿勢を計算（第2セグメントの開始姿勢として使用）
  geometry_msgs::msg::Pose second_segment_start_pose;
  const auto & last_point_first = first_clothoid_points.back();
  second_segment_start_pose.position = last_point_first;

  // 終点での進行方向を計算（最後の2点から）
  if (first_clothoid_points.size() >= 2) {
    // TODO(Sugahara): ここでyawの計算方法あってる？
    const auto & second_last_first = first_clothoid_points[first_clothoid_points.size() - 2];
    double dx = last_point_first.x - second_last_first.x;
    double dy = last_point_first.y - second_last_first.y;
    double heading = std::atan2(dy, dx);
    second_segment_start_pose.orientation =
      tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), heading));
  } else {
    second_segment_start_pose.orientation = current_segment_pose.orientation;
  }

  // 第2セグメント（終了セグメント）の処理
  const auto & second_segment = circular_path.segments[1];
  auto second_clothoid_points = convertArcToClothoidWithCorrection(
    second_segment, second_segment_start_pose, initial_velocity, wheel_base, max_steer_angle_rate,
    point_interval);

  if (second_clothoid_points.empty()) {
    std::cerr << "Failed to convert second segment to clothoid" << std::endl;
    return clothoid_paths;
  }
  clothoid_paths.push_back(second_clothoid_points);

  return clothoid_paths;
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
    // どこまで追加する？？？？
    for (size_t i = target_idx; i < centerline_path.points.size(); ++i) {
      centerline_extension.points.push_back(centerline_path.points[i]);
    }

    // centerline extensionが存在する場合、既存のpathと結合
    if (!centerline_extension.points.empty()) {
      // 重複点を避けて結合
      return utils::combinePath(clothoid_path, centerline_extension);
    }
  }
  return clothoid_path;
}

/**
 * @brief start_poseから前後に直進するposes配列を生成する関数
 * @param start_pose 開始姿勢
 * @param forward_distance 前方直進距離[m]
 * @param backward_distance 後方直進距離[m]
 * @param point_interval 点間隔[m]
 * @return poses配列（後方→start_pose→前方の順）
 */
std::vector<geometry_msgs::msg::Pose> createStraightPathToEndPose(
  const geometry_msgs::msg::Pose & start_pose, double forward_distance, double backward_distance,
  double point_interval)
{
  // 直進方向（yaw角）を計算
  const double start_yaw = tf2::getYaw(start_pose.orientation);

  // poseの配列を格納
  std::vector<geometry_msgs::msg::Pose> poses;

  // 1. 後方経路を生成（最も遠い後退点から順番に生成）
  if (backward_distance > 0.0) {
    const int backward_num_points = static_cast<int>(backward_distance / point_interval);

    for (int i = backward_num_points; i >= 1; --i) {
      geometry_msgs::msg::Pose pose;
      double distance = i * point_interval;
      pose.position.x = start_pose.position.x - distance * std::cos(start_yaw);
      pose.position.y = start_pose.position.y - distance * std::sin(start_yaw);
      pose.position.z = start_pose.position.z;
      pose.orientation = start_pose.orientation;
      poses.push_back(pose);
    }
  }

  // 2. start_poseを追加
  poses.push_back(start_pose);

  // 3. 前方経路を生成
  if (forward_distance > 0.0) {
    // 前方終了姿勢を計算
    geometry_msgs::msg::Pose forward_end_pose = start_pose;
    forward_end_pose.position.x = start_pose.position.x + forward_distance * std::cos(start_yaw);
    forward_end_pose.position.y = start_pose.position.y + forward_distance * std::sin(start_yaw);
    forward_end_pose.orientation = start_pose.orientation;

    // 前方点数を計算（start_poseは既に追加済みなので除外）
    const int forward_num_points =
      std::max(1, static_cast<int>(std::ceil(forward_distance / point_interval)));

    // 実際の間隔を再計算（等間隔にするため）
    const double actual_interval = forward_distance / forward_num_points;

    // 前方各点を生成（start_pose以降）
    for (int i = 1; i <= forward_num_points; ++i) {
      geometry_msgs::msg::Pose pose;

      if (i == forward_num_points) {
        // 最終点（正確にforward_end_poseにする）
        pose = forward_end_pose;
      } else {
        // 中間点
        const double distance = i * actual_interval;
        pose.position.x = start_pose.position.x + distance * std::cos(start_yaw);
        pose.position.y = start_pose.position.y + distance * std::sin(start_yaw);
        pose.position.z = start_pose.position.z;
        pose.orientation = start_pose.orientation;
      }

      poses.push_back(pose);
    }
  }

  return poses;
}

/**
 * @brief Find target pose along path at specified longitudinal distance
 * @param centerline_path Centerline path to search
 * @param start_pose Starting pose
 * @param longitudinal_distance Longitudinal distance to search
 * @return Target pose
 */
geometry_msgs::msg::Pose findTargetPoseAlongPath(
  const PathWithLaneId & centerline_path, const geometry_msgs::msg::Pose & start_pose,
  const double longitudinal_distance)
{
  geometry_msgs::msg::Pose target_pose = start_pose;
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

/**
 * @brief Calculate necessary longitudinal distance for circular path planning
 * @param lateral_offset Lateral offset from the path
 * @param minimum_radius Minimum turning radius
 * @return Calculated longitudinal distance
 */
double calc_necessary_longitudinal_distance(
  const double lateral_offset, const double minimum_radius)
{
  // Trial distances based on minimum radius
  const std::vector<double> trial_distances = {
    0.5 * minimum_radius, 0.75 * minimum_radius, 1.0 * minimum_radius, 1.5 * minimum_radius,
    2.0 * minimum_radius, 3.0 * minimum_radius,  4.0 * minimum_radius, 5.0 * minimum_radius,
    6.0 * minimum_radius, 8.0 * minimum_radius,  10.0 * minimum_radius};

  // Starting pose parameters (assumed at origin with 0 yaw)
  constexpr double x_start = 0.0;
  constexpr double y_start = 0.0;
  constexpr double yaw_start = 0.0;

  // Evaluation parameters
  constexpr double error_threshold = 0.5;
  constexpr double tolerance = 0.1;

  // Results tracking
  std::vector<std::pair<double, double>> evaluation_results;
  evaluation_results.reserve(trial_distances.size());

  double best_distance = 0.0;
  double best_score = -1e9;  // Prioritize longer Arc1 length
  bool found_valid = false;
  int valid_results_count = 0;

  for (const double trial_distance : trial_distances) {
    // Calculate goal position considering lateral offset
    const double x_goal =
      x_start + trial_distance * std::cos(yaw_start) + lateral_offset * (-std::sin(yaw_start));
    const double y_goal =
      y_start + trial_distance * std::sin(yaw_start) + lateral_offset * std::cos(yaw_start);

    // Calculate starting arc center (assuming clockwise rotation)
    const double center_rx = x_start + minimum_radius * std::sin(yaw_start);
    const double center_ry = y_start - minimum_radius * std::cos(yaw_start);

    // Calculate target arc radius using Al-Kashi theorem
    const double dx_goal = x_goal - center_rx;
    const double dy_goal = y_goal - center_ry;
    const double distance_to_goal = std::sqrt(dx_goal * dx_goal + dy_goal * dy_goal);

    if (distance_to_goal < 1e-6) {
      std::cout << "  Trial distance: " << std::fixed << std::setprecision(2) << trial_distance
                << " m - SKIPPED (goal too close to arc center)" << std::endl;
      continue;
    }

    const double cos_term = std::clamp((y_goal - center_ry) / distance_to_goal, -1.0, 1.0);
    const double alpha = M_PI + std::acos(cos_term);
    const double denominator = 2 * minimum_radius + 2 * distance_to_goal * std::cos(alpha);

    if (std::abs(denominator) < 1e-6) {
      std::cout << "  Trial distance: " << std::fixed << std::setprecision(2) << trial_distance
                << " m - SKIPPED (denominator too small)" << std::endl;
      continue;
    }

    const double radius_goal =
      (distance_to_goal * distance_to_goal - minimum_radius * minimum_radius) / denominator;

    // Check physical feasibility
    if (radius_goal < 0 || radius_goal < minimum_radius) {
      std::cout << "  Trial distance: " << std::fixed << std::setprecision(2) << trial_distance
                << " m - SKIPPED (radius infeasible: " << std::setprecision(3) << radius_goal << ")"
                << std::endl;
      continue;
    }

    // Calculate target arc center (assuming counter-clockwise rotation)
    const double center_lx = x_goal - radius_goal * std::sin(yaw_start);
    const double center_ly = y_goal + radius_goal * std::cos(yaw_start);

    // Validate arc connection
    const double dx_centers = center_lx - center_rx;
    const double dy_centers = center_ly - center_ry;
    const double distance_between_centers =
      std::sqrt(dx_centers * dx_centers + dy_centers * dy_centers);

    const double external_tangent_distance = minimum_radius + radius_goal;
    const double internal_tangent_distance = std::abs(minimum_radius - radius_goal);

    const bool connection_valid =
      (std::abs(distance_between_centers - external_tangent_distance) <= tolerance ||
       std::abs(distance_between_centers - internal_tangent_distance) <= tolerance ||
       (distance_between_centers > external_tangent_distance + tolerance &&
        distance_between_centers - external_tangent_distance <= 2.0) ||
       (distance_between_centers < internal_tangent_distance - tolerance &&
        internal_tangent_distance - distance_between_centers <=
          std::min(minimum_radius, radius_goal) * 0.8));

    if (!connection_valid) {
      std::cout << "  Trial distance: " << std::fixed << std::setprecision(2) << trial_distance
                << " m - SKIPPED (arc connection invalid)" << std::endl;
      continue;
    }

    // Calculate tangent point between circles
    double tangent_x, tangent_y;
    if (distance_between_centers < 1e-6) {
      tangent_x = (center_rx + center_lx) / 2.0;
      tangent_y = (center_ry + center_ly) / 2.0;
    } else if (std::abs(distance_between_centers - external_tangent_distance) <= tolerance) {
      // External tangent case
      const double ratio = minimum_radius / (minimum_radius + radius_goal);
      tangent_x = center_rx + ratio * dx_centers;
      tangent_y = center_ry + ratio * dy_centers;
    } else {
      // Other cases - use approximation
      const double ratio = 0.5;
      tangent_x = center_rx + ratio * dx_centers;
      tangent_y = center_ry + ratio * dy_centers;
    }

    // Calculate actual lateral offset achieved
    const double dx_actual = x_goal - x_start;
    const double dy_actual = y_goal - y_start;
    const double lateral_x = -std::sin(yaw_start);
    const double lateral_y = std::cos(yaw_start);
    const double actual_lateral_offset = dx_actual * lateral_x + dy_actual * lateral_y;
    const double lateral_error = std::abs(actual_lateral_offset - lateral_offset);

    // Calculate Arc1 length
    const double start_angle = std::atan2(y_start - center_ry, x_start - center_rx);
    const double tangent_angle = std::atan2(tangent_y - center_ry, tangent_x - center_rx);
    double angle_diff = tangent_angle - start_angle;

    // Adjust for clockwise direction
    if (angle_diff > 0) {
      angle_diff -= 2 * M_PI;
    }

    const double arc1_length = minimum_radius * std::abs(angle_diff);

    // Store evaluation result
    evaluation_results.emplace_back(trial_distance, arc1_length);
    valid_results_count++;

    // Update best candidate selection
    if (lateral_error <= error_threshold) {
      if (arc1_length > best_score) {
        best_score = arc1_length;
        best_distance = trial_distance;
        found_valid = true;
      }
    } else if (!found_valid && arc1_length > best_score) {
      // If no acceptable solution found yet, select the best available
      best_score = arc1_length;
      best_distance = trial_distance;
    }
  }

  // Output selection results
  std::cout << "\n--- Selection Results ---" << std::endl;
  std::cout << "Valid results: " << valid_results_count << std::endl;

  if (found_valid) {
    std::cout << "Acceptable results (error <= " << std::fixed << std::setprecision(1)
              << error_threshold << "m): found" << std::endl;
    std::cout << "Selected result: Arc1 length = " << std::setprecision(3) << best_score
              << " m, Distance = " << std::setprecision(3) << best_distance << " m" << std::endl;
  } else {
    std::cout << "No acceptable results found" << std::endl;
  }

  // Fallback if no valid solution found
  if (!found_valid && best_distance == 0.0) {
    best_distance = std::max(4.0 * minimum_radius, std::abs(lateral_offset) * 2.0);
    std::cout << "Using geometric estimation: " << std::setprecision(3) << best_distance << " m"
              << std::endl;
  }

  return best_distance;
}

/**
 * @brief Calculate circular path using relative coordinates
 * @param start_pose Starting pose
 * @param longitudinal_distance Longitudinal distance to target
 * @param lateral_distance Lateral distance to target
 * @param angle_diff Angle difference to target
 * @param minimum_radius Minimum turning radius
 * @return Composite arc path
 */
CompositeArcPath calc_circular_path(
  const geometry_msgs::msg::Pose & start_pose, const double longitudinal_distance,
  const double lateral_distance, const double angle_diff, const double minimum_radius)
{
  const double PI = M_PI;

  std::cout << "\n=== Circular Path Planning (Relative Direct) ===" << std::endl;
  std::cout << std::fixed << std::setprecision(2);
  std::cout << "Start: (" << start_pose.position.x << ", " << start_pose.position.y
            << "), yaw=" << tf2::getYaw(start_pose.orientation) * 180.0 / PI << "°" << std::endl;
  std::cout << "Relative target: longitudinal=" << longitudinal_distance
            << "m, lateral=" << lateral_distance << "m, angle_diff=" << angle_diff * 180.0 / PI
            << "°" << std::endl;

  // Calculate in relative coordinate system (origin at start point, X-axis as forward direction)
  // Start point: (0, 0, 0)
  // Goal point: (longitudinal_distance, lateral_distance, angle_diff)

  const double x_start_rel = 0.0;
  const double y_start_rel = 0.0;
  const double yaw_start_rel = 0.0;

  const double x_goal_rel = longitudinal_distance;
  const double y_goal_rel = lateral_distance;
  const double yaw_goal_rel = angle_diff;

  std::cout << "Relative coordinates - Start: (" << x_start_rel << ", " << y_start_rel
            << "), Goal: (" << x_goal_rel << ", " << y_goal_rel << ")" << std::endl;

  // Calculate starting arc center (assuming clockwise rotation)
  double C_rx_rel = x_start_rel + minimum_radius * std::sin(yaw_start_rel);
  double C_ry_rel = y_start_rel - minimum_radius * std::cos(yaw_start_rel);

  // Distance from goal point to starting arc center
  double dx_goal_rel = x_goal_rel - C_rx_rel;
  double dy_goal_rel = y_goal_rel - C_ry_rel;
  double d_goal_Cr_rel = std::sqrt(dx_goal_rel * dx_goal_rel + dy_goal_rel * dy_goal_rel);

  if (d_goal_Cr_rel < 1e-6) {
    std::cout << "Warning: Goal is too close to start arc center (distance: "
              << std::setprecision(6) << d_goal_Cr_rel << ")" << std::endl;
    return CompositeArcPath();
  }

  // Calculate radius using Al-Kashi theorem
  double cos_term = dy_goal_rel / d_goal_Cr_rel;
  cos_term = std::max(-1.0, std::min(1.0, cos_term));

  // Adjust approach angle to goal (add π for reverse direction)
  double alpha = (yaw_goal_rel + PI) + std::acos(cos_term);

  double denominator = 2 * minimum_radius + 2 * d_goal_Cr_rel * std::cos(alpha);

  if (std::abs(denominator) < 1e-6) {
    std::cout << "Warning: Denominator too small (denominator: " << std::setprecision(6)
              << denominator << ")" << std::endl;
    return CompositeArcPath();
  }

  double R_goal = (d_goal_Cr_rel * d_goal_Cr_rel - minimum_radius * minimum_radius) / denominator;

  // Check physical feasibility
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

  // Calculate goal arc center (assuming counter-clockwise rotation)
  double C_lx_rel = x_goal_rel - R_goal * std::sin(yaw_goal_rel);
  double C_ly_rel = y_goal_rel + R_goal * std::cos(yaw_goal_rel);

  // Calculate tangent point
  double dx_centers = C_lx_rel - C_rx_rel;
  double dy_centers = C_ly_rel - C_ry_rel;
  double distance_centers = std::sqrt(dx_centers * dx_centers + dy_centers * dy_centers);

  double tangent_x_rel, tangent_y_rel;

  // Calculate tangent point for external tangent case
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

  // First arc (from start point to tangent point, clockwise)
  double start_angle1 = std::atan2(y_start_rel - C_ry_rel, x_start_rel - C_rx_rel);
  double end_angle1 = std::atan2(tangent_y_rel - C_ry_rel, tangent_x_rel - C_rx_rel);
  double angle_diff1 = end_angle1 - start_angle1;

  // Adjust for clockwise direction
  if (angle_diff1 > 0) {
    angle_diff1 -= 2 * PI;
  }

  // Second arc (from tangent point to goal point, counter-clockwise)
  double start_angle2 = std::atan2(tangent_y_rel - C_ly_rel, tangent_x_rel - C_lx_rel);
  double end_angle2 = std::atan2(y_goal_rel - C_ly_rel, x_goal_rel - C_lx_rel);
  double angle_diff2 = end_angle2 - start_angle2;

  // Adjust for counter-clockwise direction
  if (angle_diff2 < 0) {
    angle_diff2 += 2 * PI;
  }

  // Prepare for transformation to global coordinate system
  const double start_yaw = tf2::getYaw(start_pose.orientation);
  const double cos_yaw = std::cos(start_yaw);
  const double sin_yaw = std::sin(start_yaw);

  // Create CompositeArcPath
  CompositeArcPath composite_path;

  // Create first arc segment
  ArcSegment arc1;
  arc1.radius = minimum_radius;
  arc1.is_clockwise = true;

  // Transform relative coordinate center to global coordinate system
  arc1.center.x = start_pose.position.x + C_rx_rel * cos_yaw - C_ry_rel * sin_yaw;
  arc1.center.y = start_pose.position.y + C_rx_rel * sin_yaw + C_ry_rel * cos_yaw;
  arc1.center.z = start_pose.position.z;

  // Set start and end poses
  arc1.start_pose = start_pose;

  // Calculate pose at tangent point (global coordinate system)
  geometry_msgs::msg::Pose tangent_pose;
  tangent_pose.position.x =
    start_pose.position.x + tangent_x_rel * cos_yaw - tangent_y_rel * sin_yaw;
  tangent_pose.position.y =
    start_pose.position.y + tangent_x_rel * sin_yaw + tangent_y_rel * cos_yaw;
  tangent_pose.position.z = start_pose.position.z;

  // Calculate orientation at tangent point (tangent direction of arc)
  double tangent_angle_global = end_angle1 + (arc1.is_clockwise ? -PI / 2 : PI / 2) + start_yaw;
  tangent_pose.orientation.x = 0.0;
  tangent_pose.orientation.y = 0.0;
  tangent_pose.orientation.z = std::sin(tangent_angle_global / 2.0);
  tangent_pose.orientation.w = std::cos(tangent_angle_global / 2.0);

  arc1.end_pose = tangent_pose;

  // Create second arc segment
  ArcSegment arc2;
  arc2.radius = R_goal;
  arc2.is_clockwise = false;

  // Transform relative coordinate center to global coordinate system
  arc2.center.x = start_pose.position.x + C_lx_rel * cos_yaw - C_ly_rel * sin_yaw;
  arc2.center.y = start_pose.position.y + C_lx_rel * sin_yaw + C_ly_rel * cos_yaw;
  arc2.center.z = start_pose.position.z;

  // Set start pose (tangent point) and end pose (goal point)
  arc2.start_pose = tangent_pose;

  // Calculate goal pose (global coordinate system)
  geometry_msgs::msg::Pose goal_pose;
  goal_pose.position.x = start_pose.position.x + x_goal_rel * cos_yaw - y_goal_rel * sin_yaw;
  goal_pose.position.y = start_pose.position.y + x_goal_rel * sin_yaw + y_goal_rel * cos_yaw;
  goal_pose.position.z = start_pose.position.z;

  // Calculate orientation at goal point
  double goal_yaw_global = start_yaw + yaw_goal_rel;
  goal_pose.orientation.x = 0.0;
  goal_pose.orientation.y = 0.0;
  goal_pose.orientation.z = std::sin(goal_yaw_global / 2.0);
  goal_pose.orientation.w = std::cos(goal_yaw_global / 2.0);

  arc2.end_pose = goal_pose;

  // Add segments
  composite_path.segments.push_back(arc1);
  composite_path.segments.push_back(arc2);

  return composite_path;
}

/**
 * @brief Convert circular path to trajectory
 * @param composite_arc_path Composite arc path
 * @param velocity Velocity for trajectory
 * @param z Z coordinate for trajectory points
 * @return Trajectory message
 */
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

  // Generate point cloud from CompositeArcPath
  std::vector<std::pair<double, double>> path_points;
  const int points_per_segment = 50;

  for (const auto & segment : composite_arc_path.segments) {
    // Generate points from each segment
    for (int i = 0; i < points_per_segment; ++i) {
      // Skip first point for segments after the first (avoid duplication)
      if (!path_points.empty() && i == 0) {
        continue;
      }

      double progress = static_cast<double>(i) / (points_per_segment - 1);

      // Calculate start and end angles
      double start_angle = segment.getStartAngle();
      double end_angle = segment.getEndAngle();
      double current_angle;

      if (segment.is_clockwise) {
        // Adjust angle for clockwise direction
        double angle_diff = end_angle - start_angle;
        if (angle_diff > 0) {
          angle_diff -= 2 * M_PI;
        }
        current_angle = start_angle + angle_diff * progress;
      } else {
        // Adjust angle for counter-clockwise direction
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

    // Set position
    point.pose.position.x = path_points[i].first;
    point.pose.position.y = path_points[i].second;
    point.pose.position.z = z;

    // Set orientation (direction to next point)
    if (i < path_points.size() - 1) {
      const double dx = path_points[i + 1].first - path_points[i].first;
      const double dy = path_points[i + 1].second - path_points[i].second;
      const double yaw = std::atan2(dy, dx);
      point.pose.orientation = autoware_utils::create_quaternion_from_yaw(yaw);
    } else {
      // Last point has same direction as previous point
      if (i > 0) {
        const double dx = path_points[i].first - path_points[i - 1].first;
        const double dy = path_points[i].second - path_points[i - 1].second;
        const double yaw = std::atan2(dy, dx);
        point.pose.orientation = autoware_utils::create_quaternion_from_yaw(yaw);
      } else {
        point.pose.orientation = autoware_utils::create_quaternion_from_yaw(0.0);
      }
    }

    // Set velocity
    point.longitudinal_velocity_mps = velocity;
    point.lateral_velocity_mps = 0.0;
    point.acceleration_mps2 = 0.0;
    point.heading_rate_rps = 0.0;
    point.front_wheel_angle_rad = 0.0;
    point.rear_wheel_angle_rad = 0.0;

    // Set time
    if (i == 0) {
      point.time_from_start.sec = 0;
      point.time_from_start.nanosec = 0;
    } else {
      const double distance = std::sqrt(
        std::pow(path_points[i].first - path_points[i - 1].first, 2) +
        std::pow(path_points[i].second - path_points[i - 1].second, 2));
      const double time_diff = distance / velocity;

      // Use builtin_interfaces::msg::Duration
      const auto prev_time = trajectory.points[i - 1].time_from_start;
      const auto time_diff_sec = static_cast<int32_t>(time_diff);
      const auto time_diff_nanosec = static_cast<uint32_t>((time_diff - time_diff_sec) * 1e9);

      point.time_from_start.sec = prev_time.sec + time_diff_sec;
      point.time_from_start.nanosec = prev_time.nanosec + time_diff_nanosec;

      // Handle nanosecond overflow
      if (point.time_from_start.nanosec >= 1000000000) {
        point.time_from_start.sec += 1;
        point.time_from_start.nanosec -= 1000000000;
      }
    }

    trajectory.points.push_back(point);
  }

  return trajectory;
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
  const std::shared_ptr<const PlannerData> & planner_data, PlannerDebugData & planner_debug_data)
{
  // =====================================================================
  // STEP 1: パラメータ設定・初期化処理
  // =====================================================================
  const double initial_velocity = parameters_.clothoid_initial_velocity;
  const double acceleration = parameters_.clothoid_acceleration;
  const std::vector<double> max_steer_angle_degs = parameters_.clothoid_max_steer_angle_degs;

  // パラメータから度をラジアンに変換
  std::vector<double> max_steer_angle;
  for (const auto & deg : max_steer_angle_degs) {
    max_steer_angle.push_back(deg * M_PI / 180.0);
  }

  const double max_steer_angle_rate_deg_per_sec =
    parameters_.clothoid_max_steer_angle_rate_deg_per_sec;
  const double max_steer_angle_rate = max_steer_angle_rate_deg_per_sec * M_PI / 180.0;
  constexpr double initial_forward_straight_distance = 3.0;  // [m] 直進区間長さ（仮）
  const double backward_distance = 3.0;                      // 後退距離[m]

  const auto & route_handler = planner_data->route_handler;
  const auto & common_parameters = planner_data->parameters;
  const double wheel_base = common_parameters.vehicle_info.wheel_base_m;
  const double backward_path_length =
    planner_data->parameters.backward_path_length + parameters_.max_back_distance;

  // =====================================================================
  // STEP 2: レーン情報の取得
  // =====================================================================
  const auto road_lanes = utils::getExtendedCurrentLanes(
    planner_data, backward_path_length, std::numeric_limits<double>::max(),
    /*forward_only_in_route*/ true);

  // pull_out_lanes を取得（shoulder lane を含む）
  const auto pull_out_lanes = getPullOutLanes(planner_data, backward_path_length);

  // road_lanes と pull_out_lanes を結合して全てのレーンを含める
  const auto all_lanes = utils::combineLanelets(road_lanes, pull_out_lanes);

  // =====================================================================
  // STEP 3: センターラインパスの生成
  // =====================================================================
  const auto row_centerline_path = utils::getCenterLinePath(
    *route_handler, road_lanes, start_pose, backward_path_length,
    std::numeric_limits<double>::max(), common_parameters);

  PathWithLaneId centerline_path =
    utils::resamplePathWithSpline(row_centerline_path, parameters_.center_line_path_interval);

  // =====================================================================
  // STEP 4: 前後直進パスの生成
  // =====================================================================
  // createStraightPathToEndPose関数を使用して前後直進経路を生成
  auto straight_poses = createStraightPathToEndPose(
    start_pose, initial_forward_straight_distance, backward_distance,
    parameters_.center_line_path_interval);

  // ここで path_with_lane_id にしなくてよい
  // straight_posesからPathPointWithLaneIdを生成
  std::vector<PathPointWithLaneId> straight_forward_points;
  for (size_t i = 0; i < straight_poses.size(); ++i) {
    PathPointWithLaneId pt;
    pt.point.pose = straight_poses[i];
    pt.point.longitudinal_velocity_mps = initial_velocity;
    pt.point.lateral_velocity_mps = 0.0;
    pt.point.heading_rate_rps = 0.0;
    pt.point.is_final = false;

    // レーンIDの設定
    std::vector<int64_t> previous_lane_ids;
    if (i > 0) {
      previous_lane_ids = straight_forward_points[i - 1].lane_ids;
    }
    setLaneIdsToPathPoint(pt, all_lanes, previous_lane_ids);

    straight_forward_points.push_back(pt);
  }

  const Pose straight_end_pose = straight_poses.back();

  // lateral_offset: returns positive value when straight_end_pose.position is on the left side
  // of the trajectory segment
  const double lateral_offset = centerline_path.points.empty()
                                  ? 0.0
                                  : autoware::motion_utils::calcLateralOffset(
                                      centerline_path.points, straight_end_pose.position);

  // =====================================================================
  // STEP 5: 各ステア角度での処理ループ
  // =====================================================================
  for (const auto & steer_angle : max_steer_angle) {
    // ===================================================================
    // STEP 5-1: クロソイドパス生成
    // ===================================================================
    // Calculate minimum radius based on the maximum steer angle
    const double minimum_radius = wheel_base / std::tan(steer_angle);

    const double longitudinal_distance =
      calc_necessary_longitudinal_distance(-lateral_offset, minimum_radius);

    const Pose target_pose = start_planner_utils::findTargetPoseAlongPath(
      centerline_path, straight_end_pose, longitudinal_distance);

    // TODO(Sugahara): ここでlateral_offset がプラスな場合は直進経路でよい。
    const auto relative_pose_info =
      start_planner_utils::calculateRelativePoseInVehicleCoordinate(straight_end_pose, target_pose);

    const auto circular_path = calc_circular_path(
      straight_end_pose, relative_pose_info.longitudinal_distance_vehicle,
      relative_pose_info.lateral_distance_vehicle, relative_pose_info.angle_diff, minimum_radius);

    if (circular_path.segments.empty()) {
      std::cerr << "No circular path segments found for steer angle " << steer_angle * 180.0 / M_PI
                << " deg." << std::endl;
      continue;
    }

    geometry_msgs::msg::Pose current_segment_pose = straight_end_pose;
    std::vector<std::vector<geometry_msgs::msg::Point>> clothoid_paths;

    // 円弧パスをクロソイド曲線に変換
    clothoid_paths = convertCircularPathToClothoidPaths(
      circular_path, current_segment_pose, initial_velocity, wheel_base, max_steer_angle_rate,
      parameters_.center_line_path_interval);

    if (clothoid_paths.empty()) {
      std::cerr << "Failed to convert circular path to clothoid paths for steer angle "
                << steer_angle * 180.0 / M_PI << " deg." << std::endl;
      continue;
    }

    // ===================================================================
    // STEP 5-2: 目標速度の取得とパス結合・リサンプリング
    // ===================================================================
    // 目標速度を取得（centerline_pathからtarget_poseに最も近い点の速度を使用）
    double target_velocity = initial_velocity;  // デフォルト値
    if (!centerline_path.points.empty()) {
      const auto target_idx =
        autoware::motion_utils::findNearestIndex(centerline_path.points, target_pose.position);
      if (target_idx < centerline_path.points.size()) {
        target_velocity = centerline_path.points[target_idx].point.longitudinal_velocity_mps;
      }
    }

    // クロソイドパスをセンターラインに結合
    PathWithLaneId path_with_lane_id = createPathWithLaneIdFromClothoidPaths(
      clothoid_paths, target_pose, initial_velocity, target_velocity, acceleration, all_lanes,
      route_handler);

    // センターラインパスとの結合
    auto combined_path = combinePathWithCenterline(path_with_lane_id, centerline_path, target_pose);

    PathWithLaneId resampled_combined_path =
      utils::resamplePathWithSpline(combined_path, parameters_.center_line_path_interval);

    // ===================================================================
    // STEP 5-3: 最終パスの作成（前後直進パスとの結合、yaw角の再計算）
    // ===================================================================
    PathWithLaneId final_path;
    final_path.header = resampled_combined_path.header;
    final_path.points = straight_forward_points;  // 前後直進パス（統合済み）

    // クロソイドパス + センターライン拡張パスを追加（重複点を除去）
    if (!resampled_combined_path.points.empty()) {
      // 重複を避けるため、最初の点をスキップして追加
      for (size_t i = 1; i < resampled_combined_path.points.size(); ++i) {
        final_path.points.push_back(resampled_combined_path.points[i]);
      }
    }

    // final_pathの座標情報を元にyaw角を再計算
    for (size_t i = 0; i < final_path.points.size(); ++i) {
      if (i < final_path.points.size() - 1) {
        // 次の点への方向を計算
        const double dx = final_path.points[i + 1].point.pose.position.x -
                          final_path.points[i].point.pose.position.x;
        const double dy = final_path.points[i + 1].point.pose.position.y -
                          final_path.points[i].point.pose.position.y;
        const double yaw = std::atan2(dy, dx);
        final_path.points[i].point.pose.orientation =
          autoware::universe_utils::createQuaternionFromYaw(yaw);
      } else {
        // 最後の点は前の点との方向から計算
        if (final_path.points.size() >= 2) {
          const double dx = final_path.points[i].point.pose.position.x -
                            final_path.points[i - 1].point.pose.position.x;
          const double dy = final_path.points[i].point.pose.position.y -
                            final_path.points[i - 1].point.pose.position.y;
          const double yaw = std::atan2(dy, dx);
          final_path.points[i].point.pose.orientation =
            autoware::universe_utils::createQuaternionFromYaw(yaw);
        } else {
          // 1点しかない場合は単位クォータニオン
          final_path.points[i].point.pose.orientation.x = 0.0;
          final_path.points[i].point.pose.orientation.y = 0.0;
          final_path.points[i].point.pose.orientation.z = 0.0;
          final_path.points[i].point.pose.orientation.w = 1.0;
        }
      }
    }

    // ===================================================================
    // STEP 5-4: 車線逸脱判定とパス検証
    // ===================================================================
    const auto lanelet_map_ptr = planner_data->route_handler->getLaneletMapPtr();

    std::vector<lanelet::Id> fused_id_start_to_end{};
    std::optional<autoware_utils::Polygon2d> fused_polygon_start_to_end = std::nullopt;

    std::vector<lanelet::Id> fused_id_crop_points{};
    std::optional<autoware_utils::Polygon2d> fused_polygon_crop_points = std::nullopt;

    // clothoid path is not separate but only one.
    auto & clothoid_path = final_path;

    // check lane_departure with path between pull_out_start to pull_out_end
    PathWithLaneId path_clothoid_start_to_end{};
    {
      const size_t pull_out_start_idx =
        autoware::motion_utils::findNearestIndex(clothoid_path.points, start_pose.position);
      const size_t pull_out_end_idx =
        autoware::motion_utils::findNearestIndex(clothoid_path.points, target_pose.position);

      path_clothoid_start_to_end.points.insert(
        path_clothoid_start_to_end.points.begin(),
        clothoid_path.points.begin() + pull_out_start_idx,
        clothoid_path.points.begin() + pull_out_end_idx + 1);
    }

    // check lane departure
    if (
      parameters_.check_clothoid_path_lane_departure &&
      boundary_departure_checker_->checkPathWillLeaveLane(
        lanelet_map_ptr, path_clothoid_start_to_end, fused_id_start_to_end,
        fused_polygon_start_to_end)) {
      std::cerr << "Lane departure detected for steer angle " << steer_angle * 180.0 / M_PI
                << " deg. Continuing to next candidate." << std::endl;
      planner_debug_data.conditions_evaluation.emplace_back("lane departure");
      continue;
    }

    // crop backward path
    const size_t start_segment_idx =
      autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(
        clothoid_path.points, start_pose, common_parameters.ego_nearest_dist_threshold,
        common_parameters.ego_nearest_yaw_threshold);

    PathWithLaneId cropped_path;
    if (parameters_.check_clothoid_path_lane_departure) {
      // レーン外の点をcrop する意味がわからない
      cropped_path = boundary_departure_checker_->cropPointsOutsideOfLanes(
        lanelet_map_ptr, clothoid_path, start_segment_idx, fused_id_crop_points,
        fused_polygon_crop_points);
      if (cropped_path.points.empty()) {
        std::cerr << "Cropped path is empty for steer angle " << steer_angle * 180.0 / M_PI
                  << " deg. Continuing to next candidate." << std::endl;
        planner_debug_data.conditions_evaluation.emplace_back("cropped path is empty");
        continue;
      }
    } else {
      // If lane departure check is disabled, use the original path without cropping
      cropped_path = clothoid_path;
    }

    // check that the path is not cropped in excess and there is not excessive longitudinal
    // deviation between the first 2 points
    auto validate_cropped_path = [&](const auto & cropped_path) -> bool {
      if (cropped_path.points.size() < 2) return false;
      const double max_long_offset = parameters_.maximum_longitudinal_deviation;
      const size_t start_segment_idx_after_crop =
        autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(
          cropped_path.points, start_pose);

      // if the start segment id after crop is not 0, then the cropping is not excessive
      if (start_segment_idx_after_crop != 0) return true;

      const auto long_offset_to_closest_point =
        autoware::motion_utils::calcLongitudinalOffsetToSegment(
          cropped_path.points, start_segment_idx_after_crop, start_pose.position);
      const auto long_offset_to_next_point =
        autoware::motion_utils::calcLongitudinalOffsetToSegment(
          cropped_path.points, start_segment_idx_after_crop + 1, start_pose.position);
      return std::abs(long_offset_to_closest_point - long_offset_to_next_point) < max_long_offset;
    };

    if (parameters_.check_clothoid_path_lane_departure && !validate_cropped_path(cropped_path)) {
      std::cerr << "Cropped path is invalid for steer angle " << steer_angle * 180.0 / M_PI
                << " deg. Continuing to next candidate." << std::endl;
      planner_debug_data.conditions_evaluation.emplace_back("cropped path is invalid");
      continue;
    }

    // Update the final path with cropped path
    clothoid_path.points = cropped_path.points;
    clothoid_path.header = planner_data->route_handler->getRouteHeader();

    // ===================================================================
    // STEP 5-5: 衝突判定
    // ===================================================================
    // Create PullOutPath for collision check
    PullOutPath temp_pull_out_path;
    temp_pull_out_path.partial_paths.push_back(clothoid_path);
    temp_pull_out_path.start_pose =
      clothoid_path.points.empty() ? start_pose : clothoid_path.points.front().point.pose;
    temp_pull_out_path.end_pose = target_pose;

    if (isPullOutPathCollided(
          temp_pull_out_path, planner_data, parameters_.shift_collision_check_distance_from_end)) {
      std::cerr << "Collision detected for steer angle " << steer_angle * 180.0 / M_PI
                << " deg. Continuing to next candidate." << std::endl;
      planner_debug_data.conditions_evaluation.emplace_back("collision");
      continue;
    }

    // ===================================================================
    // STEP 5-6: 成功時の結果返却
    // ===================================================================
    // 検証に成功したら、最終的なPullOutPathを作成して返す
    PullOutPath pull_out_path;
    pull_out_path.pairs_terminal_velocity_and_accel.push_back(
      std::make_pair(initial_velocity, acceleration));
    pull_out_path.partial_paths.push_back(clothoid_path);  // Use validated and cropped path

    pull_out_path.start_pose =
      clothoid_path.points.empty() ? start_pose : clothoid_path.points.front().point.pose;
    pull_out_path.end_pose = target_pose;

    RCLCPP_ERROR(
      rclcpp::get_logger("clothoid_pull_out"),
      "\n===========================================\n"
      "Successfully generated clothoid pull-out path with steer angle %.2f deg.\n"
      "===========================================",
      steer_angle * 180.0 / M_PI);

    planner_debug_data.conditions_evaluation.emplace_back("success");
    return pull_out_path;
  }

  // =====================================================================
  // STEP 6: 経路が生成できなかった場合
  // =====================================================================
  planner_debug_data.conditions_evaluation.emplace_back("no path found");
  return std::nullopt;
}

}  // namespace autoware::behavior_path_planner
