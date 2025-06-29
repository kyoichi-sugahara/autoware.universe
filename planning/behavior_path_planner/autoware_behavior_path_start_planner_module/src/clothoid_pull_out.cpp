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

// クロソイドセグメント構造体の定義
struct ClothoidSegment
{
  enum Type { CLOTHOID_ENTRY, CIRCULAR_ARC, CLOTHOID_EXIT };

  Type type;
  double A;           // クロソイドパラメータ
  double L;           // 弧長
  double radius;      // 半径（円弧セグメント用）
  double angle;       // 角度（円弧セグメント用）
  bool is_clockwise;  // 回転方向
  std::string description;

  explicit ClothoidSegment(Type t, double a = 0.0, double l = 0.0)
  : type(t), A(a), L(l), radius(0.0), angle(0.0), is_clockwise(true)
  {
  }
};

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

  std::cerr << "\n=== Rigid Transform Correction ===" << std::endl;

  // 1. 現在のクロソイドの幾何学的特性を取得
  auto clothoid_start = clothoid_points.front();
  auto clothoid_end = clothoid_points.back();

  // 目標の開始・終了位置を取得
  auto target_start = start_pose.position;
  auto target_end = original_segment.getPointAtAngle(original_segment.getEndAngle());

  std::cerr << "Target start: (" << target_start.x << ", " << target_start.y << ")" << std::endl;
  std::cerr << "Target end: (" << target_end.x << ", " << target_end.y << ")" << std::endl;
  std::cerr << "Clothoid start: (" << clothoid_start.x << ", " << clothoid_start.y << ")"
            << std::endl;
  std::cerr << "Clothoid end: (" << clothoid_end.x << ", " << clothoid_end.y << ")" << std::endl;

  // 2. 方向ベクトルを計算
  double clothoid_dx = clothoid_end.x - clothoid_start.x;
  double clothoid_dy = clothoid_end.y - clothoid_start.y;
  double clothoid_length = std::sqrt(clothoid_dx * clothoid_dx + clothoid_dy * clothoid_dy);

  double target_dx = target_end.x - target_start.x;
  double target_dy = target_end.y - target_start.y;
  double target_length = std::sqrt(target_dx * target_dx + target_dy * target_dy);

  std::cerr << "Clothoid vector: (" << clothoid_dx << ", " << clothoid_dy
            << "), length: " << clothoid_length << std::endl;
  std::cerr << "Target vector: (" << target_dx << ", " << target_dy
            << "), length: " << target_length << std::endl;

  // 3. スケーリング係数を計算
  double scale_factor = (clothoid_length > 1e-10) ? target_length / clothoid_length : 1.0;
  std::cerr << "Scale factor: " << scale_factor << std::endl;

  // 4. 回転角度を計算
  double clothoid_angle = std::atan2(clothoid_dy, clothoid_dx);
  double target_angle = std::atan2(target_dy, target_dx);
  double rotation_angle = target_angle - clothoid_angle;

  // 角度を [-π, π] の範囲に正規化
  while (rotation_angle > M_PI) rotation_angle -= 2 * M_PI;
  while (rotation_angle < -M_PI) rotation_angle += 2 * M_PI;

  std::cerr << "Clothoid angle: " << clothoid_angle * 180.0 / M_PI << " deg" << std::endl;
  std::cerr << "Target angle: " << target_angle * 180.0 / M_PI << " deg" << std::endl;
  std::cerr << "Rotation angle: " << rotation_angle * 180.0 / M_PI << " deg" << std::endl;

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

  // 7. 変換結果の検証
  auto final_start = corrected_points.front();
  auto final_end = corrected_points.back();

  double start_error = std::sqrt(
    (final_start.x - target_start.x) * (final_start.x - target_start.x) +
    (final_start.y - target_start.y) * (final_start.y - target_start.y));

  double end_error = std::sqrt(
    (final_end.x - target_end.x) * (final_end.x - target_end.x) +
    (final_end.y - target_end.y) * (final_end.y - target_end.y));

  std::cerr << "Final start: (" << final_start.x << ", " << final_start.y << ")" << std::endl;
  std::cerr << "Final end: (" << final_end.x << ", " << final_end.y << ")" << std::endl;
  std::cerr << "Start error: " << start_error << " m" << std::endl;
  std::cerr << "End error: " << end_error << " m" << std::endl;
  std::cerr << "====================================" << std::endl;

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

  std::cerr << "\n=== Clothoid Entry Point Generation (Numerical Integration) ===" << std::endl;
  std::cerr << "Start pose: (" << start_pose.position.x << ", " << start_pose.position.y
            << "), psi=" << start_yaw << " rad" << std::endl;
  std::cerr << "Parameters: A=" << A << ", L=" << L << ", direction_factor=" << direction_factor
            << std::endl;
  std::cerr << "Start curvature: " << start_curvature << " (1/m)" << std::endl;
  std::cerr << "Target curvature: " << target_curvature << " (1/m)" << std::endl;
  std::cerr << "Number of points: " << num_points << std::endl;

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

    // デバッグ出力（最初の数点と最後の数点のみ）
    if (i <= 3 || i >= num_points - 3) {
      std::cerr << "Point " << i << "/" << (num_points - 1) << ": progress=" << progress
                << ", current_curvature=" << current_curvature << std::endl;
      std::cerr << "  current_psi=" << current_psi << " rad (" << current_psi * 180.0 / M_PI
                << " deg)" << std::endl;
      std::cerr << "  Position: (" << point.x << ", " << point.y << ")" << std::endl;
    }

    if (i < num_points - 1) {
      double ds = L / (num_points - 1);  // 微小区間

      // 数値積分による座標更新
      current_x += std::cos(current_psi) * ds;
      current_y += std::sin(current_psi) * ds;
      current_psi += current_curvature * ds;

      // デバッグ出力（座標更新後）
      if (i <= 2 || i >= num_points - 4) {
        std::cerr << "  ds=" << ds << ", updated position: (" << current_x << ", " << current_y
                  << "), updated_psi=" << current_psi << " rad" << std::endl;
      }
    }
  }

  // 終端状態
  double final_curvature = target_curvature;
  double final_psi = current_psi;

  geometry_msgs::msg::Pose end_pose;
  end_pose.position = points.back();
  end_pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), final_psi));

  std::cerr << "Final pose: (" << end_pose.position.x << ", " << end_pose.position.y
            << "), psi=" << final_psi << " rad (" << final_psi * 180.0 / M_PI << " deg)"
            << std::endl;
  std::cerr << "Final curvature: " << final_curvature << " (1/m)" << std::endl;
  std::cerr << "=========================================" << std::endl;

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

  std::cerr << "\n=== Circular Segment Point Generation ===" << std::endl;
  std::cerr << "Start pose: (" << start_pose.position.x << ", " << start_pose.position.y
            << "), psi=" << start_yaw << " rad" << std::endl;
  std::cerr << "Parameters: angle=" << angle << ", direction_factor=" << direction_factor
            << std::endl;
  std::cerr << "Arc center: (" << center_x << ", " << center_y << ")" << std::endl;
  std::cerr << "Number of points: " << num_points << std::endl;

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
  double final_curvature = (1.0 / radius) * direction_factor;

  geometry_msgs::msg::Pose end_pose;
  end_pose.position = points.back();
  end_pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), final_psi));

  std::cerr << "Final pose: (" << end_pose.position.x << ", " << end_pose.position.y
            << "), psi=" << final_psi << " rad, curvature=" << final_curvature << std::endl;
  std::cerr << "=========================================" << std::endl;

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

  std::cerr << "\n=== Clothoid Exit Point Generation ===" << std::endl;
  std::cerr << "Start pose: (" << start_pose.position.x << ", " << start_pose.position.y
            << "), psi=" << start_yaw << " rad" << std::endl;
  std::cerr << "Parameters: L=" << L << ", direction_factor=" << direction_factor << std::endl;
  std::cerr << "Start curvature: " << start_curvature << " (1/m)" << std::endl;
  std::cerr << "Number of points: " << num_points << std::endl;

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

  std::cerr << "Final pose: (" << end_pose.position.x << ", " << end_pose.position.y
            << "), psi=" << current_psi << " rad (final curvature should be 0)" << std::endl;
  std::cerr << "=========================================" << std::endl;

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
      std::cerr << "clothoid_entry" << std::endl;
      auto result = generateClothoidEntry(segments[i], current_pose, num_points);
      segment_points = result.first;
      end_pose = result.second;
    } else if (segments[i].type == ClothoidSegment::CIRCULAR_ARC) {
      std::cerr << "circular_arc" << std::endl;
      auto result = generateCircularSegment(segments[i], current_pose, num_points);
      segment_points = result.first;
      end_pose = result.second;
    } else if (segments[i].type == ClothoidSegment::CLOTHOID_EXIT) {
      std::cerr << "exit_clothoid" << std::endl;
      // 前のセグメントがある場合はそのポインタを渡す
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
  double L_min, int num_points_per_segment = 50)
{
  std::cerr << "\n=== Arc to Clothoid Conversion ===" << std::endl;

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

  std::cerr << "Clothoid parameters:" << std::endl;
  std::cerr << "  A: " << A << std::endl;
  std::cerr << "  L: " << L << " m" << std::endl;
  std::cerr << "  Single clothoid angle α: " << alpha_clothoid * 180.0 / M_PI << "° ("
            << alpha_clothoid << " rad)" << std::endl;
  std::cerr << "  2 * α: " << 2.0 * alpha_clothoid * 180.0 / M_PI << "° (" << 2.0 * alpha_clothoid
            << " rad)" << std::endl;

  // Case分類（Case Aのみ実装）
  std::cerr << "\nCase Classification:" << std::endl;
  std::cerr << "  Condition: total_angle >= 2*α_clothoid?" << std::endl;
  std::cerr << "  " << total_angle * 180.0 / M_PI << "° >= " << 2.0 * alpha_clothoid * 180.0 / M_PI
            << "° → " << (total_angle >= 2.0 * alpha_clothoid ? "true" : "false") << std::endl;

  std::vector<ClothoidSegment> segments;

  if (total_angle >= 2.0 * alpha_clothoid) {
    // Case A: CAC(A, L, θ)
    double theta_arc = total_angle - 2.0 * alpha_clothoid;
    std::cerr << "  → CASE A: Standard CAC sequence" << std::endl;
    std::cerr << "    θ_arc (circular arc angle): " << theta_arc * 180.0 / M_PI << "° ("
              << theta_arc << " rad)" << std::endl;

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
  double L_min, int num_points_per_segment = 50)
{
  // 元のクロソイド変換を実行
  auto clothoid_points =
    convertArcToClothoid(arc_segment, start_pose, A_min, L_min, num_points_per_segment);

  if (clothoid_points.empty()) {
    std::cerr << "Clothoid conversion failed!" << std::endl;
    return clothoid_points;
  }

  std::cerr << "\n=== Applying Endpoint Correction ===" << std::endl;

  // 終点補正を適用
  auto corrected_points = correctClothoidByRigidTransform(clothoid_points, arc_segment, start_pose);

  return corrected_points;
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
  // const std::vector<double> max_steer_angle_degs = {20.0, 30.0, 40.0};
  // const std::vector<double> max_steer_angle = {
  //   max_steer_angle_degs[0] * M_PI / 180.0, max_steer_angle_degs[1] * M_PI / 180.0,
  //   max_steer_angle_degs[2] * M_PI / 180.0};
  const std::vector<double> max_steer_angle_degs = {20.0};
  const std::vector<double> max_steer_angle = {max_steer_angle_degs[0] * M_PI / 180.0};

  const double max_steer_angle_rate_deg_per_sec = 10.0;  // Assume a constant rate for simplicity
  const double max_steer_angle_rate = max_steer_angle_rate_deg_per_sec * M_PI / 180.0;
  const double velocity = 1.0;  // Assume a constant velocity for the pull-out maneuver
  const double wheel_base = planner_data->parameters.vehicle_info.wheel_base_m;

  for (const auto & steer_angle : max_steer_angle) {
    // Calculate minimum radius based on the maximum steer angle
    const double minimum_radius = wheel_base / std::tan(steer_angle);
    // std::cerr << "Minimum radius for steer angle " << steer_angle * 180.0 / M_PI
    //           << " deg: " << minimum_radius << std::endl;

    // Calculate longitudinal necessary distance for pull out
    const double longitudinal_distance =
      start_planner_utils::calc_necessary_longitudinal_distance(-lateral_offset, minimum_radius);
    // std::cerr << "Longitudinal distance for steer angle " << steer_angle * 180.0 / M_PI
    //           << " deg: " << longitudinal_distance << std::endl;
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

    // std::cerr << "start_pose: " << start_pose.position.x << ", " << start_pose.position.y << ", "
    //           << tf2::getYaw(start_pose.orientation) << std::endl;
    // std::cerr << "target_pose: " << target_pose.position.x << ", " << target_pose.position.y <<
    // ", "
    //           << tf2::getYaw(target_pose.orientation) << std::endl;

    const double dx = target_pose.position.x - start_pose.position.x;
    const double dy = target_pose.position.y - start_pose.position.y;
    const double start_yaw = tf2::getYaw(start_pose.orientation);
    const double target_yaw = tf2::getYaw(target_pose.orientation);

    // Transform to vehicle coordinate system (x: forward, y: left)
    const double longitudinal_distance_vehicle =
      dx * std::cos(start_yaw) + dy * std::sin(start_yaw);
    const double lateral_distance_vehicle = -dx * std::sin(start_yaw) + dy * std::cos(start_yaw);

    // Calculate angle difference
    double angle_diff = target_yaw - start_yaw;
    // Normalize angle to [-pi, pi]
    while (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;
    while (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;
    // std::cerr << "Vehicle coordinate relative position:" << std::endl;
    // std::cerr << "  Longitudinal (forward): " << longitudinal_distance_vehicle << " m" <<
    // std::endl; std::cerr << "  Lateral (left): " << lateral_distance_vehicle << " m" <<
    // std::endl; std::cerr << "  Angle difference: " << angle_diff << " rad (" << angle_diff *
    // 180.0 / M_PI
    //           << " deg)" << std::endl;
    const auto circular_path = start_planner_utils::calc_circular_path(
      start_pose, longitudinal_distance_vehicle, lateral_distance_vehicle, angle_diff,
      minimum_radius);

    // circular_pathが空の場合は処理を終了
    if (circular_path.segments.empty()) {
      return std::nullopt;
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

            std::cerr << "Updated pose for next segment:" << std::endl;
            std::cerr << "  Position: (" << current_segment_pose.position.x << ", "
                      << current_segment_pose.position.y << ")" << std::endl;
            std::cerr << "  Heading: " << heading << " rad (" << heading * 180.0 / M_PI << " deg)"
                      << std::endl;
          }
        }
      } else {
        std::cerr << "Failed to convert segment " << (i + 1) << " to clothoid" << std::endl;
      }
    }

    // クロソイドパスが生成された場合、PathWithLaneIdを作成
    if (!clothoid_paths.empty()) {
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
      path_with_lane_id.header = planner_data->route_handler->getRouteHeader();

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
        path_point.point.longitudinal_velocity_mps = 5.0;  // 5 m/s
        path_point.point.lateral_velocity_mps = 0.0;
        path_point.point.heading_rate_rps = 0.0;
        path_point.point.is_final = (i == all_clothoid_points.size() - 1);

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

      // PullOutPathを作成
      PullOutPath pull_out_path;
      pull_out_path.partial_paths.push_back(path_with_lane_id);
      pull_out_path.start_pose = start_pose;
      pull_out_path.end_pose = target_pose;

      // 速度と加速度のペア設定
      // TODO(Sugahara): set parameter properly
      pull_out_path.pairs_terminal_velocity_and_accel.push_back(std::make_pair(5.0, 1.0));

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

            // end_poseを最終点に更新
            if (!combined_path.points.empty()) {
              pull_out_path.end_pose = combined_path.points.back().point.pose;
            }

            std::cerr << "Successfully connected to centerline path. Combined path points: "
                      << combined_path.points.size() << std::endl;
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
