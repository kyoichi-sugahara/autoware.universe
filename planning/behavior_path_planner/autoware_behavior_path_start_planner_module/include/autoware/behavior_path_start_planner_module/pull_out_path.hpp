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

#ifndef AUTOWARE__BEHAVIOR_PATH_START_PLANNER_MODULE__PULL_OUT_PATH_HPP_
#define AUTOWARE__BEHAVIOR_PATH_START_PLANNER_MODULE__PULL_OUT_PATH_HPP_

#include "autoware/behavior_path_planner_common/utils/path_shifter/path_shifter.hpp"

#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace autoware::behavior_path_planner
{
using autoware_internal_planning_msgs::msg::PathWithLaneId;
using geometry_msgs::msg::Pose;

struct PullOutPath
{
  std::vector<PathWithLaneId> partial_paths{};
  // accelerate with constant acceleration to the target velocity
  std::vector<std::pair<double, double>> pairs_terminal_velocity_and_accel{};
  Pose start_pose{};
  Pose end_pose{};
};

/**
 * @brief 姿勢ベースの円弧セグメントを表現する構造体
 */
struct ArcSegment
{
  // 円弧の幾何学的パラメータ
  geometry_msgs::msg::Point center;     // 円弧の中心点
  double radius;                        // 半径 [m]
  geometry_msgs::msg::Pose start_pose;  // 開始姿勢
  geometry_msgs::msg::Pose end_pose;    // 終了姿勢
  bool is_clockwise;                    // 時計回りかどうか

  ArcSegment() : radius(0.0), is_clockwise(true)
  {
    center.x = center.y = center.z = 0.0;
    // start_pose, end_poseはデフォルトで初期化される
  }

  /**
   * @brief 開始角度を計算
   * @return 開始角度 [rad]
   */
  double getStartAngle() const
  {
    return std::atan2(start_pose.position.y - center.y, start_pose.position.x - center.x);
  }

  /**
   * @brief 終了角度を計算
   * @return 終了角度 [rad]
   */
  double getEndAngle() const
  {
    return std::atan2(end_pose.position.y - center.y, end_pose.position.x - center.x);
  }

  /**
   * @brief 円弧長を計算
   * @return 円弧長 [m]
   */
  double calculateArcLength() const
  {
    double start_angle = getStartAngle();
    double end_angle = getEndAngle();
    double angle_diff = std::abs(end_angle - start_angle);

    // 角度差が2πを超える場合の調整
    if (angle_diff > 2.0 * M_PI) {
      angle_diff = 2.0 * M_PI - std::fmod(angle_diff, 2.0 * M_PI);
    }
    return radius * angle_diff;
  }

  /**
   * @brief 曲率を取得（円弧では一定）
   * @return 曲率 [1/m]
   */
  double getCurvature() const { return (radius > 0.0) ? (1.0 / radius) : 0.0; }

  /**
   * @brief 指定した角度での位置を計算
   * @param angle 角度 [rad]
   * @return 位置
   */
  geometry_msgs::msg::Point getPointAtAngle(double angle) const
  {
    geometry_msgs::msg::Point point;
    point.x = center.x + radius * std::cos(angle);
    point.y = center.y + radius * std::sin(angle);
    point.z = center.z;
    return point;
  }

  /**
   * @brief 開始位置を取得
   * @return 開始位置
   */
  geometry_msgs::msg::Point getStartPoint() const { return start_pose.position; }

  /**
   * @brief 終了位置を取得
   * @return 終了位置
   */
  geometry_msgs::msg::Point getEndPoint() const { return end_pose.position; }

  /**
   * @brief 指定した角度での姿勢を計算
   * @param angle 角度 [rad]
   * @return 姿勢
   */
  geometry_msgs::msg::Pose getPoseAtAngle(double angle) const
  {
    geometry_msgs::msg::Pose pose;

    // 位置を計算
    pose.position = getPointAtAngle(angle);

    // 接線方向を計算（円弧の進行方向）
    double tangent_angle = angle + (is_clockwise ? -M_PI / 2 : M_PI / 2);

    // クォータニオンを設定
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = std::sin(tangent_angle / 2.0);
    pose.orientation.w = std::cos(tangent_angle / 2.0);

    return pose;
  }

  /**
   * @brief 開始姿勢を取得
   * @return 開始姿勢
   */
  geometry_msgs::msg::Pose getStartPose() const { return start_pose; }

  /**
   * @brief 終了姿勢を取得
   * @return 終了姿勢
   */
  geometry_msgs::msg::Pose getEndPose() const { return end_pose; }
};

/**
 * @brief 複数の円弧セグメントからなる複合円弧経路
 */
struct CompositeArcPath
{
  std::vector<ArcSegment> segments;  // 円弧セグメントの配列

  CompositeArcPath() = default;

  double calculateTotalLength() const
  {
    double length = 0.0;
    for (const auto & segment : segments) {
      length += segment.calculateArcLength();
    }
    return length;
  }
};

}  // namespace autoware::behavior_path_planner
#endif  // AUTOWARE__BEHAVIOR_PATH_START_PLANNER_MODULE__PULL_OUT_PATH_HPP_
