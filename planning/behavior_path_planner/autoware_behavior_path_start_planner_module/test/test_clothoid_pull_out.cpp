// Copyright 2024 TIER IV, Inc.
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

#include "start_planner_test_helper.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware/behavior_path_start_planner_module/clothoid_pull_out.hpp>
#include <autoware/behavior_path_start_planner_module/start_planner_module.hpp>
#include <autoware/behavior_path_start_planner_module/util.hpp>
#include <autoware/planning_test_manager/autoware_planning_test_manager_utils.hpp>
#include <autoware/route_handler/route_handler.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>
#include <autoware_utils/geometry/boost_geometry.hpp>
#include <autoware_utils/geometry/geometry.hpp>

#include <gtest/gtest.h>
#include <matplotlibcpp17/pyplot.h>
#include <pybind11/pytypes.h>
#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <numeric>
#include <optional>
#include <string>
#include <utility>
#include <vector>

using autoware::behavior_path_planner::ClothoidPullOut;
using autoware::behavior_path_planner::StartPlannerParameters;
using autoware::test_utils::get_absolute_path_to_config;
using autoware_planning_msgs::msg::LaneletRoute;
using RouteSections = std::vector<autoware_planning_msgs::msg::LaneletSegment>;
using autoware::behavior_path_planner::testing::StartPlannerTestHelper;
using autoware_planning_test_manager::utils::makeBehaviorRouteFromLaneId;

namespace autoware::behavior_path_planner
{

// Declaration of plot_footprint
void plot_footprint(
  matplotlibcpp17::axes::Axes & axes, const geometry_msgs::msg::Pose & pose,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, const std::string & color,
  const double alpha);

void plot_path_with_lane_id(
  matplotlibcpp17::axes::Axes & axes,
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path,
  const std::string & color = "red", const std::string & label = "", const double linewidth = 1.0,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info =
    autoware::vehicle_info_utils::VehicleInfo(),
  const bool draw_footprint = false)
{
  std::vector<double> xs, ys;
  std::vector<double> yaw_cos, yaw_sin;
  for (const auto & point : path.points) {
    xs.push_back(point.point.pose.position.x);
    ys.push_back(point.point.pose.position.y);
    const double yaw = autoware_utils::get_rpy(point.point.pose).z;
    yaw_cos.push_back(std::cos(yaw));
    yaw_sin.push_back(std::sin(yaw));
    axes.scatter(
      Args(xs.back(), ys.back()), Kwargs("marker"_a = "o", "color"_a = "blue", "s"_a = 10));

    // Draw footprint
    if (draw_footprint) {
      plot_footprint(axes, point.point.pose, vehicle_info, "blue", 0.1);
    }
  }
  axes.quiver(
    Args(xs, ys, yaw_cos, yaw_sin),
    Kwargs("angles"_a = "xy", "scale_units"_a = "xy", "scale"_a = 2.0));

  if (label == "") {
    axes.plot(Args(xs, ys), Kwargs("color"_a = color, "linewidth"_a = linewidth));
  } else {
    axes.plot(
      Args(xs, ys), Kwargs("color"_a = color, "linewidth"_a = linewidth, "label"_a = label));
  }
}

void plot_lanelet(
  matplotlibcpp17::axes::Axes & axes, lanelet::ConstLanelet lanelet,
  const std::string & color = "blue", const double linewidth = 0.5)
{
  const auto lefts = lanelet.leftBound();
  const auto rights = lanelet.rightBound();
  std::vector<double> xs_left, ys_left;
  for (const auto & point : lefts) {
    xs_left.push_back(point.x());
    ys_left.push_back(point.y());
  }

  std::vector<double> xs_right, ys_right;
  for (const auto & point : rights) {
    xs_right.push_back(point.x());
    ys_right.push_back(point.y());
  }

  std::vector<double> xs_center, ys_center;
  for (const auto & point : lanelet.centerline()) {
    xs_center.push_back(point.x());
    ys_center.push_back(point.y());
  }

  axes.plot(Args(xs_left, ys_left), Kwargs("color"_a = color, "linewidth"_a = linewidth));
  axes.plot(Args(xs_right, ys_right), Kwargs("color"_a = color, "linewidth"_a = linewidth));
  axes.plot(
    Args(xs_center, ys_center),
    Kwargs("color"_a = "black", "linewidth"_a = linewidth, "linestyle"_a = "dashed"));
}

void plot_footprint(
  matplotlibcpp17::axes::Axes & axes, const geometry_msgs::msg::Pose & pose,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, const std::string & color,
  const double alpha)
{
  // Calculate vehicle footprint
  const double base_to_front = vehicle_info.front_overhang_m + vehicle_info.wheel_base_m;
  const double base_to_rear = vehicle_info.rear_overhang_m;
  const double width = vehicle_info.vehicle_width_m;
  const double half_width = width / 2.0;

  // Relative coordinates of the four corners of the footprint
  std::vector<std::pair<double, double>> relative_points = {
    {base_to_front, half_width},   // Front right
    {base_to_front, -half_width},  // Front left
    {-base_to_rear, -half_width},  // Rear left
    {-base_to_rear, half_width},   // Rear right
  };

  // Calculate rotation matrix
  const double yaw = autoware_utils::get_rpy(pose).z;
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);

  // Transform footprint points
  std::vector<double> xs, ys;
  for (const auto & point : relative_points) {
    // Rotation
    const double rotated_x = point.first * cos_yaw - point.second * sin_yaw;
    const double rotated_y = point.first * sin_yaw + point.second * cos_yaw;
    // Translation
    xs.push_back(rotated_x + pose.position.x);
    ys.push_back(rotated_y + pose.position.y);
  }
  // Add the first point at the end to close the polygon
  xs.push_back(xs.front());
  ys.push_back(ys.front());

  // Draw footprint
  axes.fill(Args(xs, ys), Kwargs("color"_a = color, "alpha"_a = alpha));
}

class TestClothoidPullOut : public ::testing::Test
{
public:
  std::optional<PullOutPath> call_plan(
    const Pose & start_pose, const Pose & goal_pose,
    const std::shared_ptr<const PlannerData> & planner_data, PlannerDebugData & planner_debug_data)
  {
    return clothoid_pull_out_->plan(start_pose, goal_pose, planner_data, planner_debug_data);
  }

protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ =
      rclcpp::Node::make_shared("clothoid_pull_out", StartPlannerTestHelper::make_node_options());

    initialize_clothoid_pull_out_planner();
  }

  void TearDown() override { rclcpp::shutdown(); }

  // Member variables
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<ClothoidPullOut> clothoid_pull_out_;

private:
  void initialize_clothoid_pull_out_planner()
  {
    auto parameters = StartPlannerParameters::init(*node_);

    clothoid_pull_out_ = std::make_shared<ClothoidPullOut>(*node_, parameters);
  }
};

TEST_F(TestClothoidPullOut, DISABLED_GenerateValidClothoidPullOutPath)
{
  const auto start_pose =
    geometry_msgs::build<geometry_msgs::msg::Pose>()
      .position(geometry_msgs::build<geometry_msgs::msg::Point>().x(362.181).y(362.164).z(100.000))
      .orientation(
        geometry_msgs::build<geometry_msgs::msg::Quaternion>().x(0.0).y(0.0).z(0.709650).w(
          0.704554));
  // const auto start_pose =
  //   geometry_msgs::build<geometry_msgs::msg::Pose>()
  //     .position(geometry_msgs::build<geometry_msgs::msg::Point>().x(299.462).y(354.701).z(100.000))
  //     .orientation(
  //       geometry_msgs::build<geometry_msgs::msg::Quaternion>().x(0.0).y(0.0).z(-0.769919).w(
  //         0.638141));
  const auto goal_pose =
    geometry_msgs::build<geometry_msgs::msg::Pose>()
      .position(geometry_msgs::build<geometry_msgs::msg::Point>().x(365.658).y(507.253).z(100.000))
      .orientation(
        geometry_msgs::build<geometry_msgs::msg::Quaternion>().x(0.0).y(0.0).z(0.705897).w(
          0.708314));

  auto planner_data = std::make_shared<PlannerData>();
  planner_data->init_parameters(*node_);
  StartPlannerTestHelper::set_odometry(planner_data, start_pose);
  StartPlannerTestHelper::set_route(planner_data, 4619, 4635);
  // StartPlannerTestHelper::set_route(planner_data, 675, 720);
  // Plan the pull out path
  PlannerDebugData debug_data;
  std::cerr << "Planning clothoid pull out path..." << std::endl;
  auto result = call_plan(start_pose, goal_pose, planner_data, debug_data);

  // Assert that a valid clothoid pull out path is generated
  // ASSERT_TRUE(result.has_value()) << "clothoid pull out path generation failed.";
  // EXPECT_EQ(result->partial_paths.size(), 1UL)
  //   << "Generated clothoid pull out path does not have the expected number of partial paths.";
  // EXPECT_EQ(debug_data.conditions_evaluation.back(), "success")
  //   << "clothoid pull out path planning did not succeed.";

  // Plot the generated path
  pybind11::scoped_interpreter guard{};
  auto plt = matplotlibcpp17::pyplot::import();
  auto [fig, axes] = plt.subplots(1, 1);
  auto & ax = axes[0];

  // Plot lanelets
  const auto & lanelets = planner_data->route_handler->getLaneletMapPtr()->laneletLayer;
  for (const auto & lanelet : lanelets) {
    plot_lanelet(ax, lanelet);
  }

  // Plot start and goal poses
  ax.plot(
    Args(start_pose.position.x, start_pose.position.y),
    Kwargs("marker"_a = "x", "label"_a = "start", "markersize"_a = 20, "color"_a = "green"));
  ax.plot(
    Args(goal_pose.position.x, goal_pose.position.y),
    Kwargs("marker"_a = "x", "label"_a = "goal", "markersize"_a = 20, "color"_a = "red"));

  // Plot footprints
  plot_footprint(ax, start_pose, planner_data->parameters.vehicle_info, "green", 0.3);
  plot_footprint(ax, goal_pose, planner_data->parameters.vehicle_info, "red", 0.3);

  // Plot generated path
  for (const auto & path : result->partial_paths) {
    plot_path_with_lane_id(
      ax, path, "blue", "generated path", 2.0, planner_data->parameters.vehicle_info, true);
  }
  // Set plot limits
  const double margin = 10.0;  // 10 meters margin
  const double x_min = std::min(start_pose.position.x, goal_pose.position.x) - margin;
  const double x_max = std::max(start_pose.position.x, goal_pose.position.x) + margin;
  const double y_min = std::min(start_pose.position.y, goal_pose.position.y) - margin;
  const double y_max = std::max(start_pose.position.y, goal_pose.position.y) + margin;
  ax.set_xlim(Args(x_min, x_max));
  ax.set_ylim(Args(y_min, y_max));

  ax.set_aspect(Args("equal"));

  // タイトルに座標範囲情報を追加
  std::string title = "Circular Path vs Clothoid Path Comparison\n";
  title += "X: [" + std::to_string(static_cast<int>(x_min)) + ", " +
           std::to_string(static_cast<int>(x_max)) + "] m, ";
  title += "Y: [" + std::to_string(static_cast<int>(y_min)) + ", " +
           std::to_string(static_cast<int>(y_max)) + "] m";
  ax.set_title(Args(title));
  ax.set_xlabel(Args("X [m]"));
  ax.set_ylabel(Args("Y [m]"));
  ax.legend();

  // グリッドを追加して視認性を向上
  ax.grid(Args(true), Kwargs("alpha"_a = 0.3));

  plt.show(Args(), Kwargs("block"_a = true));

  std::cerr << "=======================================" << std::endl;
}

/**
 * @brief 複数セグメント対応の改良版クロソイド変換関数
 */
std::vector<std::vector<geometry_msgs::msg::Point>> convertMultipleArcsToClothoidWithCorrection(
  const std::vector<ArcSegment> & arc_segments, const geometry_msgs::msg::Pose & initial_start_pose,
  const std::vector<double> & A_values, const std::vector<double> & L_values,
  int num_points_per_segment = 50)
{
  std::vector<std::vector<geometry_msgs::msg::Point>> corrected_clothoid_paths;

  // セグメント間の連続性を保つための姿勢管理
  geometry_msgs::msg::Pose current_segment_pose = initial_start_pose;

  for (size_t i = 0; i < arc_segments.size(); ++i) {
    const auto & segment = arc_segments[i];

    std::cerr << "\n--- Converting Arc Segment " << (i + 1) << "/" << arc_segments.size()
              << " with Correction ---" << std::endl;

    // パラメータの取得
    double A_min = (i < A_values.size()) ? A_values[i] : 50.0;
    double L_min = (i < L_values.size()) ? L_values[i] : 10.0;

    // 補正付きクロソイド変換を実行
    auto corrected_clothoid_points = convertArcToClothoidWithCorrection(
      segment, current_segment_pose, A_min, L_min, num_points_per_segment);

    if (!corrected_clothoid_points.empty()) {
      corrected_clothoid_paths.push_back(corrected_clothoid_points);

      // 次のセグメントのために終点姿勢を更新
      if (i < arc_segments.size() - 1) {
        const auto & last_point = corrected_clothoid_points.back();

        // 終点での進行方向を計算（最後の2点から）
        if (corrected_clothoid_points.size() >= 2) {
          const auto & second_last =
            corrected_clothoid_points[corrected_clothoid_points.size() - 2];
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
      std::cerr << "Failed to convert segment " << (i + 1) << " to clothoid with correction"
                << std::endl;
    }
  }

  return corrected_clothoid_paths;
}

TEST_F(TestClothoidPullOut, PlotCircularPathGeneration)
{
  const auto start_pose =
    geometry_msgs::build<geometry_msgs::msg::Pose>()
      .position(geometry_msgs::build<geometry_msgs::msg::Point>().x(362.181).y(362.164).z(100.000))
      .orientation(
        geometry_msgs::build<geometry_msgs::msg::Quaternion>().x(0.0).y(0.0).z(0.709650).w(
          0.704554));

  const auto goal_pose =
    geometry_msgs::build<geometry_msgs::msg::Pose>()
      .position(geometry_msgs::build<geometry_msgs::msg::Point>().x(365.658).y(507.253).z(100.000))
      .orientation(
        geometry_msgs::build<geometry_msgs::msg::Quaternion>().x(0.0).y(0.0).z(0.705897).w(
          0.708314));

  auto planner_data = std::make_shared<PlannerData>();
  planner_data->init_parameters(*node_);
  StartPlannerTestHelper::set_odometry(planner_data, start_pose);
  StartPlannerTestHelper::set_route(planner_data, 4619, 4635);

  const auto & route_handler = planner_data->route_handler;
  const auto & common_parameters = planner_data->parameters;

  const double backward_path_length =
    planner_data->parameters.backward_path_length + 10.0;  // max_back_distance = 10.0と仮定
  const auto road_lanes = utils::getExtendedCurrentLanes(
    planner_data, backward_path_length, std::numeric_limits<double>::max(),
    /*forward_only_in_route*/ true);

  // Generate centerline path from road_lanes
  const auto centerline_path = utils::getCenterLinePath(
    *route_handler, road_lanes, start_pose, backward_path_length,
    std::numeric_limits<double>::max(), common_parameters);

  // Calculate lateral offset
  const double lateral_offset =
    centerline_path.points.empty()
      ? 0.0
      : autoware::motion_utils::calcLateralOffset(centerline_path.points, start_pose.position);

  const double max_steer_angle_deg = 20.0;
  const double max_steer_angle = max_steer_angle_deg * M_PI / 180.0;
  const double max_steer_angle_rate_deg_per_sec = 10.0;
  const double max_steer_angle_rate = max_steer_angle_rate_deg_per_sec * M_PI / 180.0;
  const double velocity = 1.0;
  const double wheel_base = planner_data->parameters.vehicle_info.wheel_base_m;
  // const double minimum_radius = 13.46;
  const double minimum_radius = wheel_base / std::tan(max_steer_angle);
  std::cerr << "minimum_radius: " << minimum_radius << std::endl;

  // longitudinal necessary distance for pull out
  const double longitudinal_distance =
    start_planner_utils::calc_necessary_longitudinal_distance(-lateral_offset, minimum_radius);

  // target pose calculation
  Pose target_pose = start_pose;
  if (!centerline_path.points.empty()) {
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

  // Calculate relative position in vehicle coordinate system
  const double dx = target_pose.position.x - start_pose.position.x;
  const double dy = target_pose.position.y - start_pose.position.y;
  const double start_yaw = tf2::getYaw(start_pose.orientation);
  const double target_yaw = tf2::getYaw(target_pose.orientation);

  // Transform to vehicle coordinate system
  const double longitudinal_distance_vehicle = dx * std::cos(start_yaw) + dy * std::sin(start_yaw);
  const double lateral_distance_vehicle = -dx * std::sin(start_yaw) + dy * std::cos(start_yaw);

  // Calculate angle difference
  double angle_diff = target_yaw - start_yaw;
  while (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;
  while (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;

  std::cerr << "=== Test Parameters ===" << std::endl;
  std::cerr << "Lateral offset: " << lateral_offset << std::endl;
  std::cerr << "Longitudinal distance: " << longitudinal_distance << std::endl;
  std::cerr << "Vehicle coordinate relative position:" << std::endl;
  std::cerr << "  Longitudinal (forward): " << longitudinal_distance_vehicle << " m" << std::endl;
  std::cerr << "  Lateral (left): " << lateral_distance_vehicle << " m" << std::endl;
  std::cerr << "  Angle difference: " << angle_diff << " rad (" << angle_diff * 180.0 / M_PI
            << " deg)" << std::endl;

  // calc_circular_pathの入力パラメータをデバッグ出力
  std::cerr << "\n=== calc_circular_path Input Parameters ===" << std::endl;
  std::cerr << "start_pose:" << std::endl;
  std::cerr << "  position: (" << start_pose.position.x << ", " << start_pose.position.y << ", "
            << start_pose.position.z << ")" << std::endl;
  std::cerr << "  orientation (quaternion): (" << start_pose.orientation.x << ", "
            << start_pose.orientation.y << ", " << start_pose.orientation.z << ", "
            << start_pose.orientation.w << ")" << std::endl;
  std::cerr << "  yaw: " << start_yaw << " rad (" << start_yaw * 180.0 / M_PI << " deg)"
            << std::endl;

  // calc_circular_pathを直接呼び出し
  const auto circular_path = start_planner_utils::calc_circular_path(
    start_pose, longitudinal_distance_vehicle, lateral_distance_vehicle, angle_diff,
    minimum_radius);

  // 円弧経路が生成されたことを確認
  ASSERT_FALSE(circular_path.segments.empty()) << "Circular path generation failed.";

  // 経路点を生成
  std::vector<std::pair<double, double>> path_points;
  const int points_per_segment = 50;

  // 角度変化の計算
  double total_angle_change = 0.0;
  std::cerr << "=== Arc Segment Analysis ===" << std::endl;

  for (const auto & segment : circular_path.segments) {
    const double circular_steer_angle = std::atan(wheel_base / segment.radius);
    const double circular_steer_angle_deg = circular_steer_angle * 180.0 / M_PI;
    std::cerr << "circular_steer_angle_deg: " << circular_steer_angle_deg << std::endl;
    const double minimum_steer_time = circular_steer_angle / max_steer_angle_rate;
    const double L_min = velocity * minimum_steer_time;
    const double A_min = std::sqrt(segment.radius * L_min);
    const double alpha_clothoid = (L_min * L_min) / (2.0 * A_min * A_min);
    std::cerr << "L_min: " << L_min << std::endl;
    std::cerr << "A_min: " << A_min << std::endl;
    std::cerr << "alpha_clothoid: " << alpha_clothoid << std::endl;

    // 各セグメントの角度変化を計算
    double start_angle = segment.getStartAngle();
    double end_angle = segment.getEndAngle();
    double segment_angle_change;

    if (segment.is_clockwise) {
      segment_angle_change = end_angle - start_angle;
      if (segment_angle_change > 0) {
        segment_angle_change -= 2 * M_PI;
      }
    } else {
      segment_angle_change = end_angle - start_angle;
      if (segment_angle_change < 0) {
        segment_angle_change += 2 * M_PI;
      }
    }

    total_angle_change += segment_angle_change;

    for (int i = 0; i < points_per_segment; ++i) {
      if (!path_points.empty() && i == 0) {
        continue;
      }

      double progress = static_cast<double>(i) / (points_per_segment - 1);

      double current_angle;

      if (segment.is_clockwise) {
        double angle_diff_seg = end_angle - start_angle;
        if (angle_diff_seg > 0) {
          angle_diff_seg -= 2 * M_PI;
        }
        current_angle = start_angle + angle_diff_seg * progress;
      } else {
        double angle_diff_seg = end_angle - start_angle;
        if (angle_diff_seg < 0) {
          angle_diff_seg += 2 * M_PI;
        }
        current_angle = start_angle + angle_diff_seg * progress;
      }

      auto point = segment.getPointAtAngle(current_angle);
      path_points.push_back(std::make_pair(point.x, point.y));
    }
  }

  std::cerr << "=== Total Angle Change ===" << std::endl;
  std::cerr << "Total angle change: " << total_angle_change << " rad ("
            << total_angle_change * 180.0 / M_PI << " deg)" << std::endl;
  std::cerr << "===========================" << std::endl;

  // 統計情報を出力
  std::cerr << "=== Circular Path Information ===" << std::endl;
  std::cerr << "Number of points: " << path_points.size() << std::endl;
  std::cerr << "Total path length: " << circular_path.calculateTotalLength() << " m" << std::endl;

  // 曲率情報を計算・出力
  const auto trajectory = start_planner_utils::convertCircularPathToTrajectory(circular_path);
  const auto curvatures = start_planner_utils::calcCurvatureFromTrajectory(trajectory);

  // ============================================================================
  // クロソイド変換処理を追加
  // ============================================================================
  std::cerr << "\n=== Starting Clothoid Conversion ===" << std::endl;

  // 各円弧セグメントをクロソイドに変換
  std::vector<std::vector<geometry_msgs::msg::Point>> clothoid_paths;

  // セグメント間の連続性を保つための姿勢管理
  geometry_msgs::msg::Pose current_segment_pose = start_pose;

  for (size_t i = 0; i < circular_path.segments.size(); ++i) {
    const auto & segment = circular_path.segments[i];

    std::cerr << "\n--- Converting Arc Segment " << (i + 1) << "/" << circular_path.segments.size()
              << " ---" << std::endl;

    // セグメントの開始状態をデバッグ出力
    double current_yaw = tf2::getYaw(current_segment_pose.orientation);
    std::cerr << "Segment " << (i + 1) << " start pose:" << std::endl;
    std::cerr << "  Position: (" << current_segment_pose.position.x << ", "
              << current_segment_pose.position.y << ")" << std::endl;
    std::cerr << "  Heading: " << current_yaw << " rad (" << current_yaw * 180.0 / M_PI << " deg)"
              << std::endl;

    // 車両パラメータから最適なクロソイドパラメータを計算
    const double circular_steer_angle = std::atan(wheel_base / segment.radius);
    const double minimum_steer_time = circular_steer_angle / max_steer_angle_rate;
    const double L_min = velocity * minimum_steer_time;
    const double A_min = std::sqrt(segment.radius * L_min);

    std::cerr << "Calculated clothoid parameters for segment " << (i + 1) << ":" << std::endl;
    std::cerr << "  A_min: " << A_min << std::endl;
    std::cerr << "  L_min: " << L_min << " m" << std::endl;
    std::cerr << "  Segment radius: " << segment.radius << " m" << std::endl;

    // クロソイド変換を実行
    auto clothoid_points = convertArcToClothoidWithCorrection(
      segment, current_segment_pose, A_min, L_min, points_per_segment);

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

  // プロット作成 - 左に経路、右に曲率分析を表示
  pybind11::scoped_interpreter guard{};
  auto plt = matplotlibcpp17::pyplot::import();

  // ============================================================================
  // クロソイド経路の曲率計算（プロット前に実行）
  // ============================================================================
  std::cerr << "\n=== Clothoid Path Curvature Analysis ===" << std::endl;

  // clothoid_pathsを結合
  std::vector<geometry_msgs::msg::Point> combined_clothoid_path;
  for (size_t i = 0; i < clothoid_paths.size(); ++i) {
    const auto & clothoid_path = clothoid_paths[i];

    // 最初のセグメント以外は開始点を除いて結合（重複回避）
    size_t start_idx = (i == 0) ? 0 : 1;
    for (size_t j = start_idx; j < clothoid_path.size(); ++j) {
      combined_clothoid_path.push_back(clothoid_path[j]);
    }
  }

  std::cerr << "Combined clothoid path: " << combined_clothoid_path.size() << " points"
            << std::endl;

  // 曲率計算データの準備
  std::vector<double> arc_lengths;
  std::vector<double> curvature_values;
  std::vector<double> curvature_changes;
  std::vector<double> arc_lengths_changes;
  bool has_curvature_data = false;

  if (combined_clothoid_path.size() >= 3) {
    // 結合されたクロソイド経路の曲率を計算
    auto combined_curvatures =
      autoware::behavior_path_planner::start_planner_utils::calcCurvatureFromPoints(
        combined_clothoid_path);

    std::cerr << "\n=== Combined Clothoid Path Curvature Debug ===" << std::endl;
    std::cerr << "Total points: " << combined_clothoid_path.size() << std::endl;
    std::cerr << "Curvature values count: " << combined_curvatures.size() << std::endl;

    // 各点の曲率をデバッグプリント
    for (size_t i = 0; i < combined_curvatures.size(); ++i) {
      const auto & point = combined_clothoid_path[i];
      const double curvature = combined_curvatures[i];

      std::cerr << "Point[" << i << "]: "
                << "pos=(" << std::fixed << std::setprecision(3) << point.x << ", " << point.y
                << "), "
                << "curvature=" << std::setprecision(6) << curvature << " (1/m)" << std::endl;
    }

    // 曲率統計の計算
    double max_curvature =
      *std::max_element(combined_curvatures.begin(), combined_curvatures.end());
    double min_curvature =
      *std::min_element(combined_curvatures.begin(), combined_curvatures.end());
    double avg_curvature =
      std::accumulate(combined_curvatures.begin(), combined_curvatures.end(), 0.0) /
      combined_curvatures.size();
    double sum_abs_curvature = 0.0;
    for (const auto & curvature : combined_curvatures) {
      sum_abs_curvature += std::abs(curvature);
    }
    double avg_abs_curvature = sum_abs_curvature / combined_curvatures.size();

    std::cerr << "\n=== Combined Path Curvature Statistics ===" << std::endl;
    std::cerr << "Max curvature: " << max_curvature << " (1/m)" << std::endl;
    std::cerr << "Min curvature: " << min_curvature << " (1/m)" << std::endl;
    std::cerr << "Average curvature: " << avg_curvature << " (1/m)" << std::endl;
    std::cerr << "Average absolute curvature: " << avg_abs_curvature << " (1/m)" << std::endl;

    // 弧長を計算
    arc_lengths.push_back(0.0);
    double cumulative_length = 0.0;
    for (size_t i = 1; i < combined_clothoid_path.size(); ++i) {
      double dx = combined_clothoid_path[i].x - combined_clothoid_path[i - 1].x;
      double dy = combined_clothoid_path[i].y - combined_clothoid_path[i - 1].y;
      cumulative_length += std::sqrt(dx * dx + dy * dy);
      arc_lengths.push_back(cumulative_length);
    }

    // 曲率値をコピー
    for (size_t i = 0; i < combined_curvatures.size(); ++i) {
      curvature_values.push_back(combined_curvatures[i]);
    }

    // 曲率変化率を計算
    for (size_t i = 1; i < combined_curvatures.size(); ++i) {
      curvature_changes.push_back(combined_curvatures[i] - combined_curvatures[i - 1]);
      arc_lengths_changes.push_back(arc_lengths[i]);
    }

    has_curvature_data = true;
    std::cerr << "\n=== Plotting Curvature vs Arc Length ===" << std::endl;
    std::cerr << "Total arc length: " << cumulative_length << " m" << std::endl;
    std::cerr << "Number of curvature points: " << curvature_values.size() << std::endl;
  } else {
    std::cerr << "Combined clothoid path has insufficient points for curvature calculation"
              << std::endl;
  }

  // 横並びプロット作成: 左に経路、右に曲率分析
  auto [fig, axes] = plt.subplots(1, 3, Kwargs("figsize"_a = std::make_tuple(18, 6)));
  auto & ax_path = axes[0];
  auto & ax_curvature = axes[1];
  auto & ax_curvature_change = axes[2];

  // ============================================================================
  // 左側: 経路プロット
  // ============================================================================

  // レーンレットをプロット
  const auto & lanelets = planner_data->route_handler->getLaneletMapPtr()->laneletLayer;
  for (const auto & lanelet : lanelets) {
    plot_lanelet(ax_path, lanelet);
  }

  // 開始姿勢と目標姿勢をプロット
  ax_path.plot(
    Args(start_pose.position.x, start_pose.position.y),
    Kwargs("marker"_a = "x", "label"_a = "start", "markersize"_a = 20, "color"_a = "green"));
  ax_path.plot(
    Args(target_pose.position.x, target_pose.position.y),
    Kwargs("marker"_a = "x", "label"_a = "target", "markersize"_a = 20, "color"_a = "red"));

  // フットプリントをプロット
  plot_footprint(ax_path, start_pose, planner_data->parameters.vehicle_info, "green", 0.3);
  plot_footprint(ax_path, target_pose, planner_data->parameters.vehicle_info, "red", 0.3);

  // 元の円弧経路の点をプロット
  std::vector<double> xs, ys;
  for (const auto & point : path_points) {
    xs.push_back(point.first);
    ys.push_back(point.second);
  }

  // 円弧経路を線で接続
  ax_path.plot(
    Args(xs, ys),
    Kwargs("color"_a = "blue", "linewidth"_a = 2.0, "label"_a = "circular path", "alpha"_a = 0.7));

  // 円弧経路を点でマーク
  ax_path.scatter(Args(xs, ys), Kwargs("color"_a = "blue", "s"_a = 10, "alpha"_a = 0.6));

  // クロソイド経路をプロット
  for (size_t i = 0; i < clothoid_paths.size(); ++i) {
    const auto & clothoid_path = clothoid_paths[i];

    std::vector<double> clothoid_xs, clothoid_ys;
    for (const auto & point : clothoid_path) {
      clothoid_xs.push_back(point.x);
      clothoid_ys.push_back(point.y);
    }

    // クロソイド経路を異なる色で描画（線分を削除して点のみ表示）
    std::string color = (i % 2 == 0) ? "red" : "purple";
    std::string label = (i == 0) ? "clothoid path" : "";

    // クロソイド経路の点をscatterで描画
    ax_path.scatter(
      Args(clothoid_xs, clothoid_ys),
      Kwargs("color"_a = color, "s"_a = 15, "label"_a = label, "alpha"_a = 0.8));

    // クロソイド経路の開始点と終了点をマーク
    if (!clothoid_path.empty()) {
      ax_path.plot(
        Args(clothoid_path.front().x, clothoid_path.front().y),
        Kwargs("marker"_a = "o", "color"_a = color, "markersize"_a = 8, "alpha"_a = 0.9));
      ax_path.plot(
        Args(clothoid_path.back().x, clothoid_path.back().y),
        Kwargs("marker"_a = "s", "color"_a = color, "markersize"_a = 8, "alpha"_a = 0.9));
    }
  }

  // 円弧セグメントの中心点をプロット
  for (size_t i = 0; i < circular_path.segments.size(); ++i) {
    const auto & segment = circular_path.segments[i];
    ax_path.plot(
      Args(segment.center.x, segment.center.y),
      Kwargs(
        "marker"_a = "o", "color"_a = "orange", "markersize"_a = 8,
        "label"_a = (i == 0 ? "arc centers" : "")));
  }

  // プロット範囲を設定
  const double margin = 20.0;
  const double x_min = std::min(start_pose.position.x, target_pose.position.x) - margin;
  const double x_max = std::max(start_pose.position.x, target_pose.position.x) + margin;
  const double y_min = std::min(start_pose.position.y, target_pose.position.y) - margin;
  const double y_max = std::max(start_pose.position.y, target_pose.position.y) + margin;

  ax_path.set_xlim(Args(x_min, x_max));
  ax_path.set_ylim(Args(y_min, y_max));
  ax_path.set_aspect(Args("equal"));
  ax_path.grid(Args(true), Kwargs("alpha"_a = 0.3));
  ax_path.set_title(Args("Path Comparison"));
  ax_path.set_xlabel(Args("X [m]"));
  ax_path.set_ylabel(Args("Y [m]"));
  ax_path.legend();

  // ============================================================================
  // 中央・右側: 曲率分析プロット
  // ============================================================================

  if (has_curvature_data && !curvature_values.empty()) {
    // 曲率 vs 弧長をプロット
    ax_curvature.plot(
      Args(arc_lengths, curvature_values),
      Kwargs("color"_a = "blue", "linewidth"_a = 2.0, "label"_a = "Curvature"));

    ax_curvature.set_xlabel(Args("Arc Length [m]"));
    ax_curvature.set_ylabel(Args("Curvature [1/m]"));
    ax_curvature.set_title(Args("Clothoid Path Curvature"));
    ax_curvature.grid(Args(true), Kwargs("alpha"_a = 0.3));
    ax_curvature.legend();

    // 曲率変化率をプロット
    if (!curvature_changes.empty()) {
      ax_curvature_change.plot(
        Args(arc_lengths_changes, curvature_changes),
        Kwargs("color"_a = "red", "linewidth"_a = 2.0, "label"_a = "Curvature Change Rate"));

      ax_curvature_change.set_xlabel(Args("Arc Length [m]"));
      ax_curvature_change.set_ylabel(Args("Curvature Change [1/m per step]"));
      ax_curvature_change.set_title(Args("Curvature Change Rate"));
      ax_curvature_change.grid(Args(true), Kwargs("alpha"_a = 0.3));
      ax_curvature_change.legend();

      // 曲率変化の統計を表示
      double max_change = *std::max_element(curvature_changes.begin(), curvature_changes.end());
      double min_change = *std::min_element(curvature_changes.begin(), curvature_changes.end());
      double avg_change = std::accumulate(curvature_changes.begin(), curvature_changes.end(), 0.0) /
                          curvature_changes.size();

      std::cerr << "\n=== Curvature Change Rate Statistics ===" << std::endl;
      std::cerr << "Max change: " << max_change << " (1/m per step)" << std::endl;
      std::cerr << "Min change: " << min_change << " (1/m per step)" << std::endl;
      std::cerr << "Average change: " << avg_change << " (1/m per step)" << std::endl;
    } else {
      ax_curvature_change.text(
        Args(0.5, 0.5, "No curvature change data"), Kwargs("ha"_a = "center", "va"_a = "center"));
      ax_curvature_change.set_title(Args("Curvature Change Rate (No Data)"));
    }
  } else {
    // 曲率データがない場合のメッセージ表示
    ax_curvature.text(
      Args(0.5, 0.5, "Insufficient points for\ncurvature calculation"),
      Kwargs("ha"_a = "center", "va"_a = "center"));
    ax_curvature.set_title(Args("Curvature Analysis (No Data)"));

    ax_curvature_change.text(
      Args(0.5, 0.5, "No curvature change data"), Kwargs("ha"_a = "center", "va"_a = "center"));
    ax_curvature_change.set_title(Args("Curvature Change Rate (No Data)"));
  }

  // 統計情報をコンソールに出力
  std::cerr << "\n=== Path Statistics Summary ===" << std::endl;
  std::cerr << "Circular path: " << path_points.size() << " points, "
            << static_cast<int>(circular_path.calculateTotalLength()) << " m" << std::endl;
  std::cerr << "===============================" << std::endl;

  // レイアウトを調整して表示
  fig.tight_layout();
  plt.show(Args(), Kwargs("block"_a = true));

  // ============================================================================
  // クロソイド経路の品質評価
  // ============================================================================
  std::cerr << "\n=== Clothoid Path Quality Analysis ===" << std::endl;

  for (size_t i = 0; i < clothoid_paths.size(); ++i) {
    const auto & clothoid_path = clothoid_paths[i];
    const auto & original_segment = circular_path.segments[i];

    if (clothoid_path.size() < 2) continue;

    std::cerr << "\nSegment " << (i + 1) << " Analysis:" << std::endl;

    // 経路長の比較
    double clothoid_length = 0.0;
    for (size_t j = 1; j < clothoid_path.size(); ++j) {
      double dx = clothoid_path[j].x - clothoid_path[j - 1].x;
      double dy = clothoid_path[j].y - clothoid_path[j - 1].y;
      clothoid_length += std::sqrt(dx * dx + dy * dy);
    }

    double original_length = original_segment.calculateArcLength();
    double length_error = std::abs(clothoid_length - original_length) / original_length * 100.0;

    std::cerr << "  Path length comparison:" << std::endl;
    std::cerr << "    Original arc: " << original_length << " m" << std::endl;
    std::cerr << "    Clothoid: " << clothoid_length << " m" << std::endl;
    std::cerr << "    Relative error: " << length_error << "%" << std::endl;

    // 開始点と終了点の比較
    auto original_start = original_segment.getPointAtAngle(original_segment.getStartAngle());
    auto original_end = original_segment.getPointAtAngle(original_segment.getEndAngle());

    double start_error = std::sqrt(
      std::pow(clothoid_path.front().x - original_start.x, 2) +
      std::pow(clothoid_path.front().y - original_start.y, 2));
    double end_error = std::sqrt(
      std::pow(clothoid_path.back().x - original_end.x, 2) +
      std::pow(clothoid_path.back().y - original_end.y, 2));

    std::cerr << "  Position accuracy:" << std::endl;
    std::cerr << "    Start point error: " << start_error << " m" << std::endl;
    std::cerr << "    End point error: " << end_error << " m" << std::endl;
  }

  std::cerr << "=======================================" << std::endl;
}

}  // namespace autoware::behavior_path_planner
