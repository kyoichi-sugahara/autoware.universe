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
  ax.legend();
  plt.show(Args(), Kwargs("block"_a = true));
}

TEST_F(TestClothoidPullOut, PlotCircularPathGeneration)
{
  // GenerateValidClothoidPullOutPathと同じ条件を使用
  const auto start_pose =
    geometry_msgs::build<geometry_msgs::msg::Pose>()
      .position(geometry_msgs::build<geometry_msgs::msg::Point>().x(362.181).y(362.164).z(100.000))
      .orientation(
        geometry_msgs::build<geometry_msgs::msg::Quaternion>().x(0.0).y(0.0).z(0.709650).w(
          0.704554));

  auto planner_data = std::make_shared<PlannerData>();
  planner_data->init_parameters(*node_);
  StartPlannerTestHelper::set_odometry(planner_data, start_pose);
  StartPlannerTestHelper::set_route(planner_data, 4619, 4635);

  // clothoid_pull_out.cppと同じパラメータ計算を実行
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

  const double minimum_radius = 13.46;

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

  // calc_circular_pathを直接呼び出し
  const auto circular_path = start_planner_utils::calc_circular_path(
    start_pose, longitudinal_distance_vehicle, lateral_distance_vehicle, angle_diff,
    minimum_radius);

  // 円弧経路が生成されたことを確認
  ASSERT_FALSE(circular_path.segments.empty()) << "Circular path generation failed.";

  // 経路点を生成
  std::vector<std::pair<double, double>> path_points;
  const int points_per_segment = 50;

  for (const auto & segment : circular_path.segments) {
    for (int i = 0; i < points_per_segment; ++i) {
      if (!path_points.empty() && i == 0) {
        continue;
      }

      double progress = static_cast<double>(i) / (points_per_segment - 1);

      double start_angle = segment.getStartAngle();
      double end_angle = segment.getEndAngle();
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

  // 統計情報を出力
  std::cerr << "=== Circular Path Information ===" << std::endl;
  std::cerr << "Number of segments: " << circular_path.segments.size() << std::endl;
  std::cerr << "Number of points: " << path_points.size() << std::endl;
  std::cerr << "Total path length: " << circular_path.calculateTotalLength() << " m" << std::endl;

  // 曲率情報を計算・出力
  const auto trajectory = start_planner_utils::convertCircularPathToTrajectory(circular_path);
  const auto curvatures = start_planner_utils::calcCurvatureFromTrajectory(trajectory);

  if (!curvatures.empty()) {
    double max_curvature = *std::max_element(curvatures.begin(), curvatures.end());
    double min_curvature = *std::min_element(curvatures.begin(), curvatures.end());
    double sum_curvature = std::accumulate(curvatures.begin(), curvatures.end(), 0.0);
    double avg_curvature = sum_curvature / curvatures.size();

    std::cerr << "Curvature statistics:" << std::endl;
    std::cerr << "  Maximum: " << max_curvature << " [1/m]" << std::endl;
    std::cerr << "  Minimum: " << min_curvature << " [1/m]" << std::endl;
    std::cerr << "  Average: " << avg_curvature << " [1/m]" << std::endl;
  }
  std::cerr << "=================================" << std::endl;

  // プロット作成
  pybind11::scoped_interpreter guard{};
  auto plt = matplotlibcpp17::pyplot::import();
  auto [fig, axes] = plt.subplots(1, 1);
  auto & ax = axes[0];

  // レーンレットをプロット
  const auto & lanelets = planner_data->route_handler->getLaneletMapPtr()->laneletLayer;
  for (const auto & lanelet : lanelets) {
    plot_lanelet(ax, lanelet);
  }

  // 開始姿勢と目標姿勢をプロット
  ax.plot(
    Args(start_pose.position.x, start_pose.position.y),
    Kwargs("marker"_a = "x", "label"_a = "start", "markersize"_a = 20, "color"_a = "green"));
  ax.plot(
    Args(target_pose.position.x, target_pose.position.y),
    Kwargs("marker"_a = "x", "label"_a = "target", "markersize"_a = 20, "color"_a = "red"));

  // フットプリントをプロット
  plot_footprint(ax, start_pose, planner_data->parameters.vehicle_info, "green", 0.3);
  plot_footprint(ax, target_pose, planner_data->parameters.vehicle_info, "red", 0.3);

  // 円弧経路の点をプロット
  std::vector<double> xs, ys;
  for (const auto & point : path_points) {
    xs.push_back(point.first);
    ys.push_back(point.second);
  }

  // 経路点を線で接続
  ax.plot(
    Args(xs, ys), Kwargs("color"_a = "blue", "linewidth"_a = 2.0, "label"_a = "circular path"));

  // 経路点を点でマーク
  ax.scatter(Args(xs, ys), Kwargs("color"_a = "blue", "s"_a = 10, "alpha"_a = 0.6));

  // 円弧セグメントの中心点をプロット
  for (size_t i = 0; i < circular_path.segments.size(); ++i) {
    const auto & segment = circular_path.segments[i];
    ax.plot(
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
  ax.set_xlim(Args(x_min, x_max));
  ax.set_ylim(Args(y_min, y_max));

  ax.set_aspect(Args("equal"));
  ax.set_title(Args("Circular Path Generation Test"));
  ax.legend();
  plt.show(Args(), Kwargs("block"_a = true));
}

}  // namespace autoware::behavior_path_planner
