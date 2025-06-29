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

// フレネル積分の近似計算
std::pair<double, double> fresnel(double t)
{
  // フレネル積分の級数展開による近似
  // C(t) = ∫[0,t] cos(π/2 * u²) du = t - (π/2)²t⁵/40 + (π/2)⁴t⁹/3456 - ...
  // S(t) = ∫[0,t] sin(π/2 * u²) du = (π/2)t³/6 - (π/2)³t⁷/336 + (π/2)⁵t¹¹/42240 - ...

  if (std::abs(t) < 1e-10) {
    return {0.0, t};
  }

  const double pi_half = M_PI / 2.0;
  const double t2 = t * t;
  const double t4 = t2 * t2;

  // 簡単な近似（最初の数項のみ）
  double C = t;
  double S = 0.0;

  if (std::abs(t) > 1e-6) {
    // C(t) = t - (π/2)²t⁵/40 + (π/2)⁴t⁹/3456 - ...
    double pi_half_2 = pi_half * pi_half;      // (π/2)²
    double pi_half_4 = pi_half_2 * pi_half_2;  // (π/2)⁴

    double t5 = t4 * t;
    double t9 = t4 * t4 * t;

    C = t - pi_half_2 * t5 / 40.0 + pi_half_4 * t9 / 3456.0;

    // S(t) = (π/2)t³/6 - (π/2)³t⁷/336 + (π/2)⁵t¹¹/42240 - ...
    double pi_half_3 = pi_half_2 * pi_half;  // (π/2)³
    double pi_half_5 = pi_half_4 * pi_half;  // (π/2)⁵

    double t3 = t2 * t;
    double t7 = t4 * t3;
    double t11 = t4 * t4 * t3;

    S = pi_half * t3 / 6.0 - pi_half_3 * t7 / 336.0 + pi_half_5 * t11 / 42240.0;
  }

  return {S, C};
}

/**
 * @brief クロソイドセグメント情報
 */
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
 * @brief クロソイド変換後の点を元の円弧の終点と一致するように補正する
 * @param clothoid_points クロソイド変換後の点列
 * @param original_segment 元の円弧セグメント
 * @param start_pose セグメントの開始姿勢
 * @return 補正後の点列
 */
std::vector<geometry_msgs::msg::Point> correctClothoidEndpoint(
  const std::vector<geometry_msgs::msg::Point> & clothoid_points,
  const ArcSegment & original_segment, const geometry_msgs::msg::Pose & start_pose)
{
  if (clothoid_points.size() < 2) {
    return clothoid_points;
  }

  std::vector<geometry_msgs::msg::Point> corrected_points = clothoid_points;

  // 元の円弧の終点を取得
  auto original_end = original_segment.getPointAtAngle(original_segment.getEndAngle());

  // クロソイド変換後の終点
  const auto & clothoid_end = clothoid_points.back();

  // 終点での補正量を計算
  double end_error_x = original_end.x - clothoid_end.x;
  double end_error_y = original_end.y - clothoid_end.y;

  std::cerr << "\n=== Endpoint Correction Analysis ===" << std::endl;
  std::cerr << "Original endpoint: (" << original_end.x << ", " << original_end.y << ")"
            << std::endl;
  std::cerr << "Clothoid endpoint: (" << clothoid_end.x << ", " << clothoid_end.y << ")"
            << std::endl;
  std::cerr << "End error vector: (" << end_error_x << ", " << end_error_y << ")" << std::endl;
  std::cerr << "End error magnitude: "
            << std::sqrt(end_error_x * end_error_x + end_error_y * end_error_y) << " m"
            << std::endl;

  // 円弧中心
  double center_x = original_segment.center.x;
  double center_y = original_segment.center.y;
  // double radius = original_segment.radius;

  // 補正量を円周方向と半径方向に分解
  // 終点での半径ベクトル（中心から終点への方向）
  double radial_x = original_end.x - center_x;
  double radial_y = original_end.y - center_y;
  double radial_length = std::sqrt(radial_x * radial_x + radial_y * radial_y);

  // 正規化された半径方向ベクトル
  double radial_unit_x = radial_x / radial_length;
  double radial_unit_y = radial_y / radial_length;

  // 正規化された円周方向ベクトル（半径方向に垂直）
  double tangential_unit_x, tangential_unit_y;
  if (original_segment.is_clockwise) {
    tangential_unit_x = radial_unit_y;  // 90度時計回り回転
    tangential_unit_y = -radial_unit_x;
  } else {
    tangential_unit_x = -radial_unit_y;  // 90度反時計回り回転
    tangential_unit_y = radial_unit_x;
  }

  // 補正量の円周方向と半径方向成分を計算
  double radial_correction = end_error_x * radial_unit_x + end_error_y * radial_unit_y;
  double tangential_correction = end_error_x * tangential_unit_x + end_error_y * tangential_unit_y;

  std::cerr << "Radial unit vector: (" << radial_unit_x << ", " << radial_unit_y << ")"
            << std::endl;
  std::cerr << "Tangential unit vector: (" << tangential_unit_x << ", " << tangential_unit_y << ")"
            << std::endl;
  std::cerr << "Radial correction: " << radial_correction << " m" << std::endl;
  std::cerr << "Tangential correction: " << tangential_correction << " m" << std::endl;

  // 総点数を取得
  size_t total_points = clothoid_points.size();
  std::cerr << "Total clothoid points: " << total_points << std::endl;

  // 各点を補正
  for (size_t i = 1; i < corrected_points.size(); ++i) {  // 始点(i=0)は補正しない
    double progress = static_cast<double>(i) / static_cast<double>(total_points - 1);

    // 現在の点における円弧上の対応点を推定
    // 円弧の開始角度から終了角度まで線形補間
    double start_angle = original_segment.getStartAngle();
    double end_angle = original_segment.getEndAngle();

    double current_angle;
    if (original_segment.is_clockwise) {
      double angle_diff = end_angle - start_angle;
      if (angle_diff > 0) angle_diff -= 2 * M_PI;
      current_angle = start_angle + angle_diff * progress;
    } else {
      double angle_diff = end_angle - start_angle;
      if (angle_diff < 0) angle_diff += 2 * M_PI;
      current_angle = start_angle + angle_diff * progress;
    }

    // 現在の点における半径方向と円周方向ベクトル
    double current_radial_x = std::cos(current_angle);
    double current_radial_y = std::sin(current_angle);

    double current_tangential_x, current_tangential_y;
    if (original_segment.is_clockwise) {
      current_tangential_x = current_radial_y;
      current_tangential_y = -current_radial_x;
    } else {
      current_tangential_x = -current_radial_y;
      current_tangential_y = current_radial_x;
    }

    // 補正量を進行度に比例して適用（index比例）
    double correction_factor = progress;  // 線形補正 (0から1まで)

    // 滑らかな補正のためにsin関数を使用（オプション）
    // double correction_factor = std::sin(progress * M_PI / 2.0);  // より滑らかな補正

    double applied_radial_correction = radial_correction * correction_factor;
    double applied_tangential_correction = tangential_correction * correction_factor;

    // 補正を適用
    corrected_points[i].x += applied_radial_correction * current_radial_x +
                             applied_tangential_correction * current_tangential_x;
    corrected_points[i].y += applied_radial_correction * current_radial_y +
                             applied_tangential_correction * current_tangential_y;

    // デバッグ出力（最初の数点と最後の数点のみ）
    if (i <= 3 || i >= clothoid_points.size() - 3) {
      std::cerr << "Point " << i << "/" << (total_points - 1) << ": progress=" << progress
                << ", correction_factor=" << correction_factor << std::endl;
      std::cerr << "  Original: (" << clothoid_points[i].x << ", " << clothoid_points[i].y << ")"
                << std::endl;
      std::cerr << "  Corrected: (" << corrected_points[i].x << ", " << corrected_points[i].y << ")"
                << std::endl;
      std::cerr << "  Applied radial: " << applied_radial_correction
                << ", tangential: " << applied_tangential_correction << std::endl;
    }
  }

  // 最終確認：補正後の終点誤差
  const auto & final_corrected_end = corrected_points.back();
  double final_error_x = original_end.x - final_corrected_end.x;
  double final_error_y = original_end.y - final_corrected_end.y;
  double final_error_magnitude =
    std::sqrt(final_error_x * final_error_x + final_error_y * final_error_y);

  std::cerr << "Final corrected endpoint: (" << final_corrected_end.x << ", "
            << final_corrected_end.y << ")" << std::endl;
  std::cerr << "Remaining error: " << final_error_magnitude << " m" << std::endl;
  std::cerr << "=========================================" << std::endl;

  return corrected_points;
}

/**
 * @brief エントリクロソイドセグメントを生成
 */
std::pair<std::vector<geometry_msgs::msg::Point>, geometry_msgs::msg::Pose> generateClothoidEntry(
  const ClothoidSegment & segment, const geometry_msgs::msg::Pose & start_pose, int num_points)
{
  double A = segment.A;
  double L = segment.L;
  double direction_factor = segment.is_clockwise ? -1.0 : 1.0;
  double start_yaw = tf2::getYaw(start_pose.orientation);

  std::vector<geometry_msgs::msg::Point> points;

  std::cerr << "\n=== Clothoid Entry Point Generation ===" << std::endl;
  std::cerr << "Start pose: (" << start_pose.position.x << ", " << start_pose.position.y
            << "), psi=" << start_yaw << " rad" << std::endl;
  std::cerr << "Parameters: A=" << A << ", L=" << L << ", direction_factor=" << direction_factor
            << std::endl;
  std::cerr << "Number of points: " << num_points << std::endl;

  for (int i = 0; i < num_points; ++i) {
    double s = L * i / (num_points - 1);

    geometry_msgs::msg::Point point;

    if (A > 0 && s > 0) {
      // フレネル積分による座標計算
      double t = s / (A * std::sqrt(M_PI));
      auto fresnel_result = fresnel(t);
      double S_f = fresnel_result.first;
      double C_f = fresnel_result.second;

      double x_local = A * std::sqrt(M_PI) * C_f;
      double y_local = A * std::sqrt(M_PI) * S_f * direction_factor;

      double cos_start = std::cos(start_yaw);
      double sin_start = std::sin(start_yaw);

      point.x = start_pose.position.x + x_local * cos_start - y_local * sin_start;
      point.y = start_pose.position.y + x_local * sin_start + y_local * cos_start;
      point.z = 0.0;
    } else {
      point.x = start_pose.position.x;
      point.y = start_pose.position.y;
      point.z = 0.0;
    }

    points.push_back(point);
  }

  // 終端状態
  double final_curvature = (L / (A * A)) * direction_factor;
  double final_psi = start_yaw + (L * L / (2.0 * A * A)) * direction_factor;

  geometry_msgs::msg::Pose end_pose;
  end_pose.position = points.back();
  end_pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), final_psi));

  std::cerr << "Final pose: (" << end_pose.position.x << ", " << end_pose.position.y
            << "), psi=" << final_psi << " rad, curvature=" << final_curvature << std::endl;
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

  std::vector<geometry_msgs::msg::Point> points;

  // 前のセグメント（円弧）の曲率を計算
  double start_curvature = -1.0 / segment.radius;

  std::cerr << "\n=== Clothoid Exit Point Generation ===" << std::endl;
  std::cerr << "Start pose: (" << start_pose.position.x << ", " << start_pose.position.y
            << "), psi=" << start_yaw << " rad" << std::endl;
  std::cerr << "Parameters: L=" << L << std::endl;
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
  const ArcSegment & arc_segment, const std::vector<ClothoidSegment> & segments,
  int num_points_per_segment, const geometry_msgs::msg::Pose & start_pose)
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
  const ArcSegment & arc_segment, const geometry_msgs::msg::Pose & start_pose, double A_min = 50.0,
  double L_min = 10.0, int num_points_per_segment = 50)
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
    generateClothoidPath(arc_segment, segments, num_points_per_segment, start_pose);

  return clothoid_path;
}

/**
 * @brief 改良版のクロソイド変換関数（終点補正付き）
 */
std::vector<geometry_msgs::msg::Point> convertArcToClothoidWithCorrection(
  const ArcSegment & arc_segment, const geometry_msgs::msg::Pose & start_pose, double A_min = 50.0,
  double L_min = 10.0, int num_points_per_segment = 50)
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
  auto corrected_points = correctClothoidEndpoint(clothoid_points, arc_segment, start_pose);

  return corrected_points;
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

  // start_poseと円弧セグメントの開始点比較
  if (!circular_path.segments.empty()) {
    const auto & first_segment = circular_path.segments[0];
    auto arc_start_point = first_segment.getPointAtAngle(first_segment.getStartAngle());

    // 距離差を計算
    double distance_diff = std::sqrt(
      std::pow(arc_start_point.x - start_pose.position.x, 2) +
      std::pow(arc_start_point.y - start_pose.position.y, 2) +
      std::pow(arc_start_point.z - start_pose.position.z, 2));
    std::cerr << "Distance difference: " << distance_diff << " m" << std::endl;

    // 角度差を計算
    double angle_diff_start = first_segment.getStartAngle() - start_yaw;
    while (angle_diff_start > M_PI) angle_diff_start -= 2.0 * M_PI;
    while (angle_diff_start < -M_PI) angle_diff_start += 2.0 * M_PI;
    std::cerr << "Angle difference: " << angle_diff_start << " rad ("
              << angle_diff_start * 180.0 / M_PI << " deg)" << std::endl;

    // 各円弧セグメントの詳細情報を出力
    std::cerr << "\n=== Arc Segment Details ===" << std::endl;
    for (size_t i = 0; i < circular_path.segments.size(); ++i) {
      const auto & segment = circular_path.segments[i];
      std::cerr << "Segment " << (i + 1) << ":" << std::endl;
      std::cerr << "  Center: (" << segment.center.x << ", " << segment.center.y << ")"
                << std::endl;
      std::cerr << "  Radius: " << segment.radius << std::endl;
      std::cerr << "  Start angle: " << segment.getStartAngle() << " rad ("
                << segment.getStartAngle() * 180.0 / M_PI << " deg)" << std::endl;
      std::cerr << "  End angle: " << segment.getEndAngle() << " rad ("
                << segment.getEndAngle() * 180.0 / M_PI << " deg)" << std::endl;
      std::cerr << "  Is clockwise: " << (segment.is_clockwise ? "true" : "false") << std::endl;

      auto seg_start_point = segment.getPointAtAngle(segment.getStartAngle());
      auto seg_end_point = segment.getPointAtAngle(segment.getEndAngle());
      std::cerr << "  Start point: (" << seg_start_point.x << ", " << seg_start_point.y << ")"
                << std::endl;
      std::cerr << "  End point: (" << seg_end_point.x << ", " << seg_end_point.y << ")"
                << std::endl;

      if (i == 0) {
        // 最初のセグメントの開始点とstart_poseの距離差
        double seg_distance_diff = std::sqrt(
          std::pow(seg_start_point.x - start_pose.position.x, 2) +
          std::pow(seg_start_point.y - start_pose.position.y, 2));
        std::cerr << "  Distance from original start_pose: " << seg_distance_diff << " m"
                  << std::endl;
      }
      std::cerr << std::endl;
    }
    std::cerr << "===============================" << std::endl;
  }

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

  int total_clothoid_points = 0;
  for (const auto & path : clothoid_paths) {
    total_clothoid_points += path.size();
  }

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

  // 元の円弧経路の点をプロット
  std::vector<double> xs, ys;
  for (const auto & point : path_points) {
    xs.push_back(point.first);
    ys.push_back(point.second);
  }

  // 円弧経路を線で接続
  ax.plot(
    Args(xs, ys),
    Kwargs("color"_a = "blue", "linewidth"_a = 2.0, "label"_a = "circular path", "alpha"_a = 0.7));

  // 円弧経路を点でマーク
  ax.scatter(Args(xs, ys), Kwargs("color"_a = "blue", "s"_a = 10, "alpha"_a = 0.6));

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

    // 線分描画をコメントアウト
    // ax.plot(
    //   Args(clothoid_xs, clothoid_ys),
    //   Kwargs("color"_a = color, "linewidth"_a = 3.0, "label"_a = label, "alpha"_a = 0.8));

    // クロソイド経路の点をscatterで描画
    ax.scatter(
      Args(clothoid_xs, clothoid_ys),
      Kwargs("color"_a = color, "s"_a = 15, "label"_a = label, "alpha"_a = 0.8));

    // クロソイド経路の開始点と終了点をマーク
    if (!clothoid_path.empty()) {
      ax.plot(
        Args(clothoid_path.front().x, clothoid_path.front().y),
        Kwargs("marker"_a = "o", "color"_a = color, "markersize"_a = 8, "alpha"_a = 0.9));
      ax.plot(
        Args(clothoid_path.back().x, clothoid_path.back().y),
        Kwargs("marker"_a = "s", "color"_a = color, "markersize"_a = 8, "alpha"_a = 0.9));
    }
  }

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

  std::cerr << "\n=== Plot Range Information ===" << std::endl;
  std::cerr << "Start pose: (" << start_pose.position.x << ", " << start_pose.position.y << ")"
            << std::endl;
  std::cerr << "Target pose: (" << target_pose.position.x << ", " << target_pose.position.y << ")"
            << std::endl;
  std::cerr << "Plot range with " << margin << "m margin:" << std::endl;
  std::cerr << "  X: [" << x_min << ", " << x_max << "] m (range: " << (x_max - x_min) << " m)"
            << std::endl;
  std::cerr << "  Y: [" << y_min << ", " << y_max << "] m (range: " << (y_max - y_min) << " m)"
            << std::endl;
  std::cerr << "===============================" << std::endl;

  ax.set_xlim(Args(x_min, x_max));
  ax.set_ylim(Args(y_min, y_max));

  ax.set_aspect(Args("equal"));

  // グリッドを追加して視認性を向上
  ax.grid(Args(true), Kwargs("alpha"_a = 0.3));

  // 統計情報をコンソールに出力（プロットへの追加は削除）
  std::cerr << "\n=== Path Statistics Summary ===" << std::endl;
  std::cerr << "Circular path: " << path_points.size() << " points, "
            << static_cast<int>(circular_path.calculateTotalLength()) << " m" << std::endl;
  std::cerr << "Clothoid path: " << total_clothoid_points << " points, " << clothoid_paths.size()
            << " segments" << std::endl;
  std::cerr << "===============================" << std::endl;

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
