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

#include "autoware/control_validator/control_validator.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/node_options.hpp>

#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <gtest/gtest-param-test.h>
#include <gtest/gtest.h>
#include <sys/resource.h>
#include <tf2/LinearMath/Quaternion.h>

#include <chrono>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <tuple>

using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;

Trajectory make_linear_trajectory(
  const TrajectoryPoint & start, const TrajectoryPoint & end, size_t num_points, double velocity)
{
  auto create_quaternion = [](double yaw) {
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    return tf2::toMsg(q);
  };

  double yaw = std::atan2(
    end.pose.position.y - start.pose.position.y, end.pose.position.x - start.pose.position.x);
  yaw += (velocity < 0) ? M_PI : 0;

  Trajectory trajectory;
  trajectory.points.reserve(num_points);

  for (size_t i = 0; i < num_points; ++i) {
    double ratio = static_cast<double>(i) / static_cast<double>(num_points - 1);

    TrajectoryPoint point;
    point.pose.position.x =
      start.pose.position.x + ratio * (end.pose.position.x - start.pose.position.x);
    point.pose.position.y =
      start.pose.position.y + ratio * (end.pose.position.y - start.pose.position.y);
    point.pose.orientation = create_quaternion(yaw);
    point.longitudinal_velocity_mps = static_cast<float>(velocity);
    point.lateral_velocity_mps = 0.0;

    trajectory.points.emplace_back(point);
  }

  return trajectory;
}

TrajectoryPoint make_trajectory_point(double x, double y)
{
  TrajectoryPoint point;
  point.pose.position.x = x;
  point.pose.position.y = y;
  return point;
}

size_t get_current_memory_usage()
{
  struct rusage usage;
  getrusage(RUSAGE_SELF, &usage);
  return usage.ru_maxrss;
}

class MemoryUsageTracker
{
public:
  explicit MemoryUsageTracker(const std::string & test_name)
  : test_name_(test_name),
    start_memory_(get_current_memory_usage()),
    start_time_(std::chrono::steady_clock::now())
  {
    std::cout << "Starting test: " << test_name_ << " (Initial memory: " << start_memory_ << " KB)"
              << std::endl;
  }

  ~MemoryUsageTracker()
  {
    auto end_memory = get_current_memory_usage();
    auto memory_diff =
      end_memory > start_memory_ ? end_memory - start_memory_ : start_memory_ - end_memory;
    auto end_time = std::chrono::steady_clock::now();
    auto duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time_).count();

    std::cout << "Finished test: " << test_name_ << std::endl;
    std::cout << "  Duration: " << duration << " ms" << std::endl;
    std::cout << "  Memory usage: " << end_memory
              << " KB (Diff: " << (end_memory > start_memory_ ? "+" : "-") << memory_diff << " KB)"
              << std::endl;
  }

private:
  std::string test_name_;
  size_t start_memory_;
  std::chrono::steady_clock::time_point start_time_;
};

class TrajectoryDeviationTest
: public ::testing::TestWithParam<std::tuple<Trajectory, Trajectory, double, bool>>
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    rclcpp::NodeOptions options;
    options.arguments(
      {"--ros-args", "--params-file",
       ament_index_cpp::get_package_share_directory("autoware_control_validator") +
         "/config/control_validator.param.yaml",
       "--params-file",
       ament_index_cpp::get_package_share_directory("autoware_test_utils") +
         "/config/test_vehicle_info.param.yaml"});

    node = std::make_shared<autoware::control_validator::ControlValidator>(options);
  }

  void TearDown() override { rclcpp::shutdown(); }

  std::shared_ptr<autoware::control_validator::ControlValidator> node;
};

TEST_P(TrajectoryDeviationTest, test_calc_lateral_deviation_status)
{
  MemoryUsageTracker tracker("calc_lateral_deviation_status");

  auto [reference_trajectory, predicted_trajectory, expected_deviation, expected_condition] =
    GetParam();

  // 参照軌道とテスト軌道のサイズを出力
  std::cout << "Reference trajectory size: " << reference_trajectory.points.size() << " points"
            << std::endl;
  std::cout << "Predicted trajectory size: " << predicted_trajectory.points.size() << " points"
            << std::endl;

  // 各軌道のメモリサイズを推定
  size_t ref_traj_memory =
    sizeof(Trajectory) + reference_trajectory.points.size() * sizeof(TrajectoryPoint);
  size_t pred_traj_memory =
    sizeof(Trajectory) + predicted_trajectory.points.size() * sizeof(TrajectoryPoint);

  std::cout << "Estimated reference trajectory memory: " << ref_traj_memory << " bytes"
            << std::endl;
  std::cout << "Estimated predicted trajectory memory: " << pred_traj_memory << " bytes"
            << std::endl;

  // 個々のテストケースの詳細をログ
  std::cout << "Test case details: " << std::endl;
  std::cout << "  Expected deviation: " << expected_deviation << std::endl;
  std::cout << "  Expected condition: " << (expected_condition ? "valid" : "invalid") << std::endl;

  // メモリ使用量を計測しながら実行
  size_t before_calc = get_current_memory_usage();
  auto start_time = std::chrono::steady_clock::now();

  auto [deviation, is_valid] =
    node->calc_lateral_deviation_status(predicted_trajectory, reference_trajectory);

  auto end_time = std::chrono::steady_clock::now();
  size_t after_calc = get_current_memory_usage();

  auto duration =
    std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();

  std::cout << "Calculation time: " << duration << " microseconds" << std::endl;

  // メモリ使用量の差分を安全に計算
  size_t mem_diff = after_calc > before_calc ? after_calc - before_calc : before_calc - after_calc;
  std::cout << "Memory during calculation: " << (after_calc > before_calc ? "+" : "-") << mem_diff
            << " KB" << std::endl;

  // テスト検証
  EXPECT_EQ(is_valid, expected_condition);
  EXPECT_NEAR(deviation, expected_deviation, 1e-5);
}

INSTANTIATE_TEST_SUITE_P(
  TrajectoryDeviationTests, TrajectoryDeviationTest,
  ::testing::Values(

    std::make_tuple(
      make_linear_trajectory(make_trajectory_point(0, 0), make_trajectory_point(10, 0), 1100, 1.0),
      make_linear_trajectory(
        make_trajectory_point(0, 0), make_trajectory_point(10, 0.99), 1100, 1.0),
      0.99, true),

    std::make_tuple(
      make_linear_trajectory(make_trajectory_point(0, 0), make_trajectory_point(10, 0), 11, 1.0),
      make_linear_trajectory(make_trajectory_point(0, 0), make_trajectory_point(10, 1.0), 11, 1.0),
      1.0, true),

    std::make_tuple(
      make_linear_trajectory(make_trajectory_point(0, 0), make_trajectory_point(10, 0), 11, 1.0),
      make_linear_trajectory(make_trajectory_point(0, 0), make_trajectory_point(10, 1.01), 11, 1.0),
      1.01, false),

    std::make_tuple(
      make_linear_trajectory(make_trajectory_point(0, 0), make_trajectory_point(10, 0), 11, -1.0),
      make_linear_trajectory(
        make_trajectory_point(0, 0), make_trajectory_point(10, 0.99), 11, -1.0),
      0.99, true),

    std::make_tuple(
      make_linear_trajectory(make_trajectory_point(0, 0), make_trajectory_point(10, 0), 11, -1.0),
      make_linear_trajectory(make_trajectory_point(0, 0), make_trajectory_point(10, 1.0), 11, -1.0),
      1.0, true),

    std::make_tuple(
      make_linear_trajectory(make_trajectory_point(0, 0), make_trajectory_point(10, 0), 11, -1.0),
      make_linear_trajectory(
        make_trajectory_point(0, 0), make_trajectory_point(10, 1.01), 11, -1.0),
      1.01, false),

    std::make_tuple(
      make_linear_trajectory(make_trajectory_point(0, 0), make_trajectory_point(10, 0), 11, 1.0),
      make_linear_trajectory(make_trajectory_point(11, 0), make_trajectory_point(20, 0.0), 11, 1.0),
      0.0, true),

    std::make_tuple(
      make_linear_trajectory(make_trajectory_point(11, 0), make_trajectory_point(20, 0.0), 11, 1.0),
      make_linear_trajectory(make_trajectory_point(0, 0), make_trajectory_point(10, 0), 11, 1.0),
      0.0, true),

    std::make_tuple(
      make_linear_trajectory(make_trajectory_point(0, 0), make_trajectory_point(10, 0), 11, 1.0),
      make_linear_trajectory(make_trajectory_point(1, 0), make_trajectory_point(10, 1.0), 11, 1.0),
      1.0, true),

    std::make_tuple(
      make_linear_trajectory(make_trajectory_point(0, 0), make_trajectory_point(10, 0), 11, 1.0),
      make_linear_trajectory(make_trajectory_point(-1, 0), make_trajectory_point(10, 1.0), 11, 1.0),
      1.0, true),

    std::make_tuple(
      make_linear_trajectory(make_trajectory_point(0, 0), make_trajectory_point(10, 0), 11, 1.0),
      make_linear_trajectory(make_trajectory_point(0, 0), make_trajectory_point(20, 2.0), 21, 1.0),
      1.0, true))

);

class AccelerationValidatorTest : public ::testing::TestWithParam<std::tuple<bool, double, double>>
{
public:
  bool is_in_error_range() { return acceleration_validator_->is_in_error_range(); }
  void set_desired(double x) { acceleration_validator_->desired_acc_lpf.reset(x); }
  void set_measured(double x) { acceleration_validator_->measured_acc_lpf.reset(x); }

protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    rclcpp::NodeOptions options;
    options.arguments(
      {"--ros-args", "--params-file",
       ament_index_cpp::get_package_share_directory("autoware_control_validator") +
         "/config/control_validator.param.yaml",
       "--params-file",
       ament_index_cpp::get_package_share_directory("autoware_test_utils") +
         "/config/test_vehicle_info.param.yaml"});

    node_ = std::make_shared<autoware::control_validator::ControlValidator>(options);
    acceleration_validator_ =
      std::make_shared<autoware::control_validator::AccelerationValidator>(*node_);
  }
  void TearDown() override { rclcpp::shutdown(); }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<autoware::control_validator::AccelerationValidator> acceleration_validator_;
};

TEST_P(AccelerationValidatorTest, DISABLED_test_is_in_error_range)
{
  auto [expected, des, mes] = GetParam();
  set_desired(des);
  set_measured(mes);

  ASSERT_EQ(expected, is_in_error_range());
};

INSTANTIATE_TEST_SUITE_P(
  AccelerationValidatorTests, AccelerationValidatorTest,
  ::testing::Values(
    std::make_tuple(true, 0.0, 0.0), std::make_tuple(false, 0.0, 5.0),
    std::make_tuple(false, 0.0, -5.0), std::make_tuple(true, 1.0, 1.0),
    std::make_tuple(false, 1.0, 5.0), std::make_tuple(false, 1.0, -5.0),
    std::make_tuple(true, -1.0, -1.0), std::make_tuple(false, -1.0, -5.0),
    std::make_tuple(false, -1.0, 5.0)));
