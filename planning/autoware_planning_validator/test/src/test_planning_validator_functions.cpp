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

#include "autoware/planning_validator/debug_marker.hpp"
#include "autoware/planning_validator/planning_validator.hpp"
#include "test_parameter.hpp"
#include "test_planning_validator_helper.hpp"

#include <autoware_utils/geometry/geometry.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <string>

using autoware::planning_validator::PlanningValidator;
using autoware_planning_msgs::msg::Trajectory;

TEST(PlanningValidatorTestSuite, DISABLED_checkValidFiniteValueFunction)
{
  auto validator = std::make_shared<PlanningValidator>(getNodeOptionsWithDefaultParams());

  // Valid Trajectory
  {
    Trajectory valid_traj = generateTrajectory(THRESHOLD_INTERVAL * 0.9);
    ASSERT_TRUE(validator->checkValidFiniteValue(valid_traj));
  }

  // Nan Trajectory
  {
    Trajectory nan_traj = generateNanTrajectory();
    ASSERT_FALSE(validator->checkValidFiniteValue(nan_traj));
  }

  // Inf Trajectory
  {
    Trajectory inf_traj = generateInfTrajectory();
    ASSERT_FALSE(validator->checkValidFiniteValue(inf_traj));
  }
}

TEST(PlanningValidatorTestSuite, DISABLED_checkValidIntervalFunction)
{
  auto validator = std::make_shared<PlanningValidator>(getNodeOptionsWithDefaultParams());

  // Normal Trajectory
  {
    Trajectory valid_traj = generateTrajectory(THRESHOLD_INTERVAL * 0.9);
    ASSERT_TRUE(validator->checkValidInterval(valid_traj));
  }

  // Boundary Trajectory
  {
    // Note: too small value is not supported like numerical_limits::epsilon
    const auto ep = 1.0e-5;

    Trajectory ok_bound_traj = generateTrajectory(THRESHOLD_INTERVAL - ep);
    ASSERT_TRUE(validator->checkValidInterval(ok_bound_traj));

    Trajectory ng_bound_traj = generateTrajectory(THRESHOLD_INTERVAL + ep);
    ASSERT_FALSE(validator->checkValidInterval(ng_bound_traj));
  }

  // Long Interval Trajectory
  {
    Trajectory long_interval_traj = generateTrajectory(THRESHOLD_INTERVAL * 2.0);
    ASSERT_FALSE(validator->checkValidInterval(long_interval_traj));
  }
}

TEST(PlanningValidatorTestSuite, DISABLED_checkValidCurvatureFunction)
{
  auto validator = std::make_shared<PlanningValidator>(getNodeOptionsWithDefaultParams());

  // Normal Trajectory
  {
    Trajectory valid_traj = generateTrajectory(THRESHOLD_INTERVAL * 2.0);
    ASSERT_TRUE(validator->checkValidCurvature(valid_traj));
  }

  // Invalid curvature trajectory
  {
    // TODO(Horibe): write me
  }
}

TEST(PlanningValidatorTestSuite, DISABLED_checkValidRelativeAngleFunction)
{
  auto validator = std::make_shared<PlanningValidator>(getNodeOptionsWithDefaultParams());

  // valid case
  {
    /**
     * x: 0   1 2    3  4   5 6 7 8 9 10
     * y: 0 0.1 0 -0.1  0 0.2 0 0 0 0  0
     * max relative angle is about 0.197 radian (= 11 degree)
     **/
    constexpr auto interval = 1.0;
    Trajectory valid_traj = generateTrajectory(interval);
    valid_traj.points[1].pose.position.y = 0.1;
    valid_traj.points[3].pose.position.y = -0.1;
    valid_traj.points[5].pose.position.y = 0.2;
    ASSERT_TRUE(validator->checkValidRelativeAngle(valid_traj));
  }

  // invalid case
  {
    /**
     * x: 0 1 2 3  4 5 6 7 8 9 10
     * y: 0 0 0 0 10 0 0 0 0 0 0
     * the relative angle around index [4] is about 1.4 radian (= 84 degree)
     **/
    constexpr auto interval = 1.0;
    Trajectory invalid_traj = generateTrajectory(interval);
    invalid_traj.points[4].pose.position.x = 3;
    invalid_traj.points[4].pose.position.y = 10;
    // for (auto t : invalid_traj.points) {
    //   std::cout << "p: (x , y) = " << "( "<<t.pose.position.x <<
    // " , " << t.pose.position.y <<" )"<< std::endl;
    // }
    ASSERT_FALSE(validator->checkValidRelativeAngle(invalid_traj));
  }

  {
    /** <---inverted pattern-----
     * x: 0 -1 -2 -3  -4 -5 -6 -7 -8 -9 -10
     * y: 0  0  0  0  10  0  0  0  0  0   0
     **/
    constexpr auto interval = 1.0;
    Trajectory invalid_traj = generateTrajectory(interval);
    invalid_traj.points[4].pose.position.y = 10;
    for (auto t : invalid_traj.points) {
      t.pose.position.x *= -1;
    }
    ASSERT_FALSE(validator->checkValidRelativeAngle(invalid_traj));
  }

  {
    /** vertical pattern
     * x: 0 0 0 0 10 0 0 0 0 0  0
     * y: 0 1 2 3  4 5 6 7 8 9 10
     **/
    constexpr auto interval = 1.0;
    Trajectory invalid_traj = generateTrajectory(interval);
    for (size_t i = 0; i < invalid_traj.points.size(); i++) {
      auto & p = invalid_traj.points[i].pose.position;
      p.x = 0;
      p.y = i;
    }
    invalid_traj.points[4].pose.position.x = 10;
    std::string valid_error_msg;
    ASSERT_FALSE(validator->checkValidRelativeAngle(invalid_traj));
  }
}

TEST(PlanningValidatorTestSuite, checkValidLateralJerkFunction)
{
  auto validator = std::make_shared<PlanningValidator>(getNodeOptionsWithDefaultParams());
  // Valid trajectory with normal lateral jerk
  {
    std::cerr << "1st test" << std::endl;
    Trajectory valid_traj = generateTrajectory(THRESHOLD_INTERVAL * 0.9);
    ASSERT_TRUE(validator->checkValidLateralJerk(valid_traj));
  }

  // Trajectory with straight line movement (zero lateral jerk)
  {
    std::cerr << "2nd test" << std::endl;

    // 直線運動で加速度が変化する場合でも横方向ジャークは発生しない
    std::vector<double> accel_values = {1.0, 2.0, 0.0, -1.0, -2.0};
    Trajectory zero_jerk_traj =
      generateTrajectoryWithStepAcceleration(0.5, 5.0, 0.0, 20, accel_values, 4);
    ASSERT_TRUE(validator->checkValidLateralJerk(zero_jerk_traj));
  }

  // Trajectory with sinusoidal longitudinal acceleration but straight path
  {
    std::cerr << "3rd test" << std::endl;

    Trajectory sinusoidal_accel_traj =
      generateTrajectoryWithSinusoidalAcceleration(0.5, 8.0, 0.0, 30, 2.0, 10.0);
    ASSERT_TRUE(validator->checkValidLateralJerk(sinusoidal_accel_traj));
  }

  // Trajectory with high lateral jerk (zigzag pattern)
  {
    // 直線パスでジグザグな加速度変化を持つ軌道を生成
    Trajectory high_jerk_traj = generateTrajectory(0.5);

    // Create a sharp zigzag pattern - 新しい関数は使用せず既存のコードを維持
    for (size_t i = 2; i < high_jerk_traj.points.size(); i += 4) {
      if (i < high_jerk_traj.points.size()) {
        high_jerk_traj.points[i].pose.position.y += 2.0;
        high_jerk_traj.points[i].longitudinal_velocity_mps = 5.0;
      }

      if (i + 2 < high_jerk_traj.points.size()) {
        high_jerk_traj.points[i + 2].pose.position.y -= 2.0;
        high_jerk_traj.points[i + 2].longitudinal_velocity_mps = 5.0;
      }
    }

    // Update orientations to match the path direction
    for (size_t i = 1; i < high_jerk_traj.points.size(); ++i) {
      const auto & p1 = high_jerk_traj.points[i - 1].pose.position;
      const auto & p2 = high_jerk_traj.points[i].pose.position;
      const double yaw = std::atan2(p2.y - p1.y, p2.x - p1.x);
      high_jerk_traj.points[i - 1].pose.orientation =
        autoware_utils::create_quaternion_from_yaw(yaw);
    }

    // Set the velocity high enough to generate significant lateral jerk
    for (auto & point : high_jerk_traj.points) {
      point.longitudinal_velocity_mps = 10.0;
    }
    std::cerr << "4th test" << std::endl;

    // This should fail due to high lateral jerk
    ASSERT_FALSE(validator->checkValidLateralJerk(high_jerk_traj));
  }
  /**
   * Trajectory specification:
   * --------------------------
   * Velocity (m/s):      1    1    1    1    1    2    3    3    3    3
   * Acceleration (m/s):  1    1    1    1    1    2    3    3    3    3
   * Curvature (1/m):     0    0    0   0.05 0.1  0.1  0.05  0    0    0
   * Interval ds (m):        2    2    2    2    2    2    2    2    2
   */

  // Set coordinates, velocity, and acceleration for each point
  {
    Trajectory custom_traj;
    custom_traj.header.stamp = rclcpp::Clock{RCL_ROS_TIME}.now();

    const size_t num_points = 10;
    const double point_spacing = 2.0;
    const double curve_radius = 10.0;
    std::vector<double> cumulative_distance(num_points, 0.0);
    for (size_t i = 0; i < num_points; ++i) {
      autoware_planning_msgs::msg::TrajectoryPoint p;

      if (i <= 3) {
        // index 0~3: Straight line (along x-axis)
        p.pose.position.x = i * point_spacing;
        p.pose.position.y = 0.0;
        p.pose.orientation =
          autoware_utils_geometry::create_quaternion_from_yaw(0.0);  // Facing east
      } else {
        // index 4 and beyond: Curve or straight line
        const auto & last_pose = custom_traj.points[i - 1].pose;

        // Convert quaternion to yaw angle to get the final angle
        double roll, pitch, yaw;
        tf2::Quaternion q(
          last_pose.orientation.x, last_pose.orientation.y, last_pose.orientation.z,
          last_pose.orientation.w);
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        // const double last_angle = yaw;
        const double last_x = last_pose.position.x;
        const double last_y = last_pose.position.y;

        if (i <= 6) {
          // index 4~6: Arc with curvature 1.0
          const double angle =
            (i - 3) * (point_spacing / curve_radius);  // Angle corresponding to arc length
          p.pose.position.x =
            last_x +
            curve_radius * (std::sin(angle) - std::sin((i - 4) * (point_spacing / curve_radius)));
          p.pose.position.y =
            last_y + curve_radius * (1 - std::cos(angle) -
                                     (1 - std::cos((i - 4) * (point_spacing / curve_radius))));
          p.pose.orientation = autoware_utils_geometry::create_quaternion_from_yaw(angle);
        } else {
          // index 7~9: Straight line (from index 6 point in the same direction)
          // Get the yaw angle at index 6 (final direction of the arc)
          double final_roll, final_pitch, final_yaw;
          tf2::Quaternion final_q(
            custom_traj.points[6].pose.orientation.x, custom_traj.points[6].pose.orientation.y,
            custom_traj.points[6].pose.orientation.z, custom_traj.points[6].pose.orientation.w);
          tf2::Matrix3x3(final_q).getRPY(final_roll, final_pitch, final_yaw);

          // Get the final position at index 6
          const double start_x = custom_traj.points[6].pose.position.x;
          const double start_y = custom_traj.points[6].pose.position.y;

          // Calculate points on the straight line (relative position from index 6)
          const double dx = std::cos(final_yaw) * point_spacing * (i - 6);
          const double dy = std::sin(final_yaw) * point_spacing * (i - 6);

          p.pose.position.x = start_x + dx;
          p.pose.position.y = start_y + dy;
          p.pose.orientation = autoware_utils_geometry::create_quaternion_from_yaw(final_yaw);
        }
      }

      // Set velocity and acceleration
      if (i <= 3) {
        // index 0~3: velocity 1.0, acceleration 1.0
        p.longitudinal_velocity_mps = 1.0;
        p.acceleration_mps2 = 1.0;
      } else if (i <= 6) {
        // index 4~6: velocity 1.0/2.0/3.0, acceleration 1.0/2.0/3.0
        p.longitudinal_velocity_mps = 1.0 + (i - 4);  // 1.0, 2.0, 3.0
        p.acceleration_mps2 = 1.0 + (i - 4);          // 1.0, 2.0, 3.0
      } else {
        // index 7~9: velocity 3.0, acceleration 3.0
        p.longitudinal_velocity_mps = 3.0;
        p.acceleration_mps2 = 3.0;
      }

      custom_traj.points.push_back(p);
    }

    // Check trajectory accuracy: Calculate distance between each point
    std::vector<double> segment_distances;
    for (size_t i = 1; i < custom_traj.points.size(); ++i) {
      const auto & p1 = custom_traj.points[i - 1];
      const auto & p2 = custom_traj.points[i];
      const double dx = p2.pose.position.x - p1.pose.position.x;
      const double dy = p2.pose.position.y - p1.pose.position.y;
      const double dist = std::hypot(dx, dy);
      segment_distances.push_back(dist);
    }

    std::cout << "Index\tX\tY\tYaw\tVelocity\tAccel\tCurvature\tDistance" << std::endl;

    for (size_t i = 0; i < custom_traj.points.size(); ++i) {
      const auto & p = custom_traj.points[i];
      double curvature = 0.0;

      // Set curvature (theoretical value)
      if (i >= 4 && i <= 6) {
        curvature = 1.0;  // Specified curvature
      } else {
        curvature = 0.0;  // Straight sections
      }

      // Convert quaternion to yaw angle
      double roll, pitch, yaw;
      tf2::Quaternion q(
        p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w);
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

      // Adjust output format
      std::cout << std::fixed << std::setprecision(6);  // Set to 6 decimal places
      std::cout << i << "\t" << p.pose.position.x << "\t" << p.pose.position.y << "\t" << yaw
                << "\t" << p.longitudinal_velocity_mps << "\t" << p.acceleration_mps2 << "\t"
                << curvature;

      // Add distance information (no distance for the first point)
      if (i > 0) {
        std::cout << "\t" << segment_distances[i - 1];
      } else {
        std::cout << "\t-";
      }
      std::cout << std::endl;
    }

    // Output debug information: Check if the distance between points is exactly 2.0
    std::cout << "\nDistance check between points:" << std::endl;
    double total_dist = 0.0;
    for (size_t i = 0; i < segment_distances.size(); ++i) {
      total_dist += segment_distances[i];
      std::cout << "Points " << i << "-" << (i + 1) << ": " << segment_distances[i]
                << " m (cumulative: " << total_dist << " m)" << std::endl;
    }

    // Calculate lateral jerk (final test)
    [[maybe_unused]] const bool result = validator->checkValidLateralJerk(custom_traj);
  }
}
