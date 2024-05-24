// Copyright 2021 The Autoware Foundation
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

#ifndef TRAJECTORY_FOLLOWER_TEST_UTILS_HPP_
#define TRAJECTORY_FOLLOWER_TEST_UTILS_HPP_

#include "fake_test_node/fake_test_node.hpp"
#include "motion_utils/trajectory/conversion.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

#include "geometry_msgs/msg/transform_stamped.hpp"

#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace test_utils
{
using FakeNodeFixture = autoware::tools::testing::FakeTestNode;
using TrajectoryPointArray = std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;

inline void waitForMessage(
  const std::shared_ptr<rclcpp::Node> & node, FakeNodeFixture * fixture, const bool & received_flag,
  const std::chrono::duration<int> max_wait_time = std::chrono::seconds{10LL},
  const bool fail_on_timeout = true)
{
  const auto dt{std::chrono::milliseconds{30LL}};
  auto time_passed{std::chrono::milliseconds{0LL}};
  while (!received_flag) {
    rclcpp::spin_some(node);
    rclcpp::spin_some(fixture->get_fake_node());
    std::this_thread::sleep_for(dt);
    time_passed += dt;
    if (time_passed > max_wait_time) {
      if (fail_on_timeout) {
        throw std::runtime_error(std::string("Did not receive a message soon enough"));
      } else {
        break;
      }
    }
  }
}

inline geometry_msgs::msg::TransformStamped getDummyTransform()
{
  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.transform.translation.x = 0.0;
  transform_stamped.transform.translation.y = 0.0;
  transform_stamped.transform.translation.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, 0.0);
  transform_stamped.transform.rotation.x = q.x();
  transform_stamped.transform.rotation.y = q.y();
  transform_stamped.transform.rotation.z = q.z();
  transform_stamped.transform.rotation.w = q.w();
  transform_stamped.header.frame_id = "map";
  transform_stamped.child_frame_id = "base_link";
  return transform_stamped;
}

inline TrajectoryPoint make_traj_point(const double px, const double py, const float vx)
{
  TrajectoryPoint p;
  p.pose.position.x = px;
  p.pose.position.y = py;
  p.longitudinal_velocity_mps = vx;
  return p;
}

inline Trajectory generateCurvatureTrajectory(
  std_msgs::msg::Header header, double curvature, double arc_length, double velocity)
{
  Trajectory trajectory;
  trajectory.header = header;

  // Calculate radius from curvature
  double radius =
    1 / fabs(curvature);  // Radius is the reciprocal of the absolute value of curvature
  double arc_angle = arc_length / radius;  // Total angle of the arc

  // Calculate starting angle based on curvature
  double start_angle = (curvature > 0) ? (3 * M_PI / 2) : (M_PI / 2);
  start_angle -= arc_angle / 2;

  const int points = 20;  // Number of points in the trajectory

  // Generate points along the arc in reverse order
  for (int i = points; i >= 0; --i) {
    double angle = start_angle + (arc_angle * i / points);  // Current angle of the point on the arc
    double x = radius * cos(angle);                         // X coordinate
    double y = radius * sin(angle);                         // Y coordinate
    // Adjust y-coordinate based on curvature sign
    y += (curvature > 0) ? radius : -radius;

    trajectory.points.push_back(make_traj_point(x, y, velocity));  // Speed set as per argument
  }

  return trajectory;
}

// inline Trajectory generateCurvatureTrajectory(
//   std_msgs::msg::Header header, double start_curvature, double end_curvature, double arc_length,
//   double velocity)
// {
//   Trajectory trajectory;
//   trajectory.header = header;

//   const int points = 20;  // Number of points in the trajectory

//   // Generate points along the arc in reverse order
//   for (int i = points; i >= 0; --i) {
//     double t = static_cast<double>(i) / points;
//     double curvature = start_curvature + t * (end_curvature - start_curvature);
//     double radius =
//       1 / fabs(curvature);  // Radius is the reciprocal of the absolute value of curvature
//     double arc_angle = arc_length / radius;  // Total angle of the arc

//     // Calculate starting angle based on curvature
//     double start_angle = (curvature > 0) ? (3 * M_PI / 2) : (M_PI / 2);
//     start_angle -= arc_angle / 2;

//     double angle = start_angle + (arc_angle * t);  // Current angle of the point on the arc
//     double x = radius * cos(angle);                // X coordinate
//     double y = radius * sin(angle);                // Y coordinate

//     // Adjust y-coordinate based on curvature sign
//     y += (curvature > 0) ? radius : -radius;

//     trajectory.points.push_back(make_traj_point(x, y, velocity));  // Speed set as per argument
//   }

//   return trajectory;
// }

// TODO(Horibe): modify the controller nodes so that they does not publish topics when data is not
// ready. then, remove this function.
template <typename T>
inline void spinWhile(T & node)
{
  for (size_t i = 0; i < 10; i++) {
    rclcpp::spin_some(node);
    const auto dt{std::chrono::milliseconds{100LL}};
    std::this_thread::sleep_for(dt);
  }
}

void writeTrajectoryToFile(
  const Trajectory & trajectory, std::ofstream & output_file_x, std::ofstream & output_file_y)
{
  for (const auto & point : trajectory.points) {
    output_file_x << point.pose.position.x << ",";
    output_file_y << point.pose.position.y << ",";
  }
  output_file_x << std::endl;
  output_file_y << std::endl;
}

void openOutputFilesInWriteMode(
  const std::string & trajectory_directory, std::ofstream & output_file_orig_x,
  std::ofstream & output_file_orig_y, std::ofstream & output_file_resampled_x,
  std::ofstream & output_file_resampled_y, std::ofstream & output_file_predicted_x,
  std::ofstream & output_file_predicted_y, std::ofstream & output_file_predicted_frenet_x,
  std::ofstream & output_file_predicted_frenet_y,
  std::ofstream & output_file_cgmres_predicted_frenet_x,
  std::ofstream & output_file_cgmres_predicted_frenet_y,
  std::ofstream & output_file_cgmres_predicted_x, std::ofstream & output_file_cgmres_predicted_y,
  std::ofstream & output_file_time)
{
  output_file_orig_x.open(trajectory_directory + "original_x.log");
  output_file_orig_y.open(trajectory_directory + "original_y.log");
  output_file_resampled_x.open(trajectory_directory + "resampled_x.log");
  output_file_resampled_y.open(trajectory_directory + "resampled_y.log");
  output_file_predicted_x.open(trajectory_directory + "predicted_x.log");
  output_file_predicted_y.open(trajectory_directory + "predicted_y.log");
  output_file_predicted_frenet_x.open(trajectory_directory + "predicted_frenet_x.log");
  output_file_predicted_frenet_y.open(trajectory_directory + "predicted_frenet_y.log");
  output_file_cgmres_predicted_frenet_x.open(
    trajectory_directory + "cgmres_predicted_frenet_x.log");
  output_file_cgmres_predicted_frenet_y.open(
    trajectory_directory + "cgmres_predicted_frenet_y.log");
  output_file_cgmres_predicted_x.open(trajectory_directory + "cgmres_predicted_x.log");
  output_file_cgmres_predicted_y.open(trajectory_directory + "cgmres_predicted_y.log");
  output_file_time.open(trajectory_directory + "time.log");
}

void openOutputFilesInAppendMode(
  const std::string & trajectory_directory, std::ofstream & output_file_orig_x,
  std::ofstream & output_file_orig_y, std::ofstream & output_file_resampled_x,
  std::ofstream & output_file_resampled_y, std::ofstream & output_file_predicted_x,
  std::ofstream & output_file_predicted_y, std::ofstream & output_file_predicted_frenet_x,
  std::ofstream & output_file_predicted_frenet_y,
  std::ofstream & output_file_cgmres_predicted_frenet_x,
  std::ofstream & output_file_cgmres_predicted_frenet_y,
  std::ofstream & output_file_cgmres_predicted_x, std::ofstream & output_file_cgmres_predicted_y,
  std::ofstream & output_file_time)
{
  output_file_orig_x.open(trajectory_directory + "original_x.log", std::ios::app);
  output_file_orig_y.open(trajectory_directory + "original_y.log", std::ios::app);
  output_file_resampled_x.open(trajectory_directory + "resampled_x.log", std::ios::app);
  output_file_resampled_y.open(trajectory_directory + "resampled_y.log", std::ios::app);
  output_file_predicted_x.open(trajectory_directory + "predicted_x.log", std::ios::app);
  output_file_predicted_y.open(trajectory_directory + "predicted_y.log", std::ios::app);
  output_file_predicted_frenet_x.open(
    trajectory_directory + "predicted_frenet_x.log", std::ios::app);
  output_file_predicted_frenet_y.open(
    trajectory_directory + "predicted_frenet_y.log", std::ios::app);
  output_file_cgmres_predicted_frenet_x.open(
    trajectory_directory + "cgmres_predicted_frenet_x.log", std::ios::app);
  output_file_cgmres_predicted_frenet_y.open(
    trajectory_directory + "cgmres_predicted_frenet_y.log", std::ios::app);
  output_file_cgmres_predicted_x.open(
    trajectory_directory + "cgmres_predicted_x.log", std::ios::app);
  output_file_cgmres_predicted_y.open(
    trajectory_directory + "cgmres_predicted_y.log", std::ios::app);
  output_file_time.open(trajectory_directory + "time.log", std::ios::app);
}

std::string getTrajectoryDirectory(const std::string & log_directory)
{
  std::ifstream input_file_time(log_directory + "time.log");
  std::string last_line;
  std::string line;
  if (input_file_time.is_open()) {
    while (std::getline(input_file_time, line)) {
      if (!line.empty()) {
        last_line = line;
      }
    }
    input_file_time.close();
  }

  if (!last_line.empty()) {
    double last_time_in_seconds = std::stod(last_line);
    std::string trajectory_directory =
      log_directory + "trajectory_" + std::to_string(last_time_in_seconds) + "/";
    return trajectory_directory;
  } else {
    return "";
  }
}

std::string createTrajectoryDirectory(const std::string & log_directory)
{
  // Get current process ID
  pid_t pid = getpid();
  // Get current timestamp
  struct timeval tv;
  gettimeofday(&tv, NULL);
  int64_t timestamp = static_cast<int64_t>(tv.tv_sec) * 1000000 + tv.tv_usec;
  // Create directory name
  std::stringstream ss;
  ss << "trajectory_" << pid << "_" << timestamp;
  std::string number_string = ss.str();

  // Create directory
  std::string trajectory_directory = log_directory + number_string + "/";
  int ret = mkdir(trajectory_directory.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
  if (ret != 0 && errno != EEXIST) {
    std::cerr << "Error: Failed to create directory " << trajectory_directory << std::endl;
    return "";
  }

  return trajectory_directory;
}

std::string openOutputFiles(
  const std::string & log_directory, const std::string & latest_directory,
  std::ofstream & output_file_orig_x, std::ofstream & output_file_orig_y,
  std::ofstream & output_file_resampled_x, std::ofstream & output_file_resampled_y,
  std::ofstream & output_file_predicted_x, std::ofstream & output_file_predicted_y,
  std::ofstream & output_file_predicted_frenet_x, std::ofstream & output_file_predicted_frenet_y,
  std::ofstream & output_file_cgmres_predicted_frenet_x,
  std::ofstream & output_file_cgmres_predicted_frenet_y,
  std::ofstream & output_file_cgmres_predicted_x, std::ofstream & output_file_cgmres_predicted_y,
  std::ofstream & output_file_time, const rclcpp::Time & stamp)
{
  std::string trajectory_directory;

  if (!latest_directory.empty()) {
    std::string time_log_file = log_directory + latest_directory + "/time.log";
    std::ifstream input_file_time(time_log_file);
    std::string last_line;
    std::string line;
    if (input_file_time.is_open()) {
      while (std::getline(input_file_time, line)) {
        if (!line.empty()) {
          last_line = line;
        }
      }
      input_file_time.close();
    }

    if (!last_line.empty()) {
      double last_time_in_seconds = std::stod(last_line);
      rclcpp::Time last_stamp(last_time_in_seconds);
      double time_in_seconds = stamp.seconds() + static_cast<double>(stamp.nanoseconds()) / 1e9;

      if (time_in_seconds - last_time_in_seconds <= 10.0) {
        trajectory_directory = log_directory + latest_directory + "/";
        openOutputFilesInAppendMode(
          trajectory_directory, output_file_orig_x, output_file_orig_y, output_file_resampled_x,
          output_file_resampled_y, output_file_predicted_x, output_file_predicted_y,
          output_file_predicted_frenet_x, output_file_predicted_frenet_y,
          output_file_cgmres_predicted_frenet_x, output_file_cgmres_predicted_frenet_y,
          output_file_cgmres_predicted_x, output_file_cgmres_predicted_y, output_file_time);
      } else {
        trajectory_directory = createTrajectoryDirectory(log_directory);
        openOutputFilesInWriteMode(
          trajectory_directory, output_file_orig_x, output_file_orig_y, output_file_resampled_x,
          output_file_resampled_y, output_file_predicted_x, output_file_predicted_y,
          output_file_predicted_frenet_x, output_file_predicted_frenet_y,
          output_file_cgmres_predicted_frenet_x, output_file_cgmres_predicted_frenet_y,
          output_file_cgmres_predicted_x, output_file_cgmres_predicted_y, output_file_time);
      }
    } else {
      trajectory_directory = createTrajectoryDirectory(log_directory);
      openOutputFilesInWriteMode(
        trajectory_directory, output_file_orig_x, output_file_orig_y, output_file_resampled_x,
        output_file_resampled_y, output_file_predicted_x, output_file_predicted_y,
        output_file_predicted_frenet_x, output_file_predicted_frenet_y,
        output_file_cgmres_predicted_frenet_x, output_file_cgmres_predicted_frenet_y,
        output_file_cgmres_predicted_x, output_file_cgmres_predicted_y, output_file_time);
    }
  } else {
    trajectory_directory = createTrajectoryDirectory(log_directory);
    openOutputFilesInWriteMode(
      trajectory_directory, output_file_orig_x, output_file_orig_y, output_file_resampled_x,
      output_file_resampled_y, output_file_predicted_x, output_file_predicted_y,
      output_file_predicted_frenet_x, output_file_predicted_frenet_y,
      output_file_cgmres_predicted_frenet_x, output_file_cgmres_predicted_frenet_y,
      output_file_cgmres_predicted_x, output_file_cgmres_predicted_y, output_file_time);
  }

  return trajectory_directory;
}

std::string getLatestDirectory(const std::string & log_directory)
{
  std::string latest_directory;
  std::time_t latest_time = 0;

  for (const auto & entry : std::filesystem::directory_iterator(log_directory)) {
    if (entry.is_directory()) {
      std::string directory_name = entry.path().filename().string();
      if (directory_name.find("trajectory_") == 0) {
        std::time_t directory_time = std::stoll(directory_name.substr(11));
        if (directory_time > latest_time) {
          latest_time = directory_time;
          latest_directory = directory_name;
        }
      }
    }
  }

  return latest_directory;
}

void writeTrajectoriesToFiles(
  const Trajectory & original_ref_trajectory, const Trajectory & resampled_ref_trajectory,
  const Trajectory & predicted_trajectory,
  const Trajectory & predicted_trajectory_in_frenet_coordinate,
  const Trajectory & cgmres_predicted_trajectory_in_frenet_coordinate,
  const Trajectory & cgmres_predicted_trajectory, const rclcpp::Time & stamp)
{
  // Get home directory path
  const char * home_dir = std::getenv("HOME");
  if (home_dir == nullptr) {
    std::cerr << "Error: HOME environment variable not set." << std::endl;
    return;
  }
  const std::string log_directory = std::string(home_dir) + "/.ros/log/";

  // Get the latest directory in log_directory
  std::string latest_directory = getLatestDirectory(log_directory);

  // Open output files
  std::ofstream output_file_orig_x, output_file_orig_y, output_file_resampled_x,
    output_file_resampled_y, output_file_predicted_x, output_file_predicted_y,
    output_file_predicted_frenet_x, output_file_predicted_frenet_y,
    output_file_cgmres_predicted_frenet_x, output_file_cgmres_predicted_frenet_y,
    output_file_cgmres_predicted_x, output_file_cgmres_predicted_y, output_file_time;
  std::string trajectory_directory = openOutputFiles(
    log_directory, latest_directory, output_file_orig_x, output_file_orig_y,
    output_file_resampled_x, output_file_resampled_y, output_file_predicted_x,
    output_file_predicted_y, output_file_predicted_frenet_x, output_file_predicted_frenet_y,
    output_file_cgmres_predicted_frenet_x, output_file_cgmres_predicted_frenet_y,
    output_file_cgmres_predicted_x, output_file_cgmres_predicted_y, output_file_time, stamp);

  // Write trajectories to files
  writeTrajectoryToFile(original_ref_trajectory, output_file_orig_x, output_file_orig_y);
  writeTrajectoryToFile(resampled_ref_trajectory, output_file_resampled_x, output_file_resampled_y);
  writeTrajectoryToFile(predicted_trajectory, output_file_predicted_x, output_file_predicted_y);
  writeTrajectoryToFile(
    predicted_trajectory_in_frenet_coordinate, output_file_predicted_frenet_x,
    output_file_predicted_frenet_y);
  writeTrajectoryToFile(
    cgmres_predicted_trajectory_in_frenet_coordinate, output_file_cgmres_predicted_frenet_x,
    output_file_cgmres_predicted_frenet_y);
  writeTrajectoryToFile(
    cgmres_predicted_trajectory, output_file_cgmres_predicted_x, output_file_cgmres_predicted_y);

  // Write timestamp to file
  double time_in_seconds = stamp.seconds() + static_cast<double>(stamp.nanoseconds()) / 1e9;
  output_file_time << std::fixed << std::setprecision(3) << time_in_seconds << std::endl;
}

}  // namespace test_utils

#endif  // TRAJECTORY_FOLLOWER_TEST_UTILS_HPP_
