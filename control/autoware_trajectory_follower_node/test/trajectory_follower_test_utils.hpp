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
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <sys/time.h>

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
using TrajectoryPointArray = std::vector<autoware_planning_msgs::msg::TrajectoryPoint>;
using VehicleOdometry = nav_msgs::msg::Odometry;
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;

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

inline Trajectory generateClothoidTrajectory(
  std_msgs::msg::Header header, double start_curvature, double end_curvature, double arc_length,
  double velocity, double step_length, double backward_distance = 5.0)
{
  Trajectory trajectory;
  trajectory.header = header;

  const int points =
    static_cast<int>(arc_length / step_length);  // Number of points in the trajectory
  double curvature_rate = (end_curvature - start_curvature) / arc_length;  // Curvature change rate

  // Variables for forward direction
  double x_f = 0.0;
  double y_f = 0.0;
  double theta_f = 0.0;  // Initial angle
  double curvature_f = start_curvature;

  // Variables for backward direction
  double x_b = 0.0;
  double y_b = 0.0;
  double theta_b = 0.0;  // Initial angle, curvature is zero

  const int backward_points = static_cast<int>(backward_distance / step_length);

  // Add the origin point first
  trajectory.points.push_back(make_traj_point(0.0, 0.0, velocity));

  // Generate points along the backward straight line and add them to the front
  for (int i = 1; i <= backward_points; ++i) {
    // Calculate the next point using Euler's method
    double next_x_b = x_b - step_length * cos(theta_b);
    double next_y_b = y_b - step_length * sin(theta_b);
    trajectory.points.insert(
      trajectory.points.begin(), make_traj_point(next_x_b, next_y_b, velocity));

    // Update the parameters
    x_b = next_x_b;
    y_b = next_y_b;
    // Curvature is zero for backward direction, so theta_b remains the same
  }

  // Generate points along the clothoid or constant curvature arc in forward direction
  for (int i = 1; i <= points; ++i) {  // Start from 1 to avoid duplicate point at the origin
    // Calculate the next point using Euler's method
    double next_x_f = x_f + step_length * cos(theta_f);
    double next_y_f = y_f + step_length * sin(theta_f);
    trajectory.points.push_back(make_traj_point(next_x_f, next_y_f, velocity));

    // Update the parameters
    x_f = next_x_f;
    y_f = next_y_f;
    curvature_f +=
      curvature_rate *
      step_length;  // Increment curvature (remains constant if start_curvature == end_curvature)
    theta_f += curvature_f * step_length;  // Increment angle by the current curvature
  }

  return trajectory;
}

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

// x_{k+1} &= x_{k} + v\cos\theta_{k} \, \text{d}t
// y_{k+1} &= y_{k} + v\sin\theta_{k} \, \text{d}t
// \theta_{k+1} &= \theta_{k} + \frac{v}{L} \tan\delta \, \text{d}t
inline void updateOdom(
  [[maybe_unused]] Trajectory & resampled_ref, VehicleOdometry & odom, const float steering_angle,
  const double delta_time, const float wheelbase, [[maybe_unused]] const int num_steps = 10)
{
  // double original_x = odom.pose.pose.position.x;
  // double original_y = odom.pose.pose.position.y;
  double original_yaw = tf2::getYaw(odom.pose.pose.orientation);
  double velocity = odom.twist.twist.linear.x;

  odom.pose.pose.position.x += velocity * cos(original_yaw) * delta_time;
  odom.pose.pose.position.y += velocity * sin(original_yaw) * delta_time;
  double updated_yaw = original_yaw + (velocity / wheelbase) * tan(steering_angle) * delta_time;
  odom.pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), updated_yaw));

  // double euler_x = original_x + velocity * cos(original_yaw) * delta_time;
  // double euler_y = original_y + velocity * sin(original_yaw) * delta_time;
  // double euler_yaw = original_yaw + (velocity / wheelbase) * tan(steering_angle) * delta_time;

  // double k1_x = velocity * cos(original_yaw);
  // double k1_y = velocity * sin(original_yaw);
  // double k1_yaw = (velocity / wheelbase) * tan(steering_angle);

  // double k2_x = velocity * cos(original_yaw + k1_yaw * delta_time / 2);
  // double k2_y = velocity * sin(original_yaw + k1_yaw * delta_time / 2);
  // double k2_yaw = (velocity / wheelbase) * tan(steering_angle);

  // double k3_x = velocity * cos(original_yaw + k2_yaw * delta_time / 2);
  // double k3_y = velocity * sin(original_yaw + k2_yaw * delta_time / 2);
  // double k3_yaw = (velocity / wheelbase) * tan(steering_angle);

  // double k4_x = velocity * cos(original_yaw + k3_yaw * delta_time);
  // double k4_y = velocity * sin(original_yaw + k3_yaw * delta_time);
  // double k4_yaw = (velocity / wheelbase) * tan(steering_angle);

  // double rungekutta_x = original_x + (k1_x + 2 * k2_x + 2 * k3_x + k4_x) * delta_time / 6;
  // double rungekutta_y = original_y + (k1_y + 2 * k2_y + 2 * k3_y + k4_y) * delta_time / 6;
  // double rungekutta_yaw = original_yaw + (k1_yaw + 2 * k2_yaw + 2 * k3_yaw + k4_yaw) * delta_time
  // / 6;

  // double euler_x_split = original_x;
  // double euler_y_split = original_y;
  // double euler_yaw_split = original_yaw;

  // double rungekutta_x_split = original_x;
  // double rungekutta_y_split = original_y;
  // double rungekutta_yaw_split = original_yaw;

  // double dt_step = delta_time / num_steps;

  // for (int i = 0; i < num_steps; ++i) {
  //   euler_x_split += velocity * cos(euler_yaw_split) * dt_step;
  //   euler_y_split += velocity * sin(euler_yaw_split) * dt_step;
  //   euler_yaw_split += (velocity / wheelbase) * tan(steering_angle) * dt_step;

  //   double k1_x = velocity * cos(rungekutta_yaw_split);
  //   double k1_y = velocity * sin(rungekutta_yaw_split);
  //   double k1_yaw = (velocity / wheelbase) * tan(steering_angle);

  //   double k2_x = velocity * cos(rungekutta_yaw_split + k1_yaw * dt_step / 2);
  //   double k2_y = velocity * sin(rungekutta_yaw_split + k1_yaw * dt_step / 2);
  //   double k2_yaw = (velocity / wheelbase) * tan(steering_angle);

  //   double k3_x = velocity * cos(rungekutta_yaw_split + k2_yaw * dt_step / 2);
  //   double k3_y = velocity * sin(rungekutta_yaw_split + k2_yaw * dt_step / 2);
  //   double k3_yaw = (velocity / wheelbase) * tan(steering_angle);

  //   double k4_x = velocity * cos(rungekutta_yaw_split + k3_yaw * dt_step);
  //   double k4_y = velocity * sin(rungekutta_yaw_split + k3_yaw * dt_step);
  //   double k4_yaw = (velocity / wheelbase) * tan(steering_angle);

  //   rungekutta_x_split += (k1_x + 2 * k2_x + 2 * k3_x + k4_x) * dt_step / 6;
  //   rungekutta_y_split += (k1_y + 2 * k2_y + 2 * k3_y + k4_y) * dt_step / 6;
  //   rungekutta_yaw_split += (k1_yaw + 2 * k2_yaw + 2 * k3_yaw + k4_yaw) * dt_step / 6;
  // }

  // std::cerr << "euler_x: " << euler_x << std::endl;
  // std::cerr << "euler_y: " << euler_y << std::endl;
  // std::cerr << "euler_yaw: " << euler_yaw << std::endl;

  // std::cerr << "rungekutta_x: " << rungekutta_x << std::endl;
  // std::cerr << "rungekutta_y: " << rungekutta_y << std::endl;
  // std::cerr << "rungekutta_yaw: " << rungekutta_yaw << std::endl;

  // std::cerr << "euler_x_split: " << euler_x_split << std::endl;
  // std::cerr << "euler_y_split: " << euler_y_split << std::endl;
  // std::cerr << "euler_yaw_split: " << euler_yaw_split << std::endl;

  // std::cerr << "rungekutta_x_split: " << rungekutta_x_split << std::endl;
  // std::cerr << "rungekutta_y_split: " << rungekutta_y_split << std::endl;
  // std::cerr << "rungekutta_yaw_split: " << rungekutta_yaw_split << std::endl;
}

template <typename T>
void writeValuesToFile(const std::vector<T> & values, std::ofstream & output_file)
{
  for (const auto & value : values) {
    output_file << value << ",";
  }
  output_file << std::endl;
}

void writeTrajectoryToFile(
  const Trajectory & trajectory, std::ofstream & output_file_x, std::ofstream & output_file_y)
{
  std::vector<double> x_values;
  std::vector<double> y_values;

  for (const auto & point : trajectory.points) {
    x_values.push_back(point.pose.position.x);
    y_values.push_back(point.pose.position.y);
  }

  writeValuesToFile(x_values, output_file_x);
  writeValuesToFile(y_values, output_file_y);
}

void openOutputFilesInWriteMode(
  const std::string & trajectory_directory, std::ofstream & output_file_orig_ref_x,
  std::ofstream & output_file_orig_ref_y, std::ofstream & output_file_resampled_ref_x,
  std::ofstream & output_file_resampled_ref_y, std::ofstream & output_file_resampled_k,
  std::ofstream & output_file_resampled_vx, std::ofstream & output_file_predicted_x,
  std::ofstream & output_file_predicted_y, std::ofstream & output_file_predicted_frenet_x,
  std::ofstream & output_file_predicted_frenet_y,
  std::ofstream & output_file_cgmres_predicted_frenet_x,
  std::ofstream & output_file_cgmres_predicted_frenet_y,
  std::ofstream & output_file_cgmres_predicted_x, std::ofstream & output_file_cgmres_predicted_y,
  std::ofstream & output_file_time)
{
  output_file_orig_ref_x.open(trajectory_directory + "original_ref_x.log");
  output_file_orig_ref_y.open(trajectory_directory + "original_ref_y.log");
  output_file_resampled_ref_x.open(trajectory_directory + "resampled_ref_x.log");
  output_file_resampled_ref_y.open(trajectory_directory + "resampled_ref_y.log");
  output_file_resampled_k.open(trajectory_directory + "resampled_k.log");
  output_file_resampled_vx.open(trajectory_directory + "resampled_vx.log");
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
  const std::string & trajectory_directory, std::ofstream & output_file_orig_ref_x,
  std::ofstream & output_file_orig_ref_y, std::ofstream & output_file_resampled_ref_x,
  std::ofstream & output_file_resampled_ref_y, std::ofstream & output_file_resampled_k,
  std::ofstream & output_file_resampled_vx, std::ofstream & output_file_predicted_x,
  std::ofstream & output_file_predicted_y, std::ofstream & output_file_predicted_frenet_x,
  std::ofstream & output_file_predicted_frenet_y,
  std::ofstream & output_file_cgmres_predicted_frenet_x,
  std::ofstream & output_file_cgmres_predicted_frenet_y,
  std::ofstream & output_file_cgmres_predicted_x, std::ofstream & output_file_cgmres_predicted_y,
  std::ofstream & output_file_time)
{
  output_file_orig_ref_x.open(trajectory_directory + "original_ref_x.log", std::ios::app);
  output_file_orig_ref_y.open(trajectory_directory + "original_ref_y.log", std::ios::app);
  output_file_resampled_ref_x.open(trajectory_directory + "resampled_ref_x.log", std::ios::app);
  output_file_resampled_ref_y.open(trajectory_directory + "resampled_ref_y.log", std::ios::app);
  output_file_resampled_k.open(trajectory_directory + "resampled_k.log", std::ios::app);
  output_file_resampled_vx.open(trajectory_directory + "resampled_vx.log", std::ios::app);
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
  std::ofstream & output_file_resampled_k, std::ofstream & output_file_resampled_vx,
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
          output_file_resampled_y, output_file_resampled_k, output_file_resampled_vx,
          output_file_predicted_x, output_file_predicted_y, output_file_predicted_frenet_x,
          output_file_predicted_frenet_y, output_file_cgmres_predicted_frenet_x,
          output_file_cgmres_predicted_frenet_y, output_file_cgmres_predicted_x,
          output_file_cgmres_predicted_y, output_file_time);
      } else {
        trajectory_directory = createTrajectoryDirectory(log_directory);
        openOutputFilesInWriteMode(
          trajectory_directory, output_file_orig_x, output_file_orig_y, output_file_resampled_x,
          output_file_resampled_y, output_file_resampled_k, output_file_resampled_vx,
          output_file_predicted_x, output_file_predicted_y, output_file_predicted_frenet_x,
          output_file_predicted_frenet_y, output_file_cgmres_predicted_frenet_x,
          output_file_cgmres_predicted_frenet_y, output_file_cgmres_predicted_x,
          output_file_cgmres_predicted_y, output_file_time);
      }
    } else {
      trajectory_directory = createTrajectoryDirectory(log_directory);
      openOutputFilesInWriteMode(
        trajectory_directory, output_file_orig_x, output_file_orig_y, output_file_resampled_x,
        output_file_resampled_y, output_file_resampled_k, output_file_resampled_vx,
        output_file_predicted_x, output_file_predicted_y, output_file_predicted_frenet_x,
        output_file_predicted_frenet_y, output_file_cgmres_predicted_frenet_x,
        output_file_cgmres_predicted_frenet_y, output_file_cgmres_predicted_x,
        output_file_cgmres_predicted_y, output_file_time);
    }
  } else {
    trajectory_directory = createTrajectoryDirectory(log_directory);
    openOutputFilesInWriteMode(
      trajectory_directory, output_file_orig_x, output_file_orig_y, output_file_resampled_x,
      output_file_resampled_y, output_file_resampled_k, output_file_resampled_vx,
      output_file_predicted_x, output_file_predicted_y, output_file_predicted_frenet_x,
      output_file_predicted_frenet_y, output_file_cgmres_predicted_frenet_x,
      output_file_cgmres_predicted_frenet_y, output_file_cgmres_predicted_x,
      output_file_cgmres_predicted_y, output_file_time);
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
  const Trajectory & cgmres_predicted_trajectory,
  const std::vector<double> & resampled_ref_curvature,
  const std::vector<double> & resampled_ref_velocity, const rclcpp::Time & stamp)
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
    output_file_cgmres_predicted_x, output_file_cgmres_predicted_y, output_file_resampled_k,
    output_file_resampled_vx, output_file_time;
  std::string trajectory_directory = openOutputFiles(
    log_directory, latest_directory, output_file_orig_x, output_file_orig_y,
    output_file_resampled_x, output_file_resampled_y, output_file_predicted_x,
    output_file_predicted_y, output_file_predicted_frenet_x, output_file_predicted_frenet_y,
    output_file_cgmres_predicted_frenet_x, output_file_cgmres_predicted_frenet_y,
    output_file_cgmres_predicted_x, output_file_cgmres_predicted_y, output_file_resampled_k,
    output_file_resampled_vx, output_file_time, stamp);

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
  writeValuesToFile(resampled_ref_curvature, output_file_resampled_k);
  writeValuesToFile(resampled_ref_velocity, output_file_resampled_vx);

  // Write timestamp to file
  double time_in_seconds = stamp.seconds() + static_cast<double>(stamp.nanoseconds()) / 1e9;
  output_file_time << std::fixed << std::setprecision(3) << time_in_seconds << std::endl;
}

}  // namespace test_utils

#endif  // TRAJECTORY_FOLLOWER_TEST_UTILS_HPP_
