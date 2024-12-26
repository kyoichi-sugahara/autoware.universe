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

#ifndef AUTOWARE__LATERAL_OPTIMAL_CONTROLLER__MPC_LATERAL_CONTROLLER__PARAMETERS_HPP_
#define AUTOWARE__LATERAL_OPTIMAL_CONTROLLER__MPC_LATERAL_CONTROLLER__PARAMETERS_HPP_

#include <rclcpp/node.hpp>

// #include <autoware_planning_msgs/msg/lanelet_route.hpp>
// #include <autoware_planning_msgs/msg/trajectory.hpp>
// #include <autoware_planning_msgs/msg/trajectory_point.hpp>
// #include <nav_msgs/msg/odometry.hpp>

// #include <lanelet2_core/LaneletMap.h>

// #include <map>
// #include <string>
// #include <vector>

namespace autoware::motion::control::lateral_optimal_controller
{
// using autoware::universe_utils::PoseDeviation;
// using autoware_planning_msgs::msg::LaneletRoute;
// using autoware_planning_msgs::msg::Trajectory;
// using autoware_planning_msgs::msg::TrajectoryPoint;
// using TrajectoryPoints = std::vector<TrajectoryPoint>;
// using autoware::universe_utils::LinearRing2d;
// Weight factors used in Model Predictive Control
struct MPCWeight
{
  // Weight for lateral tracking error. A larger weight leads to less lateral tracking error.
  double lat_error;

  // Weight for heading tracking error. A larger weight reduces heading tracking error.
  double heading_error;

  // Weight combining heading error and velocity. Adjusts the influence of heading error based on
  // velocity.
  double heading_error_squared_vel;

  // Weight for lateral tracking error at the terminal state of the trajectory. This improves the
  // stability of MPC.
  double terminal_lat_error;

  // Weight for heading tracking error at the terminal state of the trajectory. This improves the
  // stability of MPC.
  double terminal_heading_error;

  // Weight for the steering input. This surpress the deviation between the steering command and
  // reference steering angle calculated from curvature.
  double steering_input;

  // Adjusts the influence of steering_input weight based on velocity.
  double steering_input_squared_vel;

  // Weight for lateral jerk. Penalizes sudden changes in lateral acceleration.
  double lat_jerk;

  // Weight for steering rate. Penalizes the speed of steering angle change.
  double steer_rate;

  // Weight for steering angle acceleration. Regulates the rate of change of steering rate.
  double steer_acc;
};

struct MPCParam
{
  // Number of steps in the prediction horizon.
  int prediction_horizon;

  // Sampling time for the prediction horizon.
  double prediction_dt;

  // Threshold at which the feed-forward steering angle becomes zero.
  double zero_ff_steer_deg;

  // Time delay for compensating the steering input.
  double input_delay;

  // Limit for calculating trajectory velocity.
  double acceleration_limit;

  // Time constant for calculating trajectory velocity.
  double velocity_time_constant;

  // Minimum prediction distance used for low velocity case.
  double min_prediction_length;

  // Time constant for the steer model.
  double steer_tau;

  // Weight parameters for the MPC in nominal conditions.
  MPCWeight nominal_weight;

  // Weight parameters for the MPC in low curvature path conditions.
  MPCWeight low_curvature_weight;

  // Curvature threshold to determine when to use "low curvature" parameter settings.
  double low_curvature_thresh_curvature;
};

struct TrajectoryFilteringParam
{
  static TrajectoryFilteringParam init(rclcpp::Node & node);
  // path resampling interval [m]
  double traj_resample_dist;

  // flag of traj extending for terminal yaw
  bool extend_trajectory_for_end_yaw_control;

  // flag for path smoothing
  bool enable_path_smoothing;

  // param of moving average filter for path smoothing
  int path_filter_moving_ave_num;

  // point-to-point index distance for curvature calculation for trajectory
  int curvature_smoothing_num_traj;

  // point-to-point index distance for curvature calculation for reference steer command
  int curvature_smoothing_num_ref_steer;
};

struct MPCData
{
  // Index of the nearest point in the trajectory.
  size_t nearest_idx{};

  // Time stamp of the nearest point in the trajectory.
  double nearest_time{};

  // Pose (position and orientation) of the nearest point in the trajectory.
  Pose nearest_pose{};

  // Current steering angle.
  double steer{};

  // Predicted steering angle based on the vehicle model.
  double predicted_steer{};

  // Lateral tracking error.
  double lateral_err{};

  // Yaw (heading) tracking error.
  double yaw_err{};

  MPCData() = default;
};

/**
 * MPC matrix with the following format:
 * Xex = Aex * X0 + Bex * Uex * Wex
 * Yex = Cex * Xex
 * Cost = Xex' * Qex * Xex + (Uex - Uref_ex)' * R1ex * (Uex - Uref_ex) +  Uex' * R2ex * Uex
 */
struct MPCMatrix
{
  MatrixXd Aex;
  MatrixXd Bex;
  MatrixXd Wex;
  MatrixXd Cex;
  MatrixXd Qex;
  MatrixXd R1ex;
  MatrixXd R2ex;
  MatrixXd Uref_ex;

  MPCMatrix() = default;
};
}  // namespace autoware::motion::control::lateral_optimal_controller

#endif  // AUTOWARE__LATERAL_OPTIMAL_CONTROLLER__MPC_LATERAL_CONTROLLER__PARAMETERS_HPP_
