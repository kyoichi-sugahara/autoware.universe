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

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "autoware/trajectory_follower_node/controller_node.hpp"
#include "fake_test_node/fake_test_node.hpp"
#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "trajectory_follower_test_utils.hpp"

#include "autoware_adapi_v1_msgs/msg/operation_mode_state.hpp"
#include "autoware_control_msgs/msg/control.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "autoware_vehicle_msgs/msg/steering_report.hpp"
#include "geometry_msgs/msg/accel_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/utils.h>

#include <memory>
#include <vector>

using Controller = autoware::motion::control::trajectory_follower_node::Controller;
using Control = autoware_control_msgs::msg::Control;
using Trajectory = autoware_planning_msgs::msg::Trajectory;
using TrajectoryPoint = autoware_planning_msgs::msg::TrajectoryPoint;
using VehicleOdometry = nav_msgs::msg::Odometry;
using SteeringReport = autoware_vehicle_msgs::msg::SteeringReport;
using autoware_adapi_v1_msgs::msg::OperationModeState;
using geometry_msgs::msg::AccelWithCovarianceStamped;
using std_msgs::msg::Float64MultiArray;

using FakeNodeFixture = autoware::tools::testing::FakeTestNode;

const rclcpp::Duration one_second(1, 0);

rclcpp::NodeOptions makeNodeOptions(const bool enable_keep_stopped_until_steer_convergence = false)
{
  // Pass default parameter file to the node
  const auto share_dir =
    ament_index_cpp::get_package_share_directory("autoware_trajectory_follower_node");
  const auto longitudinal_share_dir =
    ament_index_cpp::get_package_share_directory("autoware_pid_longitudinal_controller");
  const auto lateral_share_dir =
    ament_index_cpp::get_package_share_directory("autoware_mpc_lateral_controller");
  rclcpp::NodeOptions node_options;
  node_options.append_parameter_override("lateral_controller_mode", "mpc");
  node_options.append_parameter_override("longitudinal_controller_mode", "pid");
  node_options.append_parameter_override(
    "enable_keep_stopped_until_steer_convergence",
    enable_keep_stopped_until_steer_convergence);  // longitudinal
  node_options.arguments(
    {"--ros-args", "--params-file",
     lateral_share_dir + "/param/lateral_controller_cgmres.param.yaml", "--params-file",
     //  lateral_share_dir + "/param/lateral_controller_defaults.param.yaml", "--params-file",
     longitudinal_share_dir + "/config/autoware_pid_longitudinal_controller.param.yaml",
     "--params-file", share_dir + "/test/test_vehicle_info.param.yaml", "--params-file",
     share_dir + "/test/test_nearest_search.param.yaml", "--params-file",
     share_dir + "/param/trajectory_follower_node.param.yaml"});

  return node_options;
}

std::shared_ptr<Controller> makeNode(const rclcpp::NodeOptions & node_options)
{
  std::shared_ptr<Controller> node = std::make_shared<Controller>(node_options);

  // Enable all logging in the node
  auto ret =
    rcutils_logging_set_logger_level(node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
  if (ret != RCUTILS_RET_OK) {
    std::cout << "Failed to set logging severity to DEBUG\n";
  }
  return node;
}

class ControllerTester
{
public:
  explicit ControllerTester(FakeNodeFixture * _fnf, const rclcpp::NodeOptions & node_options)
  : fnf(_fnf), node(makeNode(node_options))
  {
  }

  FakeNodeFixture * fnf;
  std::shared_ptr<Controller> node;

  Control::SharedPtr cmd_msg;
  bool received_control_command = false;
  VehicleOdometry::SharedPtr odom_msg;
  bool received_odom_msg = false;
  Trajectory::SharedPtr resampled_reference_trajectory;
  bool received_resampled_reference_trajectory = false;
  Float64MultiArray::SharedPtr resampled_reference_curvature;
  bool received_resampled_reference_curvature = false;
  Float64MultiArray::SharedPtr resampled_reference_velocity;
  bool received_resampled_reference_velocity = false;
  Trajectory::SharedPtr predicted_trajectory_in_frenet_coordinate;
  bool received_predicted_trajectory_in_frenet_coordinate = false;
  Trajectory::SharedPtr predicted_trajectory;
  bool received_predicted_trajectory = false;
  Trajectory::SharedPtr cgmres_predicted_trajectory_in_frenet_coordinate;
  bool received_cgmres_predicted_trajectory_in_frenet_coordinate = false;
  Trajectory::SharedPtr cgmres_predicted_trajectory;
  bool received_cgmres_predicted_trajectory = false;

  void publish_default_odom()
  {
    VehicleOdometry odom_msg;
    odom_msg.header.stamp = node->now();
    odom_pub->publish(odom_msg);
  };

  void publish_odom_vx(const double vx)
  {
    VehicleOdometry odom_msg;
    odom_msg.header.stamp = node->now();
    odom_msg.header.frame_id = "map";
    odom_msg.pose.pose.position.x = 0.0;
    odom_msg.pose.pose.position.y = 0.0;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.twist.twist.linear.x = vx;
    odom_pub->publish(odom_msg);
  };

  void publish_odom(const double vx, const double yaw)
  {
    VehicleOdometry odom_msg;
    odom_msg.header.stamp = node->now();
    odom_msg.header.frame_id = "map";
    odom_msg.pose.pose.position.x = 0.0;
    odom_msg.pose.pose.position.y = 0.0;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.twist.twist.linear.x = vx;
    odom_msg.pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), yaw));
    odom_pub->publish(odom_msg);
  };

  void publish_odom(const VehicleOdometry & odom) { odom_pub->publish(odom); }

  void publish_default_steer()
  {
    SteeringReport steer_msg;
    steer_msg.stamp = node->now();
    steer_pub->publish(steer_msg);
  };

  void publish_steer_angle(const double steer)
  {
    SteeringReport steer_msg;
    steer_msg.stamp = node->now();
    steer_msg.steering_tire_angle = steer;
    steer_pub->publish(steer_msg);
  };

  void publish_default_acc()
  {
    AccelWithCovarianceStamped acc_msg;
    acc_msg.header.stamp = node->now();
    accel_pub->publish(acc_msg);
  };

  void publish_autonomous_operation_mode()
  {
    OperationModeState msg;
    msg.stamp = node->now();
    msg.mode = OperationModeState::AUTONOMOUS;
    operation_mode_pub->publish(msg);
  };

  void publish_default_traj()
  {
    Trajectory traj_msg;
    traj_msg.header.stamp = node->now();
    traj_msg.header.frame_id = "map";
    traj_pub->publish(traj_msg);
  };

  void send_default_transform()
  {
    // Dummy transform: ego is at (0.0, 0.0) in map frame
    geometry_msgs::msg::TransformStamped transform = test_utils::getDummyTransform();
    transform.header.stamp = node->now();
    br->sendTransform(transform);

    // Spin for transform to be published
    test_utils::spinWhile(node);
  };

  rclcpp::Publisher<Trajectory>::SharedPtr traj_pub =
    fnf->create_publisher<Trajectory>("controller/input/reference_trajectory");

  rclcpp::Publisher<VehicleOdometry>::SharedPtr odom_pub =
    fnf->create_publisher<VehicleOdometry>("controller/input/current_odometry");

  rclcpp::Publisher<SteeringReport>::SharedPtr steer_pub =
    fnf->create_publisher<SteeringReport>("controller/input/current_steering");

  rclcpp::Publisher<AccelWithCovarianceStamped>::SharedPtr accel_pub =
    fnf->create_publisher<AccelWithCovarianceStamped>("controller/input/current_accel");

  rclcpp::Publisher<OperationModeState>::SharedPtr operation_mode_pub =
    fnf->create_publisher<OperationModeState>("controller/input/current_operation_mode");

  rclcpp::Subscription<Control>::SharedPtr cmd_sub = fnf->create_subscription<Control>(
    "controller/output/control_cmd", *fnf->get_fake_node(), [this](const Control::SharedPtr msg) {
      cmd_msg = msg;
      received_control_command = true;
    });

  rclcpp::Subscription<VehicleOdometry>::SharedPtr odom_sub =
    fnf->create_subscription<VehicleOdometry>(
      "controller/input/current_odometry", *fnf->get_fake_node(),
      [this](const VehicleOdometry::SharedPtr msg) {
        odom_msg = msg;
        received_odom_msg = true;
      });

  rclcpp::Subscription<Trajectory>::SharedPtr predicted_traj_in_frenet_sub =
    fnf->create_subscription<Trajectory>(
      "controller/debug/predicted_trajectory_in_frenet_coordinate", *fnf->get_fake_node(),
      [this](const Trajectory::SharedPtr msg) {
        predicted_trajectory_in_frenet_coordinate = msg;
        received_predicted_trajectory_in_frenet_coordinate = true;
      });

  rclcpp::Subscription<Trajectory>::SharedPtr predicted_traj_sub =
    fnf->create_subscription<Trajectory>(
      "controller/output/predicted_trajectory", *fnf->get_fake_node(),
      [this](const Trajectory::SharedPtr msg) {
        predicted_trajectory = msg;
        received_predicted_trajectory = true;
      });

  rclcpp::Subscription<Trajectory>::SharedPtr cgmres_predicted_traj_in_frenet_sub =
    fnf->create_subscription<Trajectory>(
      "controller/debug/cgmres/predicted_trajectory_in_frenet_coordinate", *fnf->get_fake_node(),
      [this](const Trajectory::SharedPtr msg) {
        cgmres_predicted_trajectory_in_frenet_coordinate = msg;
        received_cgmres_predicted_trajectory_in_frenet_coordinate = true;
      });

  rclcpp::Subscription<Trajectory>::SharedPtr cgmres_predicted_traj_sub =
    fnf->create_subscription<Trajectory>(
      "controller/debug/cgmres/predicted_trajectory", *fnf->get_fake_node(),
      [this](const Trajectory::SharedPtr msg) {
        cgmres_predicted_trajectory = msg;
        received_cgmres_predicted_trajectory = true;
      });

  rclcpp::Subscription<Trajectory>::SharedPtr resampled_ref_traj_sub =
    fnf->create_subscription<Trajectory>(
      "controller/debug/resampled_reference_trajectory", *fnf->get_fake_node(),
      [this](const Trajectory::SharedPtr msg) {
        resampled_reference_trajectory = msg;
        received_resampled_reference_trajectory = true;
      });

  rclcpp::Subscription<Float64MultiArray>::SharedPtr resampled_ref_curvature_sub =
    fnf->create_subscription<Float64MultiArray>(
      "controller/debug/resampled_reference_curvature", *fnf->get_fake_node(),
      [this](const Float64MultiArray::SharedPtr msg) {
        resampled_reference_curvature = msg;
        received_resampled_reference_curvature = true;
      });

  rclcpp::Subscription<Float64MultiArray>::SharedPtr resampled_ref_velocity_sub =
    fnf->create_subscription<Float64MultiArray>(
      "controller/debug/resampled_reference_velocity", *fnf->get_fake_node(),
      [this](const Float64MultiArray::SharedPtr msg) {
        resampled_reference_velocity = msg;
        received_resampled_reference_velocity = true;
      });

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> br =
    std::make_shared<tf2_ros::StaticTransformBroadcaster>(fnf->get_fake_node());
};

TEST_F(FakeNodeFixture, DISABLED_no_input)
{
  const auto node_options = makeNodeOptions();
  ControllerTester tester(this, node_options);

  // No published data: expect a stopped command
  test_utils::waitForMessage(
    tester.node, this, tester.received_control_command, std::chrono::seconds{1LL}, false);
  ASSERT_FALSE(tester.received_control_command);
}

TEST_F(FakeNodeFixture, DISABLED_empty_trajectory)
{
  const auto node_options = makeNodeOptions();
  ControllerTester tester(this, node_options);

  tester.send_default_transform();

  // Empty trajectory: expect a stopped command
  tester.publish_default_traj();
  tester.publish_default_odom();
  tester.publish_autonomous_operation_mode();
  tester.publish_default_acc();
  tester.publish_default_steer();

  test_utils::waitForMessage(
    tester.node, this, tester.received_control_command, std::chrono::seconds{1LL}, false);
  ASSERT_FALSE(tester.received_control_command);
}

// lateral
TEST_F(FakeNodeFixture, DISABLED_straight_trajectory)
{
  const auto node_options = makeNodeOptions();
  ControllerTester tester(this, node_options);

  tester.send_default_transform();
  tester.publish_odom_vx(1.0);
  tester.publish_autonomous_operation_mode();
  tester.publish_default_steer();
  tester.publish_default_acc();

  Trajectory traj_msg;
  traj_msg.header.stamp = tester.node->now();
  traj_msg.header.frame_id = "map";
  traj_msg.points.push_back(test_utils::make_traj_point(-1.0, 0.0, 1.0f));
  traj_msg.points.push_back(test_utils::make_traj_point(0.0, 0.0, 1.0f));
  traj_msg.points.push_back(test_utils::make_traj_point(1.0, 0.0, 1.0f));
  traj_msg.points.push_back(test_utils::make_traj_point(2.0, 0.0, 1.0f));
  tester.traj_pub->publish(traj_msg);

  test_utils::waitForMessage(tester.node, this, tester.received_control_command);
  ASSERT_TRUE(tester.received_control_command);
  // following conditions will pass even if the MPC solution does not converge
  EXPECT_EQ(tester.cmd_msg->lateral.steering_tire_angle, 0.0f);
  EXPECT_EQ(tester.cmd_msg->lateral.steering_tire_rotation_rate, 0.0f);
  EXPECT_GT(tester.cmd_msg->longitudinal.velocity, 0.0f);
  EXPECT_GT(rclcpp::Time(tester.cmd_msg->stamp), rclcpp::Time(traj_msg.header.stamp));
}

TEST_F(FakeNodeFixture, DISABLED_right_turn)
{
  const auto node_options = makeNodeOptions();
  ControllerTester tester(this, node_options);

  tester.send_default_transform();
  tester.publish_odom_vx(1.0);
  tester.publish_autonomous_operation_mode();
  tester.publish_default_steer();
  tester.publish_default_acc();

  // Right turning trajectory: expect right steering
  Trajectory traj_msg;
  traj_msg.header.stamp = tester.node->now();
  traj_msg.header.frame_id = "map";
  traj_msg.points.push_back(test_utils::make_traj_point(-1.0, -1.0, 1.0f));
  traj_msg.points.push_back(test_utils::make_traj_point(0.0, 0.0, 1.0f));
  traj_msg.points.push_back(test_utils::make_traj_point(1.0, -1.0, 1.0f));
  traj_msg.points.push_back(test_utils::make_traj_point(2.0, -2.0, 1.0f));
  tester.traj_pub->publish(traj_msg);

  test_utils::waitForMessage(tester.node, this, tester.received_control_command);
  ASSERT_TRUE(tester.received_control_command);
  EXPECT_LT(tester.cmd_msg->lateral.steering_tire_angle, 0.0f);
  EXPECT_LT(tester.cmd_msg->lateral.steering_tire_rotation_rate, 0.0f);
  EXPECT_GT(rclcpp::Time(tester.cmd_msg->stamp), rclcpp::Time(traj_msg.header.stamp));
}

TEST_F(FakeNodeFixture, constant_curvature_right_turn)
{
  const auto node_options = makeNodeOptions();
  ControllerTester tester(this, node_options);
  Trajectory ref_trajectory;

  const double velocity = 0.2;
  const double trajectory_arc_length = 20.0;
  const double start_curvature_sign = 0.0;
  const double end_curvature_sign = -0.001;
  const double step_length = 1.0;

  tester.send_default_transform();
  tester.publish_autonomous_operation_mode();
  tester.publish_default_steer();
  tester.publish_default_acc();
  tester.publish_odom_vx(1.0);

  test_utils::waitForMessage(tester.node, this, tester.received_odom_msg);

  auto publishTrajectory = [&tester, &ref_trajectory, start_curvature_sign, end_curvature_sign,
                            trajectory_arc_length, velocity, step_length]() {
    std_msgs::msg::Header header;
    header.stamp = tester.node->now();
    header.frame_id = "map";
    ref_trajectory = test_utils::generateClothoidTrajectory(
      header, start_curvature_sign, end_curvature_sign, trajectory_arc_length, velocity,
      step_length);
    tester.traj_pub->publish(ref_trajectory);
  };

  constexpr size_t iter_num = 10;
  for (size_t i = 0; i < iter_num; i++) {
    publishTrajectory();
    test_utils::waitForMessage(tester.node, this, tester.received_control_command);

    test_utils::writeTrajectoriesToFiles(
      ref_trajectory, *tester.resampled_reference_trajectory, *tester.predicted_trajectory,
      *tester.predicted_trajectory_in_frenet_coordinate,
      *tester.cgmres_predicted_trajectory_in_frenet_coordinate, *tester.cgmres_predicted_trajectory,
      tester.resampled_reference_curvature->data, tester.resampled_reference_velocity->data,
      tester.cmd_msg->stamp);
    ASSERT_TRUE(tester.received_control_command);
    tester.received_control_command = false;
  }
}

TEST_F(FakeNodeFixture, DISABLED_clothoid_right_turn)
{
  const auto node_options = makeNodeOptions();
  ControllerTester tester(this, node_options);
  Trajectory ref_trajectory;

  const double velocity = 5.0;
  const double trajectory_arc_length = 50.0;
  const double start_curvature_sign = 0.0;
  const double end_curvature_sign = -0.1;
  const double step_length = 1.0;
  const double wheel_base = 2.74;
  const double delta_time = 0.03;

  tester.send_default_transform();
  tester.publish_autonomous_operation_mode();
  tester.publish_default_steer();
  tester.publish_default_acc();
  tester.publish_odom_vx(velocity);

  test_utils::waitForMessage(tester.node, this, tester.received_odom_msg);

  auto publishTrajectory = [&tester, &ref_trajectory, start_curvature_sign, end_curvature_sign,
                            trajectory_arc_length, velocity, step_length]() {
    std_msgs::msg::Header header;
    header.stamp = tester.node->now();
    header.frame_id = "map";
    ref_trajectory = test_utils::generateClothoidTrajectory(
      header, start_curvature_sign, end_curvature_sign, trajectory_arc_length, velocity,
      step_length);
    tester.traj_pub->publish(ref_trajectory);
  };

  SteeringReport steering_status;
  steering_status.steering_tire_angle = 0.0;
  constexpr size_t iter_num = 50;
  for (size_t i = 0; i < iter_num; i++) {
    tester.publish_odom(*tester.odom_msg);
    tester.publish_steer_angle(steering_status.steering_tire_angle);

    publishTrajectory();
    test_utils::waitForMessage(tester.node, this, tester.received_control_command);

    test_utils::writeTrajectoriesToFiles(
      ref_trajectory, *tester.resampled_reference_trajectory, *tester.predicted_trajectory,
      *tester.predicted_trajectory_in_frenet_coordinate,
      *tester.cgmres_predicted_trajectory_in_frenet_coordinate, *tester.cgmres_predicted_trajectory,
      tester.resampled_reference_curvature->data, tester.resampled_reference_velocity->data,
      tester.cmd_msg->stamp);
    ASSERT_TRUE(tester.received_control_command);
    tester.received_control_command = false;

    test_utils::updateOdom(
      *tester.odom_msg, tester.cmd_msg->lateral.steering_tire_angle, delta_time, wheel_base,
      steering_status.steering_tire_angle);
  }
}

TEST_F(FakeNodeFixture, DISABLED_periodically_reference_trajectory_change)
{
  const auto node_options = makeNodeOptions();
  ControllerTester tester(this, node_options);
  Trajectory ref_trajectory;

  const double velocity = 5.0;
  const double trajectory_arc_length = 50.0;
  const double start_curvature_sign = 0.0;
  const double step_length = 1.0;

  const int steps_per_change = 10;
  const double curvature_increment = -0.02;
  double current_end_curvature_sign = -0.1;

  tester.send_default_transform();
  tester.publish_autonomous_operation_mode();
  tester.publish_default_steer();
  tester.publish_default_acc();
  tester.publish_odom_vx(velocity);

  test_utils::waitForMessage(tester.node, this, tester.received_odom_msg);

  auto publishTrajectory = [&tester, &ref_trajectory, start_curvature_sign,
                            &current_end_curvature_sign, trajectory_arc_length, velocity,
                            step_length]() {
    std_msgs::msg::Header header;
    header.stamp = tester.node->now();
    header.frame_id = "map";
    ref_trajectory = test_utils::generateClothoidTrajectory(
      header, start_curvature_sign, current_end_curvature_sign, trajectory_arc_length, velocity,
      step_length);
    tester.traj_pub->publish(ref_trajectory);
  };

  constexpr size_t iter_num = 100;
  for (size_t i = 0; i < iter_num; i++) {
    tester.publish_odom(*tester.odom_msg);

    if (i % steps_per_change == 0) {
      current_end_curvature_sign += curvature_increment;
    }

    publishTrajectory();
    test_utils::waitForMessage(tester.node, this, tester.received_control_command);

    test_utils::writeTrajectoriesToFiles(
      ref_trajectory, *tester.resampled_reference_trajectory, *tester.predicted_trajectory,
      *tester.predicted_trajectory_in_frenet_coordinate,
      *tester.cgmres_predicted_trajectory_in_frenet_coordinate, *tester.cgmres_predicted_trajectory,
      tester.resampled_reference_curvature->data, tester.resampled_reference_velocity->data,
      tester.cmd_msg->stamp);
    ASSERT_TRUE(tester.received_control_command);
    tester.received_control_command = false;
  }
}

TEST_F(FakeNodeFixture, DISABLED_right_turn_with_initial_yaw_bias)
{
  const auto node_options = makeNodeOptions();
  ControllerTester tester(this, node_options);
  Trajectory ref_trajectory;

  const double velocity = 5.0;
  const double trajectory_arc_length = 50.0;
  const double start_curvature_sign = -0.01;
  const double end_curvature_sign = -0.01;
  const double step_length = 1.0;
  const double wheel_base = 2.74;
  const double delta_time = 0.03;

  tester.send_default_transform();
  tester.publish_autonomous_operation_mode();
  tester.publish_default_steer();
  tester.publish_default_acc();
  tester.publish_odom_vx(velocity);

  test_utils::waitForMessage(tester.node, this, tester.received_odom_msg);

  auto publishTrajectory = [&tester, &ref_trajectory, start_curvature_sign, end_curvature_sign,
                            trajectory_arc_length, velocity, step_length]() {
    std_msgs::msg::Header header;
    header.stamp = tester.node->now();
    header.frame_id = "map";
    ref_trajectory = test_utils::generateClothoidTrajectory(
      header, start_curvature_sign, end_curvature_sign, trajectory_arc_length, velocity,
      step_length);

    tester.traj_pub->publish(ref_trajectory);
  };

  SteeringReport steering_status;
  steering_status.steering_tire_angle = 0.0;
  constexpr size_t iter_num = 50;
  for (size_t i = 0; i < iter_num; i++) {
    tester.publish_odom(*tester.odom_msg);
    tester.publish_steer_angle(steering_status.steering_tire_angle);

    publishTrajectory();
    test_utils::waitForMessage(tester.node, this, tester.received_control_command);

    test_utils::writeTrajectoriesToFiles(
      ref_trajectory, *tester.resampled_reference_trajectory, *tester.predicted_trajectory,
      *tester.predicted_trajectory_in_frenet_coordinate,
      *tester.cgmres_predicted_trajectory_in_frenet_coordinate, *tester.cgmres_predicted_trajectory,
      tester.resampled_reference_curvature->data, tester.resampled_reference_velocity->data,
      tester.cmd_msg->stamp);
    ASSERT_TRUE(tester.received_control_command);
    tester.received_control_command = false;

    test_utils::updateOdom(
      *tester.odom_msg, tester.cmd_msg->lateral.steering_tire_angle, delta_time, wheel_base,
      steering_status.steering_tire_angle);
  }
}

TEST_F(FakeNodeFixture, DISABLED_left_turn)
{
  const auto node_options = makeNodeOptions();
  ControllerTester tester(this, node_options);

  tester.send_default_transform();
  tester.publish_odom_vx(1.0);
  tester.publish_autonomous_operation_mode();
  tester.publish_default_steer();
  tester.publish_default_acc();

  // Left turning trajectory: expect left steering
  Trajectory traj_msg;
  traj_msg.header.stamp = tester.node->now();
  traj_msg.header.frame_id = "map";
  traj_msg.points.push_back(test_utils::make_traj_point(-1.0, 1.0, 1.0f));
  traj_msg.points.push_back(test_utils::make_traj_point(0.0, 0.0, 1.0f));
  traj_msg.points.push_back(test_utils::make_traj_point(1.0, 1.0, 1.0f));
  traj_msg.points.push_back(test_utils::make_traj_point(2.0, 2.0, 1.0f));

  test_utils::waitForMessage(tester.node, this, tester.received_control_command);
  ASSERT_TRUE(tester.received_control_command);
  EXPECT_GT(tester.cmd_msg->lateral.steering_tire_angle, 0.0f);
  EXPECT_GT(tester.cmd_msg->lateral.steering_tire_rotation_rate, 0.0f);
  EXPECT_GT(rclcpp::Time(tester.cmd_msg->stamp), rclcpp::Time(traj_msg.header.stamp));
}

TEST_F(FakeNodeFixture, DISABLED_stopped)
{
  const auto node_options = makeNodeOptions();
  ControllerTester tester(this, node_options);

  tester.send_default_transform();
  tester.publish_default_odom();
  tester.publish_autonomous_operation_mode();
  tester.publish_default_acc();

  const double steering_tire_angle = -0.5;
  tester.publish_steer_angle(steering_tire_angle);

  // Straight trajectory: expect no steering
  Trajectory traj_msg;
  traj_msg.header.stamp = tester.node->now();
  traj_msg.header.frame_id = "map";
  traj_msg.points.push_back(test_utils::make_traj_point(-1.0, 0.0, 0.0f));
  traj_msg.points.push_back(test_utils::make_traj_point(0.0, 0.0, 0.0f));
  traj_msg.points.push_back(test_utils::make_traj_point(1.0, 0.0, 0.0f));
  traj_msg.points.push_back(test_utils::make_traj_point(2.0, 0.0, 0.0f));
  tester.traj_pub->publish(traj_msg);

  test_utils::waitForMessage(tester.node, this, tester.received_control_command);
  ASSERT_TRUE(tester.received_control_command);
  EXPECT_EQ(tester.cmd_msg->lateral.steering_tire_angle, steering_tire_angle);
  EXPECT_EQ(tester.cmd_msg->lateral.steering_tire_rotation_rate, 0.0f);
  EXPECT_GT(rclcpp::Time(tester.cmd_msg->stamp), rclcpp::Time(traj_msg.header.stamp));
}

// longitudinal
TEST_F(FakeNodeFixture, DISABLED_longitudinal_keep_velocity)
{
  const auto node_options = makeNodeOptions();
  ControllerTester tester(this, node_options);

  tester.send_default_transform();
  tester.publish_odom_vx(1.0);
  tester.publish_autonomous_operation_mode();
  tester.publish_default_steer();
  tester.publish_default_acc();

  // Publish non stopping trajectory
  Trajectory traj_msg;
  traj_msg.header.stamp = tester.node->now();
  traj_msg.header.frame_id = "map";
  traj_msg.points.push_back(test_utils::make_traj_point(0.0, 0.0, 1.0f));
  traj_msg.points.push_back(test_utils::make_traj_point(50.0, 0.0, 1.0f));
  traj_msg.points.push_back(test_utils::make_traj_point(100.0, 0.0, 1.0f));
  tester.traj_pub->publish(traj_msg);

  test_utils::waitForMessage(tester.node, this, tester.received_control_command);

  ASSERT_TRUE(tester.received_control_command);
  EXPECT_DOUBLE_EQ(tester.cmd_msg->longitudinal.velocity, 1.0);
  EXPECT_DOUBLE_EQ(tester.cmd_msg->longitudinal.acceleration, 0.0);

  // Generate another control message
  tester.traj_pub->publish(traj_msg);
  test_utils::waitForMessage(tester.node, this, tester.received_control_command);
  ASSERT_TRUE(tester.received_control_command);
  EXPECT_DOUBLE_EQ(tester.cmd_msg->longitudinal.velocity, 1.0);
  EXPECT_DOUBLE_EQ(tester.cmd_msg->longitudinal.acceleration, 0.0);
}

TEST_F(FakeNodeFixture, DISABLED_longitudinal_slow_down)
{
  const auto node_options = makeNodeOptions();
  ControllerTester tester(this, node_options);

  tester.send_default_transform();
  tester.publish_default_acc();
  tester.publish_default_steer();

  const double odom_vx = 1.0;
  tester.publish_odom_vx(odom_vx);

  tester.publish_autonomous_operation_mode();

  // Publish non stopping trajectory
  Trajectory traj;
  traj.header.stamp = tester.node->now();
  traj.header.frame_id = "map";
  traj.points.push_back(test_utils::make_traj_point(0.0, 0.0, 0.5f));
  traj.points.push_back(test_utils::make_traj_point(50.0, 0.0, 0.5f));
  traj.points.push_back(test_utils::make_traj_point(100.0, 0.0, 0.5f));
  tester.traj_pub->publish(traj);

  test_utils::waitForMessage(tester.node, this, tester.received_control_command);

  ASSERT_TRUE(tester.received_control_command);
  EXPECT_LT(tester.cmd_msg->longitudinal.velocity, static_cast<float>(odom_vx));
  EXPECT_LT(tester.cmd_msg->longitudinal.acceleration, 0.0f);

  // Generate another control message
  tester.traj_pub->publish(traj);
  test_utils::waitForMessage(tester.node, this, tester.received_control_command);
  ASSERT_TRUE(tester.received_control_command);
  EXPECT_LT(tester.cmd_msg->longitudinal.velocity, static_cast<float>(odom_vx));
  EXPECT_LT(tester.cmd_msg->longitudinal.acceleration, 0.0f);
}

TEST_F(FakeNodeFixture, DISABLED_longitudinal_accelerate)
{
  const auto node_options = makeNodeOptions();
  ControllerTester tester(this, node_options);

  tester.send_default_transform();
  tester.publish_default_steer();
  tester.publish_default_acc();

  const double odom_vx = 0.5;
  tester.publish_odom_vx(odom_vx);

  tester.publish_autonomous_operation_mode();

  // Publish non stopping trajectory
  Trajectory traj;
  traj.header.stamp = tester.node->now();
  traj.header.frame_id = "map";
  traj.points.push_back(test_utils::make_traj_point(0.0, 0.0, 1.0f));
  traj.points.push_back(test_utils::make_traj_point(50.0, 0.0, 1.0f));
  traj.points.push_back(test_utils::make_traj_point(100.0, 0.0, 1.0f));
  tester.traj_pub->publish(traj);

  test_utils::waitForMessage(tester.node, this, tester.received_control_command);

  ASSERT_TRUE(tester.received_control_command);
  EXPECT_GT(tester.cmd_msg->longitudinal.velocity, static_cast<float>(odom_vx));
  EXPECT_GT(tester.cmd_msg->longitudinal.acceleration, 0.0f);

  // Generate another control message
  tester.traj_pub->publish(traj);
  test_utils::waitForMessage(tester.node, this, tester.received_control_command);
  ASSERT_TRUE(tester.received_control_command);
  EXPECT_GT(tester.cmd_msg->longitudinal.velocity, static_cast<float>(odom_vx));
  EXPECT_GT(tester.cmd_msg->longitudinal.acceleration, 0.0f);
}

TEST_F(FakeNodeFixture, DISABLED_longitudinal_stopped)
{
  const auto node_options = makeNodeOptions();
  ControllerTester tester(this, node_options);

  tester.send_default_transform();
  tester.publish_default_odom();
  tester.publish_autonomous_operation_mode();
  tester.publish_default_steer();
  tester.publish_default_acc();

  // Publish stopping trajectory
  Trajectory traj;
  traj.header.stamp = tester.node->now();
  traj.header.frame_id = "map";
  traj.points.push_back(test_utils::make_traj_point(0.0, 0.0, 0.0f));
  traj.points.push_back(test_utils::make_traj_point(50.0, 0.0, 0.0f));
  traj.points.push_back(test_utils::make_traj_point(100.0, 0.0, 0.0f));
  tester.traj_pub->publish(traj);

  test_utils::waitForMessage(tester.node, this, tester.received_control_command);

  ASSERT_TRUE(tester.received_control_command);
  EXPECT_DOUBLE_EQ(tester.cmd_msg->longitudinal.velocity, 0.0f);
  EXPECT_LT(
    tester.cmd_msg->longitudinal.acceleration,
    0.0f);  // when stopped negative acceleration to brake
}

TEST_F(FakeNodeFixture, DISABLED_longitudinal_reverse)
{
  const auto node_options = makeNodeOptions();
  ControllerTester tester(this, node_options);

  tester.send_default_transform();

  tester.publish_default_odom();
  tester.publish_autonomous_operation_mode();
  tester.publish_default_steer();
  tester.publish_default_acc();

  // Publish reverse
  Trajectory traj;
  traj.header.stamp = tester.node->now();
  traj.header.frame_id = "map";
  traj.points.push_back(test_utils::make_traj_point(0.0, 0.0, -1.0f));
  traj.points.push_back(test_utils::make_traj_point(50.0, 0.0, -1.0f));
  traj.points.push_back(test_utils::make_traj_point(100.0, 0.0, -1.0f));
  tester.traj_pub->publish(traj);

  test_utils::waitForMessage(tester.node, this, tester.received_control_command);

  ASSERT_TRUE(tester.received_control_command);
  EXPECT_LT(tester.cmd_msg->longitudinal.velocity, 0.0f);
  EXPECT_GT(tester.cmd_msg->longitudinal.acceleration, 0.0f);
}

TEST_F(FakeNodeFixture, DISABLED_longitudinal_emergency)
{
  const auto node_options = makeNodeOptions();
  ControllerTester tester(this, node_options);

  tester.send_default_transform();
  tester.publish_default_odom();
  tester.publish_autonomous_operation_mode();
  tester.publish_default_steer();
  tester.publish_default_acc();

  // Publish trajectory starting away from the current ego pose
  Trajectory traj;
  traj.header.stamp = tester.node->now();
  traj.header.frame_id = "map";
  traj.points.push_back(test_utils::make_traj_point(10.0, 0.0, 1.0f));
  traj.points.push_back(test_utils::make_traj_point(50.0, 0.0, 1.0f));
  traj.points.push_back(test_utils::make_traj_point(100.0, 0.0, 1.0f));
  tester.traj_pub->publish(traj);

  test_utils::waitForMessage(tester.node, this, tester.received_control_command);

  ASSERT_TRUE(tester.received_control_command);
  // Emergencies (e.g., far from trajectory) produces braking command (0 vel, negative accel)
  EXPECT_DOUBLE_EQ(tester.cmd_msg->longitudinal.velocity, 0.0f);
  EXPECT_LT(tester.cmd_msg->longitudinal.acceleration, 0.0f);
}

TEST_F(FakeNodeFixture, DISABLED_longitudinal_not_check_steer_converged)
{
  const auto node_options = makeNodeOptions();
  ControllerTester tester(this, node_options);

  tester.send_default_transform();
  tester.publish_default_odom();
  tester.publish_autonomous_operation_mode();
  tester.publish_default_acc();

  // steering_tire_angle has to be larger than the threshold to check convergence.
  const double steering_tire_angle = -0.5;
  tester.publish_steer_angle(steering_tire_angle);

  // Publish trajectory starting away from the current ego pose
  Trajectory traj;
  traj.header.stamp = tester.node->now();
  traj.header.frame_id = "map";
  traj.points.push_back(test_utils::make_traj_point(0.0, 0.0, 1.0f));
  traj.points.push_back(test_utils::make_traj_point(50.0, 0.0, 1.0f));
  traj.points.push_back(test_utils::make_traj_point(100.0, 0.0, 1.0f));
  tester.traj_pub->publish(traj);

  test_utils::waitForMessage(tester.node, this, tester.received_control_command);

  ASSERT_TRUE(tester.received_control_command);
  // Not keep stopped state when the lateral control is not converged.
  EXPECT_DOUBLE_EQ(tester.cmd_msg->longitudinal.velocity, 1.0f);
}

TEST_F(FakeNodeFixture, DISABLED_longitudinal_check_steer_converged)
{
  // set enable_keep_stopped_until_steer_convergence true
  const auto node_options = makeNodeOptions(true);
  ControllerTester tester(this, node_options);

  tester.send_default_transform();
  tester.publish_default_odom();
  tester.publish_autonomous_operation_mode();
  tester.publish_default_acc();

  // steering_tire_angle has to be larger than the threshold to check convergence.
  const double steering_tire_angle = -0.5;
  tester.publish_steer_angle(steering_tire_angle);

  // Publish trajectory starting away from the current ego pose
  Trajectory traj;
  traj.header.stamp = tester.node->now();
  traj.header.frame_id = "map";
  traj.points.push_back(test_utils::make_traj_point(0.0, 0.0, 1.0f));
  traj.points.push_back(test_utils::make_traj_point(50.0, 0.0, 1.0f));
  traj.points.push_back(test_utils::make_traj_point(100.0, 0.0, 1.0f));
  tester.traj_pub->publish(traj);

  {  // Check if the ego can keep stopped when the steering is not converged.
    tester.traj_pub->publish(traj);
    test_utils::waitForMessage(tester.node, this, tester.received_control_command);

    ASSERT_TRUE(tester.received_control_command);
    // Keep stopped state when the lateral control is not converged.
    EXPECT_DOUBLE_EQ(tester.cmd_msg->longitudinal.velocity, 0.0f);
  }

  {  // Check if the ego can keep stopped after the following sequence
    // 1. not converged -> 2. converged -> 3. not converged
    tester.publish_steer_angle(0.0);
    tester.traj_pub->publish(traj);
    test_utils::waitForMessage(tester.node, this, tester.received_control_command);

    tester.publish_steer_angle(steering_tire_angle);
    tester.traj_pub->publish(traj);
    test_utils::waitForMessage(tester.node, this, tester.received_control_command);

    ASSERT_TRUE(tester.received_control_command);
    // Keep stopped state when the lateral control is not converged.
    EXPECT_DOUBLE_EQ(tester.cmd_msg->longitudinal.velocity, 0.0f);
  }
}
