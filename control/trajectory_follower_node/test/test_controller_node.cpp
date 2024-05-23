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
#include "fake_test_node/fake_test_node.hpp"
#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rclcpp/time.hpp"
#include "rcutils/time.h"
#include "rosbag2_cpp/writers/sequential_writer.hpp"
#include "rosbag2_storage/serialized_bag_message.hpp"
#include "rosbag2_storage/topic_metadata.hpp"
#include "trajectory_follower_node/controller_node.hpp"
#include "trajectory_follower_test_utils.hpp"

#include "autoware_adapi_v1_msgs/msg/operation_mode_state.hpp"
#include "autoware_auto_control_msgs/msg/ackermann_lateral_command.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "autoware_auto_vehicle_msgs/msg/steering_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/vehicle_odometry.hpp"
#include "geometry_msgs/msg/accel_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <memory>
#include <random>
#include <vector>

using Controller = autoware::motion::control::trajectory_follower_node::Controller;
using AckermannControlCommand = autoware_auto_control_msgs::msg::AckermannControlCommand;
using Trajectory = autoware_auto_planning_msgs::msg::Trajectory;
using TrajectoryPoint = autoware_auto_planning_msgs::msg::TrajectoryPoint;
using VehicleOdometry = nav_msgs::msg::Odometry;
using SteeringReport = autoware_auto_vehicle_msgs::msg::SteeringReport;
using autoware_adapi_v1_msgs::msg::OperationModeState;
using geometry_msgs::msg::AccelWithCovarianceStamped;

using FakeNodeFixture = autoware::tools::testing::FakeTestNode;

const rclcpp::Duration one_second(1, 0);

rclcpp::NodeOptions makeNodeOptions(const bool enable_keep_stopped_until_steer_convergence = false)
{
  // Pass default parameter file to the node
  const auto share_dir = ament_index_cpp::get_package_share_directory("trajectory_follower_node");
  const auto longitudinal_share_dir =
    ament_index_cpp::get_package_share_directory("pid_longitudinal_controller");
  const auto lateral_share_dir =
    ament_index_cpp::get_package_share_directory("mpc_lateral_controller");
  rclcpp::NodeOptions node_options;
  node_options.append_parameter_override("lateral_controller_mode", "mpc");
  node_options.append_parameter_override("longitudinal_controller_mode", "pid");
  node_options.append_parameter_override(
    "enable_keep_stopped_until_steer_convergence", enable_keep_stopped_until_steer_convergence);
  node_options.arguments(
    {"--ros-args", "--params-file",
     lateral_share_dir + "/param/lateral_controller_cgmres.param.yaml", "--params-file",
     //  lateral_share_dir + "/param/lateral_controller_defaults.param.yaml", "--params-file",
     longitudinal_share_dir + "/param/longitudinal_controller_defaults.param.yaml", "--params-file",
     share_dir + "/test/test_vehicle_info.param.yaml", "--params-file",
     share_dir + "/test/test_nearest_search.param.yaml", "--params-file",
     share_dir + "/param/trajectory_follower_node.param.yaml"});

  return node_options;
}

std::shared_ptr<Controller> makeNode(const rclcpp::NodeOptions & node_options)
{
  const auto node = std::make_shared<Controller>(node_options);
  if (
    rcutils_logging_set_logger_level(node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG) !=
    RCUTILS_RET_OK) {
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
    traj_pub = fnf->create_publisher<Trajectory>("controller/input/reference_trajectory");
    odom_pub = fnf->create_publisher<VehicleOdometry>("controller/input/current_odometry");
    steer_pub = fnf->create_publisher<SteeringReport>("controller/input/current_steering");
    accel_pub = fnf->create_publisher<AccelWithCovarianceStamped>("controller/input/current_accel");
    operation_mode_pub =
      fnf->create_publisher<OperationModeState>("controller/input/current_operation_mode");

    cmd_sub = fnf->create_subscription<AckermannControlCommand>(
      "controller/output/control_cmd", *fnf->get_fake_node(),
      [this](const AckermannControlCommand::SharedPtr msg) {
        cmd_msg = msg;
        received_control_command = true;
      });

    predicted_traj_in_frenet_sub = fnf->create_subscription<Trajectory>(
      "controller/debug/predicted_trajectory_in_frenet_coordinate", *fnf->get_fake_node(),
      [this](const Trajectory::SharedPtr msg) {
        predicted_trajectory_in_frenet_coordinate = msg;
        received_predicted_trajectory_in_frenet_coordinate = true;
      });

    predicted_traj_sub = fnf->create_subscription<Trajectory>(
      "controller/output/predicted_trajectory", *fnf->get_fake_node(),
      [this](const Trajectory::SharedPtr msg) {
        predicted_trajectory = msg;
        received_predicted_trajectory = true;
      });

    cgmres_predicted_traj_in_frenet_sub = fnf->create_subscription<Trajectory>(
      "controller/debug/cgmres/predicted_trajectory_in_frenet_coordinate", *fnf->get_fake_node(),
      [this](const Trajectory::SharedPtr msg) {
        cgmres_predicted_trajectory_in_frenet_coordinate = msg;
        received_cgmres_predicted_trajectory_in_frenet_coordinate = true;
      });

    cgmres_predicted_traj_sub = fnf->create_subscription<Trajectory>(
      "controller/debug/cgmres/predicted_trajectory", *fnf->get_fake_node(),
      [this](const Trajectory::SharedPtr msg) {
        cgmres_predicted_trajectory = msg;
        received_cgmres_predicted_trajectory = true;
      });

    resampled_ref_traj_sub = fnf->create_subscription<Trajectory>(
      "controller/debug/resampled_reference_trajectory", *fnf->get_fake_node(),
      [this](const Trajectory::SharedPtr msg) {
        resampled_reference_trajectory = msg;
        received_resampled_reference_trajectory = true;
      });

    br = std::make_shared<tf2_ros::StaticTransformBroadcaster>(fnf->get_fake_node());
  }

  void publish_default_odom()
  {
    VehicleOdometry odom_msg;
    odom_msg.header.stamp = node->now();
    odom_pub->publish(odom_msg);
  }

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
  }

  void publish_default_steer()
  {
    SteeringReport steer_msg;
    steer_msg.stamp = node->now();
    steer_pub->publish(steer_msg);
  }

  void publish_steer_angle(const double steer)
  {
    SteeringReport steer_msg;
    steer_msg.stamp = node->now();
    steer_msg.steering_tire_angle = steer;
    steer_pub->publish(steer_msg);
  }

  void publish_default_acc()
  {
    AccelWithCovarianceStamped acc_msg;
    acc_msg.header.stamp = node->now();
    accel_pub->publish(acc_msg);
  }

  void publish_autonomous_operation_mode()
  {
    OperationModeState msg;
    msg.stamp = node->now();
    msg.mode = OperationModeState::AUTONOMOUS;
    operation_mode_pub->publish(msg);
  }

  void publish_default_traj()
  {
    Trajectory traj_msg;
    traj_msg.header.stamp = node->now();
    traj_msg.header.frame_id = "map";
    traj_pub->publish(traj_msg);
  }

  void send_default_transform()
  {
    geometry_msgs::msg::TransformStamped transform = test_utils::getDummyTransform();
    transform.header.stamp = node->now();
    br->sendTransform(transform);
    test_utils::spinWhile(node);
  }

  FakeNodeFixture * fnf;
  std::shared_ptr<Controller> node;

  AckermannControlCommand::SharedPtr cmd_msg;
  bool received_control_command = false;

  Trajectory::SharedPtr resampled_reference_trajectory;
  bool received_resampled_reference_trajectory = false;
  Trajectory::SharedPtr predicted_trajectory_in_frenet_coordinate;
  bool received_predicted_trajectory_in_frenet_coordinate = false;
  Trajectory::SharedPtr predicted_trajectory;
  bool received_predicted_trajectory = false;
  Trajectory::SharedPtr cgmres_predicted_trajectory_in_frenet_coordinate;
  bool received_cgmres_predicted_trajectory_in_frenet_coordinate = false;
  Trajectory::SharedPtr cgmres_predicted_trajectory;
  bool received_cgmres_predicted_trajectory = false;

  rclcpp::Publisher<Trajectory>::SharedPtr traj_pub;
  rclcpp::Publisher<VehicleOdometry>::SharedPtr odom_pub;
  rclcpp::Publisher<SteeringReport>::SharedPtr steer_pub;
  rclcpp::Publisher<AccelWithCovarianceStamped>::SharedPtr accel_pub;
  rclcpp::Publisher<OperationModeState>::SharedPtr operation_mode_pub;

  rclcpp::Subscription<AckermannControlCommand>::SharedPtr cmd_sub;
  rclcpp::Subscription<Trajectory>::SharedPtr predicted_traj_in_frenet_sub;
  rclcpp::Subscription<Trajectory>::SharedPtr predicted_traj_sub;
  rclcpp::Subscription<Trajectory>::SharedPtr cgmres_predicted_traj_in_frenet_sub;
  rclcpp::Subscription<Trajectory>::SharedPtr cgmres_predicted_traj_sub;
  rclcpp::Subscription<Trajectory>::SharedPtr resampled_ref_traj_sub;

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> br;
};

// TEST_F(FakeNodeFixture, no_input)
// {
//   const auto node_options = makeNodeOptions();
//   ControllerTester tester(this, node_options);

//   // No published data: expect a stopped command
//   test_utils::waitForMessage(
//     tester.node, this, tester.received_control_command, std::chrono::seconds{1LL}, false);
//   ASSERT_FALSE(tester.received_control_command);
// }

// TEST_F(FakeNodeFixture, empty_trajectory)
// {
//   const auto node_options = makeNodeOptions();
//   ControllerTester tester(this, node_options);

//   tester.send_default_transform();

//   // Empty trajectory: expect a stopped command
//   tester.publish_default_traj();
//   tester.publish_default_odom();
//   tester.publish_autonomous_operation_mode();
//   tester.publish_default_acc();
//   tester.publish_default_steer();

//   test_utils::waitForMessage(
//     tester.node, this, tester.received_control_command, std::chrono::seconds{1LL}, false);
//   ASSERT_FALSE(tester.received_control_command);
// }

// lateral
// TEST_F(FakeNodeFixture, straight_trajectory)
// {
//   const auto node_options = makeNodeOptions();
//   ControllerTester tester(this, node_options);

//   tester.send_default_transform();
//   tester.publish_odom_vx(1.0);
//   tester.publish_autonomous_operation_mode();
//   tester.publish_default_steer();
//   tester.publish_default_acc();

//   Trajectory traj_msg;
//   traj_msg.header.stamp = tester.node->now();
//   traj_msg.header.frame_id = "map";
//   traj_msg.points.push_back(test_utils::make_traj_point(-1.0, 0.0, 1.0f));
//   traj_msg.points.push_back(test_utils::make_traj_point(0.0, 0.0, 1.0f));
//   traj_msg.points.push_back(test_utils::make_traj_point(1.0, 0.0, 1.0f));
//   traj_msg.points.push_back(test_utils::make_traj_point(2.0, 0.0, 1.0f));
//   tester.traj_pub->publish(traj_msg);

//   test_utils::waitForMessage(tester.node, this, tester.received_control_command);
//   ASSERT_TRUE(tester.received_control_command);
//   // following conditions will pass even if the MPC solution does not converge
//   EXPECT_EQ(tester.cmd_msg->lateral.steering_tire_angle, 0.0f);
//   EXPECT_EQ(tester.cmd_msg->lateral.steering_tire_rotation_rate, 0.0f);
//   EXPECT_GT(tester.cmd_msg->longitudinal.speed, 0.0f);
//   EXPECT_GT(rclcpp::Time(tester.cmd_msg->stamp), rclcpp::Time(traj_msg.header.stamp));
// }

// TEST_F(FakeNodeFixture, right_turn)
// {
//   const auto node_options = makeNodeOptions();
//   ControllerTester tester(this, node_options);

//   tester.send_default_transform();
//   tester.publish_odom_vx(1.0);
//   tester.publish_autonomous_operation_mode();
//   tester.publish_default_steer();
//   tester.publish_default_acc();

//   // Right turning trajectory with a constant curvature of -0.5: expect right steering
//   Trajectory traj_msg;
//   std_msgs::msg::Header header;
//   header.stamp = tester.node->now();
//   header.frame_id = "map";
//   traj_msg = test_utils::generateCurvatureTrajectory(header, -0.05, 4.0, 1.0);
//   tester.traj_pub->publish(traj_msg);

//   test_utils::waitForMessage(tester.node, this, tester.received_control_command);
//   ASSERT_TRUE(tester.received_control_command);
//   test_utils::writeTrajectoriesToFiles(
//     traj_msg, *tester.resampled_reference_trajectory,
//     *tester.predicted_trajectory_in_frenet_coordinate, header.stamp);
//   // ASSERT_TRUE(tester.received_resampled_reference_trajectory);

//   std::cerr << "lat steer tire angle: " << tester.cmd_msg->lateral.steering_tire_angle <<
//   std::endl; std::cerr << "lat steer tire rotation rate: "
//             << tester.cmd_msg->lateral.steering_tire_rotation_rate << std::endl;
//   EXPECT_LT(tester.cmd_msg->lateral.steering_tire_angle, 0.0f);
//   EXPECT_LT(tester.cmd_msg->lateral.steering_tire_rotation_rate, 0.0f);
//   EXPECT_GT(rclcpp::Time(tester.cmd_msg->stamp), rclcpp::Time(traj_msg.header.stamp));
// }

TEST_F(FakeNodeFixture, right_turn_convergence)
{
  const auto node_options = makeNodeOptions();
  ControllerTester tester(this, node_options);
  Trajectory ref_trajectory;
  tester.send_default_transform();
  tester.publish_odom_vx(1.0);
  tester.publish_autonomous_operation_mode();
  tester.publish_default_steer();
  tester.publish_default_acc();

  auto publishTrajectory = [&tester, &ref_trajectory](double curvature_sign) {
    std_msgs::msg::Header header;
    header.stamp = tester.node->now();
    header.frame_id = "map";
    ref_trajectory = test_utils::generateCurvatureTrajectory(header, curvature_sign, 5.0, 1.0);
    tester.traj_pub->publish(ref_trajectory);
  };

  const double curvature_sign = -0.06;
  // std::random_device rd;
  // std::mt19937 gen(rd());
  // std::uniform_real_distribution<> dis(-0.2, 0.2);

  // double curvature_sign = dis(gen);
  constexpr size_t iter_num = 10;
  for (size_t i = 0; i < iter_num; i++) {
    // curvature_sign = curvature_sign - 0.005;
    publishTrajectory(curvature_sign);
    test_utils::waitForMessage(tester.node, this, tester.received_control_command);

    test_utils::writeTrajectoriesToFiles(
      ref_trajectory, *tester.resampled_reference_trajectory, *tester.predicted_trajectory,
      *tester.predicted_trajectory_in_frenet_coordinate,
      *tester.cgmres_predicted_trajectory_in_frenet_coordinate, *tester.cgmres_predicted_trajectory,
      tester.cmd_msg->stamp);
    ASSERT_TRUE(tester.received_control_command);
    std::cerr << "lat steer tire angle: " << tester.cmd_msg->lateral.steering_tire_angle
              << std::endl;
    std::cerr << "lat steer tire rotation rate: "
              << tester.cmd_msg->lateral.steering_tire_rotation_rate << std::endl;
    // EXPECT_LT(tester.cmd_msg->lateral.steering_tire_angle, 0.0f);
    // EXPECT_LT(tester.cmd_msg->lateral.steering_tire_rotation_rate, 0.0f);
    tester.received_control_command = false;
  }

  // ASSERT_TRUE(tester.received_resampled_reference_trajectory);

  // const auto save_directory = "/home/kyoichi-sugahara/workspace/log/reference_trajectory";
  // save_message_to_rosbag(
  //   save_directory, tester.resampled_reference_trajectory,
  //   "controller/debug/resampled_reference_trajectory");

  // std::cerr << "tester.cmd_msg's stamp: " << tester.cmd_msg->stamp.sec << "s "
  //           << tester.cmd_msg->stamp.nanosec << "ns" << std::endl;
  // std::cerr << "traj_msg.header.stamp: " << traj_msg.header.stamp.sec << "s "
  //           << traj_msg.header.stamp.nanosec << "ns" << std::endl;
  // EXPECT_GT(rclcpp::Time(tester.cmd_msg->stamp), rclcpp::Time(traj_msg.header.stamp));
}

// TEST_F(FakeNodeFixture, left_turn)
// {
//   const auto node_options = makeNodeOptions();
//   ControllerTester tester(this, node_options);

//   tester.send_default_transform();
//   tester.publish_odom_vx(1.0);
//   tester.publish_autonomous_operation_mode();
//   tester.publish_default_steer();
//   tester.publish_default_acc();

//   // Left turning trajectory: expect left steering
//   Trajectory traj_msg;
//   traj_msg.header.stamp = tester.node->now();
//   traj_msg.header.frame_id = "map";
//   traj_msg.points.push_back(test_utils::make_traj_point(-1.0, 1.0, 1.0f));
//   traj_msg.points.push_back(test_utils::make_traj_point(0.0, 0.0, 1.0f));
//   traj_msg.points.push_back(test_utils::make_traj_point(1.0, 1.0, 1.0f));
//   traj_msg.points.push_back(test_utils::make_traj_point(2.0, 2.0, 1.0f));
//   tester.traj_pub->publish(traj_msg);

//   test_utils::waitForMessage(tester.node, this, tester.received_control_command);
//   ASSERT_TRUE(tester.received_control_command);
//   std::cerr << "lat steer tire angle: " << tester.cmd_msg->lateral.steering_tire_angle <<
//   std::endl; std::cerr << "lat steer tire rotation rate: "
//             << tester.cmd_msg->lateral.steering_tire_rotation_rate << std::endl;
//   EXPECT_GT(tester.cmd_msg->lateral.steering_tire_angle, 0.0f);
//   EXPECT_GT(tester.cmd_msg->lateral.steering_tire_rotation_rate, 0.0f);
//   EXPECT_GT(rclcpp::Time(tester.cmd_msg->stamp), rclcpp::Time(traj_msg.header.stamp));
// }

// TEST_F(FakeNodeFixture, stopped)
// {
//   const auto node_options = makeNodeOptions();
//   ControllerTester tester(this, node_options);

//   tester.send_default_transform();
//   tester.publish_default_odom();
//   tester.publish_autonomous_operation_mode();
//   tester.publish_default_acc();

//   const double steering_tire_angle = -0.5;
//   tester.publish_steer_angle(steering_tire_angle);

//   // Straight trajectory: expect no steering
//   Trajectory traj_msg;
//   traj_msg.header.stamp = tester.node->now();
//   traj_msg.header.frame_id = "map";
//   traj_msg.points.push_back(test_utils::make_traj_point(-1.0, 0.0, 0.0f));
//   traj_msg.points.push_back(test_utils::make_traj_point(0.0, 0.0, 0.0f));
//   traj_msg.points.push_back(test_utils::make_traj_point(1.0, 0.0, 0.0f));
//   traj_msg.points.push_back(test_utils::make_traj_point(2.0, 0.0, 0.0f));
//   tester.traj_pub->publish(traj_msg);

//   test_utils::waitForMessage(tester.node, this, tester.received_control_command);
//   ASSERT_TRUE(tester.received_control_command);
//   EXPECT_EQ(tester.cmd_msg->lateral.steering_tire_angle, steering_tire_angle);
//   EXPECT_EQ(tester.cmd_msg->lateral.steering_tire_rotation_rate, 0.0f);
//   EXPECT_GT(rclcpp::Time(tester.cmd_msg->stamp), rclcpp::Time(traj_msg.header.stamp));
// }

// longitudinal
// TEST_F(FakeNodeFixture, longitudinal_keep_velocity)
// {
//   const auto node_options = makeNodeOptions();
//   ControllerTester tester(this, node_options);

//   tester.send_default_transform();
//   tester.publish_odom_vx(1.0);
//   tester.publish_autonomous_operation_mode();
//   tester.publish_default_steer();
//   tester.publish_default_acc();

//   // Publish non stopping trajectory
//   Trajectory traj_msg;
//   traj_msg.header.stamp = tester.node->now();
//   traj_msg.header.frame_id = "map";
//   traj_msg.points.push_back(test_utils::make_traj_point(0.0, 0.0, 1.0f));
//   traj_msg.points.push_back(test_utils::make_traj_point(50.0, 0.0, 1.0f));
//   traj_msg.points.push_back(test_utils::make_traj_point(100.0, 0.0, 1.0f));
//   tester.traj_pub->publish(traj_msg);

//   test_utils::waitForMessage(tester.node, this, tester.received_control_command);

//   ASSERT_TRUE(tester.received_control_command);
//   EXPECT_DOUBLE_EQ(tester.cmd_msg->longitudinal.speed, 1.0);
//   EXPECT_DOUBLE_EQ(tester.cmd_msg->longitudinal.acceleration, 0.0);

//   // Generate another control message
//   tester.traj_pub->publish(traj_msg);
//   test_utils::waitForMessage(tester.node, this, tester.received_control_command);
//   ASSERT_TRUE(tester.received_control_command);
//   EXPECT_DOUBLE_EQ(tester.cmd_msg->longitudinal.speed, 1.0);
//   EXPECT_DOUBLE_EQ(tester.cmd_msg->longitudinal.acceleration, 0.0);
// }

// TEST_F(FakeNodeFixture, longitudinal_slow_down)
// {
//   const auto node_options = makeNodeOptions();
//   ControllerTester tester(this, node_options);

//   tester.send_default_transform();
//   tester.publish_default_acc();
//   tester.publish_default_steer();

//   const double odom_vx = 1.0;
//   tester.publish_odom_vx(odom_vx);

//   tester.publish_autonomous_operation_mode();

//   // Publish non stopping trajectory
//   Trajectory traj;
//   traj.header.stamp = tester.node->now();
//   traj.header.frame_id = "map";
//   traj.points.push_back(test_utils::make_traj_point(0.0, 0.0, 0.5f));
//   traj.points.push_back(test_utils::make_traj_point(50.0, 0.0, 0.5f));
//   traj.points.push_back(test_utils::make_traj_point(100.0, 0.0, 0.5f));
//   tester.traj_pub->publish(traj);

//   test_utils::waitForMessage(tester.node, this, tester.received_control_command);

//   ASSERT_TRUE(tester.received_control_command);
//   EXPECT_LT(tester.cmd_msg->longitudinal.speed, static_cast<float>(odom_vx));
//   EXPECT_LT(tester.cmd_msg->longitudinal.acceleration, 0.0f);

//   // Generate another control message
//   tester.traj_pub->publish(traj);
//   test_utils::waitForMessage(tester.node, this, tester.received_control_command);
//   ASSERT_TRUE(tester.received_control_command);
//   EXPECT_LT(tester.cmd_msg->longitudinal.speed, static_cast<float>(odom_vx));
//   EXPECT_LT(tester.cmd_msg->longitudinal.acceleration, 0.0f);
// }

// TEST_F(FakeNodeFixture, longitudinal_accelerate)
// {
//   const auto node_options = makeNodeOptions();
//   ControllerTester tester(this, node_options);

//   tester.send_default_transform();
//   tester.publish_default_steer();
//   tester.publish_default_acc();

//   const double odom_vx = 0.5;
//   tester.publish_odom_vx(odom_vx);

//   tester.publish_autonomous_operation_mode();

//   // Publish non stopping trajectory
//   Trajectory traj;
//   traj.header.stamp = tester.node->now();
//   traj.header.frame_id = "map";
//   traj.points.push_back(test_utils::make_traj_point(0.0, 0.0, 1.0f));
//   traj.points.push_back(test_utils::make_traj_point(50.0, 0.0, 1.0f));
//   traj.points.push_back(test_utils::make_traj_point(100.0, 0.0, 1.0f));
//   tester.traj_pub->publish(traj);

//   test_utils::waitForMessage(tester.node, this, tester.received_control_command);

//   ASSERT_TRUE(tester.received_control_command);
//   EXPECT_GT(tester.cmd_msg->longitudinal.speed, static_cast<float>(odom_vx));
//   EXPECT_GT(tester.cmd_msg->longitudinal.acceleration, 0.0f);

//   // Generate another control message
//   tester.traj_pub->publish(traj);
//   test_utils::waitForMessage(tester.node, this, tester.received_control_command);
//   ASSERT_TRUE(tester.received_control_command);
//   EXPECT_GT(tester.cmd_msg->longitudinal.speed, static_cast<float>(odom_vx));
//   EXPECT_GT(tester.cmd_msg->longitudinal.acceleration, 0.0f);
// }

// TEST_F(FakeNodeFixture, longitudinal_stopped)
// {
//   const auto node_options = makeNodeOptions();
//   ControllerTester tester(this, node_options);

//   tester.send_default_transform();
//   tester.publish_default_odom();
//   tester.publish_autonomous_operation_mode();
//   tester.publish_default_steer();
//   tester.publish_default_acc();

//   // Publish stopping trajectory
//   Trajectory traj;
//   traj.header.stamp = tester.node->now();
//   traj.header.frame_id = "map";
//   traj.points.push_back(test_utils::make_traj_point(0.0, 0.0, 0.0f));
//   traj.points.push_back(test_utils::make_traj_point(50.0, 0.0, 0.0f));
//   traj.points.push_back(test_utils::make_traj_point(100.0, 0.0, 0.0f));
//   tester.traj_pub->publish(traj);

//   test_utils::waitForMessage(tester.node, this, tester.received_control_command);

//   ASSERT_TRUE(tester.received_control_command);
//   EXPECT_DOUBLE_EQ(tester.cmd_msg->longitudinal.speed, 0.0f);
//   EXPECT_LT(
//     tester.cmd_msg->longitudinal.acceleration,
//     0.0f);  // when stopped negative acceleration to brake
// }

// TEST_F(FakeNodeFixture, longitudinal_reverse)
// {
//   const auto node_options = makeNodeOptions();
//   ControllerTester tester(this, node_options);

//   tester.send_default_transform();

//   tester.publish_default_odom();
//   tester.publish_autonomous_operation_mode();
//   tester.publish_default_steer();
//   tester.publish_default_acc();

//   // Publish reverse
//   Trajectory traj;
//   traj.header.stamp = tester.node->now();
//   traj.header.frame_id = "map";
//   traj.points.push_back(test_utils::make_traj_point(0.0, 0.0, -1.0f));
//   traj.points.push_back(test_utils::make_traj_point(50.0, 0.0, -1.0f));
//   traj.points.push_back(test_utils::make_traj_point(100.0, 0.0, -1.0f));
//   tester.traj_pub->publish(traj);

//   test_utils::waitForMessage(tester.node, this, tester.received_control_command);

//   ASSERT_TRUE(tester.received_control_command);
//   EXPECT_LT(tester.cmd_msg->longitudinal.speed, 0.0f);
//   EXPECT_GT(tester.cmd_msg->longitudinal.acceleration, 0.0f);
// }

// TEST_F(FakeNodeFixture, longitudinal_emergency)
// {
//   const auto node_options = makeNodeOptions();
//   ControllerTester tester(this, node_options);

//   tester.send_default_transform();
//   tester.publish_default_odom();
//   tester.publish_autonomous_operation_mode();
//   tester.publish_default_steer();
//   tester.publish_default_acc();

//   // Publish trajectory starting away from the current ego pose
//   Trajectory traj;
//   traj.header.stamp = tester.node->now();
//   traj.header.frame_id = "map";
//   traj.points.push_back(test_utils::make_traj_point(10.0, 0.0, 1.0f));
//   traj.points.push_back(test_utils::make_traj_point(50.0, 0.0, 1.0f));
//   traj.points.push_back(test_utils::make_traj_point(100.0, 0.0, 1.0f));
//   tester.traj_pub->publish(traj);

//   test_utils::waitForMessage(tester.node, this, tester.received_control_command);

//   ASSERT_TRUE(tester.received_control_command);
//   // Emergencies (e.g., far from trajectory) produces braking command (0 vel, negative accel)
//   EXPECT_DOUBLE_EQ(tester.cmd_msg->longitudinal.speed, 0.0f);
//   EXPECT_LT(tester.cmd_msg->longitudinal.acceleration, 0.0f);
// }

// TEST_F(FakeNodeFixture, longitudinal_not_check_steer_converged)
// {
//   const auto node_options = makeNodeOptions();
//   ControllerTester tester(this, node_options);

//   tester.send_default_transform();
//   tester.publish_default_odom();
//   tester.publish_autonomous_operation_mode();
//   tester.publish_default_acc();

//   // steering_tire_angle has to be larger than the threshold to check convergence.
//   const double steering_tire_angle = -0.5;
//   tester.publish_steer_angle(steering_tire_angle);

//   // Publish trajectory starting away from the current ego pose
//   Trajectory traj;
//   traj.header.stamp = tester.node->now();
//   traj.header.frame_id = "map";
//   traj.points.push_back(test_utils::make_traj_point(0.0, 0.0, 1.0f));
//   traj.points.push_back(test_utils::make_traj_point(50.0, 0.0, 1.0f));
//   traj.points.push_back(test_utils::make_traj_point(100.0, 0.0, 1.0f));
//   tester.traj_pub->publish(traj);

//   test_utils::waitForMessage(tester.node, this, tester.received_control_command);

//   ASSERT_TRUE(tester.received_control_command);
//   // Not keep stopped state when the lateral control is not converged.
//   EXPECT_DOUBLE_EQ(tester.cmd_msg->longitudinal.speed, 1.0f);
// }

// TEST_F(FakeNodeFixture, longitudinal_check_steer_converged)
// {
//   // set enable_keep_stopped_until_steer_convergence true
//   const auto node_options = makeNodeOptions(true);
//   ControllerTester tester(this, node_options);

//   tester.send_default_transform();
//   tester.publish_default_odom();
//   tester.publish_autonomous_operation_mode();
//   tester.publish_default_acc();

//   // steering_tire_angle has to be larger than the threshold to check convergence.
//   const double steering_tire_angle = -0.5;
//   tester.publish_steer_angle(steering_tire_angle);

//   // Publish trajectory starting away from the current ego pose
//   Trajectory traj;
//   traj.header.stamp = tester.node->now();
//   traj.header.frame_id = "map";
//   traj.points.push_back(test_utils::make_traj_point(0.0, 0.0, 1.0f));
//   traj.points.push_back(test_utils::make_traj_point(50.0, 0.0, 1.0f));
//   traj.points.push_back(test_utils::make_traj_point(100.0, 0.0, 1.0f));
//   tester.traj_pub->publish(traj);

//   test_utils::waitForMessage(tester.node, this, tester.received_control_command);

//   ASSERT_TRUE(tester.received_control_command);
//   // Keep stopped state when the lateral control is not converged.
//   EXPECT_DOUBLE_EQ(tester.cmd_msg->longitudinal.speed, 0.0f);
// }
