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

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware/behavior_path_start_planner_module/geometric_pull_out.hpp>
#include <autoware/behavior_path_start_planner_module/start_planner_module.hpp>
#include <autoware/behavior_path_start_planner_module/util.hpp>
#include <autoware/lane_departure_checker/lane_departure_checker.hpp>
#include <autoware/route_handler/route_handler.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_planning_test_manager/autoware_planning_test_manager_utils.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>

#include <gtest/gtest.h>

#include <iostream>
#include <memory>
#include <string>
#include <vector>

using autoware::behavior_path_planner::GeometricPullOut;
using autoware::behavior_path_planner::StartPlannerParameters;
using autoware::lane_departure_checker::LaneDepartureChecker;
using autoware::test_utils::get_absolute_path_to_config;
using autoware_planning_msgs::msg::LaneletRoute;
using RouteSections = std::vector<autoware_planning_msgs::msg::LaneletSegment>;
using autoware_planning_test_manager::utils::makeBehaviorRouteFromLaneId;

namespace autoware::behavior_path_planner
{

class TestGeometricPullOut : public ::testing::Test
{
public:
  std::optional<PullOutPath> plan(
    const Pose & start_pose, const Pose & goal_pose, PlannerDebugData & planner_debug_data)
  {
    return geometric_pull_out->plan(start_pose, goal_pose, planner_debug_data);
  }

  std::shared_ptr<autoware::route_handler::RouteHandler> route_handler;
  autoware::vehicle_info_utils::VehicleInfo vehicle_info;
  std::shared_ptr<GeometricPullOut> geometric_pull_out;
  std::shared_ptr<LaneDepartureChecker> lane_departure_checker;

protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    init_param();
    init_module();
  }
  void TearDown() override { rclcpp::shutdown(); }

private:
  void init_param()
  {
    auto node_options = get_node_options();
    auto node = rclcpp::Node::make_shared("geometric_pull_out", node_options);

    vehicle_info = autoware::vehicle_info_utils::VehicleInfoUtils(*node).getVehicleInfo();

    // lanelet map
    const auto shoulder_map_path = autoware::test_utils::get_absolute_path_to_lanelet_map(
      "autoware_test_utils", "road_shoulder/lanelet2_map.osm");
    const auto map_bin_msg = autoware::test_utils::make_map_bin_msg(shoulder_map_path, 0.5);
    // load map
    route_handler = std::make_shared<autoware::route_handler::RouteHandler>(map_bin_msg);

    lane_departure_checker = std::make_shared<LaneDepartureChecker>();
    lane_departure_checker->setVehicleInfo(vehicle_info);

    auto parameters = std::make_shared<StartPlannerParameters>();
    auto time_keeper = std::make_shared<autoware::universe_utils::TimeKeeper>();

    autoware::lane_departure_checker::Param lane_departure_checker_params{};
    lane_departure_checker_params.footprint_extra_margin =
      parameters->lane_departure_check_expansion_margin;

    lane_departure_checker->setParam(lane_departure_checker_params);
    parameters->parallel_parking_parameters.pull_out_max_steer_angle = 0.35;
    parameters->parallel_parking_parameters.pull_out_arc_path_interval = 1.0;
    parameters->parallel_parking_parameters.center_line_path_interval = 1.0;
    parameters->th_moving_object_velocity = 1.0;

    geometric_pull_out =
      std::make_shared<GeometricPullOut>(*node, *parameters, lane_departure_checker, time_keeper);
  }

  rclcpp::NodeOptions get_node_options() const
  {
    auto node_options = rclcpp::NodeOptions{};

    const auto common_param =
      get_absolute_path_to_config("autoware_test_utils", "test_common.param.yaml");
    const auto nearest_search_param =
      get_absolute_path_to_config("autoware_test_utils", "test_nearest_search.param.yaml");
    const auto vehicle_info_param =
      get_absolute_path_to_config("autoware_test_utils", "test_vehicle_info.param.yaml");

    std::string bpp_dir{"autoware_behavior_path_planner"};
    const auto bpp_param = get_absolute_path_to_config(bpp_dir, "behavior_path_planner.param.yaml");
    const auto drivable_area_expansion_param =
      get_absolute_path_to_config(bpp_dir, "drivable_area_expansion.param.yaml");
    const auto scene_module_manager_param =
      get_absolute_path_to_config(bpp_dir, "scene_module_manager.param.yaml");

    const auto start_planner_param = get_absolute_path_to_config(
      "autoware_behavior_path_start_planner_module", "start_planner.param.yaml");

    autoware::test_utils::updateNodeOptions(
      node_options,
      {common_param, nearest_search_param, vehicle_info_param, bpp_param,
       drivable_area_expansion_param, scene_module_manager_param, start_planner_param});
    return node_options;
  }

  void init_module()
  {
    // geometric_pull_out = std::make_shared<GeometricPullOut>(
    //   *route_handler->getNode(), route_handler->getStartPlannerParameters(),
    //   route_handler->getLaneDepartureChecker(), route_handler->getTimeKeeper());
  }
};

TEST_F(TestGeometricPullOut, NormalPullOutPlan)
{
  PlannerData planner_data;

  // Set odometry with start pose
  auto odometry = std::make_shared<nav_msgs::msg::Odometry>();
  // const geometry_msgs::msg::Pose start_pose =
  //   geometry_msgs::build<geometry_msgs::msg::Pose>()
  //     .position(geometry_msgs::build<geometry_msgs::msg::Point>().x(362.225).y(378.580).z(100.000))
  //     .orientation(
  //       geometry_msgs::build<geometry_msgs::msg::Quaternion>().x(0.0).y(0.0).z(0.709157).w(
  //         0.705051));

  // const geometry_msgs::msg::Pose goal_pose =
  //   geometry_msgs::build<geometry_msgs::msg::Pose>()
  //     .position(geometry_msgs::build<geometry_msgs::msg::Point>().x(365.575).y(450.062).z(100.000))
  //     .orientation(
  //       geometry_msgs::build<geometry_msgs::msg::Quaternion>().x(0.0).y(0.0).z(0.706060).w(
  //         0.708152));
  const geometry_msgs::msg::Pose start_pose =
    geometry_msgs::build<geometry_msgs::msg::Pose>()
      .position(geometry_msgs::build<geometry_msgs::msg::Point>().x(362.181).y(362.164).z(100.000))
      .orientation(
        geometry_msgs::build<geometry_msgs::msg::Quaternion>().x(0.0).y(0.0).z(0.709650).w(
          0.704554));

  const geometry_msgs::msg::Pose goal_pose =
    geometry_msgs::build<geometry_msgs::msg::Pose>()
      .position(geometry_msgs::build<geometry_msgs::msg::Point>().x(365.658).y(507.253).z(100.000))
      .orientation(
        geometry_msgs::build<geometry_msgs::msg::Quaternion>().x(0.0).y(0.0).z(0.705897).w(
          0.708314));

  // Find path lanelets between start and goal
  LaneletRoute route;
  LaneletRoute route_msg;
  RouteSections route_sections;
  lanelet::ConstLanelets all_route_lanelets;
  lanelet::ConstLanelets path_lanelets;

  route_handler->planPathLaneletsBetweenCheckpoints(start_pose, goal_pose, &path_lanelets);
  for (const auto & lane : path_lanelets) {
    all_route_lanelets.push_back(lane);
  }
  route_handler->setRouteLanelets(path_lanelets);
  const auto local_route_sections = route_handler->createMapSegments(path_lanelets);

  route_sections =
    autoware::test_utils::combineConsecutiveRouteSections(route_sections, local_route_sections);
  for (const auto & route_section : route_sections) {
    for (const auto & primitive : route_section.primitives) {
      std::cerr << "primitive: " << primitive.id << std::endl;
    }
    std::cerr << "preferred_primitive id : " << route_section.preferred_primitive.id << std::endl;
  }
  route_handler->setRouteLanelets(all_route_lanelets);
  route.segments = route_sections;

  route.allow_modification = false;
  route_handler->setRoute(route);

  // Set current pose as odometry
  odometry->pose.pose = start_pose;
  odometry->header.frame_id = "map";
  // odometry->header.stamp = route_handler->getNode()->now();
  planner_data.self_odometry = odometry;

  // Set route handler
  planner_data.route_handler = route_handler;

  // Set parameters
  planner_data.parameters.backward_path_length = 5.0;   // Example value
  planner_data.parameters.forward_path_length = 100.0;  // Example value
  planner_data.parameters.wheel_base = 2.79;
  planner_data.parameters.wheel_tread = 1.64;
  planner_data.parameters.front_overhang = 1.0;
  planner_data.parameters.left_over_hang = 0.128;

  // Update planner with new data
  geometric_pull_out->setPlannerData(std::make_shared<PlannerData>(planner_data));

  PlannerDebugData debug_data;
  auto result = plan(start_pose, goal_pose, debug_data);

  ASSERT_TRUE(result.has_value()) << "Failed to generate pull out path";
  if (result) {
    EXPECT_FALSE(result->partial_paths.empty()) << "Generated path is empty";
    EXPECT_EQ(debug_data.conditions_evaluation.back(), "success");
  }
}

// TEST_F(TestGeometricPullOut, NoValidPathPlan)
// {
//   const Pose start_pose{
//     {0.0, 20.0, 0.0},  // Start pose too far from road
//     {0.0, 0.0, 0.0, 1.0}
//   };
//   const Pose goal_pose{
//     {10.0, 0.0, 0.0},
//     {0.0, 0.0, 0.0, 1.0}
//   };

//   PlannerDebugData debug_data;

//   auto result = plan(start_pose, goal_pose, debug_data);

//   EXPECT_FALSE(result.has_value());
//   EXPECT_EQ(debug_data.conditions_evaluation.back(), "no path found");
// }

// TEST_F(TestGeometricPullOut, PullOutWithDifferentParameters)
// {
//   // Modify planner parameters
//   auto parameters = std::make_shared<StartPlannerParameters>();
//   parameters->backward_path_length = 10.0;
//   auto time_keeper = std::make_shared<autoware::universe_utils::TimeKeeper>();

//   geometric_pull_out =
//     std::make_shared<GeometricPullOut>(*route_handler->getNode(), *parameters,
//     lane_departure_checker, time_keeper);

//   const Pose start_pose{
//     {0.0, 5.0, 0.0},
//     {0.0, 0.0, 0.0, 1.0}
//   };
//   const Pose goal_pose{
//     {10.0, 0.0, 0.0},
//     {0.0, 0.0, 0.0, 1.0}
//   };

//   PlannerDebugData debug_data;

//   auto result = plan(start_pose, goal_pose, debug_data);

//   ASSERT_TRUE(result.has_value());
//   if (result) {
//     EXPECT_FALSE(result->partial_paths.empty());
//     EXPECT_FALSE(result->pairs_terminal_velocity_and_accel.empty());
//   }
// }

// TEST_F(TestGeometricPullOut, PathEndPointsMatchInputPoses)
// {
//   const Pose start_pose{
//     {0.0, 5.0, 0.0},
//     {0.0, 0.0, 0.0, 1.0}
//   };
//   const Pose goal_pose{
//     {10.0, 0.0, 0.0},
//     {0.0, 0.0, 0.0, 1.0}
//   };

//   PlannerDebugData debug_data;

//   auto result = plan(start_pose, goal_pose, debug_data);

//   ASSERT_TRUE(result.has_value());
//   if (result) {
//     // Check if start and end poses match input
//     const auto & output_start = result->start_pose;
//     const auto & output_end = result->end_pose;

//     EXPECT_NEAR(output_start.position.x, start_pose.position.x, 0.1);
//     EXPECT_NEAR(output_start.position.y, start_pose.position.y, 0.1);
//     EXPECT_NEAR(output_end.position.x, goal_pose.position.x, 0.1);
//     EXPECT_NEAR(output_end.position.y, goal_pose.position.y, 0.1);
//   }
// }
}  // namespace autoware::behavior_path_planner
