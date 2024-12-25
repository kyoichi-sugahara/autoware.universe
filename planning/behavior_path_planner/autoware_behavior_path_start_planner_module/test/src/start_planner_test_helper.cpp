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
// pull_out_test_utils.cpp
#include "start_planner_test_helper.hpp"

#include <autoware_test_utils/autoware_test_utils.hpp>

#include <memory>

namespace autoware::behavior_path_planner::testing
{
using autoware::test_utils::get_absolute_path_to_config;

rclcpp::NodeOptions StartPlannerTestHelper::make_node_options()
{
  // Load common configuration files
  auto node_options = rclcpp::NodeOptions{};

  const auto common_param_path =
    get_absolute_path_to_config("autoware_test_utils", "test_common.param.yaml");
  const auto nearest_search_param_path =
    get_absolute_path_to_config("autoware_test_utils", "test_nearest_search.param.yaml");
  const auto vehicle_info_param_path =
    get_absolute_path_to_config("autoware_test_utils", "test_vehicle_info.param.yaml");
  const auto behavior_path_planner_param_path = get_absolute_path_to_config(
    "autoware_behavior_path_planner", "behavior_path_planner.param.yaml");
  const auto drivable_area_expansion_param_path = get_absolute_path_to_config(
    "autoware_behavior_path_planner", "drivable_area_expansion.param.yaml");
  const auto scene_module_manager_param_path = get_absolute_path_to_config(
    "autoware_behavior_path_planner", "scene_module_manager.param.yaml");
  const auto start_planner_param_path = get_absolute_path_to_config(
    "autoware_behavior_path_start_planner_module", "start_planner.param.yaml");

  autoware::test_utils::updateNodeOptions(
    node_options, {common_param_path, nearest_search_param_path, vehicle_info_param_path,
                   behavior_path_planner_param_path, drivable_area_expansion_param_path,
                   scene_module_manager_param_path, start_planner_param_path});

  return node_options;
}

std::shared_ptr<LaneDepartureChecker> StartPlannerTestHelper::make_lane_departure_checker(
  rclcpp::Node & node)
{
  const auto vehicle_info = autoware::vehicle_info_utils::VehicleInfoUtils(node).getVehicleInfo();
  auto lane_departure_checker = std::make_shared<LaneDepartureChecker>();
  lane_departure_checker->setVehicleInfo(vehicle_info);

  autoware::lane_departure_checker::Param lane_departure_checker_params{};
  lane_departure_checker->setParam(lane_departure_checker_params);

  return lane_departure_checker;
}

std::shared_ptr<const PlannerData> StartPlannerTestHelper::make_planner_data(
  rclcpp::Node & node, const geometry_msgs::msg::Pose & start_pose, const int route_start_lane_id,
  const int route_goal_lane_id)
{
  auto planner_data = std::make_shared<PlannerData>();
  planner_data->init_parameters(node);

  // Load a sample lanelet map and create a route handler
  const auto shoulder_map_path = autoware::test_utils::get_absolute_path_to_lanelet_map(
    "autoware_test_utils", "road_shoulder/lanelet2_map.osm");
  const auto map_bin_msg = autoware::test_utils::make_map_bin_msg(shoulder_map_path, 0.5);
  auto route_handler = std::make_shared<autoware::route_handler::RouteHandler>(map_bin_msg);

  // Set up current odometry at start pose
  auto odometry = std::make_shared<nav_msgs::msg::Odometry>();
  odometry->pose.pose = start_pose;
  odometry->header.frame_id = "map";
  planner_data->self_odometry = odometry;

  // Setup route
  const auto route = makeBehaviorRouteFromLaneId(
    route_start_lane_id, route_goal_lane_id, "autoware_test_utils",
    "road_shoulder/lanelet2_map.osm");
  route_handler->setRoute(route);

  // Update planner data with the route handler
  planner_data->route_handler = route_handler;

  return planner_data;
}

}  // namespace autoware::behavior_path_planner::testing
