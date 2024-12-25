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
#pragma once

#include <autoware/behavior_path_start_planner_module/start_planner_module.hpp>
#include <autoware/lane_departure_checker/lane_departure_checker.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>

namespace autoware::behavior_path_planner::testing
{

class StartPlannerTestHelper
{
public:
  static rclcpp::NodeOptions make_node_options();

  static std::shared_ptr<LaneDepartureChecker> make_lane_departure_checker(rclcpp::Node & node);

  static std::shared_ptr<const PlannerData> make_planner_data(
    [[maybe_unused]] rclcpp::Node & node,
    [[maybe_unused]] const geometry_msgs::msg::Pose & start_pose,
    [[maybe_unused]] const int route_start_lane_id, [[maybe_unused]] const int route_goal_lane_id);
};

}  // namespace autoware::behavior_path_planner::testing
