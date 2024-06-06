// Copyright 2023 TIER IV, Inc.
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

#ifndef MANAGER_HPP_
#define MANAGER_HPP_

#include "scene_dynamic_obstacle_stop.hpp"

#include <behavior_velocity_planner_common/plugin_interface.hpp>
#include <behavior_velocity_planner_common/plugin_wrapper.hpp>
#include <behavior_velocity_planner_common/scene_module_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tier4_planning_msgs/msg/path_with_lane_id.hpp>

#include <memory>
#include <string>
#include <vector>

namespace behavior_velocity_planner
{
class DynamicObstacleStopModuleManager : public SceneModuleManagerInterface
{
public:
  explicit DynamicObstacleStopModuleManager(rclcpp::Node & node);

  const char * getModuleName() override { return "dynamic_obstacle_stop"; }

private:
  using PlannerParam = dynamic_obstacle_stop::PlannerParam;

  PlannerParam planner_param_;
  int64_t module_id_;

  void launchNewModules(const tier4_planning_msgs::msg::PathWithLaneId & path) override;

  std::function<bool(const std::shared_ptr<SceneModuleInterface> &)> getModuleExpiredFunction(
    const tier4_planning_msgs::msg::PathWithLaneId & path) override;
};

class DynamicObstacleStopModulePlugin : public PluginWrapper<DynamicObstacleStopModuleManager>
{
};

}  // namespace behavior_velocity_planner

#endif  // MANAGER_HPP_