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
#include <autoware_test_utils/autoware_test_utils.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

using autoware::behavior_path_planner::GeometricPullOut;
using autoware::behavior_path_planner::StartPlannerParameters;
using autoware::lane_departure_checker::LaneDepartureChecker;
using autoware::test_utils::get_absolute_path_to_config;

namespace autoware::behavior_path_planner
{

class TestGeometricPullOut : public ::testing::Test
{
public:
  // double calcPullOutLongitudinalDistance(
  //   const double lon_acc, const double shift_time, const double shift_length,
  //   const double max_curvature, const double min_distance)
  // {
  //   return geometric_pull_out->calcPullOutLongitudinalDistance(
  //     lon_acc, shift_time, shift_length, max_curvature, min_distance);
  // }

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

// TEST_F(GeometricPullOut, calcPullOutLongitudinalDistance)
// {
//   const double lon_acc = 1.0;
//   const double shift_time = 1.0;
//   const double shift_length = 1.0;
//   const double max_curvature = 0.1;
//   const double min_distance = 1.0;
//   calcPullOutLongitudinalDistance(lon_acc, shift_time, shift_length, max_curvature,
//   min_distance);
// }
}  // namespace autoware::behavior_path_planner
