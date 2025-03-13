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

#include "autoware/mpc_lateral_controller/parameters.hpp"

#include <autoware_utils/ros/parameter.hpp>
#include <rclcpp/node.hpp>

#include <string>

namespace autoware::motion::control::mpc_lateral_controller
{

using autoware_utils::get_or_declare_parameter;

TrajectoryFilteringParam TrajectoryFilteringParam::init(rclcpp::Node & node)
{
  TrajectoryFilteringParam p;
  p.traj_resample_dist = get_or_declare_parameter<double>(node, "traj_resample_dist");
  p.extend_trajectory_for_end_yaw_control =
    get_or_declare_parameter<bool>(node, "extend_trajectory_for_end_yaw_control");
  p.enable_path_smoothing = get_or_declare_parameter<bool>(node, "enable_path_smoothing");
  p.path_filter_moving_ave_num = get_or_declare_parameter<int>(node, "path_filter_moving_ave_num");
  p.curvature_smoothing_num_traj =
    get_or_declare_parameter<int>(node, "curvature_smoothing_num_traj");
  p.curvature_smoothing_num_ref_steer =
    get_or_declare_parameter<int>(node, "curvature_smoothing_num_ref_steer");

  return p;
}

}  // namespace autoware::motion::control::mpc_lateral_controller
