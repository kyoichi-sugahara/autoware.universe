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

#include "autoware/lateral_optimal_controller/parameters.hpp"

#include <autoware/universe_utils/ros/parameter.hpp>
#include <rclcpp/node.hpp>

#include <string>

namespace autoware::motion::control::lateral_optimal_controller
{
using autoware::universe_utils::getOrDeclareParameter;

TrajectoryFilteringParam TrajectoryFilteringParam::init(rclcpp::Node & node)
{
  TrajectoryFilteringParam p;
  p.traj_resample_dist = getOrDeclareParameter<double>(node, "traj_resample_dist");
  p.extend_trajectory_for_end_yaw_control =
    getOrDeclareParameter<bool>(node, "extend_trajectory_for_end_yaw_control");
  p.enable_path_smoothing = getOrDeclareParameter<bool>(node, "enable_path_smoothing");
  p.path_filter_moving_ave_num = getOrDeclareParameter<int>(node, "path_filter_moving_ave_num");
  p.curvature_smoothing_num_traj = getOrDeclareParameter<int>(node, "curvature_smoothing_num_traj");
  p.curvature_smoothing_num_ref_steer =
    getOrDeclareParameter<int>(node, "curvature_smoothing_num_ref_steer");

  return p;
}

}  // namespace autoware::motion::control::lateral_optimal_controller
