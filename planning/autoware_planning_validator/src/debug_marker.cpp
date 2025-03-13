// Copyright 2022 Tier IV, Inc.
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

#include "autoware/planning_validator/debug_marker.hpp"

#include <autoware/motion_utils/marker/marker_helper.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/ros/marker_helper.hpp>

#include <memory>
#include <string>
#include <vector>

using autoware_utils::Point2d;
using visualization_msgs::msg::Marker;

PlanningValidatorDebugMarkerPublisher::PlanningValidatorDebugMarkerPublisher(rclcpp::Node * node)
: node_(node)
{
  debug_viz_pub_ =
    node_->create_publisher<visualization_msgs::msg::MarkerArray>("~/debug/marker", 1);

  virtual_wall_pub_ =
    node_->create_publisher<visualization_msgs::msg::MarkerArray>("~/virtual_wall", 1);
}

void PlanningValidatorDebugMarkerPublisher::clearMarkers()
{
  marker_array_.markers.clear();
  marker_array_virtual_wall_.markers.clear();
}

void PlanningValidatorDebugMarkerPublisher::pushPoseMarker(
  const autoware_planning_msgs::msg::TrajectoryPoint & p, const std::string & ns, int id)
{
  pushPoseMarker(p.pose, ns, id);
}

void PlanningValidatorDebugMarkerPublisher::pushPoseMarker(
  const geometry_msgs::msg::Pose & pose, const std::string & ns, int id)
{
  using autoware_utils::create_marker_color;

  // append arrow marker
  std_msgs::msg::ColorRGBA color;
  if (id == 0)  // Red
  {
    color = create_marker_color(1.0, 0.0, 0.0, 0.999);
  }
  if (id == 1)  // Green
  {
    color = create_marker_color(0.0, 1.0, 0.0, 0.999);
  }
  if (id == 2)  // Blue
  {
    color = create_marker_color(0.0, 0.0, 1.0, 0.999);
  }
  Marker marker = autoware_utils::create_default_marker(
    "map", node_->get_clock()->now(), ns, getMarkerId(ns), Marker::ARROW,
    autoware_utils::create_marker_scale(0.2, 0.1, 0.3), color);
  marker.lifetime = rclcpp::Duration::from_seconds(0.2);
  marker.pose = pose;

  marker_array_.markers.push_back(marker);
}

void PlanningValidatorDebugMarkerPublisher::pushFootprintMarker(
  const geometry_msgs::msg::Pose & pose,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, const std::string & ns)
{
  Marker marker = autoware_utils::create_default_marker(
    "map", node_->get_clock()->now(), ns, getMarkerId(ns), Marker::LINE_STRIP,
    autoware_utils::create_marker_scale(0.1, 0.1, 0.1),
    autoware_utils::create_marker_color(1.0, 0.0, 0.0, 0.999));
  const double half_width = vehicle_info.vehicle_width_m / 2.0;
  const double base_to_front = vehicle_info.vehicle_length_m - vehicle_info.rear_overhang_m;
  const double base_to_rear = vehicle_info.rear_overhang_m;

  marker.points.push_back(
    autoware_utils::calc_offset_pose(pose, base_to_front, -half_width, 0.0).position);
  marker.points.push_back(
    autoware_utils::calc_offset_pose(pose, base_to_front, half_width, 0.0).position);
  marker.points.push_back(
    autoware_utils::calc_offset_pose(pose, -base_to_rear, half_width, 0.0).position);
  marker.points.push_back(
    autoware_utils::calc_offset_pose(pose, -base_to_rear, -half_width, 0.0).position);
  marker.points.push_back(marker.points.front());
  marker.lifetime = rclcpp::Duration::from_seconds(0.2);
  marker_array_.markers.push_back(marker);
}

void PlanningValidatorDebugMarkerPublisher::pushBoxMarker(
  const boost::geometry::model::box<Point2d> & polygon_box, const std::string & ns)
{
  Marker marker = autoware_utils::create_default_marker(
    "map", node_->get_clock()->now(), ns, getMarkerId(ns), Marker::LINE_STRIP,
    autoware_utils::create_marker_scale(0.1, 0.1, 0.1),
    autoware_utils::create_marker_color(0.0, 1.0, 0.0, 0.999));

  // Box has min_corner() and max_corner() methods to get the corners
  const auto & min_corner = polygon_box.min_corner();
  const auto & max_corner = polygon_box.max_corner();

  // Reserve space for 5 points (4 corners + closing point)
  marker.points.reserve(5);

  // Create the 4 corners of the box
  marker.points.push_back(autoware_utils::create_point(min_corner.x(), min_corner.y(), 0.0));
  marker.points.push_back(autoware_utils::create_point(max_corner.x(), min_corner.y(), 0.0));
  marker.points.push_back(autoware_utils::create_point(max_corner.x(), max_corner.y(), 0.0));
  marker.points.push_back(autoware_utils::create_point(min_corner.x(), max_corner.y(), 0.0));
  // Close the box by adding the first point again
  marker.points.push_back(autoware_utils::create_point(min_corner.x(), min_corner.y(), 0.0));

  marker_array_.markers.push_back(marker);
}

void PlanningValidatorDebugMarkerPublisher::pushWarningMsg(
  const geometry_msgs::msg::Pose & pose, const std::string & msg)
{
  visualization_msgs::msg::Marker marker = autoware_utils::create_default_marker(
    "map", node_->get_clock()->now(), "warning_msg", 0, Marker::TEXT_VIEW_FACING,
    autoware_utils::create_marker_scale(0.0, 0.0, 1.0),
    autoware_utils::create_marker_color(1.0, 0.1, 0.1, 0.999));
  marker.lifetime = rclcpp::Duration::from_seconds(0.2);
  marker.pose = pose;
  marker.text = msg;
  marker_array_virtual_wall_.markers.push_back(marker);
}

void PlanningValidatorDebugMarkerPublisher::pushVirtualWall(const geometry_msgs::msg::Pose & pose)
{
  const auto now = node_->get_clock()->now();
  const auto stop_wall_marker = autoware::motion_utils::createStopVirtualWallMarker(
    pose, "autoware_planning_validator", now, 0);
  autoware_utils::append_marker_array(stop_wall_marker, &marker_array_virtual_wall_, now);
}

void PlanningValidatorDebugMarkerPublisher::publish()
{
  debug_viz_pub_->publish(marker_array_);
  virtual_wall_pub_->publish(marker_array_virtual_wall_);
}
