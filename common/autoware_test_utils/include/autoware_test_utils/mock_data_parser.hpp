// Copyright 2024 TIER IV
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

#ifndef AUTOWARE_TEST_UTILS__MOCK_DATA_PARSER_HPP_
#define AUTOWARE_TEST_UTILS__MOCK_DATA_PARSER_HPP_

#include <autoware_planning_msgs/msg/lanelet_primitive.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/lanelet_segment.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tier4_planning_msgs/msg/path_with_lane_id.hpp>

#include <yaml-cpp/yaml.h>

#include <string>
#include <vector>

namespace autoware::test_utils
{
using autoware_planning_msgs::msg::LaneletPrimitive;
using autoware_planning_msgs::msg::LaneletRoute;
using autoware_planning_msgs::msg::LaneletSegment;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;
using std_msgs::msg::Header;
using tier4_planning_msgs::msg::PathPointWithLaneId;
using tier4_planning_msgs::msg::PathWithLaneId;

/**
 * @brief Parses a YAML node and converts it into an object of type T.
 *
 * This function extracts data from the given YAML node and converts it into an object of type T.
 * If no specialization exists for T, it will result in a compile-time error.
 *
 * @tparam T The type of object to parse the node contents into.
 * @param node The YAML node to be parsed.
 * @return T An object of type T containing the parsed data.
 */
template <typename T>
T parse(const YAML::Node & node);

template <>
Pose parse(const YAML::Node & node);

template <>
LaneletPrimitive parse(const YAML::Node & node);

template <>
std::vector<LaneletPrimitive> parse(const YAML::Node & node);

template <>
std::vector<LaneletSegment> parse(const YAML::Node & node);

template <>
std::vector<Point> parse(const YAML::Node & node);

template <>
Header parse(const YAML::Node & node);

template <>
std::vector<PathPointWithLaneId> parse(const YAML::Node & node);

/**
 * @brief Parses a YAML file and converts it into an object of type T.
 *
 * This function reads the specified YAML file and converts its contents into an object of the given
 * type T. If no specialization exists for T, it will result in a compile-time error.
 *
 * @tparam T The type of object to parse the file contents into.
 * @param filename The path to the YAML file to be parsed.
 * @return T An object of type T containing the parsed data.
 */
template <typename T>
T parse(const std::string & filename);

template <>
LaneletRoute parse(const std::string & filename);

template <>
PathWithLaneId parse(const std::string & filename);
}  // namespace autoware::test_utils

#endif  // AUTOWARE_TEST_UTILS__MOCK_DATA_PARSER_HPP_
