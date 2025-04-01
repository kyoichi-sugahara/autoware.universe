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

#include "autoware/planning_validator/utils.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/geometry/boost_polygon_utils.hpp>
#include <autoware_utils/geometry/geometry.hpp>

#include <boost/geometry/algorithms/intersects.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::planning_validator
{
using autoware_utils::calc_curvature;
using autoware_utils::calc_distance2d;
using autoware_utils::get_point;

namespace
{
void takeBigger(double & v_max, size_t & i_max, double v, size_t i)
{
  if (v_max < v) {
    v_max = v;
    i_max = i;
  }
}
void takeSmaller(double & v_min, size_t & i_min, double v, size_t i)
{
  if (v_min > v) {
    v_min = v;
    i_min = i;
  }
}
}  // namespace

std::pair<double, size_t> getAbsMaxValAndIdx(const std::vector<double> & v)
{
  const auto iter = std::max_element(
    v.begin(), v.end(), [](const auto & a, const auto & b) { return std::abs(a) < std::abs(b); });
  const auto idx = std::distance(v.begin(), iter);
  return {std::abs(*iter), idx};
}

// Do not interpolate.
Trajectory resampleTrajectory(const Trajectory & trajectory, const double min_interval)
{
  Trajectory resampled;
  resampled.header = trajectory.header;

  if (trajectory.points.empty()) {
    return resampled;
  }

  resampled.points.push_back(trajectory.points.front());
  for (size_t i = 1; i < trajectory.points.size(); ++i) {
    const auto prev = resampled.points.back();
    const auto curr = trajectory.points.at(i);
    if (calc_distance2d(prev, curr) > min_interval) {
      resampled.points.push_back(curr);
    }
  }
  return resampled;
}

// calculate curvature from three points with curvature_distance
void calcCurvature(
  const Trajectory & trajectory, std::vector<double> & curvature_arr,
  const double curvature_distance)
{
  if (trajectory.points.size() < 3) {
    curvature_arr = std::vector<double>(trajectory.points.size(), 0.0);
    return;
  }

  // calc arc length array: arc_length(3) - arc_length(0) is distance from point(3) to point(0)
  std::vector<double> arc_length(trajectory.points.size(), 0.0);
  for (size_t i = 1; i < trajectory.points.size(); ++i) {
    arc_length.at(i) =
      arc_length.at(i - 1) + calc_distance2d(trajectory.points.at(i - 1), trajectory.points.at(i));
  }

  // initialize with 0 curvature
  curvature_arr = std::vector<double>(trajectory.points.size(), 0.0);

  size_t first_distant_index = 0;
  size_t last_distant_index = trajectory.points.size() - 1;
  for (size_t i = 1; i < trajectory.points.size() - 1; ++i) {
    // find the previous point
    size_t prev_idx = 0;
    for (size_t j = i - 1; j > 0; --j) {
      if (arc_length.at(i) - arc_length.at(j) > curvature_distance) {
        if (first_distant_index == 0) {
          first_distant_index = i;  // save first index that meets distance requirement
        }
        prev_idx = j;
        break;
      }
    }

    // find the next point
    size_t next_idx = trajectory.points.size() - 1;
    for (size_t j = i + 1; j < trajectory.points.size(); ++j) {
      if (arc_length.at(j) - arc_length.at(i) > curvature_distance) {
        last_distant_index = i;  // save last index that meets distance requirement
        next_idx = j;
        break;
      }
    }

    const auto p1 = get_point(trajectory.points.at(prev_idx));
    const auto p2 = get_point(trajectory.points.at(i));
    const auto p3 = get_point(trajectory.points.at(next_idx));
    try {
      curvature_arr.at(i) = autoware_utils::calc_curvature(p1, p2, p3);
    } catch (...) {
      curvature_arr.at(i) = 0.0;  // maybe distance is too close
    }
  }

  // use previous or last curvature where the distance is not enough
  for (size_t i = first_distant_index; i > 0; --i) {
    curvature_arr.at(i - 1) = curvature_arr.at(i);
  }
  for (size_t i = last_distant_index; i < curvature_arr.size() - 1; ++i) {
    curvature_arr.at(i + 1) = curvature_arr.at(i);
  }
}

std::pair<double, size_t> calcMaxCurvature(const Trajectory & trajectory)
{
  if (trajectory.points.size() < 3) {
    return {0.0, 0};
  }

  std::vector<double> curvature_arr;
  calcCurvature(trajectory, curvature_arr);

  const auto max_curvature_it = std::max_element(curvature_arr.begin(), curvature_arr.end());
  const size_t index = std::distance(curvature_arr.begin(), max_curvature_it);

  return {*max_curvature_it, index};
}

void calc_interval_distance(
  const Trajectory & trajectory, std::vector<double> & interval_distance_arr)
{
  if (trajectory.points.size() < 2) {
    interval_distance_arr = std::vector<double>(trajectory.points.size() - 1, 0.0);
    return;
  }

  interval_distance_arr = std::vector<double>(trajectory.points.size() - 1, 0.0);
  for (size_t i = 0; i < trajectory.points.size() - 1; ++i) {
    const auto d = calc_distance2d(trajectory.points.at(i), trajectory.points.at(i + 1));
    interval_distance_arr.at(i) = d;
  }
}

std::pair<double, size_t> calcMaxIntervalDistance(const Trajectory & trajectory)
{
  if (trajectory.points.size() < 2) {
    return {0.0, 0};
  }
  std::vector<double> interval_distance_arr;
  calc_interval_distance(trajectory, interval_distance_arr);

  const auto max_interval_it =
    std::max_element(interval_distance_arr.begin(), interval_distance_arr.end());
  const size_t max_index = std::distance(interval_distance_arr.begin(), max_interval_it);

  return {*max_interval_it, max_index};
}

void calc_lateral_acceleration(
  const Trajectory & trajectory, std::vector<double> & lateral_acceleration_arr)
{
  if (trajectory.points.size() < 2) {
    lateral_acceleration_arr = std::vector<double>(trajectory.points.size(), 0.0);
    return;
  }

  std::vector<double> curvatures;
  calcCurvature(trajectory, curvatures);

  lateral_acceleration_arr = std::vector<double>(trajectory.points.size(), 0.0);
  for (size_t i = 0; i < trajectory.points.size(); ++i) {
    const auto v_lon = trajectory.points.at(i).longitudinal_velocity_mps;
    const auto a_lon = trajectory.points.at(i).acceleration_mps2;

    // Component 1: Centrifugal acceleration from curvature (v^2 * κ)
    const auto lat_acc_curve = v_lon * v_lon * curvatures.at(i);

    // Component 2: Lateral projection of longitudinal acceleration
    const auto theta = std::atan2(curvatures.at(i) * v_lon * v_lon, a_lon);
    const auto lat_acc_from_lon = a_lon * std::sin(theta);

    lateral_acceleration_arr.at(i) =
      std::sqrt(lat_acc_curve * lat_acc_curve + lat_acc_from_lon * lat_acc_from_lon);
  }
}

std::pair<double, size_t> calcMaxLateralAcceleration(const Trajectory & trajectory)
{
  std::vector<double> lateral_acceleration_arr;
  calc_lateral_acceleration(trajectory, lateral_acceleration_arr);

  const auto max_it =
    std::max_element(lateral_acceleration_arr.begin(), lateral_acceleration_arr.end());
  const size_t max_index = std::distance(lateral_acceleration_arr.begin(), max_it);

  return {*max_it, max_index};
}

/**
 * @brief Calculate time interval between two points assuming constant acceleration
 * @param v1 Initial velocity [m/s]
 * @param v2 Final velocity [m/s]
 * @param a Acceleration [m/s^2]
 * @param ds Distance interval [m]
 * @return Time interval [s]
 */
void calc_interval_time(const Trajectory & trajectory, std::vector<double> & time_interval_arr)
{
  // Return empty array if trajectory has less than 2 points
  if (trajectory.points.size() < 2) {
    time_interval_arr.clear();
    return;
  }

  // Calculate distances between points
  std::vector<double> interval_distance_arr;
  calc_interval_distance(trajectory, interval_distance_arr);

  // Reserve space for time intervals (one less than number of points)
  time_interval_arr.resize(trajectory.points.size() - 1);

  constexpr double epsilon = 1e-6;  // Threshold for near-zero values

  // Calculate time interval for each segment
  for (size_t i = 0; i < trajectory.points.size() - 1; ++i) {
    const double v_current_lon = trajectory.points[i].longitudinal_velocity_mps;
    const double v_next_lon = trajectory.points[i + 1].longitudinal_velocity_mps;
    const double a_current_lon = trajectory.points[i].acceleration_mps2;
    const double ds = interval_distance_arr[i];

    // Handle zero distance case
    if (std::abs(ds) < epsilon) {
      time_interval_arr[i] = 0.0;
      continue;
    }

    // Special case for near-zero acceleration
    if (std::abs(a_current_lon) < epsilon) {
      const double v_avg = (v_current_lon + v_next_lon) / 2.0;
      time_interval_arr[i] = (std::abs(v_avg) < epsilon) ? 0.0 : ds / v_avg;
      continue;
    }

    // For non-zero acceleration, use: ds = v_current_lon * dt + 0.5 * a_current_lon * dt^2
    const double discriminant = v_current_lon * v_current_lon + 2.0 * a_current_lon * ds;

    if (discriminant >= 0.0) {
      // Standard solution from quadratic formula
      const double dt = (std::sqrt(discriminant) - v_current_lon) / a_current_lon;
      time_interval_arr[i] = std::max(0.0, dt);  // Ensure non-negative time
    } else {
      // Fallback to average velocity if quadratic solution fails
      const double v_avg = (v_current_lon + v_next_lon) / 2.0;
      time_interval_arr[i] = (std::abs(v_avg) < epsilon) ? 0.0 : ds / v_avg;
    }
  }
}

// {
//   constexpr double epsilon = 1e-6;  // Threshold for near-zero values

//   // Handle zero distance case
//   if (std::abs(ds) < epsilon) {
//     return 0.0;
//   }

//   // Special case for near-zero acceleration
//   if (std::abs(a_current_lon) < epsilon) {
//     const double v_avg = (v_current_lon + v_next_lon) / 2.0;
//     return (std::abs(v_avg) < epsilon) ? 0.0 : ds / v_avg;
//   }

//   // For non-zero acceleration, use: ds = v1 * dt + 0.5 * a * dt^2
//   const double discriminant = v_current_lon * v_current_lon + 2.0 * a_current_lon * ds;

//   if (discriminant >= 0.0) {
//     // Standard solution from quadratic formula
//     const double dt = (std::sqrt(discriminant) - v_current_lon) / a_current_lon;
//     return std::max(0.0, dt);  // Ensure non-negative time
//   }

//   // Fallback to average velocity if quadratic solution fails
//   const double v_avg = (v_current_lon + v_next_lon) / 2.0;
//   return (std::abs(v_avg) < epsilon) ? 0.0 : ds / v_avg;
// }

// /**
//  * @brief Calculate time from start for each point in trajectory
//  * @param trajectory Target trajectory
//  * @param time_from_start_arr Output array of time from start for each point
//  */
// void calc_time_from_start(const Trajectory & trajectory, std::vector<double> &
// time_from_start_arr)
// {
//   // Handle empty trajectory
//   if (trajectory.points.empty()) {
//     time_from_start_arr.clear();
//     return;
//   }

//   // Handle single-point trajectory
//   if (trajectory.points.size() == 1) {
//     time_from_start_arr.assign(1, 0.0);
//     return;
//   }

//   // Prepare output array
//   time_from_start_arr.clear();
//   time_from_start_arr.reserve(trajectory.points.size());

//   // Calculate distances between points
//   std::vector<double> interval_distance_arr;
//   calc_interval_distance(trajectory, interval_distance_arr);

//   // First point starts at time = 0
//   double accumulated_time = 0.0;
//   time_from_start_arr.push_back(accumulated_time);

//   // Calculate time for each subsequent point
//   for (size_t i = 0; i < trajectory.points.size() - 1; ++i) {
//     const double v_current_lon = trajectory.points[i].longitudinal_velocity_mps;
//     const double v_next_lon = trajectory.points[i + 1].longitudinal_velocity_mps;
//     const double a_current_lon = trajectory.points[i].acceleration_mps2;
//     const double ds = interval_distance_arr[i];

//     const double dt = calc_time_interval(v_current_lon, v_next_lon, a_current_lon, ds);
//     accumulated_time += dt;

//     time_from_start_arr.push_back(accumulated_time);
//   }
// }

// std::pair<double, size_t> calc_max_lateral_jerk(
//   const Trajectory & trajectory, const double wheelbase)
// {
//   std::vector<double> steering_array;
//   calcSteeringAngles(trajectory, wheelbase, steering_array);

//   if (steering_array.size() < 2) {
//     return {0.0, 0};
//   }

//   double max_lateral_jerk = 0.0;
//   size_t max_index = 0;
//   for (size_t i = 0; i < steering_array.size() - 1; ++i) {
//     const auto delta_s = calc_distance2d(trajectory.points.at(i), trajectory.points.at(i + 1));
//     const auto dt = delta_s /
//     std::max(trajectory.points.at(i).longitudinal_velocity_mps, 1.0e-5);

//     const auto steer_prev = steering_array.at(i);
//     const auto steer_next = steering_array.at(i + 1);

//     const auto steer_rate = (steer_next - steer_prev) / dt;
//     takeBigger(max_lateral_jerk, max_index, std::abs(steer_rate), i);
//   }

//   return {max_lateral_jerk, max_index};
// }

std::pair<double, size_t> getMaxLongitudinalAcc(const Trajectory & trajectory)
{
  double max_acc = 0.0;
  size_t max_index = 0;
  for (size_t i = 0; i < trajectory.points.size(); ++i) {
    takeBigger(max_acc, max_index, trajectory.points.at(i).acceleration_mps2, i);
  }
  return {max_acc, max_index};
}

std::pair<double, size_t> getMinLongitudinalAcc(const Trajectory & trajectory)
{
  double min_acc = 0.0;
  size_t min_index = 0;
  for (size_t i = 0; i < trajectory.points.size(); ++i) {
    takeSmaller(min_acc, min_index, trajectory.points.at(i).acceleration_mps2, i);
  }
  return {min_acc, min_index};
}

std::pair<double, size_t> calcMaxRelativeAngles(const Trajectory & trajectory)
{
  // We need at least three points to compute relative angle
  const size_t relative_angle_points_num = 3;
  if (trajectory.points.size() < relative_angle_points_num) {
    return {0.0, 0};
  }

  double max_relative_angles = 0.0;
  size_t max_index = 0;

  for (size_t i = 0; i <= trajectory.points.size() - relative_angle_points_num; ++i) {
    const auto & p1 = trajectory.points.at(i).pose.position;
    const auto & p2 = trajectory.points.at(i + 1).pose.position;
    const auto & p3 = trajectory.points.at(i + 2).pose.position;

    const auto angle_a = autoware_utils::calc_azimuth_angle(p1, p2);
    const auto angle_b = autoware_utils::calc_azimuth_angle(p2, p3);

    // convert relative angle to [-pi ~ pi]
    const auto relative_angle = std::abs(autoware_utils::normalize_radian(angle_b - angle_a));

    takeBigger(max_relative_angles, max_index, std::abs(relative_angle), i);
  }

  return {max_relative_angles, max_index};
}

void calcSteeringAngles(
  const Trajectory & trajectory, const double wheelbase, std::vector<double> & steering_array)
{
  const auto curvatureToSteering = [](const auto k, const auto wheelbase) {
    return std::atan(k * wheelbase);
  };

  std::vector<double> curvatures;
  calcCurvature(trajectory, curvatures);

  steering_array.clear();
  for (const auto k : curvatures) {
    steering_array.push_back(curvatureToSteering(k, wheelbase));
  }
}

std::pair<double, size_t> calcMaxSteeringAngles(
  const Trajectory & trajectory, const double wheelbase)
{
  std::vector<double> steering_array;
  calcSteeringAngles(trajectory, wheelbase, steering_array);

  return getAbsMaxValAndIdx(steering_array);
}

std::pair<double, size_t> calcMaxSteeringRates(
  const Trajectory & trajectory, const double wheelbase)
{
  if (trajectory.points.size() < 1) {
    return {0.0, 0};
  }

  std::vector<double> steering_array;
  calcSteeringAngles(trajectory, wheelbase, steering_array);

  double max_steering_rate = 0.0;
  size_t max_index = 0;
  for (size_t i = 0; i < trajectory.points.size() - 1; ++i) {
    const auto & p_prev = trajectory.points.at(i);
    const auto & p_next = trajectory.points.at(i + 1);
    const auto delta_s = calc_distance2d(p_prev, p_next);
    const auto v = 0.5 * (p_next.longitudinal_velocity_mps + p_prev.longitudinal_velocity_mps);
    const auto dt = delta_s / std::max(v, 1.0e-5);

    const auto steer_prev = steering_array.at(i);
    const auto steer_next = steering_array.at(i + 1);

    const auto steer_rate = (steer_next - steer_prev) / dt;
    takeBigger(max_steering_rate, max_index, std::abs(steer_rate), i);
  }

  return {max_steering_rate, max_index};
}

std::optional<std::pair<std::vector<TrajectoryPoint>, std::vector<Box>>> check_collision(
  const PredictedObjects & predicted_objects, const Trajectory & trajectory,
  const geometry_msgs::msg::Point & current_ego_position, const VehicleInfo & vehicle_info,
  const double trajectory_to_object_distance_threshold,
  const double ego_to_object_distance_threshold, const double time_tolerance_threshold)
{
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> filtered_trajectory;

  filtered_trajectory.reserve(trajectory.points.size());

  for (size_t i = 0; i < trajectory.points.size(); ++i) {
    const auto & point = trajectory.points[i];
    const double dist_to_point = autoware::motion_utils::calcSignedArcLength(
      trajectory.points, current_ego_position, size_t(i));

    // Only include points that are ahead of current position (positive distance)
    if (dist_to_point > 0.0) {
      filtered_trajectory.push_back(point);
    }
  }
  if (filtered_trajectory.empty()) {
    return std::nullopt;
  }
  // Calculate timestamps for each trajectory point
  motion_utils::calculate_time_from_start(filtered_trajectory, current_ego_position);

  const auto & ego_rtree = make_ego_footprint_rtree(filtered_trajectory, vehicle_info);

  const auto filtered_objects = filter_objects(
    predicted_objects, filtered_trajectory, trajectory_to_object_distance_threshold,
    ego_to_object_distance_threshold);
  if (!filtered_objects) {
    return std::nullopt;
  }

  std::vector<BoxTimeIndexPair> predicted_object_rtree_nodes;

  // Check each predicted object for potential collisions
  for (const auto & object : filtered_objects.value().objects) {
    const auto & highest_confidence_path = find_highest_confidence_path(object);
    if (!highest_confidence_path) {
      continue;
    }

    const double predicted_time_step = highest_confidence_path.value().time_step.sec +
                                       highest_confidence_path.value().time_step.nanosec * 1e-9;

    make_predicted_object_rtree(
      highest_confidence_path.value(), object.shape, predicted_time_step,
      predicted_object_rtree_nodes);
  }

  Rtree predicted_object_rtree(
    predicted_object_rtree_nodes.begin(), predicted_object_rtree_nodes.end());

  const auto & collision_index_set =
    detect_collisions(ego_rtree, predicted_object_rtree, time_tolerance_threshold);
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> collision_points;
  std::vector<Box> collision_boxes;

  if (!collision_index_set.empty()) {
    collision_points.reserve(collision_index_set.size());
    collision_boxes.reserve(collision_index_set.size());

    for (const auto & [ego_index, obj_box] : collision_index_set) {
      collision_points.push_back(filtered_trajectory[ego_index]);
      collision_boxes.push_back(obj_box);
    }
  }

  return (collision_points.empty() && collision_boxes.empty())
           ? std::nullopt
           : std::make_optional(std::make_pair(collision_points, collision_boxes));
}

Rtree make_ego_footprint_rtree(
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory,
  const VehicleInfo & vehicle_info)
{
  autoware_utils::MultiPolygon2d trajectory_footprints;
  const double base_to_front = vehicle_info.wheel_base_m + vehicle_info.front_overhang_m;
  const double base_to_rear = vehicle_info.rear_overhang_m;

  for (const auto & p : trajectory)
    trajectory_footprints.push_back(autoware_utils::to_footprint(
      p.pose, base_to_front, base_to_rear, vehicle_info.vehicle_width_m));
  std::vector<BoxTimeIndexPair> rtree_nodes;

  rtree_nodes.reserve(trajectory_footprints.size());
  for (auto i = 0UL; i < trajectory_footprints.size(); ++i) {
    const auto box =
      boost::geometry::return_envelope<autoware_utils::Box2d>(trajectory_footprints[i]);
    const double time =
      trajectory[i].time_from_start.sec + trajectory[i].time_from_start.nanosec * 1e-9;
    rtree_nodes.emplace_back(box, std::make_pair(time, i));
  }
  return Rtree(rtree_nodes);
}

std::optional<PredictedObjects> filter_objects(
  const PredictedObjects & objects,
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory,
  const double trajectory_to_object_distance_threshold,
  const double ego_to_object_distance_threshold)
{
  PredictedObjects filtered_objects;

  for (const auto & object : objects.objects) {
    const auto & object_position = object.kinematics.initial_pose_with_covariance.pose.position;
    const size_t nearest_index =
      autoware::motion_utils::findNearestIndex(trajectory, object_position);
    const double trajectory_to_object_distance =
      autoware_utils::calc_distance2d(trajectory[nearest_index], object_position);
    const double ego_to_object_distance =
      autoware_utils::calc_distance2d(trajectory.front().pose.position, object_position);

    if (
      trajectory_to_object_distance < trajectory_to_object_distance_threshold &&
      ego_to_object_distance < ego_to_object_distance_threshold) {
      filtered_objects.objects.push_back(object);
    }
  }

  return filtered_objects.objects.empty() ? std::nullopt : std::make_optional(filtered_objects);
}

std::optional<PredictedPath> find_highest_confidence_path(const PredictedObject & object)
{
  const auto & paths = object.kinematics.predicted_paths;

  if (paths.empty()) {
    return std::nullopt;
  }

  const auto max_confidence_it = std::max_element(
    paths.begin(), paths.end(),
    [](const PredictedPath & a, const PredictedPath & b) { return a.confidence < b.confidence; });

  return *max_confidence_it;
}

void make_predicted_object_rtree(
  const PredictedPath & highest_confidence_path, const Shape & object_shape,
  const double predicted_time_step, std::vector<BoxTimeIndexPair> & predicted_object_rtree_nodes)
{
  for (size_t j = 0; j < highest_confidence_path.path.size(); ++j) {
    const auto & pose = highest_confidence_path.path[j];
    const double predicted_time = j * predicted_time_step;

    const auto predicted_polygon = autoware_utils::to_polygon2d(pose, object_shape);

    const auto box = boost::geometry::return_envelope<autoware_utils::Box2d>(predicted_polygon);

    predicted_object_rtree_nodes.emplace_back(box, std::make_pair(predicted_time, j));
  }
}

std::vector<std::pair<size_t, Box>> detect_collisions(
  const Rtree & ego_rtree, const Rtree & predicted_object_rtree, double time_tolerance)
{
  std::vector<std::pair<size_t, Box>> collision_sets;

  for (const auto & ego_value : ego_rtree) {
    const auto & ego_box = ego_value.first;
    const double ego_time = ego_value.second.first;

    std::vector<BoxTimeIndexPair> potential_collisions;
    predicted_object_rtree.query(
      boost::geometry::index::intersects(ego_box) &&
        boost::geometry::index::satisfies([&](const BoxTimeIndexPair & obj_value) {
          return std::fabs(obj_value.second.first - ego_time) <= time_tolerance;
        }),
      std::back_inserter(potential_collisions));

    for (const auto & obj_value : potential_collisions) {
      if (boost::geometry::intersects(ego_box, obj_value.first)) {
        collision_sets.emplace_back(ego_value.second.second, obj_value.first);
      }
    }
  }

  return collision_sets;
}

bool checkFinite(const TrajectoryPoint & point)
{
  const auto & p = point.pose.position;
  const auto & o = point.pose.orientation;

  using std::isfinite;
  const bool p_result = isfinite(p.x) && isfinite(p.y) && isfinite(p.z);
  const bool quat_result = isfinite(o.x) && isfinite(o.y) && isfinite(o.z) && isfinite(o.w);
  const bool v_result = isfinite(point.longitudinal_velocity_mps);
  const bool w_result = isfinite(point.heading_rate_rps);
  const bool a_result = isfinite(point.acceleration_mps2);

  return quat_result && p_result && v_result && w_result && a_result;
}

void shiftPose(geometry_msgs::msg::Pose & pose, double longitudinal)
{
  const auto yaw = tf2::getYaw(pose.orientation);
  pose.position.x += std::cos(yaw) * longitudinal;
  pose.position.y += std::sin(yaw) * longitudinal;
}

}  // namespace autoware::planning_validator
