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

#ifndef NLP_INTERFACE__TYPES__CGMRES_PARAMETERS_HPP_
#define NLP_INTERFACE__TYPES__CGMRES_PARAMETERS_HPP_

// #include <autoware/universe_utils/geometry/boost_geometry.hpp>
// #include <autoware/universe_utils/geometry/pose_deviation.hpp>
// #include <rclcpp/node.hpp>

// #include <autoware_planning_msgs/msg/lanelet_route.hpp>
// #include <autoware_planning_msgs/msg/trajectory.hpp>
// #include <autoware_planning_msgs/msg/trajectory_point.hpp>
// #include <nav_msgs/msg/odometry.hpp>

// #include <lanelet2_core/LaneletMap.h>

#include <cmath>  // for std::exp
#include <map>
#include <stdexcept>  // for std::invalid_argument
#include <string>
#include <vector>

namespace autoware::nlp_interface::types
{

struct CGMRESSolverSettings
{
  /// @brief Maximum number of iterations of the ZeroHorizonOCPSolver method.
  /// Has nothing to do with SingleShootingCGMRESSolver or MultipleShootingCGMRESSolver.
  /// Default value is 100.
  ///
  size_t max_iter = 100;

  ///
  /// @brief Termination criterion of the ZeroHorizonOCPSolver method.
  /// Has nothing to do with SingleShootingCGMRESSolver or MultipleShootingCGMRESSolver.
  /// Must be non-negative. Default value is 1.0e-04.
  ///
  double opterr_tol = 1.0e-04;

  ///
  /// @brief Epsilon of the finite difference approximation. Must be positive.
  /// Default value is 1.0e-08.
  ///
  double finite_difference_epsilon = 1.0e-08;

  ///
  /// @brief The sampling time of MPC and used in SingleShootingCGMRESSolver
  /// and MultipleShootingCGMRESSolver. Has nothing to do with ZeroHorizonOCPSolver.
  /// Must be positive. Default is 0.001.
  ///
  double sampling_time = 0.001;

  ///
  /// @brief The stabilization parameter of the continuation method.
  /// Typical value is the reciprocal of SolverSettings::sampling_time (sampling period of MPC).
  /// Used in SingleShootingCGMRESSolver and MultipleShootingCGMRESSolver.
  /// Has nothing to do with ZeroHorizonOCPSolver.
  /// Must be positive. Default is 1000.0.
  ///
  double zeta = 1000.0;

  ///
  /// @brief The minimum value of the dummy inputs.
  /// Mainly used in MultipleShootingCGMRESSolver.
  /// In SingleShootingCGMRESSolver and ZeroHorizonOCPSolver, this value is
  /// used only in SingleShootingCGMRESSolver::init_dummy_mu() and
  /// ZeroHorizonOCPSolver::init_dummy_mu(). Must be non-negaive. Default is 1.0e-03.
  ///
  double min_dummy = 1.0e-03;

  ///
  /// @brief Verbose level. 0: no printings. 1-2: print some things. Default is 0.
  ///
  size_t verbose_level = 0;
};

struct Horizon
{
  ///
  /// @brief The parameter of the horizon length. Must be positive.
  ///
  double Tf = 0.0;

  ///
  /// @brief The parameter of the time-varying horizon length. Default is 0.0 (i.e., fixed-length
  /// horizon).
  ///
  double alpha = 0.0;

  ///
  /// @brief The parameter of the time-varying horizon length. Default is 0.0.
  ///
  double t0 = 0.0;

  ///
  /// @brief Indicates if the horizon length is time-varying.
  ///
  bool time_varying_length = false;

  ///
  /// @brief Gets the length of the horizon.
  /// @param[in] t The initial time of the horizon. If this horizon is time-varying (i.e., alpha >
  /// 0.0), then this value must not be less than t0.
  /// @return The horizon length at time t.
  ///
  double T(const double t) const
  {
    if (time_varying_length) {
      if (t < t0) {
        throw std::invalid_argument(
          "[Horizon]: 't' must be greater than or equal to 't0' (" + std::to_string(t0) + ") !");
      }
      return Tf * (1.0 - std::exp(-alpha * (t - t0)));
    }
    return Tf;
  }

  ///
  /// @brief Resets the length of the horizon (for time-varying horizon).
  /// @param[in] t0_new The parameter of the time-varying horizon length.
  ///
  void reset(const double t0_new) { t0 = t0_new; }
};

}  // namespace autoware::nlp_interface::types

#endif  // NLP_INTERFACE__TYPES__CGMRES_PARAMETERS_HPP_
