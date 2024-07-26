// Copyright 2018-2021 Tier IV, Inc. All rights reserved.
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

#include "autoware/pid_longitudinal_controller/pid.hpp"

#include <algorithm>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

namespace autoware::motion::control::pid_longitudinal_controller
{
PIDController::PIDController()
: m_virtual_displacement_error(0.0),
  m_virtual_displacement_error_integral(0.0),
  m_prev_error(0.0),
  m_is_gains_set(false),
  m_is_limits_set(false)
{
}

double PIDController::calculate(
  const double error, const double dt, const bool erase_integral,
  std::vector<double> & pid_contributions)
{
  std::cerr << "DEBUG: PID calculation started" << std::endl;
  std::cerr << "DEBUG: Input error: " << error << ", dt: " << dt
            << ", erase_integral: " << erase_integral << std::endl;

  if (!m_is_gains_set || !m_is_limits_set) {
    std::cerr << "ERROR: PID not initialized. Gains set: " << m_is_gains_set
              << ", Limits set: " << m_is_limits_set << std::endl;
    throw std::runtime_error("Trying to calculate uninitialized PID");
  }

  const auto & p = m_params;
  std::cerr << "DEBUG: PID parameters - kp: " << p.kp << ", ki: " << p.ki << ", kd: " << p.kd
            << std::endl;

  double prev_virtual_displacement_error = m_virtual_displacement_error;
  m_virtual_displacement_error = erase_integral ? 0.0 : m_virtual_displacement_error + error * dt;
  std::cerr << "DEBUG: Virtual displacement error: " << m_virtual_displacement_error
            << " (previous: " << prev_virtual_displacement_error << ")" << std::endl;

  double ret_p = p.kp * m_virtual_displacement_error;
  double raw_ret_p = ret_p;
  ret_p = std::min(std::max(ret_p, p.min_ret_p), p.max_ret_p);
  std::cerr << "DEBUG: P term: " << ret_p << " (raw: " << raw_ret_p << ", min: " << p.min_ret_p
            << ", max: " << p.max_ret_p << ")" << std::endl;

  double prev_virtual_displacement_error_integral = m_virtual_displacement_error_integral;
  m_virtual_displacement_error_integral =
    erase_integral ? 0.0
                   : m_virtual_displacement_error_integral + m_virtual_displacement_error * dt;
  std::cerr << "DEBUG: Virtual displacement error integral: "
            << m_virtual_displacement_error_integral
            << " (previous: " << prev_virtual_displacement_error_integral << ")" << std::endl;

  double raw_ret_i = p.ki * m_virtual_displacement_error_integral;
  const double ret_i = std::min(std::max(raw_ret_i, p.min_ret_i), p.max_ret_i);
  std::cerr << "DEBUG: I term: " << ret_i << " (raw: " << raw_ret_i << ", min: " << p.min_ret_i
            << ", max: " << p.max_ret_i << ")" << std::endl;

  double ret_d = p.kd * error;
  double raw_ret_d = ret_d;
  ret_d = std::min(std::max(ret_d, p.min_ret_d), p.max_ret_d);
  std::cerr << "DEBUG: D term: " << ret_d << " (raw: " << raw_ret_d << ", min: " << p.min_ret_d
            << ", max: " << p.max_ret_d << ")" << std::endl;

  std::cerr << "DEBUG: Previous error: " << m_prev_error << ", Current error: " << error
            << std::endl;
  m_prev_error = error;

  pid_contributions.resize(3);
  pid_contributions.at(0) = ret_p;
  pid_contributions.at(1) = ret_i;
  pid_contributions.at(2) = ret_d;
  std::cerr << "DEBUG: PID contributions - P: " << ret_p << ", I: " << ret_i << ", D: " << ret_d
            << std::endl;

  double ret = ret_p + ret_i + ret_d;
  double raw_ret = ret;
  ret = std::min(std::max(ret, p.min_ret), p.max_ret);
  std::cerr << "DEBUG: Final output: " << ret << " (raw: " << raw_ret << ", min: " << p.min_ret
            << ", max: " << p.max_ret << ")" << std::endl;

  std::cerr << "DEBUG: PID calculation completed" << std::endl;
  return ret;
}

void PIDController::setGains(const double kp, const double ki, const double kd)
{
  m_params.kp = kp;
  m_params.ki = ki;
  m_params.kd = kd;
  m_is_gains_set = true;
}

void PIDController::setLimits(
  const double max_ret, const double min_ret, const double max_ret_p, const double min_ret_p,
  const double max_ret_i, const double min_ret_i, const double max_ret_d, const double min_ret_d)
{
  m_params.max_ret = max_ret;
  m_params.min_ret = min_ret;
  m_params.max_ret_p = max_ret_p;
  m_params.min_ret_p = min_ret_p;
  m_params.max_ret_d = max_ret_d;
  m_params.min_ret_d = min_ret_d;
  m_params.max_ret_i = max_ret_i;
  m_params.min_ret_i = min_ret_i;
  m_is_limits_set = true;
}

void PIDController::reset()
{
  m_virtual_displacement_error = 0.0;
  m_virtual_displacement_error_integral = 0.0;
  m_prev_error = 0.0;
}
}  // namespace autoware::motion::control::pid_longitudinal_controller
