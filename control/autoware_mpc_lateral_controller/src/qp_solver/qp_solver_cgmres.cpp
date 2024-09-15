// Copyright 2018-2021 The Autoware Foundation
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

#include "autoware/mpc_lateral_controller/qp_solver/qp_solver_cgmres.hpp"

#include <Eigen/Dense>

#include <string>
#include <vector>

namespace autoware::motion::control::mpc_lateral_controller
{
QPSolverCGMRES::QPSolverCGMRES(
  const rclcpp::Logger & logger, const std::string & log_dir,
  const cgmres::SolverSettings & solver_settings, const cgmres::Horizon & horizon,
  const double wheel_base, const double steer_tau)
: logger_{logger},
  cgmres_logger_(log_dir),
  settings_{solver_settings},
  external_reference_(std::make_shared<cgmres::OCP_lateral_control::ExternalReference>())
{
  ocp_.external_reference = external_reference_;
  ocp_.wheel_base = wheel_base;
  ocp_.steer_tau = steer_tau;
  initializer_ =
    cgmres::ZeroHorizonOCPSolver<cgmres::OCP_lateral_control, kmax_init>(ocp_, settings_);
  mpc_ = cgmres::SingleShootingCGMRESSolver<cgmres::OCP_lateral_control, N, kmax>(
    ocp_, horizon, settings_);
  settings_.disp(std::cerr);
  solver_settings.disp(std::cerr);
  ocp_.disp(std::cerr);
  mpc_.disp(std::cerr);
}

void QPSolverCGMRES::updateEquation(
  const MPCTrajectory & resampled_ref_trajectory, const double steer_tau)
{
  static constexpr double WHEEL_BASE = 2.74;

  ocp_.wheel_base = WHEEL_BASE;
  ocp_.steer_tau = steer_tau;

  const size_t trajectory_size = resampled_ref_trajectory.size();

  external_reference_->curvature_ref_array.resize(trajectory_size);
  external_reference_->v_ref_array.resize(trajectory_size);

  for (size_t i = 0; i < trajectory_size; ++i) {
    external_reference_->curvature_ref_array[i] = resampled_ref_trajectory.smooth_k[i];
    external_reference_->v_ref_array[i] = resampled_ref_trajectory.vx[i];
  }
}

bool QPSolverCGMRES::solveCGMRES(
  const Eigen::VectorXd & x0, Eigen::VectorXd & u, double & opt_error,
  Eigen::VectorXd & opt_error_array, const bool warm_start)
{
  // Define the initial time and initial state.
  cgmres::Vector<3> x;
  x << x0(0), x0(1), x0(2);

  // ocp_.disp(std::cerr);
  // mpc_.disp(std::cerr);

  if (initialized_time_ == rclcpp::Time(0, 0, RCL_ROS_TIME)) {
    initialized_time_ = cgmres_clock->now();
  }

  const auto current_time = cgmres_clock->now();
  const double time_since_initialized = (current_time - initialized_time_).nanoseconds() * 1.0e-6;

  if (warm_start) {
    mpc_.update(time_since_initialized, x);
  } else {
    // Initialize the solution of the C/GMRES method.
    cgmres::Vector<1> uc0 = cgmres::Vector<1>::Zero();
    initializer_.set_uc(uc0);
    initializer_.solve(time_since_initialized, x);
    mpc_.set_uc(initializer_.ucopt());
    mpc_.init_dummy_mu();
    mpc_.update(time_since_initialized, x);
  }

  const auto & uopt = mpc_.uopt();
  u.resize(uopt.size());
  for (size_t i = 0; i < uopt.size(); ++i) {
    u(i) = uopt[i](0, 0);
  }

  cgmres_logger_.save(
    time_since_initialized, x, uopt[0], uopt, mpc_.initial_solution(), mpc_.updated_solution(),
    mpc_.optError(), mpc_.gmres_iter());

  opt_error = mpc_.optError();

  opt_error_array = mpc_.optErrorArray();
  // Detailed comments on the structure and meaning of opt_error_array
  /*
  Structure of opt_error_array (for the case N=50, nu=1, nb=1):

  Total number of elements: 50 * (1 + 2 * 1) = 150

  Layout:
  [hu_0, hdummy_0, hmu_0, hu_1, hdummy_1, hmu_1, ..., hu_49, hdummy_49, hmu_49]

  Meaning of each element:
  - hu_i     (index 3i):   Gradient of the Hamiltonian with respect to the control input at the i-th
  time step The closer to zero, the more optimal the control input is
  - hdummy_i (index 3i+1): Condition related to the dummy variable at the i-th time step
                           Used to handle constraints smoothly
  - hmu_i    (index 3i+2): Condition related to the multiplier (Lagrange multiplier) at the i-th
  time step Indicates the satisfaction of constraints

  Examples:
  - opt_error_array[0]   : Optimality error for control input at the first time step
  - opt_error_array[1]   : Optimality error for dummy variable at the first time step
  - opt_error_array[2]   : Optimality error for multiplier at the first time step
  - opt_error_array[3]   : Optimality error for control input at the second time step
  ...
  - opt_error_array[147] : Optimality error for control input at the last time step
  - opt_error_array[148] : Optimality error for dummy variable at the last time step
  - opt_error_array[149] : Optimality error for multiplier at the last time step

  Notes:
  - The smaller the L2 norm of this array, the closer the obtained solution is to the optimal
  solution
  - During debugging or performance analysis, examining the value of each element individually
    can help identify which part of the optimization process may be problematic
  */

  return true;
}

bool QPSolverCGMRES::solve(
  const Eigen::MatrixXd & h_mat, const Eigen::MatrixXd & f_vec, const Eigen::MatrixXd & a,
  const Eigen::VectorXd & lb, const Eigen::VectorXd & ub, const Eigen::VectorXd & lb_a,
  const Eigen::VectorXd & ub_a, Eigen::VectorXd & u)
{
  const Eigen::Index raw_a = a.rows();
  const Eigen::Index col_a = a.cols();
  const Eigen::Index dim_u = ub.size();
  Eigen::MatrixXd Identity = Eigen::MatrixXd::Identity(dim_u, dim_u);

  // convert matrix to vector for cgmressolver
  std::vector<double> f(&f_vec(0), f_vec.data() + f_vec.cols() * f_vec.rows());

  std::vector<double> lower_bound;
  std::vector<double> upper_bound;

  for (int i = 0; i < dim_u; ++i) {
    lower_bound.push_back(lb(i));
    upper_bound.push_back(ub(i));
  }

  for (int i = 0; i < col_a; ++i) {
    lower_bound.push_back(lb_a(i));
    upper_bound.push_back(ub_a(i));
  }

  Eigen::MatrixXd cgmresA = Eigen::MatrixXd(dim_u + col_a, raw_a);
  cgmresA << Identity, a;

  /* execute optimization */
  auto result = cgmressolver_.optimize(h_mat, cgmresA, f, lower_bound, upper_bound);

  std::vector<double> U_cgmres = std::get<0>(result);
  u = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 1>>(
    &U_cgmres[0], static_cast<Eigen::Index>(U_cgmres.size()), 1);

  const int status_val = std::get<3>(result);
  if (status_val != 1) {
    RCLCPP_WARN(logger_, "optimization failed : %s", cgmressolver_.getStatusMessage().c_str());
    return false;
  }
  const auto has_nan =
    std::any_of(U_cgmres.begin(), U_cgmres.end(), [](const auto v) { return std::isnan(v); });
  if (has_nan) {
    RCLCPP_WARN(logger_, "optimization failed: result contains NaN values");
    return false;
  }

  // polish status: successful (1), unperformed (0), (-1) unsuccessful
  int status_polish = std::get<2>(result);
  if (status_polish == -1 || status_polish == 0) {
    const auto s = (status_polish == 0) ? "Polish process is not performed in cgmres."
                                        : "Polish process failed in cgmres.";
    RCLCPP_INFO(logger_, "%s The required accuracy is met, but the solution can be inaccurate.", s);
    return true;
  }
  return true;
}
}  // namespace autoware::motion::control::mpc_lateral_controller
