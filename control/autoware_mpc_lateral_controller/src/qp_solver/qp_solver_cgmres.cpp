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
  const cgmres::SolverSettings & solver_settings, const cgmres::Horizon & horizon)
: logger_{logger},
  cgmres_logger_(log_dir),
  settings_{solver_settings},
  external_reference_(std::make_shared<cgmres::OCP_lateral_control::ExternalReference>()),
  initializer_(ocp_, settings_)
{
  ocp_.external_reference = external_reference_;
  mpc_ = cgmres::SingleShootingCGMRESSolver<cgmres::OCP_lateral_control, N, kmax>(
    ocp_, horizon, settings_);
  settings_.disp(std::cerr);
  solver_settings.disp(std::cerr);
  ocp_.disp(std::cerr);
  mpc_.disp(std::cerr);
}

void QPSolverCGMRES::updateEquation(const MPCTrajectory & resampled_ref_trajectory)
{
  // calculate the average curvature of the reference trajectory
  double curvature_sum = 0.0;
  for (size_t i = 0; i < resampled_ref_trajectory.k.size(); ++i) {
    curvature_sum += resampled_ref_trajectory.k.at(i);
  }
  const double average_curvature = curvature_sum / resampled_ref_trajectory.k.size();
  // set the external reference ptr
  ocp_.curvature_in_reference_trajectory = average_curvature;
  ocp_.u_ref[0] = std::atan(average_curvature * ocp_.wheel_base);
  external_reference_->curvature_ref_array = resampled_ref_trajectory.k;
  external_reference_->v_ref_array = resampled_ref_trajectory.vx;
}

bool QPSolverCGMRES::solveCGMRES(
  const Eigen::VectorXd & x0, Eigen::VectorXd & u, const bool warm_start)
{
  // Define the initial time and initial state.
  cgmres::Vector<3> x;
  x << x0(0), x0(1), x0(2);

  // ocp_.disp(std::cerr);
  // mpc_.disp(std::cerr);

  if (!is_initialized_) {
    initialized_time_ = std::chrono::system_clock::now();
    is_initialized_ = true;
  }
  const double time_from_last_initialized = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                              std::chrono::system_clock::now() - initialized_time_)
                                              .count() *
                                            1.0e-6;

  if (warm_start) {
    mpc_.update(time_from_last_initialized, x);
  } else {
    // Initialize the solution of the C/GMRES method.
    cgmres::Vector<1> uc0;
    uc0 << 0.0;
    initializer_.set_uc(uc0);
    initializer_.solve(time_from_last_initialized, x);
    mpc_.set_uc(initializer_.ucopt());
    mpc_.init_dummy_mu();
    mpc_.update(time_from_last_initialized, x);
  }

  std::vector<double> U_cgmres(mpc_.uopt().size());
  for (size_t i = 0; i < mpc_.uopt().size(); ++i) {
    U_cgmres[i] = mpc_.uopt()[i](0);
  }
  u = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 1>>(
    &U_cgmres[0], static_cast<Eigen::Index>(U_cgmres.size()), 1);

  cgmres_logger_.save(
    time_from_last_initialized, x, mpc_.uopt()[0], mpc_.uopt(), mpc_.initial_solution(),
    mpc_.updated_solution(), mpc_.optError(), mpc_.gmres_iter());

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
