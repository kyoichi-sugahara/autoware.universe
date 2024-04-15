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

#include "mpc_lateral_controller/qp_solver/qp_solver_cgmres.hpp"

#include <Eigen/Dense>

#include <string>
#include <vector>

namespace autoware::motion::control::mpc_lateral_controller
{
QPSolverCGMRES::QPSolverCGMRES(const rclcpp::Logger & logger)
: logger_{logger},
  cgmres_logger_("../log/trajectory_following"),
  mpc_(
    ocp_, cgmres::Horizon(0.1, 0.0),
    settings_),  // Direct initialization of Horizon in the initializer list.
  initializer_(ocp_, settings_)
{
  // Solver settings.
  settings_.sampling_time = 0.03;  // sampling period
  settings_.zeta = 33.3;
  settings_.finite_difference_epsilon = 1e-08;
  // Initialization settings.
  settings_.max_iter = 50;
  settings_.opterr_tol = 1e-06;
  settings_.verbose_level = 0;
}

bool QPSolverCGMRES::solveCGMRES(
  const Eigen::VectorXd & x0, const MPCTrajectory & resampled_ref_trajectory,
  const double prediction_dt, Eigen::VectorXd & u, const bool warm_start)
{
  std::cerr << "prediction_dt: " << prediction_dt << std::endl;
  // // Define the horizon.
  const double alpha = 0.0;
  [[maybe_unused]] cgmres::Horizon horizon(prediction_dt, alpha);

  // Define the initial time and initial state.
  // state は 横偏差、ヨー角、ステアリング角度の3つ
  cgmres::Vector<3> x;
  x << x0(0), x0(1), x0(2);

  // calculate the average curvature of the reference trajectory
  double curvature_sum = 0.0;
  for (size_t i = 0; i < resampled_ref_trajectory.k.size(); ++i) {
    curvature_sum += resampled_ref_trajectory.k.at(i);
  }
  const double average_curvature = curvature_sum / resampled_ref_trajectory.k.size();
  // set the external reference ptr
  ocp_.curvature_in_reference_trajectory = average_curvature;
  ocp_.u_ref[0] = std::atan(average_curvature * ocp_.wheel_base);
  std::cerr << "average_curvature: " << average_curvature << std::endl;
  std::cerr << "ocp_.u_ref[0]: " << ocp_.u_ref[0] << std::endl;

  if (!is_initialized_) {
    initialized_time_ = std::chrono::system_clock::now();
    is_initialized_ = true;
  }

  if (warm_start) {
    const double time_from_last_initialized =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now() - initialized_time_)
        .count() *
      1.0e-6;
    mpc_.update(time_from_last_initialized, x);
    u = mpc_.uopt()[0];
    RCLCPP_DEBUG(logger_, "\n\n\n time_from_last_initialized = %e \n", time_from_last_initialized);
    RCLCPP_DEBUG(logger_, "updated u = %e \n\n\n", mpc_.uopt()[0].value());
    // cgmres_logger_.save(
    //   time_from_last_initialized, x, mpc_.uopt()[0], mpc_.uopt(), mpc_.optError(),
    //   mpc_.normDiff(), mpc_.StandardDeviation());
  } else {
    // Initialize the solution of the C/GMRES method.
    cgmres::Vector<1> uc0;
    uc0 << 0.0;
    initializer_.set_uc(uc0);
    const double t0 = 0.0;
    initializer_.solve(t0, x);
    u = initializer_.uopt();
    mpc_.set_uc(initializer_.ucopt());
    mpc_.init_dummy_mu();
    RCLCPP_DEBUG(logger_, "\n\n\n u = %e \n\n\n", u(0));
  }

  // std::cout << "MPC used in this simulation:" << std::endl;
  // std::cout << mpc_ << std::endl;
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