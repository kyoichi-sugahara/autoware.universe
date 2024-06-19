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

#ifndef AUTOWARE__MPC_LATERAL_CONTROLLER__QP_SOLVER__QP_SOLVER_CGMRES_HPP_
#define AUTOWARE__MPC_LATERAL_CONTROLLER__QP_SOLVER__QP_SOLVER_CGMRES_HPP_

#include "autoware/mpc_lateral_controller/mpc_cgmres.hpp"
#include "autoware/mpc_lateral_controller/qp_solver/qp_solver_interface.hpp"
#include "cgmres/horizon.hpp"
#include "cgmres/logger.hpp"
#include "cgmres/single_shooting_cgmres_solver.hpp"
#include "cgmres/solver_settings.hpp"
#include "cgmres/zero_horizon_ocp_solver.hpp"
#include "osqp_interface/osqp_interface.hpp"
#include "rclcpp/rclcpp.hpp"

#include <memory>
#include <string>

namespace autoware::motion::control::mpc_lateral_controller
{

/// Solver for QP problems using the CGMRES library
class QPSolverCGMRES : public QPSolverInterface
{
public:
  /**
   * @brief constructor
   */
  explicit QPSolverCGMRES(
    const rclcpp::Logger & logger, const std::string & log_dir,
    const cgmres::SolverSettings & solver_settings, const cgmres::Horizon & horizon);

  /**
   * @brief destructor
   */
  virtual ~QPSolverCGMRES() = default;

  bool solve(
    const Eigen::MatrixXd & h_mat, const Eigen::MatrixXd & f_vec, const Eigen::MatrixXd & a,
    const Eigen::VectorXd & lb, const Eigen::VectorXd & ub, const Eigen::VectorXd & lb_a,
    const Eigen::VectorXd & ub_a, Eigen::VectorXd & u) override;
  void updateEquation(const MPCTrajectory & resampled_ref_trajectory) override;
  bool solveCGMRES(const Eigen::VectorXd & x0, Eigen::VectorXd & u, const bool warm_start) override;

  int64_t getTakenIter() const override { return cgmressolver_.getTakenIter(); }
  double getRunTime() const override { return cgmressolver_.getRunTime(); }
  double getObjVal() const override { return cgmressolver_.getObjVal(); }

private:
  autoware::common::osqp::OSQPInterface cgmressolver_;
  cgmres::OCP_lateral_control ocp_;

  static constexpr int N = 50;  // Number of discretization grids of the horizon. Must be positive.
  static constexpr int kmax = 5;  // Maximum number of the GMRES iterations. Must be positive.
  static constexpr int kmax_init =
    1;  // Maximum number of the GMRES iterations for initializer. Must be positive.

  rclcpp::Logger logger_;
  cgmres::Logger cgmres_logger_;
  cgmres::SolverSettings settings_;
  std::shared_ptr<cgmres::OCP_lateral_control::ExternalReference> external_reference_;
  cgmres::SingleShootingCGMRESSolver<cgmres::OCP_lateral_control, N, kmax> mpc_;
  cgmres::ZeroHorizonOCPSolver<cgmres::OCP_lateral_control, kmax_init> initializer_;
  bool is_initialized_ = false;
  std::chrono::time_point<std::chrono::system_clock>
    initialized_time_;  // First MPC solution timestamp.
};
}  // namespace autoware::motion::control::mpc_lateral_controller
#endif  // AUTOWARE__MPC_LATERAL_CONTROLLER__QP_SOLVER__QP_SOLVER_CGMRES_HPP_
