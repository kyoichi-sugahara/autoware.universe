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

#ifndef MPC_LATERAL_CONTROLLER__QP_SOLVER__QP_SOLVER_CGMRES_HPP_
#define MPC_LATERAL_CONTROLLER__QP_SOLVER__QP_SOLVER_CGMRES_HPP_

#include "cgmres/horizon.hpp"
#include "cgmres/logger.hpp"
#include "cgmres/single_shooting_cgmres_solver.hpp"
#include "cgmres/solver_settings.hpp"
#include "cgmres/zero_horizon_ocp_solver.hpp"
#include "mpc_lateral_controller/mpc_cgmres.hpp"
#include "mpc_lateral_controller/qp_solver/qp_solver_interface.hpp"
#include "osqp_interface/osqp_interface.hpp"
#include "rclcpp/rclcpp.hpp"

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
  explicit QPSolverCGMRES(const rclcpp::Logger & logger, const std::string & log_dir);

  /**
   * @brief destructor
   */
  virtual ~QPSolverCGMRES() = default;

  /**
   * @brief solve QP problem : minimize j = u' * h_mat * u + f_vec' * u without constraint
   * @param [in] h_mat parameter matrix in object function
   * @param [in] f_vec parameter matrix in object function
   * @param [in] a parameter matrix for constraint lb_a < a*u < ub_a (not used here)
   * @param [in] lb parameter matrix for constraint lb < U < ub (not used here)
   * @param [in] ub parameter matrix for constraint lb < U < ub (not used here)
   * @param [in] lb_a parameter matrix for constraint lb_a < a*u < ub_a (not used here)
   * @param [in] ub_a parameter matrix for constraint lb_a < a*u < ub_a (not used here)
   * @param [out] u optimal variable vector
   * @return true if the problem was solved
   */
  bool solve(
    const Eigen::MatrixXd & h_mat, const Eigen::MatrixXd & f_vec, const Eigen::MatrixXd & a,
    const Eigen::VectorXd & lb, const Eigen::VectorXd & ub, const Eigen::VectorXd & lb_a,
    const Eigen::VectorXd & ub_a, Eigen::VectorXd & u) override;
  bool solveCGMRES(
    const Eigen::VectorXd & x0, const MPCTrajectory & resampled_ref_trajectory,
    double prediction_dt, Eigen::VectorXd & u, const int prediction_horizon,
    const bool warm_start) override;

  int64_t getTakenIter() const override { return cgmressolver_.getTakenIter(); }
  double getRunTime() const override { return cgmressolver_.getRunTime(); }
  double getObjVal() const override { return cgmressolver_.getObjVal(); }

private:
  autoware::common::osqp::OSQPInterface cgmressolver_;
  cgmres::OCP_lateral_control ocp_;

  static constexpr int N = 50;         // CGMRESソルバーの予測ステップ数
  static constexpr int kmax = 5;       // CGMRESソルバーの最大反復回数
  static constexpr int kmax_init = 1;  // 初期化用ソルバーの最大反復回数

  rclcpp::Logger logger_;
  cgmres::Logger cgmres_logger_;
  cgmres::SolverSettings settings_;
  cgmres::SingleShootingCGMRESSolver<cgmres::OCP_lateral_control, N, kmax> mpc_;
  cgmres::ZeroHorizonOCPSolver<cgmres::OCP_lateral_control, kmax_init> initializer_;
  bool is_initialized_ = false;
  std::chrono::time_point<std::chrono::system_clock>
    initialized_time_;  // First MPC solution timestamp.
};
}  // namespace autoware::motion::control::mpc_lateral_controller
#endif  // MPC_LATERAL_CONTROLLER__QP_SOLVER__QP_SOLVER_CGMRES_HPP_