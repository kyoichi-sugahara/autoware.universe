// Copyright 2023 TIER IV, Inc.
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

#include "nlp_interface/cgmres_interface.hpp"

#include <cmath>
#include <stdexcept>

namespace autoware::common
{

CGMRESInterface::CGMRESInterface(const bool enable_warm_start)
: NLPInterface(enable_warm_start),
  horizon_(10.0),
  dt_(0.1),
  max_iterations_(100),
  is_solved_(false),
  iteration_count_(0),
  status_("Not solved")
{
}

bool CGMRESInterface::isSolved() const
{
  return is_solved_;
}

int CGMRESInterface::getIterationNumber() const
{
  return iteration_count_;
}

std::string CGMRESInterface::getStatus() const
{
  return status_;
}

void CGMRESInterface::updateEpsAbs(const double /* eps_abs */)
{
  // Implement if needed
}

void CGMRESInterface::updateEpsRel(const double /* eps_rel */)
{
  // Implement if needed
}

void CGMRESInterface::updateVerbose(const bool /* verbose */)
{
  // Implement if needed
}

void CGMRESInterface::setHorizon(double horizon)
{
  horizon_ = horizon;
}

void CGMRESInterface::setTimeStep(double dt)
{
  dt_ = dt;
}

void CGMRESInterface::setMaxIterations(int max_iterations)
{
  max_iterations_ = max_iterations;
}

void CGMRESInterface::initializeProblemImpl(
  const std::function<double(const std::vector<double> &)> & /* objective */,
  const std::function<std::vector<double>(const std::vector<double> &)> & /* constraints */,
  const std::vector<double> & x0, const std::vector<double> & lbx, const std::vector<double> & ubx,
  const std::vector<double> & /* lbg */, const std::vector<double> & /* ubg */)
{
  // Convert the problem to CGMRES format
  // This is a simplified example and may need to be adjusted based on your specific requirements
  initial_state_ = Eigen::Map<const Eigen::VectorXd>(x0.data(), x0.size());
  lower_bound_ = Eigen::Map<const Eigen::VectorXd>(lbx.data(), lbx.size());
  upper_bound_ = Eigen::Map<const Eigen::VectorXd>(ubx.data(), ubx.size());

  // Set up system dynamics, stage cost, and terminal cost functions
  // These would need to be derived from the given objective and constraints
  // The exact implementation will depend on your problem formulation
}

std::vector<double> CGMRESInterface::optimizeImpl()
{
  Eigen::VectorXd solution = solveCGMRES();

  std::vector<double> result(solution.data(), solution.data() + solution.size());
  return result;
}

Eigen::VectorXd CGMRESInterface::solveCGMRES()
{
  // Implement the CGMRES algorithm here
  // This is a placeholder implementation
  Eigen::VectorXd solution = Eigen::VectorXd::Zero(variables_num_.value());

  for (int i = 0; i < max_iterations_; ++i) {
    // Perform CGMRES iteration
    // Update solution

    iteration_count_ = i + 1;

    // Check for convergence
    if (checkConvergence(solution)) {  // 実際の収束判定関数を実装する必要があります
      is_solved_ = true;
      status_ = "Optimal";
      break;
    }
  }

  if (!is_solved_) {
    status_ = "Max iterations reached";
  }

  return solution;
}

// 収束判定関数の例（実際のアプリケーションに合わせて実装する必要があります）
bool CGMRESInterface::checkConvergence(const Eigen::VectorXd & solution)
{
  // 実装例：ソリューションの変化が十分小さいかどうかをチェック
  static Eigen::VectorXd prev_solution;
  if (prev_solution.size() == 0) {
    prev_solution = solution;
    return false;
  }

  double change = (solution - prev_solution).norm();
  prev_solution = solution;

  return change < 1e-6;  // 収束判定の閾値
}

}  // namespace autoware::common
