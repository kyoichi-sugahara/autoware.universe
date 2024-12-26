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

#ifndef NLP_INTERFACE__CGMRES_INTERFACE_HPP_
#define NLP_INTERFACE__CGMRES_INTERFACE_HPP_

#include "nlp_interface/nlp_interface.hpp"

#include <Eigen/Dense>

#include <functional>
#include <string>
#include <vector>

namespace autoware::common
{

class CGMRESInterface : public NLPInterface
{
public:
  explicit CGMRESInterface(const bool enable_warm_start);
  ~CGMRESInterface() override = default;

  bool isSolved() const override;
  int getIterationNumber() const override;
  std::string getStatus() const override;

  void updateEpsAbs(const double eps_abs) override;
  void updateEpsRel(const double eps_rel) override;
  void updateVerbose(const bool verbose) override;

  // CGMRES specific parameters
  void setHorizon(double horizon);
  void setTimeStep(double dt);
  void setMaxIterations(int max_iterations);

protected:
  void initializeProblemImpl(
    const std::function<double(const std::vector<double> &)> & objective,
    const std::function<std::vector<double>(const std::vector<double> &)> & constraints,
    const std::vector<double> & x0, const std::vector<double> & lbx,
    const std::vector<double> & ubx, const std::vector<double> & lbg,
    const std::vector<double> & ubg) override;

  std::vector<double> optimizeImpl() override;

private:
  // CGMRES specific members
  double horizon_;
  double dt_;
  int max_iterations_;
  bool is_solved_;
  int iteration_count_;
  std::string status_;

  // Problem specific members
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> system_dynamics_;
  std::function<double(const Eigen::VectorXd &, const Eigen::VectorXd &)> stage_cost_;
  std::function<double(const Eigen::VectorXd &)> terminal_cost_;
  Eigen::VectorXd initial_state_;
  Eigen::VectorXd lower_bound_;
  Eigen::VectorXd upper_bound_;

  // CGMRES algorithm implementation
  Eigen::VectorXd solveCGMRES();

  // Convergence check
  bool checkConvergence(const Eigen::VectorXd & solution);
};

}  // namespace autoware::common

#endif  // NLP_INTERFACE__CGMRES_INTERFACE_HPP_
