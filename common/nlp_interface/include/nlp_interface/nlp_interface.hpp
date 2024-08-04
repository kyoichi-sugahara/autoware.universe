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

#ifndef NLP_INTERFACE__NLP_INTERFACE_HPP_
#define NLP_INTERFACE__NLP_INTERFACE_HPP_

#include <Eigen/Core>

#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace autoware::common
{
class NLPInterface
{
public:
  explicit NLPInterface(const bool enable_warm_start) : enable_warm_start_(enable_warm_start) {}

  virtual ~NLPInterface() = default;

  std::vector<double> optimize(
    const std::function<double(const std::vector<double> &)> & objective,
    const std::function<std::vector<double>(const std::vector<double> &)> & constraints,
    const std::vector<double> & x0, const std::vector<double> & lbx,
    const std::vector<double> & ubx, const std::vector<double> & lbg,
    const std::vector<double> & ubg);

  virtual bool isSolved() const = 0;
  virtual int getIterationNumber() const = 0;
  virtual std::string getStatus() const = 0;

  virtual void updateEpsAbs([[maybe_unused]] const double eps_abs) = 0;
  virtual void updateEpsRel([[maybe_unused]] const double eps_rel) = 0;
  virtual void updateVerbose([[maybe_unused]] const bool verbose) {}

protected:
  bool enable_warm_start_{false};

  void initializeProblem(
    const std::function<double(const std::vector<double> &)> & objective,
    const std::function<std::vector<double>(const std::vector<double> &)> & constraints,
    const std::vector<double> & x0, const std::vector<double> & lbx,
    const std::vector<double> & ubx, const std::vector<double> & lbg,
    const std::vector<double> & ubg);

  virtual void initializeProblemImpl(
    const std::function<double(const std::vector<double> &)> & objective,
    const std::function<std::vector<double>(const std::vector<double> &)> & constraints,
    const std::vector<double> & x0, const std::vector<double> & lbx,
    const std::vector<double> & ubx, const std::vector<double> & lbg,
    const std::vector<double> & ubg) = 0;

  virtual std::vector<double> optimizeImpl() = 0;

  std::optional<size_t> variables_num_{std::nullopt};
  std::optional<size_t> constraints_num_{std::nullopt};
};
}  // namespace autoware::common

#endif  // NLP_INTERFACE__NLP_INTERFACE_HPP_
