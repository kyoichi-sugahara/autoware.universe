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

#include "nlp_interface/nlp_interface.hpp"

#include <functional>
#include <iostream>
#include <string>
#include <vector>

namespace autoware::common
{
void NLPInterface::initializeProblem(
  const std::function<double(const std::vector<double> &)> & objective,
  const std::function<std::vector<double>(const std::vector<double> &)> & constraints,
  const std::vector<double> & x0, const std::vector<double> & lbx, const std::vector<double> & ubx,
  const std::vector<double> & lbg, const std::vector<double> & ubg)
{
  // check if arguments are valid
  std::stringstream ss;
  if (x0.size() != lbx.size() || x0.size() != ubx.size()) {
    ss << "Inconsistent sizes for x0, lbx, and ubx. x0.size() = " << x0.size()
       << ", lbx.size() = " << lbx.size() << ", ubx.size() = " << ubx.size();
    throw std::invalid_argument(ss.str());
  }
  if (lbg.size() != ubg.size()) {
    ss << "Inconsistent sizes for lbg and ubg. lbg.size() = " << lbg.size()
       << ", ubg.size() = " << ubg.size();
    throw std::invalid_argument(ss.str());
  }

  initializeProblemImpl(objective, constraints, x0, lbx, ubx, lbg, ubg);

  variables_num_ = x0.size();
  constraints_num_ = lbg.size();
}

std::vector<double> NLPInterface::optimize(
  const std::function<double(const std::vector<double> &)> & objective,
  const std::function<std::vector<double>(const std::vector<double> &)> & constraints,
  const std::vector<double> & x0, const std::vector<double> & lbx, const std::vector<double> & ubx,
  const std::vector<double> & lbg, const std::vector<double> & ubg)
{
  initializeProblem(objective, constraints, x0, lbx, ubx, lbg, ubg);
  const auto result = optimizeImpl();

  return result;
}
}  // namespace autoware::common
