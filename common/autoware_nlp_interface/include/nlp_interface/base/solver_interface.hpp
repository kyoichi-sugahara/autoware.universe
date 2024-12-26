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

#ifndef NLP_INTERFACE__BASE__SOLVER_INTERFACE_HPP_
#define NLP_INTERFACE__BASE__SOLVER_INTERFACE_HPP_

namespace autoware::nlp_interface::base
{

template <typename ParameterType, typename... OptimizationArgs>
class SolverInterface
{
public:
  explicit SolverInterface(const ParameterType & params) : params_(params)
  {
    validate_parameters();
  }
  virtual void optimize(OptimizationArgs &&... args) = 0;

protected:
  const ParameterType & get_parameters() const { return params_; }

private:
  virtual void validate_parameters() {}
  const ParameterType params_;
};

}  // namespace autoware::nlp_interface::base

#endif  // NLP_INTERFACE__BASE__SOLVER_INTERFACE_HPP_
