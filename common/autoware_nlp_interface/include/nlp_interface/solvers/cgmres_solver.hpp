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

#ifndef NLP_INTERFACE__SOLVERS__CGMRES_SOLVER_HPP_
#define NLP_INTERFACE__SOLVERS__CGMRES_SOLVER_HPP_

#include "nlp_interface/base/solver_interface.hpp"
#include "nlp_interface/types/cgmres_parameters.hpp"

#include <Eigen/Dense>

#include <functional>
#include <string>
#include <vector>

namespace autoware::nlp_interface::solvers
{
class CGMRESSolver final : public base::SolverInterface<types::CGMRESSolverSettings>
{
public:
  explicit CGMRESSolver(const types::CGMRESSolverSettings & params);

  // インターフェースの実装
  void optimize() override;

private:
  // パラメータバリデーション
  void validate_parameters() override;

  // 内部メソッド

  // 内部状態
  std::vector<double> workspace_;
  std::vector<double> gradient_;
  std::vector<double> direction_;
  double residual_norm_;
  bool is_initialized_;
};

}  // namespace autoware::nlp_interface::solvers

#endif  // NLP_INTERFACE__SOLVERS__CGMRES_SOLVER_HPP_
