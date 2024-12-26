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

#include "nlp_interface/solvers/cgmres_solver.hpp"

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::nlp_interface::solvers
{

CGMRESSolver::CGMRESSolver(const types::CGMRESSolverSettings & params)
: base::SolverInterface<types::CGMRESSolverSettings>(params),
  workspace_(),
  gradient_(),
  direction_(),
  residual_norm_(0.0),
  is_initialized_(false)
{
}

void CGMRESSolver::optimize()
{
  if (!is_initialized_) {
    // Initialize workspace and other vectors if needed
    // Size should be determined from the problem dimensions in params
    // workspace_.resize(/* appropriate size */);
    // gradient_.resize(/* appropriate size */);
    // direction_.resize(/* appropriate size */);
    is_initialized_ = true;
  }

  // CGMRES algorithm implementation
  // 1. Calculate gradient
  // 2. Determine search direction
  // 3. Update solution
  // 4. Check convergence

  // This is where you would implement the actual CGMRES algorithm
  // using workspace_, gradient_, and direction_ vectors
}

void CGMRESSolver::validate_parameters()
{
  // Validate solver specific parameters
}

}  // namespace autoware::nlp_interface::solvers
