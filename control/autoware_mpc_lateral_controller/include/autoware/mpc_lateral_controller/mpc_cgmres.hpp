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

// This file was automatically generated by autogenu-jupyter
// (https://github.com/ohtsukalab/autogenu-jupyter). The autogenu-jupyter copyright holders make no
// ownership claim of its contents.

#ifndef AUTOWARE__MPC_LATERAL_CONTROLLER__MPC_CGMRES_HPP_
#define AUTOWARE__MPC_LATERAL_CONTROLLER__MPC_CGMRES_HPP_

#define _USE_MATH_DEFINES
#define CGMRES_EIGEN_CONST_CAST(TYPE, OBJ) const_cast<TYPE &>(OBJ.derived())

#include <Eigen/Core>

#include <array>
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <vector>

namespace cgmres
{

///
/// @class OCP_lateral_control
/// @brief Definition of the optimal control problem (OCP) of lateral_control.
///
class OCP_lateral_control
{
public:
  ///
  /// @brief Dimension of the state.
  ///
  static constexpr int nx = 3;

  ///
  /// @brief Dimension of the control input.
  ///
  static constexpr int nu = 1;

  ///
  /// @brief Dimension of the equality constraints.
  ///
  static constexpr int nc = 0;

  ///
  /// @brief Dimension of the Fischer-Burmeister function (already counded in nc).
  ///
  static constexpr int nh = 0;

  ///
  /// @brief Dimension of the concatenation of the control input and equality constraints.
  ///
  static constexpr int nuc = nu + nc;

  ///
  /// @brief Dimension of the bound constraints on the control input.
  ///
  static constexpr int nub = 1;

  double wheel_base;  // arctan(wheel base * curvature_in_reference_trajectory)
  double steer_tau;

  std::vector<double> curvature_ref_array;
  std::vector<double> v_ref_array;
  std::vector<std::array<double, nu>> u_ref_array;

  std::array<double, nx> q = {1.0, 0.1, 0.0};
  std::array<double, nx> q_terminal = {1.0, 0.1, 0.0};
  std::array<double, nx> x_ref = {0, 0, 0};
  std::array<double, nu> r = {1.0};

  static constexpr std::array<int, 1> ubound_indices = {0};
  std::array<double, 1> umin = {-5.0};
  std::array<double, 1> umax = {5.0};
  std::array<double, 1> dummy_weight = {0.1};

  void disp(std::ostream & os) const
  {
    os << "OCP_lateral_control:" << std::endl;
    os << "  nx:  " << nx << std::endl;
    os << "  nu:  " << nu << std::endl;
    os << "  nc:  " << nc << std::endl;
    os << "  nh:  " << nh << std::endl;
    os << "  nuc: " << nuc << std::endl;
    os << "  nub: " << nub << std::endl;
    os << "  wheel_base: " << wheel_base << std::endl;
    os << "  steer_tau: " << steer_tau << std::endl;
    Eigen::IOFormat fmt(4, 0, ", ", "", "[", "]");
    Eigen::IOFormat intfmt(1, 0, ", ", "", "[", "]");
    os << "  curvature_ref_array: "
       << Eigen::Map<const Eigen::VectorXd>(curvature_ref_array.data(), curvature_ref_array.size())
            .transpose()
            .format(fmt)
       << std::endl;
    os << "  v_ref_array: "
       << Eigen::Map<const Eigen::VectorXd>(v_ref_array.data(), v_ref_array.size())
            .transpose()
            .format(fmt)
       << std::endl;
    os << "  u_ref_array: "
       << Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>>(
            u_ref_array[0].data(), u_ref_array.size(), nu)
            .format(fmt)
       << std::endl;
    os << "  q: " << Eigen::Map<const Eigen::VectorXd>(q.data(), q.size()).transpose().format(fmt)
       << std::endl;
    os << "  q_terminal: "
       << Eigen::Map<const Eigen::VectorXd>(q_terminal.data(), q_terminal.size())
            .transpose()
            .format(fmt)
       << std::endl;
    os << "  x_ref: "
       << Eigen::Map<const Eigen::VectorXd>(x_ref.data(), x_ref.size()).transpose().format(fmt)
       << std::endl;
    os << "  r: " << Eigen::Map<const Eigen::VectorXd>(r.data(), r.size()).transpose().format(fmt)
       << std::endl;
    os << std::endl;
    os << "  ubound_indices: "
       << Eigen::Map<const Eigen::VectorXi>(ubound_indices.data(), ubound_indices.size())
            .transpose()
            .format(intfmt)
       << std::endl;
    os << "  umin: "
       << Eigen::Map<const Eigen::VectorXd>(umin.data(), umin.size()).transpose().format(fmt)
       << std::endl;
    os << "  umax: "
       << Eigen::Map<const Eigen::VectorXd>(umax.data(), umax.size()).transpose().format(fmt)
       << std::endl;
    os << "  dummy_weight: "
       << Eigen::Map<const Eigen::VectorXd>(dummy_weight.data(), dummy_weight.size())
            .transpose()
            .format(fmt)
       << std::endl;
  }

  friend std::ostream & operator<<(std::ostream & os, const OCP_lateral_control & ocp)
  {
    ocp.disp(os);
    return os;
  }

  ///
  /// @class ExternalReference
  /// @brief External reference of the lateral controller
  ///
  struct ExternalReference
  {
    std::vector<double> curvature_ref_array;
    std::vector<double> v_ref_array;
    std::vector<double> u_ref_array;
  };

  ///
  /// @brief Shared ptr to the external reference of the lateral controller.
  ///
  std::shared_ptr<ExternalReference> external_reference = nullptr;

  ///
  /// @brief Synchrozies the internal parameters of this OCP with the external references.
  /// This method is called at the beginning of each MPC update.
  ///
  void synchronize()
  {
    if (external_reference != nullptr) {
      curvature_ref_array = external_reference->curvature_ref_array;
      v_ref_array = external_reference->v_ref_array;
      u_ref_array.resize(curvature_ref_array.size());
      for (size_t i = 0; i < curvature_ref_array.size(); i++) {
        u_ref_array[i][0] = std::atan(curvature_ref_array[i] * wheel_base);
      }

    } else {
      // std::cerr << "external_reference is nullptr" << std::endl;
    }
  }

  ///
  /// @brief Computes the state equation dx = f(t, x, u).
  /// @param[in] t Time.
  /// @param[in] x State.
  /// @param[in] u Control input.
  /// @param[out] dx Evaluated value of the state equation.
  /// @remark This method is intended to be used inside of the cgmres solvers and does not check
  /// size of each argument. Use the overloaded method if you call this outside of the cgmres
  /// solvers.
  ///
  void eval_f(const double t, const int i, const double * x, const double * u, double * dx) const
  {
    dx[0] = v_ref_array[i] * sin(x[1]);
    dx[1] = -curvature_ref_array[i] * v_ref_array[i] * cos(x[1]) +
            v_ref_array[i] * tan(x[2]) / wheel_base;
    dx[2] = -(-u[0] + x[2]) / steer_tau;
  }

  ///
  /// @brief Computes the partial derivative of terminal cost with respect to state,
  /// i.e., phix = dphi/dx(t, x).
  /// @param[in] t Time.
  /// @param[in] x State.
  /// @param[out] phix Evaluated value of the partial derivative of terminal cost.
  /// @remark This method is intended to be used inside of the cgmres solvers and does not check
  /// size of each argument. Use the overloaded method if you call this outside of the cgmres
  /// solvers.
  ///
  void eval_phix(const double t, const double * x, double * phix) const
  {
    phix[0] = (1.0 / 2.0) * q_terminal[0] * (2 * x[0] - 2 * x_ref[0]);
    phix[1] = (1.0 / 2.0) * q_terminal[1] * (2 * x[1] - 2 * x_ref[1]);
    phix[2] = (1.0 / 2.0) * q_terminal[2] * (2 * x[2] - 2 * x_ref[2]);
  }

  ///
  /// @brief Computes the partial derivative of the Hamiltonian with respect to state,
  /// i.e., hx = dH/dx(t, x, u, lmd).
  /// @param[in] t Time.
  /// @param[in] x State.
  /// @param[in] u Concatenatin of the control input and Lagrange multiplier with respect to the
  /// equality constraints.
  /// @param[in] lmd Costate.
  /// @param[out] hx Evaluated value of the partial derivative of the Hamiltonian.
  /// @remark This method is intended to be used inside of the cgmres solvers and does not check
  /// size of each argument. Use the overloaded method if you call this outside of the cgmres
  /// solvers.
  ///
  void eval_hx(
    const double t, const int i, const double * x, const double * u, const double * lmd,
    double * hx) const
  {
    const double v_ref = v_ref_array[i];
    const double curvature = curvature_ref_array[i];
    const double tan_x2 = tan(x[2]);
    const double sec_x2_squared = 1.0 + tan_x2 * tan_x2;

    hx[0] = q[0] * (x[0] - x_ref[0]);
    hx[1] = curvature * lmd[1] * v_ref * sin(x[1]) + lmd[0] * v_ref * cos(x[1]) +
            q[1] * (x[1] - x_ref[1]);
    hx[2] =
      -lmd[2] / steer_tau + q[2] * (x[2] - x_ref[2]) + lmd[1] * v_ref * sec_x2_squared / wheel_base;
  }

  ///
  /// @brief Computes the partial derivative of the Hamiltonian with respect to control input and
  /// the equality constraints, i.e., hu = dH/du(t, x, u, lmd).
  /// @param[in] t Time.
  /// @param[in] x State.
  /// @param[in] u Concatenatin of the control input and Lagrange multiplier with respect to the
  /// equality constraints.
  /// @param[in] lmd Costate.
  /// @param[out] hu Evaluated value of the partial derivative of the Hamiltonian.
  /// @remark This method is intended to be used inside of the cgmres solvers and does not check
  /// size of each argument. Use the overloaded method if you call this outside of the cgmres
  /// solvers.
  ///
  void eval_hu(
    const double t, const int i, const double * x, const double * u, const double * lmd,
    double * hu) const
  {
    hu[0] = lmd[2] / steer_tau + (1.0 / 2.0) * r[0] * (2 * u[0] - 2 * u_ref_array[i][0]);
  }
};

}  // namespace cgmres

#endif  // AUTOWARE__MPC_LATERAL_CONTROLLER__MPC_CGMRES_HPP_