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

#ifndef AUTOWARE_MPC_LATERAL_CONTROLLER__MPC_CGMRES_HPP_
#define AUTOWARE_MPC_LATERAL_CONTROLLER__MPC_CGMRES_HPP_

#define _USE_MATH_DEFINES

#include "cgmres/detail/macros.hpp"
#include "cgmres/types.hpp"

#include <array>
#include <cmath>
#include <iostream>
#include <memory>
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

  double v_in_reference_trajectory = 1.0;
  double curvature_in_reference_trajectory = -0.5;
  double smoothed_curvature_in_reference_trajectory = -0.5;
  double wheel_base = 2.74;  // arctan(wheel base * curvature_in_reference_trajectory)
  double tau_ = 0.3;

  std::vector<double> curvature_ref_array;
  std::vector<double> v_ref_array;
  std::vector<std::array<double, nu>> u_ref_array;

  std::array<double, nx> q = {1.0, 0.1, 0.0};
  std::array<double, nx> q_terminal = {1.0, 0.1, 0.0};
  std::array<double, nx> x_ref = {0, 0, 0};
  std::array<double, nu> u_ref = {-0.948853649067464};
  std::array<double, nu> r = {1.0};

  static constexpr std::array<int, 1> ubound_indices = {0};
  std::array<double, 1> umin = {-15.0};
  std::array<double, 1> umax = {15.0};
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
    os << std::endl;
    os << "  v_in_reference_trajectory: " << v_in_reference_trajectory << std::endl;
    os << "  curvature_in_reference_trajectory: " << curvature_in_reference_trajectory << std::endl;
    os << "  smoothed_curvature_in_reference_trajectory: "
       << smoothed_curvature_in_reference_trajectory << std::endl;
    os << "  wheel_base: " << wheel_base << std::endl;
    os << "  tau: " << tau_ << std::endl;
    os << std::endl;
    Eigen::IOFormat fmt(4, 0, ", ", "", "[", "]");
    Eigen::IOFormat intfmt(1, 0, ", ", "", "[", "]");
    os << "  q: " << Map<const VectorX>(q.data(), q.size()).transpose().format(fmt) << std::endl;
    os << "  q_terminal: "
       << Map<const VectorX>(q_terminal.data(), q_terminal.size()).transpose().format(fmt)
       << std::endl;
    os << "  x_ref: " << Map<const VectorX>(x_ref.data(), x_ref.size()).transpose().format(fmt)
       << std::endl;
    os << "  u_ref: " << Map<const VectorX>(u_ref.data(), u_ref.size()).transpose().format(fmt)
       << std::endl;
    os << "  r: " << Map<const VectorX>(r.data(), r.size()).transpose().format(fmt) << std::endl;
    os << std::endl;
    os << "  ubound_indices: "
       << Map<const VectorXi>(ubound_indices.data(), ubound_indices.size())
            .transpose()
            .format(intfmt)
       << std::endl;
    os << "  umin: " << Map<const VectorX>(umin.data(), umin.size()).transpose().format(fmt)
       << std::endl;
    os << "  umax: " << Map<const VectorX>(umax.data(), umax.size()).transpose().format(fmt)
       << std::endl;
    os << "  dummy_weight: "
       << Map<const VectorX>(dummy_weight.data(), dummy_weight.size()).transpose().format(fmt)
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
      u_ref_array.reserve(curvature_ref_array.size());
      for (size_t i = 0; i < curvature_ref_array.size(); i++) {
        u_ref_array[i][0] = std::atan(curvature_ref_array[i] * wheel_base);
      }
      curvature_in_reference_trajectory = curvature_ref_array[0];
      v_in_reference_trajectory = v_ref_array[0];
      u_ref[0] = u_ref_array[0][0];

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
  void eval_f(const double t, const double * x, const double * u, double * dx) const
  {
    dx[0] = v_in_reference_trajectory * sin(x[1]);
    dx[1] = -curvature_in_reference_trajectory * v_in_reference_trajectory * cos(x[1]) +
            v_in_reference_trajectory * tan(x[2]) / wheel_base;
    dx[2] = -(-u[0] + x[2]) / tau_;
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
    dx[2] = -(-u[0] + x[2]) / tau_;
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
    const double t, const double * x, const double * u, const double * lmd, double * hx) const
  {
    const double x0 = lmd[1] * v_in_reference_trajectory;
    hx[0] = (1.0 / 2.0) * q[0] * (2 * x[0] - 2 * x_ref[0]);
    hx[1] = curvature_in_reference_trajectory * x0 * sin(x[1]) +
            lmd[0] * v_in_reference_trajectory * cos(x[1]) +
            (1.0 / 2.0) * q[1] * (2 * x[1] - 2 * x_ref[1]);
    hx[2] = -lmd[2] / tau_ + (1.0 / 2.0) * q[2] * (2 * x[2] - 2 * x_ref[2]) +
            x0 * (pow(tan(x[2]), 2) + 1) / wheel_base;
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
    const double x0 = lmd[1] * v_ref_array[i];
    hx[0] = (1.0 / 2.0) * q[0] * (2 * x[0] - 2 * x_ref[0]);
    hx[1] = curvature_ref_array[i] * x0 * sin(x[1]) + lmd[0] * v_ref_array[i] * cos(x[1]) +
            (1.0 / 2.0) * q[1] * (2 * x[1] - 2 * x_ref[1]);
    hx[2] = -lmd[2] / tau_ + (1.0 / 2.0) * q[2] * (2 * x[2] - 2 * x_ref[2]) +
            x0 * (pow(tan(x[2]), 2) + 1) / wheel_base;
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
    const double t, const double * x, const double * u, const double * lmd, double * hu) const
  {
    hu[0] = lmd[2] / tau_ + (1.0 / 2.0) * r[0] * (2 * u[0] - 2 * u_ref[0]);
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
    hu[0] = lmd[2] / tau_ + (1.0 / 2.0) * r[0] * (2 * u[0] - 2 * u_ref_array[i][0]);
  }

  ///
  /// @brief Computes the state equation dx = f(t, x, u).
  /// @param[in] t Time.
  /// @param[in] x State. Size must be nx.
  /// @param[in] u Control input. Size must be nu.
  /// @param[out] dx Evaluated value of the state equation. Size must be nx.
  ///
  template <typename VectorType1, typename VectorType2, typename VectorType3>
  void eval_f(
    const double t, const MatrixBase<VectorType1> & x, const MatrixBase<VectorType2> & u,
    const MatrixBase<VectorType3> & dx, const int i = 0) const
  {
    if (x.size() != nx) {
      throw std::invalid_argument("[OCP]: x.size() must be " + std::to_string(nx));
    }
    if (u.size() != nu) {
      throw std::invalid_argument("[OCP]: u.size() must be " + std::to_string(nu));
    }
    if (dx.size() != nx) {
      throw std::invalid_argument("[OCP]: dx.size() must be " + std::to_string(nx));
    }
    if (i == 0) {
      eval_f(
        t, x.derived().data(), u.derived().data(), CGMRES_EIGEN_CONST_CAST(VectorType3, dx).data());
    } else {
      eval_f(
        t, i, x.derived().data(), u.derived().data(),
        CGMRES_EIGEN_CONST_CAST(VectorType3, dx).data());
    }
  }

  ///
  /// @brief Computes the partial derivative of terminal cost with respect to state,
  /// i.e., phix = dphi/dx(t, x).
  /// @param[in] t Time.
  /// @param[in] x State. Size must be nx.
  /// @param[out] phix Evaluated value of the partial derivative of terminal cost. Size must be nx.
  ///
  template <typename VectorType1, typename VectorType2>
  void eval_phix(
    const double t, const MatrixBase<VectorType1> & x, const MatrixBase<VectorType2> & phix) const
  {
    if (x.size() != nx) {
      throw std::invalid_argument("[OCP]: x.size() must be " + std::to_string(nx));
    }
    if (phix.size() != nx) {
      throw std::invalid_argument("[OCP]: phix.size() must be " + std::to_string(nx));
    }
    eval_phix(t, x.derived().data(), CGMRES_EIGEN_CONST_CAST(VectorType2, phix).data());
  }

  ///
  /// @brief Computes the partial derivative of the Hamiltonian with respect to the state,
  /// i.e., hx = dH/dx(t, x, u, lmd).
  /// @param[in] t Time.
  /// @param[in] x State. Size must be nx.
  /// @param[in] uc Concatenatin of the control input and Lagrange multiplier with respect to the
  /// equality constraints. Size must be nuc.
  /// @param[in] lmd Costate.  Size must be nx.
  /// @param[out] hx Evaluated value of the partial derivative of the Hamiltonian. Size must be nx.
  ///
  template <typename VectorType1, typename VectorType2, typename VectorType3, typename VectorType4>
  void eval_hx(
    const double t, const MatrixBase<VectorType1> & x, const MatrixBase<VectorType2> & uc,
    const MatrixBase<VectorType3> & lmd, const MatrixBase<VectorType4> & hx, const int i = 0) const
  {
    if (x.size() != nx) {
      throw std::invalid_argument("[OCP]: x.size() must be " + std::to_string(nx));
    }
    if (uc.size() != nuc) {
      throw std::invalid_argument("[OCP]: uc.size() must be " + std::to_string(nuc));
    }
    if (lmd.size() != nx) {
      throw std::invalid_argument("[OCP]: lmd.size() must be " + std::to_string(nx));
    }
    if (hx.size() != nuc) {
      throw std::invalid_argument("[OCP]: hx.size() must be " + std::to_string(nx));
    }
    if (i == 0) {
      eval_hx(
        t, x.derived().data(), uc.derived().data(), lmd.derived().data(),
        CGMRES_EIGEN_CONST_CAST(VectorType4, hx).data());
    } else {
      eval_hx(
        t, i, x.derived().data(), uc.derived().data(), lmd.derived().data(),
        CGMRES_EIGEN_CONST_CAST(VectorType4, hx).data());
    }
  }

  ///
  /// @brief Computes the partial derivative of the Hamiltonian with respect to control input and
  /// the equality constraints, i.e., hu = dH/du(t, x, u, lmd).
  /// @param[in] t Time.
  /// @param[in] x State. Size must be nx.
  /// @param[in] uc Concatenatin of the control input and Lagrange multiplier with respect to the
  /// equality constraints. Size must be nuc.
  /// @param[in] lmd Costate. Size must be nx.
  /// @param[out] hu Evaluated value of the partial derivative of the Hamiltonian. Size must be nuc.
  ///
  template <typename VectorType1, typename VectorType2, typename VectorType3, typename VectorType4>
  void eval_hu(
    const double t, const MatrixBase<VectorType1> & x, const MatrixBase<VectorType2> & uc,
    const MatrixBase<VectorType3> & lmd, const MatrixBase<VectorType4> & hu) const
  {
    if (x.size() != nx) {
      throw std::invalid_argument("[OCP]: x.size() must be " + std::to_string(nx));
    }
    if (uc.size() != nuc) {
      throw std::invalid_argument("[OCP]: uc.size() must be " + std::to_string(nuc));
    }
    if (lmd.size() != nx) {
      throw std::invalid_argument("[OCP]: lmd.size() must be " + std::to_string(nx));
    }
    if (hu.size() != nuc) {
      throw std::invalid_argument("[OCP]: hu.size() must be " + std::to_string(nuc));
    }
    eval_hu(
      t, x.derived().data(), uc.derived().data(), lmd.derived().data(),
      CGMRES_EIGEN_CONST_CAST(VectorType4, hu).data());
  }
};

}  // namespace cgmres

#endif  // AUTOWARE_MPC_LATERAL_CONTROLLER__MPC_CGMRES_HPP_
