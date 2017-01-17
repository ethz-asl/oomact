/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 *                                                                            *
 * This program is free software; you can redistribute it and/or modify       *
 * it under the terms of the Lesser GNU General Public License as published by*
 * the Free Software Foundation; either version 3 of the License, or          *
 * (at your option) any later version.                                        *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the              *
 * Lesser GNU General Public License for more details.                        *
 *                                                                            *
 * You should have received a copy of the Lesser GNU General Public License   *
 * along with this program. If not, see <http://www.gnu.org/licenses/>.       *
 ******************************************************************************/

#include "aslam/calibration/error-terms/ErrorTermLinearVelocity.h"

#include <Eigen/Dense>

using namespace aslam::backend;

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

    ErrorTermLinearVelocity::ErrorTermLinearVelocity(const EuclideanExpression& r_v_mr,
                                                     const EuclideanExpression& r_v_mr_m,
                                                     const Covariance& sigma2_v) :
        _r_v_mr(r_v_mr),
        _r_v_mr_m(r_v_mr_m),
        _sigma2_v(sigma2_v){
      setInvR(sigma2_v.inverse());
      DesignVariable::set_t dv;
      _r_v_mr.getDesignVariables(dv);
      _r_v_mr_m.getDesignVariables(dv);
      setDesignVariablesIterator(dv.begin(), dv.end());
    }

    ErrorTermLinearVelocity::ErrorTermLinearVelocity(const ErrorTermLinearVelocity& other) :
        ErrorTermFs<3>(other),
        _r_v_mr(other._r_v_mr),
        _r_v_mr_m(other._r_v_mr_m),
        _sigma2_v(other._sigma2_v){
    }

    ErrorTermLinearVelocity& ErrorTermLinearVelocity::operator =
        (const ErrorTermLinearVelocity& other) {
      if (this != &other) {
        ErrorTermFs<3>::operator=(other);
       _r_v_mr = other._r_v_mr;
       _r_v_mr_m = other._r_v_mr_m;
       _sigma2_v = other._sigma2_v;
      }
      return *this;
    }

    ErrorTermLinearVelocity::~ErrorTermLinearVelocity() {
    }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    double ErrorTermLinearVelocity::evaluateErrorImplementation() {
      error_t error;
      error = _r_v_mr_m.toValue() - _r_v_mr.toValue();
      setError(error);
      return evaluateChiSquaredError();
    }

    void ErrorTermLinearVelocity::evaluateJacobiansImplementation(JacobianContainer&
        jacobians) {
      Eigen::Matrix3d Jv = Eigen::Matrix3d::Identity();
      _r_v_mr.evaluateJacobians(jacobians, -Jv);
      Eigen::Matrix3d Jvm = Eigen::Matrix3d::Identity();
      _r_v_mr_m.evaluateJacobians(jacobians, Jvm);
    }

  }
}
