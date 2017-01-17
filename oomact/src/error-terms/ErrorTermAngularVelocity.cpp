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

#include "aslam/calibration/error-terms/ErrorTermAngularVelocity.h"

#include <Eigen/Dense>

using namespace aslam::backend;

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

  ErrorTermAngularVelocity::ErrorTermAngularVelocity(const aslam::backend::EuclideanExpression& r_w_mr,
                                                     const aslam::backend::EuclideanExpression& r_w_mr_m,
                                                     const Covariance& sigma2_ang_velZ) :
        _r_w_mr(r_w_mr),
        _r_w_mr_m(r_w_mr_m),
        _sigma2_ang_velZ(sigma2_ang_velZ) {
      setInvR(_sigma2_ang_velZ.inverse());
      DesignVariable::set_t dv;
      _r_w_mr.getDesignVariables(dv);
      _r_w_mr_m.getDesignVariables(dv);
      setDesignVariablesIterator(dv.begin(), dv.end());
    }

  ErrorTermAngularVelocity::ErrorTermAngularVelocity(const ErrorTermAngularVelocity& other) :
        ErrorTermFs<1>(other),
        _r_w_mr(other. _r_w_mr),
        _r_w_mr_m(other. _r_w_mr_m),
        _sigma2_ang_velZ(other._sigma2_ang_velZ){
    }

  ErrorTermAngularVelocity& ErrorTermAngularVelocity::operator =
        (const ErrorTermAngularVelocity& other) {
      if (this != &other) {
        ErrorTermFs<1>::operator=(other);
        _r_w_mr = other._r_w_mr;
        _r_w_mr_m = other._r_w_mr_m;
       _sigma2_ang_velZ = other._sigma2_ang_velZ;
      }
      return *this;
    }

    ErrorTermAngularVelocity::~ErrorTermAngularVelocity() {
    }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    double ErrorTermAngularVelocity::evaluateErrorImplementation() {
      error_t error;
      const double wz = _r_w_mr.toEuclidean()(2);
      const double wzm = _r_w_mr_m.toEuclidean()(2);

      error(0) = wz - wzm;

      setError(error);
      return evaluateChiSquaredError();
    }

    void ErrorTermAngularVelocity::evaluateJacobiansImplementation(JacobianContainer&
        jacobians) {
      Eigen::Matrix<double, 1, 3> Jw = Eigen::Matrix<double, 1, 3>::Zero();
      Eigen::Matrix<double, 1, 3> Jwm = Eigen::Matrix<double, 1, 3>::Zero();


      Jw(2) = 1.0;
      Jwm(2) = 1.0;

      _r_w_mr.evaluateJacobians(jacobians, Jw);
      _r_w_mr_m.evaluateJacobians(jacobians, -Jwm);
    }

  }
}
