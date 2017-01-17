/******************************************************************************
 * Copyright (C) 2014 by Jerome Maye                                          *
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

#include "aslam/calibration/error-terms/ErrorTermWheel.h"

#include <cmath>

#include <Eigen/Dense>

using namespace aslam::backend;

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

    ErrorTermWheel::ErrorTermWheel(const EuclideanExpression& v_w_mw, const
        ScalarExpression& r, double measurement, const Covariance& sigma2Wheel) :
        _v_w_mw(v_w_mw),
        _r(r),
        _measurement(measurement),
        _sigma2Wheel(sigma2Wheel){
      setInvR((Eigen::Matrix<double, 1, 1>() << (1/ _sigma2Wheel)).finished());
      DesignVariable::set_t dv;
      _v_w_mw.getDesignVariables(dv);
      _r.getDesignVariables(dv);
      setDesignVariablesIterator(dv.begin(), dv.end());
    }

    ErrorTermWheel::ErrorTermWheel(const ErrorTermWheel& other) :
        ErrorTermFs<1>(other),
        _v_w_mw(other._v_w_mw),
        _r(other._r),
        _measurement(other._measurement),
        _sigma2Wheel(other._sigma2Wheel){
    }

    ErrorTermWheel& ErrorTermWheel::operator = (const ErrorTermWheel& other) {
      if (this != &other) {
        ErrorTermFs<1>::operator=(other);
       _v_w_mw= other._v_w_mw;
       _r = other._r;
       _measurement = other._measurement;
       _sigma2Wheel = other._sigma2Wheel;
      }
      return *this;
    }

    ErrorTermWheel::~ErrorTermWheel() {
    }

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

    double ErrorTermWheel::getMeasurement() const {
      return _measurement;
    }

    void ErrorTermWheel::setMeasurement(double measurement) {
      _measurement = measurement;
    }

    const ErrorTermWheel::Covariance& ErrorTermWheel::getCovariance() const {
      return _sigma2Wheel;
    }

    void ErrorTermWheel::setCovariance(const Covariance& sigma2Wheel) {
      _sigma2Wheel = sigma2Wheel;
    }


/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    double ErrorTermWheel::evaluateErrorImplementation() {
      error_t error;
      const double v0 = _v_w_mw.toEuclidean()(0);
      const double r = _r.toScalar();
      error(0) = v0/r - _measurement;
      setError(error);
      return evaluateChiSquaredError();
    }

    void ErrorTermWheel::evaluateJacobiansImplementation(JacobianContainer&
        jacobians) {
      const double r = _r.toScalar();
      const double v0 = _v_w_mw.toValue()(0);

      Eigen::Matrix<double, 1, 3> J_w_v_mw = Eigen::Matrix<double, 1, 3>::Zero();
      J_w_v_mw(0,0) = 1.0/r;
      Eigen::Matrix<double, 1, 1> J_r;
      J_r(0,0) = -v0/(r*r);
      _v_w_mw.evaluateJacobians(jacobians, J_w_v_mw);
      _r.evaluateJacobians(jacobians, J_r);
    }

  }
}
