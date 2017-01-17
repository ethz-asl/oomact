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

#include "aslam/calibration/error-terms/ErrorTermWheelsZ.h"

#include <Eigen/Dense>

using namespace aslam::backend;

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

  ErrorTermWheelsZ::ErrorTermWheelsZ(const aslam::backend::EuclideanExpression& v_w_mwl,
                                     const aslam::backend::EuclideanExpression& v_w_mwr,
                                     const Covariance& sigma2_vert_vel) :
        _v_w_mwl(v_w_mwl),
        _v_w_mwr(v_w_mwr),
        _sigma2_vert_vel(sigma2_vert_vel) {
      setInvR(_sigma2_vert_vel.inverse());
      DesignVariable::set_t dv;
      _v_w_mwl.getDesignVariables(dv);
      _v_w_mwr.getDesignVariables(dv);
      setDesignVariablesIterator(dv.begin(), dv.end());
    }

  ErrorTermWheelsZ::ErrorTermWheelsZ(const ErrorTermWheelsZ& other) :
        ErrorTermFs<1>(other),
        _v_w_mwl(other. _v_w_mwl),
        _v_w_mwr(other. _v_w_mwr),
        _sigma2_vert_vel(other._sigma2_vert_vel){
    }

  ErrorTermWheelsZ& ErrorTermWheelsZ::operator =
        (const ErrorTermWheelsZ& other) {
      if (this != &other) {
        ErrorTermFs<1>::operator=(other);
        _v_w_mwl = other._v_w_mwl;
        _v_w_mwr = other._v_w_mwr;
       _sigma2_vert_vel = other._sigma2_vert_vel;
      }
      return *this;
    }

    ErrorTermWheelsZ::~ErrorTermWheelsZ() {
    }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    double ErrorTermWheelsZ::evaluateErrorImplementation() {
      error_t error;
      const double vzl = _v_w_mwl.toEuclidean()(2);
      const double vzr = _v_w_mwr.toEuclidean()(2);

      error(0) = vzl + vzr;

      setError(error);
      return evaluateChiSquaredError();
    }

    void ErrorTermWheelsZ::evaluateJacobiansImplementation(JacobianContainer&
        jacobians) {
      Eigen::Matrix<double, 1, 3> Jl = Eigen::Matrix<double, 1, 3>::Zero();
      Eigen::Matrix<double, 1, 3> Jr = Eigen::Matrix<double, 1, 3>::Zero();


      Jl(2) = 1.0;
      Jr(2) = 1.0;

      _v_w_mwl.evaluateJacobians(jacobians, Jl);
      _v_w_mwr.evaluateJacobians(jacobians,Jr);
    }

  }
}
