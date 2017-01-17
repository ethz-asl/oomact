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

#include "aslam/calibration/error-terms/ErrorTermTangency.h"

#include <Eigen/Dense>

using namespace aslam::backend;

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

  ErrorTermTangency::ErrorTermTangency(const aslam::backend::EuclideanExpression& v_r_mr_cross_i,
                                       const Covariance& sigma2_tangency_constraint, ErrorTermGroupReference etgr) :
      ErrorTermGroupMember(etgr),
        _v_r_mr_cross_i(v_r_mr_cross_i),
        _sigma2_tangency_constraint(sigma2_tangency_constraint) {
      setInvR(_sigma2_tangency_constraint.inverse());
      DesignVariable::set_t dv;
      _v_r_mr_cross_i.getDesignVariables(dv);
      setDesignVariablesIterator(dv.begin(), dv.end());
    }

    ErrorTermTangency::~ErrorTermTangency() {
    }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    double ErrorTermTangency::evaluateErrorImplementation() {
      error_t error;
      error = _v_r_mr_cross_i.toValue();
      setError(error);
      return evaluateChiSquaredError();
    }

    void ErrorTermTangency::evaluateJacobiansImplementation(JacobianContainer&
        jacobians) {
      _v_r_mr_cross_i.evaluateJacobians(jacobians);
    }

  }
}
