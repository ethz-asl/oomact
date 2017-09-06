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

#include "aslam/calibration/error-terms/ErrorTermPosition.h"

#include <Eigen/Dense>

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

    ErrorTermPosition::ErrorTermPosition(const aslam::backend::EuclideanExpression& p,
        const Eigen::Vector3d & pm,
        const Covariance& sigma2,
        ErrorTermGroupReference etgr) :
        ErrorTermGroupMember(etgr),
        pm_(pm),p_(p) {
      setInvR(sigma2.inverse());
      aslam::backend::DesignVariable::set_t dv;
      p_.getDesignVariables(dv);
      setDesignVariablesIterator(dv.begin(), dv.end());
    }

    ErrorTermPosition::ErrorTermPosition(const aslam::backend::EuclideanExpression& p, const PositionMeasurement& pm, const Covariance& sigma2, ErrorTermGroupReference etgr) :
      ErrorTermPosition(p, pm.p, sigma2, etgr) {
    }

    double ErrorTermPosition::evaluateErrorImplementation() {
      const auto p = p_.evaluate();
      setError(p-pm_);
      return evaluateChiSquaredError();
    }

    void ErrorTermPosition::evaluateJacobiansImplementation(aslam::backend::JacobianContainer& jacobians) {
      p_.evaluateJacobians(jacobians);
    }

  }
}

Eigen::VectorXd aslam::calibration::ErrorTermPosition::getPrediction() const {
  return p_.evaluate();
}


Eigen::VectorXd aslam::calibration::ErrorTermPosition::getMeasurement() const {
  return pm_;
}
