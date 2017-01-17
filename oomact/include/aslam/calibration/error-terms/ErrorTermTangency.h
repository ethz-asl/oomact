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

/** \file ErrorTermVelocities.h
    \brief This file defines the ErrorTermVelocities class, which implements an
           error term for velocities returned by a pose sensor.
  */

#ifndef ASLAM_CALIBRATION_ERROR_TERM_TANGENCY_H
#define ASLAM_CALIBRATION_ERROR_TERM_TANGENCY_H

#include <aslam/backend/ErrorTerm.hpp>
#include <aslam/backend/EuclideanExpression.hpp>
#include <aslam/calibration/error-terms/ErrorTermGroup.h>

namespace aslam {
  namespace calibration {

    /** The class ErrorTermVelocities implements an error term for velocities
        returned by a pose sensor such as the Applanix.
        \brief Velocities error term
      */
    class ErrorTermTangency: public aslam::backend::ErrorTermFs<3>, public ErrorTermGroupMember {
    public:
      /** \name Types definitions
        @{
        */
      /// Covariance type
      typedef Eigen::Matrix<double, 3, 3> Covariance;
      /** @}
        */

      ErrorTermTangency(const aslam::backend::EuclideanExpression& v_r_mr_cross_i,
                        const Covariance& sigma2_tangency_constraint, ErrorTermGroupReference etgr);

      virtual ~ErrorTermTangency();
      /** @}
        */

    protected:
      /** \name Protected methods
        @{
        */
      /// Evaluate the error term and return the weighted squared error
      virtual double evaluateErrorImplementation();
      /// Evaluate the Jacobians
      virtual void evaluateJacobiansImplementation(
        aslam::backend::JacobianContainer& _jacobians);
      /** @}
        */

      /** \name Protected members
        @{
        */
      /// Linear velocity cross versor i
      aslam::backend::EuclideanExpression _v_r_mr_cross_i;

      /// Covariance matrix of the constraint
      Covariance _sigma2_tangency_constraint;

      /** @}
        */
    };

  }
}

#endif // ASLAM_CALIBRATION_ERROR_TERM_TANGENCY_H
