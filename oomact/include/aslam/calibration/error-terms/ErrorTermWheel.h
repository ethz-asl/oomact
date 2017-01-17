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

/** \file ErrorTermWheel.h
    \brief This file defines the ErrorTermWheel class, which implements an error
           term for a wheel.
  */

#ifndef ASLAM_CALIBRATION_CAR_ERROR_TERM_WHEEL_H
#define ASLAM_CALIBRATION_CAR_ERROR_TERM_WHEEL_H

#include <aslam/backend/ErrorTerm.hpp>
#include <aslam/backend/EuclideanExpression.hpp>
#include <aslam/backend/ScalarExpression.hpp>

namespace aslam {
  namespace calibration {

    /** The class ErrorTermWheel implements an error term for a wheel.
        \brief Wheel error term
      */
    class ErrorTermWheel :
      public aslam::backend::ErrorTermFs<1> {
    public:
      // Required by Eigen for fixed-size matrices members
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      /** \name Types definitions
        @{
        */
      /// Covariance type
      typedef double Covariance;
      /** @}
        */

      /** \name Constructors/destructor
        @{
        */
      /**
       * Constructs the error term from input data and design variable.
       * \brief Constructs the error term
       *
       * @param w_v_mw \f$^{v}\mathbf{v}^{mw}\f$ linear velocity of
       *   the wheel w.r. to mapping frame, expressed in wheel frame (which is equal to v)
       * @param r scaling factor (radius)
       * @param measurement odometry measurement (rad/s) (\f$v\f$)
       * @param sigma2Wheel covariance matrix of the odometry measurement
       * @param frontEnabled front wheel flag
       */
      ErrorTermWheel(const aslam::backend::EuclideanExpression& v_w_mw, const
        aslam::backend::ScalarExpression& r, double measurement, const Covariance& sigma2Wheel);
      /// Copy constructor
      ErrorTermWheel(const ErrorTermWheel& other);
      /// Assignment operator
      ErrorTermWheel& operator = (const ErrorTermWheel& other);
      /// Destructor
      virtual ~ErrorTermWheel();
      /** @}
        */

      /** \name Accessors
        @{
        */
      /// Returns the measurement
      double getMeasurement() const;
      /// Sets the measurement
      void setMeasurement(double measurement);
      /// Returns the covariance of the measurement
      const Covariance& getCovariance() const;
      /// Sets the covariance of the measurement
      void setCovariance(const Covariance& sigma2Wheel);
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
      /**
       * Design variable representing the linear velocity of the wheel
       * w.r. to the mapping frame, expressed in the vehicle frame.
       * \brief Scaled linear velocity
       */
      aslam::backend::EuclideanExpression _v_w_mw;
      /// Scaling factor
      aslam::backend::ScalarExpression _r;
      /// Measured odometry
      double _measurement;
      /// Covariance of the measurement
      Covariance _sigma2Wheel;
      /** @}
        */

    };

  }
}

#endif // ASLAM_CALIBRATION_CAR_ERROR_TERM_WHEEL_H
