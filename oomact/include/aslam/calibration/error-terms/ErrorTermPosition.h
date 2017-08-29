#ifndef ASLAM_CALIBRATION_CAR_ERROR_TERM_POSITION_H
#define ASLAM_CALIBRATION_CAR_ERROR_TERM_POSITION_H

#include <aslam/backend/ErrorTerm.hpp>
#include <aslam/backend/TransformationExpression.hpp>
#include <aslam/calibration/error-terms/ErrorTermGroup.h>

#include "aslam/calibration/data/PositionMeasurement.h"
namespace aslam {
  namespace calibration {

    /** The class ErrorTermPosition implements an error term for a position sensor such
        as Leica.
        \brief Position error term
      */
    class ErrorTermPosition :
      public aslam::backend::ErrorTermFs<3>, public ErrorTermGroupMember {
    public:
      // Required by Eigen for fixed-size matrices members
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      /** \name Types definitions
        @{
        */
      /// Covariance type
      typedef Eigen::Matrix<double, 3, 3> Covariance; //TODO: CHECK if 6x6 is okay or switch to 3x3
      /** @}
        */

      /** \name Constructors/destructor
        @{
        */
      /** 
       * Constructs the error term from input data and design variables
       * \brief Constructs the error term
       * 
       * @param T position to compare with
       * @param Tm position measurement (\f$[x,y,z,\theta_z,\theta_y,\theta_x]\f$)
       * @param sigma2 Covariance matrix of the position measurement
       */
      ErrorTermPosition(
          const aslam::backend::EuclideanExpression& p,
          const Eigen::Vector3d & pm,
          const Covariance& sigma2,
          ErrorTermGroupReference etgr);

      ErrorTermPosition(
          const aslam::backend::EuclideanExpression& p,
          const PositionMeasurement & pm,
          const Covariance& sigma2,
          ErrorTermGroupReference etgr);

      virtual Eigen::VectorXd getPrediction() const;
      Eigen::VectorXd getMeasurement() const;

      virtual ~ErrorTermPosition() = default;
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
      // Input measurement
      Eigen::Vector3d pm_;

      /// Predicted position using estimated design variables
      aslam::backend::EuclideanExpression p_;
      /** @}
        */

    };

  }
}

#endif // ASLAM_CALIBRATION_CAR_ERROR_TERM_POSITION_H
