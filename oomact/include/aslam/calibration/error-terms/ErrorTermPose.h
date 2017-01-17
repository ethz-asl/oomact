#ifndef ASLAM_CALIBRATION_CAR_ERROR_TERM_POSE_H
#define ASLAM_CALIBRATION_CAR_ERROR_TERM_POSE_H

#include <aslam/backend/ErrorTerm.hpp>
#include <aslam/backend/TransformationExpression.hpp>
#include <aslam/calibration/error-terms/ErrorTermGroup.h>

#include "aslam/calibration/data/PoseMeasurement.h"
namespace aslam {
  namespace calibration {

    /** The class ErrorTermPose implements an error term for a pose sensor such
        as an Applanix.
        \brief Pose error term
      */
    class ErrorTermPose :
      public aslam::backend::ErrorTermFs<6>, public ErrorTermGroupMember {
    public:
      // Required by Eigen for fixed-size matrices members
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      /** \name Types definitions
        @{
        */
      /// Covariance type
      typedef Eigen::Matrix<double, 6, 6> Covariance;
      /** @}
        */

      /** \name Constructors/destructor
        @{
        */
      /** 
       * Constructs the error term from input data and design variables
       * \brief Constructs the error term
       * 
       * @param T pose to compare with
       * @param Tm pose measurement (\f$[x,y,z,\theta_z,\theta_y,\theta_x]\f$)
       * @param sigma2 Covariance matrix of the pose measurement
       */
      ErrorTermPose(
          const aslam::backend::TransformationExpression& T,
          const Eigen::Vector3d & t,
          const Eigen::Vector4d & q,
          const Covariance& sigma2,
          ErrorTermGroupReference r);

      ErrorTermPose(
          const aslam::backend::TransformationExpression& T, const PoseMeasurement & pm, ErrorTermGroupReference etgr
          );

      virtual Eigen::VectorXd getPrediction() const;
      Eigen::VectorXd getMeasurement() const;

      virtual ~ErrorTermPose() = default;
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
      Eigen::Vector4d _q;
      Eigen::Matrix3d _C;

      Eigen::Vector3d _t;

      /// Pose design variable
      aslam::backend::TransformationExpression _T;
      /** @}
        */

    };

  }
}

#endif // ASLAM_CALIBRATION_CAR_ERROR_TERM_POSE_H
