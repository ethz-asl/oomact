#ifndef ASLAM_CALIBRATION_ERROR_TERM_POINT_TO_PLAIN_H
#define ASLAM_CALIBRATION_ERROR_TERM_POINT_TO_PLAIN_H

#include <aslam/backend/ErrorTerm.hpp>
#include <aslam/backend/ScalarExpression.hpp>
#include <aslam/backend/EuclideanExpression.hpp>
#include <aslam/calibration/error-terms/ErrorTermGroup.h>

namespace aslam {
  namespace calibration {

    /** The class ErrorTermCloudAssociation implements an error term for an Imu sensor acceleration measurement
        \brief Acceleration error term
      */
    class ErrorTermPointToPlain :
      //  public aslam::backend::ErrorTerm{
      public aslam::backend::ErrorTermFs<1>, public ErrorTermGroupMember {
    public:
      // Required by Eigen for fixed-size matrices members
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      /** \name Types definitions
        @{
        */
      /// Covariance type
      // TODO: C Check well this covariance (error is a scalar)
      //typedef Eigen::Matrix<double, 3, 3> Covariance;

      // Error covariance
      typedef Eigen::Matrix<double, 1, 1> Covariance;

      // Measurement Covariance
      typedef Eigen::Matrix<double, 3, 3> CovarianceInput;

      /// Measurement type
      typedef Eigen::Matrix<double, 3, 1> Input;
      /** @}
        */

      /** \name Constructors/destructor
        @{
        */
      /** 
       * Constructs the error term from input data and design variables
       * \brief Constructs the error term
       * 
       * @param m_a_mr acceleration to compare with (acceleration in the mapping frame)
       * @param am acceleration measurement (\f$[x,y,z]\f$) in F_i, i_am_mi
       * @param sigma2 Covariance matrix of the pose measurement
       */

      ErrorTermPointToPlain(const Input& n_a,
                                const Input& p_a,
                                const aslam::backend::EuclideanExpression & p_A_b,
                                const double weight,
                                const CovarianceInput& sigma2_n_a,
                                const CovarianceInput& sigma2_p_a,
                                const CovarianceInput& sigma2_p_b,
                                ErrorTermGroupReference etgr);

      /// Copy constructor
      ErrorTermPointToPlain(const ErrorTermPointToPlain& other) = default;
      /// Assignment operator
      ErrorTermPointToPlain& operator = (const ErrorTermPointToPlain& other) = default;

      /// Destructor
      virtual ~ErrorTermPointToPlain();
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
      aslam::backend::EuclideanExpression _p_m_ab;

      /// Measured acceleration
      Input _n_a;
      Input _p_a;

      /** @}
        */

    };

  }
}

#endif // ASLAM_CALIBRATION_ERROR_TERM_POINT_TO_PLAIN_H
