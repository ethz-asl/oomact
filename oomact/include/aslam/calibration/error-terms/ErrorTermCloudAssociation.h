#ifndef ASLAM_CALIBRATION_CAR_ERROR_TERM_CLOUD_ASSOCIATION_H
#define ASLAM_CALIBRATION_CAR_ERROR_TERM_CLOUD_ASSOCIATION_H

#include <aslam/backend/ErrorTerm.hpp>
#include <aslam/backend/Scalar.hpp>
#include <aslam/backend/ScalarExpression.hpp>
#include <aslam/backend/EuclideanExpression.hpp>
#include <aslam/backend/RotationExpression.hpp>
#include <aslam/backend/TransformationExpression.hpp>
//#include <aslam/backend/TransformationExpression.hpp>



namespace aslam {
  namespace calibration {

    /** The class ErrorTermCloudAssociation implements an error term for an Imu sensor acceleration measurement
        \brief Acceleration error term
      */
    class ErrorTermCloudAssociation :
      //  public aslam::backend::ErrorTerm{
      public aslam::backend::ErrorTermFs<1> {
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

      ErrorTermCloudAssociation(const aslam::backend::TransformationExpression& T_m_v_a,
                                const aslam::backend::TransformationExpression& T_m_v_b,
                                const aslam::backend::RotationExpression& R_m_v_a,
                                const Input& n_a,
                                const Input& p_a,
                                const Input& p_b,
                                const double weight,
                                const CovarianceInput& sigma2_n_a,
                                const CovarianceInput& sigma2_p_a,
                                const CovarianceInput& sigma2_p_b);
                                //const Covariance& sigma2);

      /// Copy constructor
      ErrorTermCloudAssociation(const ErrorTermCloudAssociation& other) = default;
      /// Assignment operator
      ErrorTermCloudAssociation& operator = (const ErrorTermCloudAssociation& other) = default;

      /// Destructor
      virtual ~ErrorTermCloudAssociation();
      /** @}
        */

      /** \name Accessors
        @{
        */
      /// Returns the input measurement
      const Input& getInput() const;
      /// Returns the input measurement
      Input& getInput();
      /// Sets the input measurement
      void setInput(const Input& am);
      /// Returns the covariance of the measurement
      const Covariance& getCovariance() const;
      /// Returns the covariance of the measurement
      Covariance& getCovariance();
      /// Sets the covariance of the measurement
      void setCovariance(const Covariance& sigma2);
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
      /// Velodyne PointToPlane error design variable
      // aslam::backend::EuclideanExpression _pointToPlane_prediction;
      aslam::backend::ScalarExpression _pointToPlane_prediction;

      // Error design variables, just a scalar is not good, unless implementing it
      aslam::backend::EuclideanExpression _n_m_a;
      aslam::backend::EuclideanExpression _p_m_ab;
      /// Measured acceleration
      Input _n_a;
      Input _p_a;
      Input _p_b;

      // Weight
      double _w;
      /// Covariance matrix of the pose measurement
      Covariance _sigma2;
      /** @}
        */

    };

  }
}

#endif // ASLAM_CALIBRATION_CAR_ERROR_TERM_CLOUD_ASSOCIATION_H
