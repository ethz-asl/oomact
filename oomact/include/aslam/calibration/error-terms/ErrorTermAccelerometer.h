#ifndef ASLAM_CALIBRATION_CAR_ERROR_TERM_ACCELEROMETER_H
#define ASLAM_CALIBRATION_CAR_ERROR_TERM_ACCELEROMETER_H

#include "MeasurementErrorTerm.h"
#include <aslam/backend/EuclideanExpression.hpp>
#include <aslam/backend/RotationExpression.hpp>

namespace aslam {
  namespace calibration {

    /** The class ErrorTermAccelerometer implements an error term for an Imu sensor acceleration measurement
        \brief Acceleration error term
      */
    class ErrorTermAccelerometer : public MeasurementErrorTerm<3, backend::EuclideanExpression> {
    public:
      typedef MeasurementErrorTerm<3, backend::EuclideanExpression> Parent;

      // Required by Eigen for fixed-size matrices members
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

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
      ErrorTermAccelerometer(const aslam::backend::EuclideanExpression& a_m_mi,
                             const aslam::backend::RotationExpression& R_i_m,
                             const aslam::backend::EuclideanExpression& g_m,
                             const aslam::backend::EuclideanExpression& bias,
                             const Input& am, const Covariance& sigma2,
                             const ErrorTermGroupReference & etgr = ErrorTermGroupReference());

      /// Copy constructor
      ErrorTermAccelerometer(const ErrorTermAccelerometer& other) = default;

      /// Destructor
      virtual ~ErrorTermAccelerometer(){}
      /** @}
        */
    };
  }
}

#endif // ASLAM_CALIBRATION_CAR_ERROR_TERM_ACCELEROMETER_H
