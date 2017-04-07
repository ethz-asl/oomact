#ifndef ASLAM_CALIBRATION_CAR_ERROR_TERM_GYROSCOPE_H
#define ASLAM_CALIBRATION_CAR_ERROR_TERM_GYROSCOPE_H

#include "MeasurementErrorTerm.h"
#include <aslam/backend/EuclideanExpression.hpp>
#include <aslam/calibration/error-terms/ErrorTermGroup.h>

namespace aslam {
  namespace calibration {

  /** The class ErrorTermGyroscope implements an error term for an Imu sensor acceleration measurement
        \brief Acceleration error term
   */
    class ErrorTermGyroscope : public MeasurementErrorTerm<3, backend::EuclideanExpression> {
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
       * @param _i_w_mr angular speed to compare with (i_w_mr = i_w_m_i, due to body rigidity)
       * @param _bias evaluation of the bias term
       * @param wm measured angular measurement (\f$[wx,wy,wz]\f$)
       * @param sigma2 Covariance matrix of the pose measurement
       */
      ErrorTermGyroscope(const aslam::backend::EuclideanExpression& _w_i_mi,
                         const aslam::backend::EuclideanExpression& _bias,
                         const Input& wm, const Covariance& sigma2,
                         const ErrorTermGroupReference & etgr = ErrorTermGroupReference());

      /// Copy constructor
      ErrorTermGyroscope(const ErrorTermGyroscope& other) = default;

      /// Destructor
      virtual ~ErrorTermGyroscope(){}
      /** @}
       */
    };
  }
}

#endif // ASLAM_CALIBRATION_CAR_ERROR_TERM_GYROSCOPE_H
