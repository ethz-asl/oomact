#ifndef ASLAM_CALIBRATION_CAR_ERROR_TERM_POSE_H
#define ASLAM_CALIBRATION_CAR_ERROR_TERM_POSE_H

#include <aslam/backend/ErrorTerm.hpp>
#include <aslam/backend/TransformationExpression.hpp>
#include <aslam/calibration/error-terms/ErrorTermGroup.h>

#include "aslam/calibration/data/PoseMeasurement.h"
namespace aslam {
namespace calibration {
class ErrorTermPose : public aslam::backend::ErrorTermFs<6>, public ErrorTermGroupMember {
 public:
  // Required by Eigen for fixed-size matrices members
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef Eigen::Matrix<double, 6, 6> Covariance;

  ErrorTermPose(const aslam::backend::TransformationExpression& T, const Eigen::Vector3d & t, const Eigen::Vector4d & q, const Covariance& cov, ErrorTermGroupReference etgr);

  ErrorTermPose(const aslam::backend::TransformationExpression& T, const Eigen::Vector3d & t, const Eigen::Vector4d & q, const Eigen::Matrix3d & cov_t, const Eigen::Matrix3d & cov_q, ErrorTermGroupReference etgr);

  ErrorTermPose(const aslam::backend::TransformationExpression& T, const PoseMeasurement & pm, const Eigen::Matrix3d & cov_t, const Eigen::Matrix3d & cov_r, ErrorTermGroupReference etgr);

  virtual ~ErrorTermPose() = default;

  virtual Eigen::VectorXd getPrediction() const;
  virtual Eigen::VectorXd getMeasurement() const;

 protected:
  double evaluateErrorImplementation() override;
  void evaluateJacobiansImplementation(aslam::backend::JacobianContainer& jacobians) override;

  Eigen::Vector4d _q;
  Eigen::Matrix3d _C;
  Eigen::Vector3d _t;

  aslam::backend::TransformationExpression _T;
};

}
}

#endif // ASLAM_CALIBRATION_CAR_ERROR_TERM_POSE_H
