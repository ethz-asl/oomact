#ifndef ASLAM_CALIBRATION_GYROSCOPE_MEASUREMENT_H
#define ASLAM_CALIBRATION_GYROSCOPE_MEASUREMENT_H

#include <Eigen/Core>

namespace aslam {
namespace calibration {
struct GyroscopeMeasurement {
  /// Angular velocity vector [rad/s]
  Eigen::Vector3d w;
  /// Covariance. Set (0,0) to 0.0 if not available.
  Eigen::Matrix3d cov;

  GyroscopeMeasurement() = default;
  GyroscopeMeasurement(const Eigen::Vector3d & w, const Eigen::Matrix3d & cov = Eigen::Matrix3d::Zero()) :
    w(w), cov(cov) {}
};
}
}

#endif // ASLAM_CALIBRATION_GYROSCOPE_MEASUREMENT_H
