#ifndef ASLAM_CALIBRATION_POSE_MEASUREMENT_H
#define ASLAM_CALIBRATION_POSE_MEASUREMENT_H

#include <Eigen/Core>

namespace aslam {
namespace calibration {

struct PoseMeasurement {
  /// Translation
  Eigen::Vector3d t;
  /// Quaternions in JPL convention (XYZW)
  Eigen::Vector4d q;
  //TODO B switch to quaternion class
  static constexpr bool USE_XYZW_ORDER = true;  // TODO B Switch Quaternion convention.
  static constexpr bool USE_JPL_MULT = true;  // TODO B Switch Quaternion convention.
};

}
}

#endif // ASLAM_CALIBRATION_POSE_MEASUREMENT_H
