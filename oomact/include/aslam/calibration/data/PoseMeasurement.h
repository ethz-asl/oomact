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
};

}
}

#endif // ASLAM_CALIBRATION_POSE_MEASUREMENT_H
