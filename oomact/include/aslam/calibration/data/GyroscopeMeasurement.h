#ifndef ASLAM_CALIBRATION_GYROSCOPE_MEASUREMENT_H
#define ASLAM_CALIBRATION_GYROSCOPE_MEASUREMENT_H

#include <Eigen/Core>

namespace aslam {
namespace calibration {
struct GyroscopeMeasurement {
  Eigen::Vector3d w;
};
}
}

#endif // ASLAM_CALIBRATION_GYROSCOPE_MEASUREMENT_H
