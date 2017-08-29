#ifndef ASLAM_CALIBRATION_ACCELEROMETER_MEASUREMENT_H
#define ASLAM_CALIBRATION_ACCELEROMETER_MEASUREMENT_H

#include <Eigen/Core>

namespace aslam {
namespace calibration {
struct AccelerometerMeasurement {
  Eigen::Vector3d a;
};
}
}

#endif // ASLAM_CALIBRATION_ACCELEROMETER_MEASUREMENT_H
