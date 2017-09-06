#ifndef ASLAM_CALIBRATION_POSITION_MEASUREMENT_H
#define ASLAM_CALIBRATION_POSITION_MEASUREMENT_H

#include <Eigen/Core>

namespace aslam {
namespace calibration {
struct PositionMeasurement {
  Eigen::Vector3d p;
};
}
}

#endif // ASLAM_CALIBRATION_POSITION_MEASUREMENT_H
