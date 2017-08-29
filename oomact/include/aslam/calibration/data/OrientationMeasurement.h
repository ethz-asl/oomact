#ifndef ASLAM_CALIBRATION_ORIENTATION_MEASUREMENT_H
#define ASLAM_CALIBRATION_ORIENTATION_MEASUREMENT_H

#include <Eigen/Core>

namespace aslam {
namespace calibration {
struct OrientationMeasurement {
  Eigen::Vector4d q;
};

}
}

#endif // ASLAM_CALIBRATION_ORIENTATION_MEASUREMENT_H
