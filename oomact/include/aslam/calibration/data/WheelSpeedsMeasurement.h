#ifndef ASLAM_CALIBRATION_WHEEL_SPEEDS_MEASUREMENT_H
#define ASLAM_CALIBRATION_WHEEL_SPEEDS_MEASUREMENT_H

namespace aslam {
namespace calibration {

struct WheelSpeedsMeasurement {
  /// Left, right wheel speed measurement in rad/s
  double left, right;
};

}
}

#endif // ASLAM_CALIBRATION_WHEEL_SPEEDS_MEASUREMENT_H
