#include "aslam/calibration/ros/internal/Tools.h"

#include <aslam/calibration/model/Sensor.h>

namespace aslam {
namespace calibration {
namespace ros {
const Sensor& getSensorFromModule(const Module& m) {
  return dynamic_cast<const Sensor& >(m);
}

const std::string& getNameFromSensor(const Sensor& s) {
  return s.getName();
}

Eigen::Matrix3d rosArray9CovToEigenMatrix3(const boost::array<double, 9> & cov){
  Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> m(cov.data());
  return m;
}

} /* namespace ros */
} /* namespace calibration */
} /* namespace aslam */

