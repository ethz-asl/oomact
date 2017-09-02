#ifndef HA117799C_989A_498F_8881_4E13B56A0095
#define HA117799C_989A_498F_8881_4E13B56A0095

#include <Eigen/Dense>
#include <aslam/calibration/input/InputReceiverI.h>
#include <aslam/calibration/Timestamp.hpp>
#include <boost/array.hpp>

namespace aslam {
namespace calibration {
class Sensor;

namespace ros {

template <typename Msg>
Timestamp getTimestampFromHeader(const Msg& m) {
  return Timestamp::fromNumerator(m.header.stamp.toNSec());
}

Eigen::Matrix3d rosArray9CovToEigenMatrix3(const boost::array<double, 9> & cov);

template <typename Input>
Eigen::Vector3d rosVector3dToEigenVector3(Input & v){
  return Eigen::Vector3d(v.x, v.y, v.z);
}

template <typename Input>
Eigen::Vector4d rosQuaternionToVector4dXYZW(Input & v, bool conjugate){
  return conjugate ? Eigen::Vector4d(-v.x, -v.y, -v.z, v.w) : Eigen::Vector4d(v.x, v.y, v.z, v.w);
}

const Sensor & getSensorFromModule(const Module & m);

template <typename T>
const Sensor & getSensorFromReceiver(const InputReceiverIT<T> & r){
  return getSensorFromModule(r.getModule());
}

const std::string & getNameFromSensor(const Sensor & s);

} /* namespace ros */
} /* namespace calibration */
} /* namespace aslam */

#endif /* HA117799C_989A_498F_8881_4E13B56A0095 */
