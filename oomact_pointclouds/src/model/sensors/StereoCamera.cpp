#include <aslam/calibration/model/sensors/StereoCamera.hpp>
#include <glog/logging.h>


namespace aslam {
namespace calibration {

StereoCamera::StereoCamera(Model& model, std::string name, sm::value_store::ValueStoreRef config)
  : PointCloudSensor(model, name, config)
{
  std::string base = name + "/";
  useSemiGlobalMatcher = config.getBool(base + "useSemiGlobalMatcher");
  numDisparities = config.getDouble(base + "numDisparities");
  minimalX = config.getDouble(base + "minimalX");
  maximalX = config.getDouble(base + "maximalX");
  xyVarianceFactor = config.getDouble(base + "xyVarianceFactor");
  zVarianceFactor = config.getDouble(base + "zVarianceFactor");
  focalLength = config.getDouble(base + "focalLength");
  baseLine = config.getDouble(base + "baseLine");
  preScaleFactor = config.getDouble(base + "preScaleFactor", 1.0);
}

void StereoCamera::writeConfig(std::ostream& out) const {
  PointCloudSensor::writeConfig(out);
  out << ", useSemiGlobalMatcher=" << useSemiGlobalMatcher
      << ", numDisparities=" << numDisparities
      << ", minimalX=" << minimalX
      << ", maximalX=" << maximalX
      << ", xyVarianceFactor=" << xyVarianceFactor
      << ", zVarianceFactor=" << zVarianceFactor
      << ", focalLength=" << focalLength
      << ", baseLine=" << baseLine
      << ", preScaleFactor="<< getPreScaleFactor()
      ;
}


StereoCamera::~StereoCamera() {
}

Eigen::Matrix3d StereoCamera::covPoint(bool /*useSurfaceNormal*/, const Eigen::Vector3d& pInSensorFrame, const Eigen::Vector3d& /*nInSensorFrame*/) const {
  Eigen::Vector3d sDdiag;
  const double z = pInSensorFrame.z();
  double xyVariance = xyVarianceFactor * z / focalLength;
  sDdiag << xyVariance, xyVariance, z * z / ( focalLength * baseLine);
  Eigen::Matrix3d sD = Eigen::Matrix3d::Zero();
  sD.diagonal() = sDdiag;
  return sD * sD;
}

} /* namespace calibration */
} /* namespace aslam */
