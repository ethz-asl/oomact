#include <aslam/calibration/model/sensors/Lidar.h>

#include <glog/logging.h>

#include <sm/value_store/LayeredValueStore.hpp>

#include <aslam/calibration/laser-scanner/laser3d.h>
#include <aslam/calibration/model/ModuleTools.h>

namespace aslam {
namespace calibration {

Lidar::Lidar(Model& model, const std::string& name, sm::value_store::ValueStoreRef config) :
  PointCloudSensor(model, name, config)
{
  auto myConfig = getMyConfig();
  minimalDistance = myConfig.getDouble("minimalDistance");
  maximalDistance = myConfig.getDouble("maximalDistance");
  nanPolicy_.checkForNans = myConfig.getBool("checkForNans", nanPolicy_.checkForNans);
  nanPolicy_.nansAreFine = myConfig.getBool("nansAreFine", nanPolicy_.nansAreFine);

  auto noiseVs = myConfig.getChild("noise");
  vertAngleVariance = noiseVs.getDouble("vertAngleVariance");
  horizAngleVariance = noiseVs.getDouble("horizAngleVariance");
  rangeVariance = noiseVs.getDouble("rangeVariance");
  beamDiverAngle = noiseVs.getDouble("beamDiverAngle");
  noiseGainX = noiseVs.getDouble("noiseGainX");
  noiseGainY = noiseVs.getDouble("noiseGainY");
  noiseGainZ = noiseVs.getDouble("noiseGainZ");
}

Eigen::Matrix3d Lidar::covPoint(bool useSurfaceNormal, const Eigen::Vector3d& pInSensorFrame, const Eigen::Vector3d& nInSensorFrame) const {
  return covPointSurface(useSurfaceNormal, pInSensorFrame, nInSensorFrame, sqrt(rangeVariance), sqrt(vertAngleVariance), sqrt(horizAngleVariance), sqrt(beamDiverAngle), noiseGainX, noiseGainY, noiseGainZ);
}

void Lidar::writeConfig(std::ostream& out) const {
  PointCloudSensor::writeConfig(out);
  out << ", range in [" << minimalDistance << ", " << maximalDistance << "]m";
  MODULE_WRITE_PARAM(vertAngleVariance);
  MODULE_WRITE_PARAM(horizAngleVariance);
  MODULE_WRITE_PARAM(rangeVariance);
  MODULE_WRITE_PARAM(beamDiverAngle);
  MODULE_WRITE_PARAM(noiseGainX);
  MODULE_WRITE_PARAM(noiseGainY);
  MODULE_WRITE_PARAM(noiseGainZ);
}
} /* namespace calibration */
} /* namespace aslam */

