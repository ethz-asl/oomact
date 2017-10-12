#include <aslam/calibration/model/sensors/Lidar.h>

#include <glog/logging.h>

#include <sm/value_store/LayeredValueStore.hpp>

#include <aslam/calibration/laser-scanner/laser3d.h>
#include <aslam/calibration/model/ModuleTools.h>

namespace aslam {
namespace calibration {

Lidar::Lidar(Model& model, const std::string& name, sm::value_store::ValueStoreRef config) :
  PointCloudSensor(model, name, config),
  lidarConfig(config.getChild(name))
{
  const std::string lidarModel = lidarConfig.getString("model", std::string());

  if(!lidarModel.empty()){
    LOG(INFO) << "Using model=" << lidarModel << " for Lidar " << name << ".";
    std::shared_ptr<sm::value_store::LayeredValueStore> lvs = std::make_shared<LayeredValueStore>();
    lvs->add(config.getChild(name).getValueStoreSharedPtr());
    lvs->add(config.getChild(lidarModel).getValueStoreSharedPtr());
    lidarConfig = ValueStoreRef(lvs);
  }

  minimalDistance = lidarConfig.getDouble("minimalDistance");
  maximalDistance = lidarConfig.getDouble("maximalDistance");

  std::string base = "noise/";

  vertAngleVariance = lidarConfig.getDouble(base + "vertAngleVariance");
  horizAngleVariance = lidarConfig.getDouble(base + "horizAngleVariance");
  rangeVariance = lidarConfig.getDouble(base + "rangeVariance");
  beamDiverAngle = lidarConfig.getDouble(base + "beamDiverAngle");
  noiseGainX = lidarConfig.getDouble(base + "noiseGainX");
  noiseGainY = lidarConfig.getDouble(base + "noiseGainY");
  noiseGainZ = lidarConfig.getDouble(base + "noiseGainZ");

  nanPolicy_.checkForNans = lidarConfig.getBool("checkForNans", nanPolicy_.checkForNans);
  nanPolicy_.nansAreFine = lidarConfig.getBool("nansAreFine", nanPolicy_.nansAreFine);
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

