#include <aslam/calibration/model/sensors/Lidar2d.hpp>

#include <aslam/calibration/model/ModuleTools.h>

namespace aslam {
namespace calibration {

Lidar2d::Lidar2d(Model& model, std::string name, sm::value_store::ValueStoreRef config) :
  Lidar(model, name, config)
{
  minimalAngle = lidarConfig.getDouble("minimalAngle");  // TODO B use mirror information
  maximalAngle = lidarConfig.getDouble("maximalAngle");
  angularResolution = lidarConfig.getDouble("angularResolution");
  measurementTimeIncrement = lidarConfig.getDouble("measurementTimeIncrement");
}

Lidar2d::~Lidar2d() {
}

void Lidar2d::writeConfig(std::ostream& out) const {
  Lidar::writeConfig(out);
  out << ", angle in [" << minimalAngle << ", " << maximalAngle << "]rad";
  MODULE_WRITE_PARAM(angularResolution);
  MODULE_WRITE_PARAM(measurementTimeIncrement);
}


} /* namespace calibration */
} /* namespace aslam */
