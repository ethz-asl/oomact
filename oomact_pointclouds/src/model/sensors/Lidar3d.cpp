#include <aslam/calibration/model/sensors/Lidar3d.hpp>

#include <aslam/calibration/model/ModuleTools.h>

namespace aslam {
namespace calibration {

Lidar3d::Lidar3d(Model& model, const std::string& name, sm::value_store::ValueStoreRef config) :
    Lidar(model, name, config),
    normalsCovariance_{lidarConfig.getChild("noise/normalsCov"), 3, isUsed()}
{
}

Lidar3d::~Lidar3d() {
}

void Lidar3d::writeConfig(std::ostream& out) const {
  Lidar3d::writeConfig(out);
  MODULE_WRITE_PARAM(normalsCovariance_);
}

} /* namespace calibration */
} /* namespace aslam */
