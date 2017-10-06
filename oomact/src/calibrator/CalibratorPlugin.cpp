#include <aslam/calibration/calibrator/CalibratorPlugin.h>

#include <aslam/calibration/calibrator/CalibratorI.hpp>

namespace aslam {
namespace calibration {

CalibratorPlugin::CalibratorPlugin(CalibratorI& calibrator) : calibrator_(calibrator) {
}

CalibratorPlugin::~CalibratorPlugin() {
}


const aslam::calibration::Model& CalibratorPlugin::getModel() const {
  return getCalibrator().getModel();
}

aslam::calibration::Model& CalibratorPlugin::getModel() {
  return getCalibrator().getModel();
}

} /* namespace calibration */
} /* namespace aslam */
