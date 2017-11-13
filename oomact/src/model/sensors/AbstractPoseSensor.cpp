#include <aslam/calibration/model/sensors/AbstractPoseSensor.h>

#include <aslam/calibration/data/MeasurementsContainer.h>
#include <aslam/calibration/data/PoseMeasurement.h>
#include <aslam/calibration/data/StorageI.h>
#include <aslam/calibration/model/ModuleTools.h>
#include <aslam/calibration/model/sensors/PoseSensorI.h>

namespace aslam {
namespace calibration {

AbstractPoseSensor::AbstractPoseSensor(Model& model, std::string name, sm::value_store::ValueStoreRef config) :
  Sensor(model, name, config),
  storageConnector_(this),
  invertInput_(getMyConfig().getBool("invertInput", false)),
  covPosition_(getMyConfig().getChild("covPosition"), 3),
  covOrientation_(getMyConfig().getChild("covOrientation"), 3)
{
}

void AbstractPoseSensor::writeConfig(std::ostream& out) const {
  Sensor::writeConfig(out);
  MODULE_WRITE_PARAM(invertInput_);
  MODULE_WRITE_PARAM(covPosition_);
  MODULE_WRITE_PARAM(covOrientation_);
}

AbstractPoseSensor::~AbstractPoseSensor() {
}

bool AbstractPoseSensor::hasMeasurements(const ModuleStorage & storage) const {
  return storageConnector_.hasData(storage);
}

const PoseMeasurements& AbstractPoseSensor::getAllMeasurements(const ModuleStorage & storage) const {
  return storageConnector_.getDataFrom(storage);
}

PoseMeasurements& AbstractPoseSensor::getMeasurementsMutable(ModuleStorage & storage) const {
  return storageConnector_.getDataFrom(storage);
}

} /* namespace calibration */
} /* namespace aslam */

