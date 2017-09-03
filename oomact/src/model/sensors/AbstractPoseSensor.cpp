#include <aslam/calibration/model/sensors/AbstractPoseSensor.h>

#include <aslam/calibration/data/MeasurementsContainer.h>
#include <aslam/calibration/data/PoseMeasurement.h>
#include <aslam/calibration/data/StorageI.h>
#include <aslam/calibration/model/ModuleTools.h>
#include <aslam/calibration/model/sensors/PoseSensorI.hpp>

namespace aslam {
namespace calibration {

AbstractPoseSensor::AbstractPoseSensor(Model& model, std::string name, sm::value_store::ValueStoreRef config) :
  Sensor(model, name, config),
  storageConnector_(this),
  invertInput_(getMyConfig().getBool("invertInput", false))
{
}

void AbstractPoseSensor::writeConfig(std::ostream& out) const {
  MODULE_WRITE_FLAG(invertInput_);
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

