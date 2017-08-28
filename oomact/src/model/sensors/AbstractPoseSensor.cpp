#include <aslam/calibration/model/sensors/AbstractPoseSensor.h>

#include <aslam/calibration/data/MeasurementsContainer.h>
#include <aslam/calibration/data/PoseMeasurement.h>
#include <aslam/calibration/data/StorageI.h>
#include <aslam/calibration/model/sensors/PoseSensorI.hpp>

namespace aslam {
namespace calibration {

AbstractPoseSensor::AbstractPoseSensor(Model& model, std::string name, sm::value_store::ValueStoreRef config) :
  Sensor(model, name, config),
  storageConnector_(this)
{
}

AbstractPoseSensor::~AbstractPoseSensor() {
}

bool AbstractPoseSensor::hasMeasurements(ConstStorage & storage) const {
  return storageConnector_.hasData(storage);
}

const PoseMeasurements& AbstractPoseSensor::getAllMeasurements(const Storage & storage) const {
  return storageConnector_.getDataFrom(storage);
}

PoseMeasurements& AbstractPoseSensor::getAllMeasurements(Storage & storage) const {
  return storageConnector_.getDataFrom(storage);
}

} /* namespace calibration */
} /* namespace aslam */

