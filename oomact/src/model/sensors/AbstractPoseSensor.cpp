#include <aslam/calibration/model/sensors/AbstractPoseSensor.h>
#include <aslam/calibration/data/MeasurementsContainer.h>
#include <aslam/calibration/data/PoseMeasurement.h>

namespace aslam {
namespace calibration {

AbstractPoseSensor::AbstractPoseSensor(Model& model, std::string name, sm::value_store::ValueStoreRef config) :
  Sensor(model, name, config)
{
}

AbstractPoseSensor::~AbstractPoseSensor() {
}

PoseMeasurements AbstractPoseSensor::getMeasurements(Timestamp from, Timestamp till) const {
  auto & allMeasurements = getAllMeasurements();
  PoseMeasurements poses;
  std::copy_if(allMeasurements.begin(), allMeasurements.end(), poses.begin(), [=](const PoseMeasurements::value_type & p){
    auto t = Timestamp(p.first);
    return t <= till && t >= from;
  });
  return poses;

}

} /* namespace calibration */
} /* namespace aslam */

