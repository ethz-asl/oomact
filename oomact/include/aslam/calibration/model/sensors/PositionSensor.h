#ifndef POSITION_SENSOR_HPP_
#define POSITION_SENSOR_HPP_

#include <aslam/calibration/input/InputReceiverI.h>
#include <aslam/calibration/model/Sensor.h>
#include <aslam/calibration/tools/Interval.h>
#include <sm/deprecation.h>

namespace aslam {
namespace calibration {

struct PositionMeasurement;
template <typename T> struct MeasurementsContainer;
typedef MeasurementsContainer<PositionMeasurement> PositionMeasurements;

//TODO: Some comments will be helpful here, for example see PoseMeasurement.h

class PositionSensor : public Sensor, public InputReceiverIT<PositionMeasurement> {
 public:
  PositionSensor(Model & model, std::string name, sm::value_store::ValueStoreRef config);
  virtual ~PositionSensor();

  void addMeasurement(Timestamp t, const PositionMeasurement& position);
  void addMeasurement(Timestamp t, const Eigen::Vector3d & p);

  SM_DEPRECATED("Use the overload with swapped arguments instead.")
  void addMeasurement(const PositionMeasurement& position, Timestamp t){
    addMeasurement(t, position);
  }

  void addInputTo(Timestamp t, const PositionMeasurement & position, ModuleStorage & s) const override;

  virtual void clearMeasurements() override;
  void addMeasurementErrorTerms(CalibratorI & calib, const CalibrationConfI & ec, ErrorTermReceiver & problem, bool observeOnly) const override;
 private:
  std::shared_ptr<PositionMeasurements> measurements;

  Covariance covPosition;   // One noise model for all measurements
  const Frame& targetFrame; // Target Frame observed by the Position sensor
 protected:
   void writeConfig(std::ostream & out) const override;
};

} /* namespace calibration */
} /* namespace aslam */

#endif /* MOTION_CAPTURE_SENSOR_HPP_ */
