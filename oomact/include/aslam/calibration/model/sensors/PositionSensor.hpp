#ifndef POSITION_SENSOR_HPP_
#define POSITION_SENSOR_HPP_

#include <aslam/calibration/model/Sensor.hpp>
#include <aslam/calibration/tools/Interval.hpp>

namespace aslam {
namespace calibration {

struct PositionMeasurement;
template <typename T> struct MeasurementsContainer;
typedef MeasurementsContainer<PositionMeasurement> PositionMeasurements;

//TODO: Some comments will be helpful here, for example see PoseMeasurement.h

class PositionSensor : public Sensor {
 public:
  PositionSensor(Model & model, std::string name, sm::value_store::ValueStoreRef config);

  virtual ~PositionSensor();

  virtual void clearMeasurements() override;

  void addMeasurementErrorTerms(CalibratorI & calib, const EstConf & ec, ErrorTermReceiver & problem, bool observeOnly) const override;

  void addMeasurement(const PositionMeasurement& Position, const Timestamp t);

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
