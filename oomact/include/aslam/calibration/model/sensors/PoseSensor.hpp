#ifndef POSE_SENSOR_HPP_
#define POSE_SENSOR_HPP_

#include <aslam/calibration/model/sensors/AbstractPoseSensor.h>
#include <aslam/calibration/tools/Interval.hpp>

namespace aslam {
namespace calibration {

class PoseSensor : public AbstractPoseSensor {
 public:
  PoseSensor(Model & model, std::string name, sm::value_store::ValueStoreRef config);

  virtual ~PoseSensor();

  virtual void clearMeasurements() override;

  void addMeasurementErrorTerms(CalibratorI & calib, const EstConf & ec, ErrorTermReceiver & problem, bool observeOnly) const override;

  void addMeasurement(const PoseMeasurement& pose, const Timestamp t);

  virtual bool hasMeasurements() const override;
  virtual const PoseMeasurements & getMeasurements() const override;
 private:
  std::shared_ptr<PoseMeasurements> measurements;

  Covariance covPosition, covOrientation;   // One noise model for all measurements

  const Frame& targetFrame;

 protected:
   void writeConfig(std::ostream & out) const override;
};

} /* namespace calibration */
} /* namespace aslam */

#endif /* MOTION_CAPTURE_SENSOR_HPP_ */
