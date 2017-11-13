#ifndef POSE_SENSOR_HPP_
#define POSE_SENSOR_HPP_

#include <aslam/calibration/input/InputReceiverI.h>
#include <aslam/calibration/model/sensors/AbstractPoseSensor.h>
#include <aslam/calibration/tools/Interval.h>

namespace aslam {
namespace calibration {

class PoseSensor : public AbstractPoseSensor, public InputReceiverIT<PoseMeasurement> {
 public:
  PoseSensor(Model & model, std::string name, sm::value_store::ValueStoreRef config = sm::value_store::ValueStoreRef());

  virtual ~PoseSensor();

  virtual void addMeasurementErrorTerms(CalibratorI & calib, const CalibrationConfI & ec, ErrorTermReceiver & problem, bool observeOnly) const override;

  void addInputTo(Timestamp t, const PoseMeasurement& pose, ModuleStorage & storage) const override;
  void addMeasurement(Timestamp t, const PoseMeasurement& pose, ModuleStorage & storage) const;
  void addMeasurement(Timestamp t, const Eigen::Vector4d & quat, const Eigen::Vector3d & trans, ModuleStorage & storage) const;

  const Frame& getTargetFrame() const override {
    return targetFrame_;
  }

  const static PoseMeasurement Outlier;
 private:
  const Frame& targetFrame_;

  bool absoluteMeasurements_;

  bool isOutlier(const PoseMeasurement & p) const;
 protected:
  void writeConfig(std::ostream & out) const override;
};

} /* namespace calibration */
} /* namespace aslam */

#endif /* MOTION_CAPTURE_SENSOR_HPP_ */
