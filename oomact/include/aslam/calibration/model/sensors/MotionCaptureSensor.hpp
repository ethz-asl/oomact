#ifndef MOTION_CAPTURE_SENSOR_HPP_
#define MOTION_CAPTURE_SENSOR_HPP_

#include <aslam/calibration/tools/Interval.hpp>
#include <aslam/calibration/model/sensors/AbstractPoseSensor.h>

namespace aslam {
namespace calibration {
class MotionCaptureSource;
struct MotionCaptureSensorMeasurements;

struct PoseMeasurement;
template <typename T> struct MeasurementsContainer;
typedef MeasurementsContainer<PoseMeasurement> PoseMeasurements;


class MotionCaptureSystem : public Module, public PoseCv, public DelayCv, public CalibratableMinimal, public Activatable {
 public:
  MotionCaptureSystem(Model & model, std::string name, sm::value_store::ValueStoreRef config);

 protected:
  void writeConfig(std::ostream & out) const override;
  virtual void setActive(bool spatial, bool temporal) override;
  void registerWithModel() override;
};

class MotionCaptureSensor : public AbstractPoseSensor {
 public:
  MotionCaptureSensor(MotionCaptureSystem & motionCaptureSystem, std::string name, sm::value_store::ValueStoreRef config);

  virtual ~MotionCaptureSensor();

  void preProcessNewWindow(CalibratorI & calib) override;

  virtual void clearMeasurements() override;

  void addMeasurementErrorTerms(CalibratorI & calib, const EstConf & ec, ErrorTermReceiver & problem, bool observeOnly) const override;

  Interval getSnappedWindow(CalibratorI & calib, const Interval & i);

  SensorType getType() const override;

  const std::shared_ptr<MotionCaptureSource>& getMotionCaptureSource() const {
    return motionCaptureSource;
  }

  void setMotionCaptureSource(const std::shared_ptr<MotionCaptureSource>& motionCaptureSource) {
    this->motionCaptureSource = motionCaptureSource;
  }

  bool hasMeasurements() const override {
    return bool(measurements);
  }

  const PoseMeasurements & getAllMeasurements() const override;
  PoseMeasurements getMeasurements(Timestamp from, Timestamp till) const override;

  const Frame& getTargetFrame() const override {
    return motionCaptureSystem.getParentFrame();
  }

 private:
  MotionCaptureSystem & motionCaptureSystem;
  std::shared_ptr<MotionCaptureSource> motionCaptureSource;
  std::shared_ptr<MotionCaptureSensorMeasurements> measurements;

  Covariance covPosition, covOrientation;
};

} /* namespace calibration */
} /* namespace aslam */

#endif /* MOTION_CAPTURE_SENSOR_HPP_ */
