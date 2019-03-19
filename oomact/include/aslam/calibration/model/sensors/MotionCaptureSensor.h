#ifndef MOTION_CAPTURE_SENSOR_HPP_
#define MOTION_CAPTURE_SENSOR_HPP_

#include <aslam/calibration/data/StorageI.h>
#include <aslam/calibration/model/Module.h>
#include <aslam/calibration/model/sensors/AbstractPoseSensor.h>
#include <aslam/calibration/tools/Interval.h>

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

  void addMeasurementErrorTerms(CalibratorI & calib, const CalibrationConfI & ec, ErrorTermReceiver & problem, bool observeOnly) const override;

  Interval getSnappedWindow(CalibratorI & calib, const Interval & i);

  const PoseMeasurements & fetchMeasurementsFromSourceInto(Timestamp from, Timestamp till, ModuleStorage & storage) const;

  const std::shared_ptr<MotionCaptureSource>& getMotionCaptureSource() const {
    return motionCaptureSource;
  }

  void setMotionCaptureSource(const std::shared_ptr<MotionCaptureSource>& motionCaptureSource) {
    this->motionCaptureSource = motionCaptureSource;
  }

  const Frame& getTargetFrame() const override {
    return motionCaptureSystem.getReferenceFrame();
  }
 private:
  MotionCaptureSystem & motionCaptureSystem;
  std::shared_ptr<MotionCaptureSource> motionCaptureSource;
};

} /* namespace calibration */
} /* namespace aslam */

#endif /* MOTION_CAPTURE_SENSOR_HPP_ */
