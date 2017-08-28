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

  virtual void addMeasurementErrorTerms(CalibratorI & calib, const EstConf & ec, ErrorTermReceiver & problem, bool observeOnly) const override;

  void addMeasurement(const PoseMeasurement& pose, const Timestamp t, Storage & storage) const;
  void addMeasurement(const Eigen::Vector4d & quat, const Eigen::Vector3d & trans, const Timestamp t, Storage & storage) const;

  const Covariance& getCovOrientation() const {
    return covOrientation_;
  }

  const Covariance& getCovPosition() const {
    return covPosition_;
  }

  const Frame& getTargetFrame() const override {
    return targetFrame_;
  }

  const static PoseMeasurement Outlier;
 private:
  Covariance covPosition_, covOrientation_; // One noise model for all measurements

  const Frame& targetFrame_;

  bool absoluteMeasurements_;

  bool isOutlier(const PoseMeasurement & p) const;
 protected:
  void writeConfig(std::ostream & out) const override;
};

} /* namespace calibration */
} /* namespace aslam */

#endif /* MOTION_CAPTURE_SENSOR_HPP_ */
