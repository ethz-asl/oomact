#ifndef INCLUDE_ASLAM_CALIBRATION_SENSOR_HPP_
#define INCLUDE_ASLAM_CALIBRATION_SENSOR_HPP_

#include <aslam/backend/Scalar.hpp>

#include <aslam/calibration/calibrator/CalibrationConfI.h>
#include <aslam/calibration/model/Module.h>
#include <aslam/calibration/model/CalibrationVariable.h>
#include <aslam/calibration/model/fragments/PoseCv.h>
#include <aslam/calibration/model/fragments/DelayCv.h>
#include <aslam/calibration/SensorId.h>
#include <aslam/calibration/tools/Interval.h>

namespace aslam {
namespace calibration {
class CalibratorI;
class ModelAtTime;
class Model;

/// Scalar calibration variable
typedef CalibrationDesignVariable<aslam::backend::Scalar> ScalarCv;
/// Shared pointer to the scalar
typedef boost::shared_ptr<ScalarCv> ScalarCvSp;

class Sensor : public Module, public PoseCv, public DelayCv, public ObserverMinimal, public CalibratableMinimal, public Activatable {
 public:
  Sensor(Model & model, std::string name, sm::value_store::ValueStoreRef config);
  virtual ~Sensor();


  BoundedTimeExpression getBoundedTimestampExpression(const CalibratorI& calib, Timestamp t) const;
  aslam::backend::TransformationExpression getTransformationExpressionToAtMeasurementTimestamp(const CalibratorI & calib, Timestamp t, const Frame & to, bool ignoreBounds = false) const;
  virtual aslam::backend::TransformationExpression getTransformationExpressionTo(const ModelAtTime & robotModel, const Frame & to) const;
  sm::kinematics::Transformation getTransformationTo(const ModelAtTime & robotModel, const Frame & to) const;
  sm::kinematics::Transformation getTransformationTo(const CalibratorI & calib, const Frame & to) const;

  bool operator == (const Sensor & other) const { return this == &other; };
  bool operator != (const Sensor & other) const { return !(*this == other); };

  friend std::ostream & operator << (std::ostream & out, const Sensor & sensor);

  double getMaximalExpectedGap() const {
    return maximalExpectedGap;
  }

  const boost::shared_ptr<aslam::backend::MEstimator>& getMEstimator() const {
    return mEstimator;
  }

  virtual Interval getCurrentMeasurementTimestampRange(const CalibratorI & calib) const;

  const SensorId& getId() const {
    return id;
  }

  operator const SensorId& () const {
    return getId();
  }
 protected:
  const SensorId id;
  void writeConfig(std::ostream & out) const override;
  void registerWithModel() override;
  void setActive(bool spatial, bool temporal) override;

  void setMEstimator(const boost::shared_ptr<aslam::backend::MEstimator>& mEstimator) {
    this->mEstimator = mEstimator;
  }

 private:
  double maximalExpectedGap;
  boost::shared_ptr<aslam::backend::MEstimator> mEstimator;
};

}
}


#endif /* INCLUDE_ASLAM_CALIBRATION_SENSOR_HPP_ */
