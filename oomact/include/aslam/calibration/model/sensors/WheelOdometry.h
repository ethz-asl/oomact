#ifndef H2E963BC0_3111_4293_88B0_3AB8B916345D
#define H2E963BC0_3111_4293_88B0_3AB8B916345D

#include <aslam/calibration/data/MeasurementsContainer.h>
#include <aslam/calibration/model/Module.h>
#include <aslam/calibration/model/Sensor.hpp>
#include <aslam/calibration/data/WheelSpeedsMeasurement.h>
#include "../../Timestamp.hpp"

namespace aslam {
namespace calibration {

class WheelOdometry : public Sensor {
 public:
  WheelOdometry(Model & model, const std::string & name, sm::value_store::ValueStoreRef config);

  void addMeasurementErrorTerms(CalibratorI & calib, const EstConf & ec, ErrorTermReceiver & problem, bool observeOnly) const override;
  void clearMeasurements() override;

  void addMeasurement(CalibratorI & calib, Timestamp t, const WheelSpeedsMeasurement & m) const;

  virtual ~WheelOdometry() = default;

  const ScalarCvSp& getL() const {
    return L;
  }

  const ScalarCvSp& getWheelRadiusL() const {
    return R_l;
  }

  const ScalarCvSp& getWheelRadiusR() const {
    return R_r;
  }

  const MeasurementsContainer<WheelSpeedsMeasurement> & getMeasurements() const {
    return measurements_ ;
  }

  bool hasTooFewMeasurements() const override;
 protected:
  void writeConfig(std::ostream& out) const override;
  void registerWithModel() override;
  void setActive(bool spatial, bool temporal) override;

 public:
  /// Axe length
  ScalarCvSp L;
  /// Left wheel radius
  ScalarCvSp R_l;
  /// Right wheel radius
  ScalarCvSp R_r;

  const double assumedWheelBase, assumedWheelRadiusLeft, assumedWheelRadiusRight;
 private:
  mutable MeasurementsContainer<WheelSpeedsMeasurement> measurements_;
  double maximallyPossibleRotationalVelocity = 5.0; //TODO C make maximallyPossibleRotationalVelocity and maximallyPossibleTranslationalVelocity parameters;
  double maximallyPossibleTranslationalVelocity = 3.0;
  double lwVariance;
  double rwVariance;
  int minimalMeasurementsPerBatch;
  const Frame & groundFrame_;
};

} /* namespace calibration */
} /* namespace aslam */

#endif /* H2E963BC0_3111_4293_88B0_3AB8B916345D */
