#ifndef H657A8A48_567D_4372_A5E6_CA05BA2932B4
#define H657A8A48_567D_4372_A5E6_CA05BA2932B4

#ifndef CALIBRATOR_HIGH_LEVEL
#include <aslam/calibration/data/MeasurementsContainer.h>
#endif

#include <aslam/calibration/model/Module.h>
#include <aslam/calibration/model/StateCarrier.h>
#include <aslam/calibration/model/Sensor.hpp>

namespace aslam {
namespace backend {
class ErrorTermReceiver;
}
namespace calibration {
struct AccelerometerMeasurement;
struct GyroscopeMeasurement;
class TrajectoryCarrier;

class Imu;

class BiasBatchState;

class Bias : public StateCarrier, public NamedMinimal {
 public:
  Bias(Module & m, const std::string & name, sm::value_store::ValueStoreRef config);

  bool isUsingSpline() const { return !biasVector; }

  aslam::backend::EuclideanExpression getBiasExpression(Timestamp t) const;

  void setActive(bool active){
    if(biasVector){
      biasVector->setActive(active);
    }
  }

  void initState(CalibratorI & calib);
  void addToBatch(bool stateActive, BatchStateReceiver & batchStateReceiver, DesignVariableReceiver & problem);
  void registerCalibrationVariables(Model & model);
 private:
  EuclideanPointCvSp biasVector;
  aslam::backend::EuclideanExpression biasVectorExpression;
  std::shared_ptr<TrajectoryCarrier> biasSplineCarrier;
  std::shared_ptr<BiasBatchState> state_;
  friend Imu;
};


class Imu : public Sensor, public StateCarrier {
 public:
#ifdef CALIBRATOR_HIGH_LEVEL
  class Measurements;
#else
  struct Measurements {
    MeasurementsContainer<AccelerometerMeasurement> accelerometer;
    MeasurementsContainer<GyroscopeMeasurement> gyroscope;
  };

  bool hasMeasurements() const {
    return bool(measurements_);
  }

  const MeasurementsContainer<AccelerometerMeasurement> & getAccelerometerMeasurements() const {
    assert(measurements_);
    return measurements_->accelerometer;
  }

  const MeasurementsContainer<GyroscopeMeasurement> & getGyroscopeMeasurements() const {
    assert(measurements_);
    return measurements_->gyroscope;
  }
#endif

  bool initState(CalibratorI & calib) override;
  void addToBatch(const Activator & stateActivator, BatchStateReceiver & batchStateReceiver, DesignVariableReceiver & problem) override;

  Imu(Model & model, const std::string & name, sm::value_store::ValueStoreRef config);

  void clearMeasurements() override;
  void addAccelerometerMeasurement(CalibratorI & calib, const AccelerometerMeasurement& data, Timestamp timestamp) const;
  void addGyroscopeMeasurement(CalibratorI & calib, const GyroscopeMeasurement& data, Timestamp timestamp) const;
  bool hasTooFewMeasurements() const override;

  void addPriorFactors(CalibratorI & calib, backend::ErrorTermReceiver & errorTermReceiver, double priorFactor) const;

  double getMaximalTimeGap() const;

  virtual ~Imu();
 protected:
  void registerWithModel() override;
  void setActive(bool spatial, bool temporal) override;

 private:
  void addMeasurementErrorTerms(CalibratorI & calib, const EstConf & ec, backend::ErrorTermReceiver & errorTermReceiver, bool observeOnly) const override;
  std::shared_ptr<Measurements> measurements_;
  const bool useAcc_, useGyro_;

  /// Variance for Accelerometer X measurements
  double accXVariance;
  /// Variance for Accelerometer Y measurements
  double accYVariance;
  /// Variance for Accelerometer Z measurements
  double accZVariance;
  /// Statistical random walk value for the accelerometer
  double accRandomWalk;

  /// Variance for Gyroscope X measurements
  double gyroXVariance;
  /// Variance for Gyroscope Y measurements
  double gyroYVariance;
  /// Variance for Gyroscope Z measurements
  double gyroZVariance;
  /// Statistical random walk value for the accelerometer
  double gyroRandomWalk;

  Bias accBias, gyroBias;

  int minimalMeasurementsPerBatch;
  const Frame & inertiaFrame;
};

} /* namespace calibration */
} /* namespace aslam */

#endif /* H657A8A48_567D_4372_A5E6_CA05BA2932B4 */
