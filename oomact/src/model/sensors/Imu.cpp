#include <aslam/calibration/model/sensors/Imu.h>

#include <glog/logging.h>

#include <boost/make_shared.hpp>
#include <bsplines/NsecTimePolicy.hpp>
#include <bsplines/EuclideanBSpline.hpp>
#include <aslam/backend/QuadraticIntegralError.hpp>
#include <aslam/backend/Vector2RotationQuaternionExpressionAdapter.hpp>
#include <aslam/splines/OPTBSpline.hpp>

#include "aslam/calibration/calibrator/CalibratorI.hpp"
#include "aslam/calibration/data/AccelerometerMeasurement.h"
#include "aslam/calibration/data/GyroscopeMeasurement.h"
#include <aslam/calibration/DesignVariableReceiver.hpp>
#include <aslam/calibration/algo/splinesToFile.h>
#include <aslam/calibration/model/Model.h>
#include <aslam/calibration/model/fragments/TrajectoryCarrier.h>
#include <aslam/calibration/model/ModuleTools.h>
#include <aslam/calibration/model/StateCarrier.h>
#include "aslam/calibration/error-terms/ErrorTermAccelerometer.h"
#include "aslam/calibration/error-terms/ErrorTermGyroscope.h"
#include "aslam/calibration/tools/ErrorTermStatisticsWithProblemAndPredictor.h"
#include "aslam/calibration/tools/SplineWriter.h"


namespace aslam {
namespace calibration {

Imu::Imu(Model& model, const std::string& name, sm::value_store::ValueStoreRef config) :
    Sensor(model, name, config),
    measurements_(std::make_shared<Measurements>()),
    useAcc_(getMyConfig().getBool("acc/used", true)),
    useGyro_(getMyConfig().getBool("gyro/used", true)),
    enforceAccCovariance_(getMyConfig().getBool("acc/enforceCovariance", false)),
    enforceGyroCovariance_(getMyConfig().getBool("gyro/enforceCovariance", false)),
    accBias(*this, "accBias", getMyConfig().getChild("acc")),
    gyroBias(*this, "gyroBias", getMyConfig().getChild("gyro")),
    minimalMeasurementsPerBatch(getMyConfig().getInt("minimalMeasurementsPerBatch", 100)),
    inertiaFrame(getModel().getFrame(getMyConfig().getString("inertiaFrame")))
{
  if(isUsed()){
    SM_ASSERT_GE(std::runtime_error, minimalMeasurementsPerBatch, 0, "");

    if(useAcc_){
      accXVariance = getMyConfig().getDouble("acc/noise/accXVariance");
      accYVariance = getMyConfig().getDouble("acc/noise/accYVariance");
      accZVariance = getMyConfig().getDouble("acc/noise/accZVariance");
      accRandomWalk = getMyConfig().getDouble("acc/noise/accRandomWalk");
    }

    if(useGyro_){
      gyroXVariance = getMyConfig().getDouble("gyro/noise/gyroXVariance");
      gyroYVariance = getMyConfig().getDouble("gyro/noise/gyroYVariance");
      gyroZVariance = getMyConfig().getDouble("gyro/noise/gyroZVariance");
      gyroRandomWalk = getMyConfig().getDouble("gyro/noise/gyroRandomWalk");
    }
  }

//TODO C support MESTIMATORS:  setMEstimator(boost::shared_ptr<aslam::backend::MEstimator>(new aslam::backend::CauchyMEstimator(10)));
}

void Imu::writeConfig(std::ostream& out) const {
  MODULE_WRITE_PARAMETER(inertiaFrame);

  //TODO D write bias mode

  MODULE_WRITE_FLAG(useAcc_);
  if(useAcc_){
    MODULE_WRITE_PARAMETER(accXVariance);
    MODULE_WRITE_PARAMETER(accYVariance);
    MODULE_WRITE_PARAMETER(accZVariance);
    MODULE_WRITE_FLAG(enforceAccCovariance_);
    if(accBias.isUsingSpline()){
      MODULE_WRITE_PARAMETER(accRandomWalk);
    }
  }
  MODULE_WRITE_FLAG(useGyro_);
  if(useGyro_){
    MODULE_WRITE_PARAMETER(gyroXVariance);
    MODULE_WRITE_PARAMETER(gyroYVariance);
    MODULE_WRITE_PARAMETER(gyroZVariance);
    MODULE_WRITE_FLAG(enforceGyroCovariance_);
    if(accBias.isUsingSpline()){
      MODULE_WRITE_PARAMETER(gyroRandomWalk);
    }
  }

  MODULE_WRITE_PARAMETER(minimalMeasurementsPerBatch);
}

void Imu::registerWithModel() {
  Sensor::registerWithModel();
  if(useAcc_)
    accBias.registerCalibrationVariables(getModel());
  if(useGyro_)
    gyroBias.registerCalibrationVariables(getModel());
}

Imu::~Imu() = default;

typedef aslam::splines::OPTBSpline<typename bsplines::EuclideanBSpline<Eigen::Dynamic, 3, bsplines::NsecTimePolicy>::CONF>::BSpline BiasSpline;

class BiasBatchState : public BatchState {
 public:
  BiasBatchState(const TrajectoryCarrier & carrier, const std::string & name);
  void writeToFile(const CalibratorI & calib, const std::string & pathPrefix) const override;
  void addToProblem(DesignVariableReceiver & problem) const;

 private:
  std::string name_;
  BiasSpline biasSpline;
  friend Bias;
  friend Imu;
};


Bias::Bias(Module & m, const std::string & name, sm::value_store::ValueStoreRef config) :
    NamedMinimal(name)
{
  if (config.getBool("hasBias", true)) {
    biasVector = m.createCVIfUsed<EuclideanPointCv>(config.getChild("biasVector"), name);
    if (biasVector) {
      mode_ = Mode::Vector;
      biasVectorExpression = biasVector->toExpression();
    } else if (m.isUsed()) {
      mode_ = Mode::Spline;
      biasSplineCarrier.reset(new TrajectoryCarrier(config.getChild("biasSpline")));
    }
  } else {
    mode_ = Mode::None;
  }
}

void Bias::registerCalibrationVariables(Model& model) {
  if (biasVector) {
    model.addCalibrationVariables({biasVector});
  }
}

void Bias::initState(CalibratorI & calib){
  if (isUsingSpline()) {
    state_ = std::make_shared<BiasBatchState>(*biasSplineCarrier, getName());
    const auto & interval = calib.getCurrentEffectiveBatchInterval();
    const double elapsedTime = interval.getElapsedTime();
    const int numSegments = std::ceil(biasSplineCarrier->getKnotsPerSecond() * elapsedTime);
    LOG(INFO)<< "using IMU bias numSegments=" << numSegments << " for " << elapsedTime << " seconds";

    //TODO D make bias initial guess based on IMU measurements!

    state_->biasSpline.initConstantUniformSpline(interval.start, interval.end, numSegments, Eigen::Vector3d::Zero());
  }
}

void Bias::addToBatch(bool stateActive, BatchStateReceiver & batchStateReceiver, DesignVariableReceiver & problem) {
  if(state_){
    problem.addSplineDesignVariables(state_->biasSpline, stateActive);
    batchStateReceiver.addBatchState(*this, state_);
  }
}

bool Imu::initState(CalibratorI& calib) {
  if(isUsed()){
    accBias.initState(calib);
    gyroBias.initState(calib);
  }
  return true;
}
void Imu::addToBatch(const Activator & stateActivator, BatchStateReceiver & batchStateReceiver, DesignVariableReceiver & problem) {
  const bool stateActive = stateActivator.isActive(*this);
  if(useAcc_){
    accBias.setActive(stateActive);
    accBias.addToBatch(stateActive, batchStateReceiver, problem);
  }
  if(useGyro_){
    gyroBias.setActive(stateActive);
    gyroBias.addToBatch(stateActive, batchStateReceiver, problem);
  }
}

void Imu::setActive(bool spatial, bool temporal) {
  Sensor::setActive(spatial, temporal);
}

BiasBatchState::BiasBatchState(const TrajectoryCarrier & carrier, const std::string & name)
    : name_(name), biasSpline(carrier.getSplineOrder())
{
}

void BiasBatchState::writeToFile(const CalibratorI & calib, const std::string& pathPrefix) const {
  writeSpline(biasSpline, calib.getOptions().getSplineOutputSamplePeriod(), pathPrefix + name_);
}


void Imu::addAccelerometerMeasurement(CalibratorI & calib, const AccelerometerMeasurement& data, Timestamp timestamp) const {
  calib.addMeasurementTimestamp(timestamp, *this);
  measurements_->accelerometer.emplace_back(timestamp, data);
}

void Imu::addGyroscopeMeasurement(CalibratorI & calib, const GyroscopeMeasurement& data, Timestamp timestamp) const {
  calib.addMeasurementTimestamp(timestamp, *this);
  measurements_->gyroscope.emplace_back(timestamp, data);
}

void Imu::addInputTo(Timestamp t, const AccelerometerMeasurement& input, ModuleStorage&) const {
  measurements_->accelerometer.emplace_back(t, input);
}

void Imu::addInputTo(Timestamp t, const GyroscopeMeasurement& input, ModuleStorage&) const {
  measurements_->gyroscope.emplace_back(t, input);
}

void Imu::clearMeasurements() {
  measurements_->accelerometer.clear();
  measurements_->gyroscope.clear();
}


using namespace aslam::backend;

template<typename BSpline_>
struct BiasModelIntegrationErrorExpressionFactory {
  typedef BSpline_ BSplineT;
  typedef typename BSplineT::time_t TimeT;
  enum { PointSize = 3 };
  typedef VectorExpression<PointSize> value_expression_t;
  typedef Eigen::Matrix<double, 1, 1>  value_t;

  const BSplineT & _bspline;
  const Eigen::Matrix<double, PointSize, PointSize> _sqrtInvR;

  BiasModelIntegrationErrorExpressionFactory(const BSplineT & bspline, Eigen::Matrix<double, PointSize, PointSize> sqrtInvR) : _bspline(bspline), _sqrtInvR(sqrtInvR) {}

  inline value_expression_t operator()(TimeT time) const {
    return _bspline.template getExpressionFactoryAt<1>(time).getValueExpression(1);
  }
};

template <typename SplineT>
void addBiasModelErrorTerms(CalibratorI & calib, std::string name, ErrorTermReceiver & errorTermReceiver, const SplineT & spline, const Eigen::MatrixXd & sqrtInvR, bool observeOnly, int numberOfIntegrationPoints = -1){
  BiasModelIntegrationErrorExpressionFactory<SplineT> integrationFunctor(spline, sqrtInvR);
  if(numberOfIntegrationPoints < 0){
    numberOfIntegrationPoints = (spline.getAbsoluteNumberOfSegments() + spline.getSplineOrder()) * 2;
  }

  ErrorTermStatisticsWithProblemAndPredictor stat(calib, name + "Bias", errorTermReceiver, observeOnly);

  aslam::backend::integration::addQuadraticIntegralExpressionErrorTerms<aslam::backend::integration::DefaultAlgorithm>(stat, spline.getMinTime(), spline.getMaxTime(), numberOfIntegrationPoints, integrationFunctor, sqrtInvR);

  stat.printInto(LOG(INFO));
}

template <typename T, typename ErrorTermFactory>
void addImuErrorTerms(CalibratorI & calib, const Imu & imu, std::string name, const MeasurementsContainer<T>& measurements, ErrorTermFactory etFactory, ErrorTermReceiver & errorTermReceiver, bool observeOnly) {
  LOG(INFO) << "Adding " << measurements.size() << " " << name << " error terms";

  ErrorTermStatisticsWithProblemAndPredictor statWPAP(calib, name, errorTermReceiver, observeOnly);
  const Interval & interval = calib.getCurrentEffectiveBatchInterval();

  Timestamp minTime = interval.end, maxTime = interval.start;

  for (auto & m : measurements) {
    Timestamp timestamp = m.first;
    if (!interval.contains(timestamp, imu)){
      LOG(INFO) << name << " measurement out of spline range at " << calib.secsSinceStart(timestamp) << "s.";
      continue;
    }

    if(minTime > timestamp) minTime = timestamp;
    if(maxTime < timestamp) maxTime = timestamp;

    auto e = etFactory(timestamp, m.second);

    if(imu.getMEstimator()){
      e->setMEstimatorPolicy(imu.getMEstimator());
    }

    VLOG(2) << "Cost function " << name << " : " << e->evaluateError() << " count: " << statWPAP.getCounter() << " timestamp: " << calib.secsSinceStart(timestamp) << "s,";
    statWPAP.add(timestamp, e);
  }
  statWPAP.printInto(LOG(INFO)) << " Between " << calib.secsSinceStart(minTime) << "s and " << calib.secsSinceStart(maxTime) << "s.";
}

template <typename T>
bool isCovarianceAvailable(T&t){
  return t.cov(0,0) > 0.0;
}

void Imu::addMeasurementErrorTerms(CalibratorI & calib, const EstConf & /*ec*/, ErrorTermReceiver & errorTermReceiver, bool observeOnly) const {
  if(useAcc_){
    const std::string accelerometerName = getName() + "Accelerometer";
    if(accBias.isUsingSpline()){
      addBiasModelErrorTerms(calib, accelerometerName, errorTermReceiver, accBias.state_->biasSpline, Eigen::Matrix3d::Identity() / accRandomWalk, observeOnly);
    }

    //TODO C solve gravity vector problem. Each model has a gravity vector?
    auto g_m = calib.getModel().getGravity().getVectorExpression();
    Eigen::Matrix3d covarianceMatrix = Eigen::Vector3d(accXVariance, accYVariance, accZVariance).asDiagonal();
    ErrorTermGroupReference etgr(getName() + "Acc");
    addImuErrorTerms(
        calib, *this, accelerometerName, measurements_->accelerometer,
        [&, this](const Timestamp timestamp, const AccelerometerMeasurement & m){
          auto modelAt = calib.getModelAt(*this, timestamp, 2, {false});
          return boost::make_shared<ErrorTermAccelerometer>(
              modelAt.getAcceleration(getFrame(), inertiaFrame),
              getTransformationExpressionTo(modelAt, inertiaFrame).toRotationExpression().inverse(),
              g_m,
              accBias.getBiasExpression(timestamp),
              m.a,
              isCovarianceAvailable(m) && !enforceAccCovariance_ ? m.cov : covarianceMatrix,
              etgr);
        },
        errorTermReceiver,
        observeOnly
      );
  }
  if(useGyro_){
    const std::string gyroscopeName = getName() + "Gyroscope";
    if(gyroBias.isUsingSpline()){
      addBiasModelErrorTerms(calib, gyroscopeName, errorTermReceiver, gyroBias.state_->biasSpline, Eigen::Matrix3d::Identity() / gyroRandomWalk, observeOnly);
    }
    Eigen::Matrix3d covarianceMatrix = Eigen::Vector3d(gyroXVariance, gyroYVariance, gyroZVariance).asDiagonal();
    ErrorTermGroupReference etgr(getName() + "Gyro");
    addImuErrorTerms(
        calib, *this, gyroscopeName, measurements_->gyroscope,
        [&, this](const Timestamp timestamp, const GyroscopeMeasurement & m){
          auto modelAt = calib.getModelAt(*this,  timestamp, 1, {false});
          return boost::make_shared<ErrorTermGyroscope>(
              getTransformationExpressionTo(modelAt, inertiaFrame).toRotationExpression().inverse() * modelAt.getAngularVelocity(getParentFrame(), inertiaFrame),
              gyroBias.getBiasExpression(timestamp),
              m.w,
              isCovarianceAvailable(m) && !enforceGyroCovariance_ ? m.cov : covarianceMatrix,
              etgr
            );
        },
        errorTermReceiver,
        observeOnly
      );
  }
}

double Imu::getMaximalTimeGap() const {
  if(measurements_){
    Timestamp gGap(measurements_->gyroscope.getMaximalTimeGap());
    Timestamp aGap(measurements_->accelerometer.getMaximalTimeGap());
    return std::max(double(gGap), double(aGap));
  } else {
    return 0.0;
  }
}

bool Imu::hasTooFewMeasurements() const {
  return getGyroscopeMeasurements().size() < size_t(minimalMeasurementsPerBatch);
}

aslam::backend::EuclideanExpression Bias::getBiasExpression(Timestamp t) const {
  if(isUsingSpline()){
    SM_ASSERT_NOTNULL(std::runtime_error, state_, "");
    return state_->biasSpline.getExpressionFactoryAt<0>(t).getValueExpression();
  } else {
    return biasVectorExpression;
  }
}

void Imu::addPriorFactors(CalibratorI & calib, ErrorTermReceiver & errorTermReceiver, double priorFactor) const {
  const double invSigma = 1e-2 * priorFactor;
  Timestamp startTime = calib.getCurrentEffectiveBatchInterval().start;
  if(accBias.isUsingSpline()) errorTermReceiver.addErrorTerm(backend::toErrorTerm(accBias.getBiasExpression(startTime), Eigen::Matrix3d::Identity()*invSigma));
  if(gyroBias.isUsingSpline()) errorTermReceiver.addErrorTerm(backend::toErrorTerm(gyroBias.getBiasExpression(startTime), Eigen::Matrix3d::Identity()*invSigma));
}

} /* namespace calibration */
} /* namespace aslam */
