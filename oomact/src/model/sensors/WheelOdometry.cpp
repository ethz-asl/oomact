#include <aslam/calibration/model/sensors/WheelOdometry.h>

#include <ostream>

#include <glog/logging.h>

#include <aslam/backend/EuclideanExpression.hpp>
#include <aslam/backend/OptimizationProblemBase.hpp>
#include <aslam/backend/RotationExpression.hpp>
#include <aslam/backend/ScalarExpression.hpp>

#include <aslam/calibration/algo/PredictionWriter.h>
#include "aslam/calibration/calibrator/CalibratorI.h"
#include "aslam/calibration/data/WheelSpeedsMeasurement.h"
#include <aslam/calibration/model/Model.h>
#include <aslam/calibration/model/ModuleTools.h>
#include "aslam/calibration/error-terms/ErrorTermWheel.h"
#include <aslam/calibration/tools/ErrorTermStatistics.h>
#include <aslam/calibration/tools/Interval.h>

using aslam::backend::EuclideanExpression;
using aslam::backend::RotationExpression;
using aslam::backend::ScalarExpression;

namespace aslam {
namespace calibration {

WheelOdometry::WheelOdometry(Model& model, const std::string& name, sm::value_store::ValueStoreRef config) :
  Sensor(model, name, config),
  L(createCVIfUsed<ScalarCv>("wheelBase", "L")),
  R_l(createCVIfUsed<ScalarCv>("wheelRadiusLeft", "R_l")),
  R_r(createCVIfUsed<ScalarCv>("wheelRadiusRight", "R_r")),
  assumedWheelBase(getMyConfig().getDouble("assumedWheelBase")),
  assumedWheelRadiusLeft(getMyConfig().getDouble("assumedWheelRadiusLeft")),
  assumedWheelRadiusRight(getMyConfig().getDouble("assumedWheelRadiusRight")),
  lwVariance(getMyConfig().getDouble("noise/lwVariance")),
  rwVariance(getMyConfig().getDouble("noise/rwVariance")),
  minimalMeasurementsPerBatch(getMyConfig().getInt("minimalMeasurementsPerBatch", 10)),
  groundFrame_(getModel().getFrame(getMyConfig().getString("groundFrame")))
{
  if(isUsed()){
    SM_ASSERT_GE(std::runtime_error, minimalMeasurementsPerBatch, 0, "");
    LOG(INFO) << "Using wheel odometry.";
    if(isToBeCalibrated())
      LOG(INFO) << "Calibrating wheel odometry!";
    if(!L->isToBeEstimated())
      LOG(INFO) << "Not estimating wheel distance L!";
    if(getDelayVariable().isToBeEstimated())
      LOG(INFO) << "Estimating wheelDelay!";

    LOG(INFO) << "Assuming a wheel delay of " << double(getDelay()) << " in [" <<
        double(getDelayLowerBound()) << "," << double(getDelayUpperBound()) << "].";
  }
}


void WheelOdometry::registerWithModel() {
  Sensor::registerWithModel();
  getModel().addCalibrationVariables({L, R_l, R_r});
}

void WheelOdometry::setActive(bool spatial, bool temporal){
  R_l->setActive(spatial);
  R_r->setActive(spatial);
  L->setActive(spatial && L->isToBeEstimated());
  DelayCv::setActive(temporal);
}

void WheelOdometry::writeConfig(std::ostream& out) const {
  Sensor::writeConfig(out);
  MODULE_WRITE_PARAM(assumedWheelBase);
  MODULE_WRITE_PARAM(assumedWheelRadiusLeft);
  MODULE_WRITE_PARAM(assumedWheelRadiusRight);
}

void WheelOdometry::addMeasurement(CalibratorI & calib, const Timestamp t, const WheelSpeedsMeasurement& data) const {
  if(isUsed()){
    calib.addMeasurementTimestamp(t, *this);
    measurements_.push_back(std::make_pair(t, data));
  } else {
    calib.addMeasurementTimestamp(t, *this);
  }
}

void WheelOdometry::addMeasurementErrorTerms(CalibratorI & calib, const CalibrationConfI & /*ec*/, ErrorTermReceiver & problem, bool observeOnly) const {
  LOG(INFO) << "Adding " << 2 * measurements_.size() << " wheels error terms";

  Timestamp minTime = Timestamp::Numerator(std::numeric_limits<Timestamp::Integer>::max()), maxTime = InvalidTimestamp();

  auto R_l = getWheelRadiusL()->toExpression();
  auto R_r = getWheelRadiusR()->toExpression();
  auto L = getL()->toExpression();

  ErrorTermStatistics
    lwES(getName() + ".lw" + (observeOnly  ? " (OBSERVER)" : "")),
    rwES(getName() + ".rw" + (observeOnly  ? " (OBSERVER)" : ""));

  auto predictions = calib.createPredictionCollector(getName());

  const Interval & validRange = calib.getCurrentEffectiveBatchInterval();

  auto & timeDelay = getDelayExpression();

  Timestamp prevTimestamp = validRange.start;

  const auto lHalf = (L * 0.5);

  for (auto & m : measurements_) {
    if(Timestamp(m.first) <= validRange.start){
      VLOG(1) << "Skipping out of bounds wheel measurement at " << calib.secsSinceStart(m.first) << ".";
      continue;
    }

    // first of the pair of measurements in measurements container
    SM_ASSERT_LE(std::runtime_error, prevTimestamp, m.first, "Expecting monotone increasing timestamps in measurements");
    auto timestamp = ((m.first + prevTimestamp) / Timestamp(2.0));
    prevTimestamp = m.first;

    auto timestampDelayed = backend::GenericScalarExpression<Timestamp>(timestamp) - timeDelay;
    auto lBound = Timestamp(timestamp) - getDelayUpperBound();
    auto uBound = Timestamp(timestamp) - getDelayLowerBound();

    if(uBound > validRange.end || lBound < validRange.start){
      VLOG(1) << "Skipping wheel measurement that could go out of bounds at " << calib.secsSinceStart(timestamp) << ".";
      continue;
    }

    if(minTime > timestamp) minTime = timestamp;
    if(maxTime < timestamp) maxTime = timestamp;

    auto robot = calib.getModelAt({timestampDelayed, lBound, uBound}, 1, {});

    EuclideanExpression v_r_mwl, v_r_mwr;
    RotationExpression R_m_r;

    /*else{// if(_options.useBaseSpeed){ TODO C check this out ?
      auto R_m_r = Vector2RotationQuaternionExpressionAdapter::adapt(
          rotationExpressionFactory.getValueExpression());
      auto v_m_mr = EuclideanExpression(
          translationExpressionFactory.getValueExpression(1));
      auto v_r_mr = R_m_r.inverse() * v_m_mr;

      auto w_m_mr = -EuclideanExpression(rotationExpressionFactory.getAngularVelocityExpression());
      auto w_r_mr = R_m_r.inverse() * w_m_mr;

      auto v_r_mr_m = EuclideanExpression(Eigen::Vector3d(1.0, 0.0, 0.0)) * (R_l * m.second.left + R_r * m.second.right) * 0.5;

      auto w_r_mr_m = EuclideanExpression(Eigen::Vector3d(0.0, 0.0, 1.0)) * (R_r * m.second.right - R_l * m.second.left);
      // Z speed constraint
      double sigma2_vx = (_options.lwVariance * R_l_val*R_l_val + _options.rwVariance * R_r_val * R_r_val)/0.25;
      double sigma2_wz = (_options.lwVariance * R_l_val*R_l_val + _options.rwVariance * R_r_val * R_r_val);///(L_val * L_val);
      auto e_v = boost::make_shared<ErrorTermLinearVelocity>(v_r_mr,
                                                       v_r_mr_m,
                                                       Eigen::Vector3d(sigma2_vx,
                                                                       _options.vyVariance,
                                                                       _options.vzVariance).asDiagonal());
      errorTermReceiver.addErrorTerm(e_v);
      auto e_w = boost::make_shared<ErrorTermAngularVelocity>(w_r_mr * L,
                                                              w_r_mr_m,
                                                              (Eigen::Matrix<double,1,1>() << (sigma2_wz)).finished());
      errorTermReceiver.addErrorTerm(e_w);

      initJ += e_w->evaluateError() + e_v->evaluateError();
      count++;
      if(_options.writeLogFile)
        DLOG(ERROR) << "Cost function Speed: Linear " << e_v->evaluateError() << " Angular: "
        << e_w->evaluateError() <<  " count: " << count << " timestamp: " << timestamp;
    }*/
    {
      auto R_m_r = getTransformationExpressionTo(robot, groundFrame_).toRotationExpression();
      auto v_m_mr = robot.getVelocity(groundFrame_, getReferenceFrame());
      auto v_r_mr = R_m_r.inverse() * v_m_mr;

      auto w_m_mr = robot.getAngularVelocity(groundFrame_, getReferenceFrame());
      auto w_r_mr = R_m_r.inverse() * w_m_mr;

      // Computing the velocity in the wheel frame:
      // v_r_mwl = v_wl_mwl, since the two frames are aligned
      auto t_r_rwl = EuclideanExpression(Eigen::Vector3d(0.0, 1.0, 0.0)) * lHalf;
      auto t_r_rwr = EuclideanExpression(Eigen::Vector3d(0.0, -1.0, 0.0)) * lHalf;

      v_r_mwl = v_r_mr + w_r_mr.cross(t_r_rwl);
      v_r_mwr = v_r_mr + w_r_mr.cross(t_r_rwr);
    }

    // TODO B improve Odometry error model (this is basically a different elliptic distribution) and make thresholds parameters or use self-tuning
    auto motionBasedFactor = [&](double v){
      if(fabs(v) < 1e-6){
        return 1e-4;
      }
      return 1.;
    };

    auto e_rlw = boost::make_shared<ErrorTermWheel>(v_r_mwl, R_l, m.second.left, motionBasedFactor(m.second.left) * lwVariance);
    auto e_rrw = boost::make_shared<ErrorTermWheel>(v_r_mwr, R_r, m.second.right, motionBasedFactor(m.second.right) * rwVariance);

    if(!observeOnly){
      problem.addErrorTerm(e_rlw);
      problem.addErrorTerm(e_rrw);
    }
    lwES.add(e_rlw);
    rwES.add(e_rrw);

    if(calib.getOptions().getPredictResults()){
      predictions->add([=](std::ostream &outPred, std::ostream &outMeasure, std::ostream &outErr, std::ostream & outNormalizedErr){
        e_rlw->evaluateError();
        e_rrw->evaluateError(); //TODO C cleanup wheel error term (using MeasurementErrorTerm) !
        outPred << timestamp << " " << (double)(e_rlw->error()(0) + e_rlw->getMeasurement()) << " " << (double)(e_rrw->error()(0) + e_rrw->getMeasurement()) <<std::endl;
        outMeasure << timestamp << " " << (double)(e_rlw->getMeasurement()) << " " << (double)(e_rrw->getMeasurement()) <<std::endl;
        outErr << timestamp << " " << (double) e_rlw->error()(0) << " "  << (double) e_rrw->error()(0) <<std::endl;
        outNormalizedErr << timestamp << " " << (double) (e_rlw->sqrtInvR() * e_rlw->error())(0) << " "  << (double) (e_rlw->sqrtInvR() * e_rlw->error())(0) << std::endl;
      });
    }
  }
  lwES.printInto(LOG(INFO));
  rwES.printInto(LOG(INFO));
}

void WheelOdometry::clearMeasurements() {
  measurements_.clear();
}
//TODO C deduplicate hasTooFewMeasurements (WheelOdometry, Imu)
bool WheelOdometry::hasTooFewMeasurements() const {
  return measurements_.size() < size_t(minimalMeasurementsPerBatch);
}

} /* namespace calibration */
} /* namespace aslam */
