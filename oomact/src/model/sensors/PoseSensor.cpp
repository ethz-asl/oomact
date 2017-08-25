#include <aslam/calibration/model/sensors/PoseSensor.hpp>

#include <cmath>
#include <memory>

#include <aslam/backend/TransformationExpression.hpp>
#include <boost/make_shared.hpp>
#include <glog/logging.h>

#include "aslam/calibration/calibrator/CalibratorI.hpp"
#include <aslam/calibration/data/MeasurementsContainer.h>
#include "aslam/calibration/error-terms/ErrorTermPose.h"
#include <aslam/calibration/model/Model.h>
#include <aslam/calibration/model/Sensor.hpp>
#include <aslam/calibration/model/ModuleTools.h>
#include <aslam/calibration/tools/ErrorTermStatistics.h>
#include <aslam/calibration/tools/ErrorTermStatisticsWithProblemAndPredictor.h>

namespace aslam {
namespace calibration {


void PoseSensor::addMeasurement(const Eigen::Vector4d& quat, const Eigen::Vector3d& trans, const Timestamp t) {
  PoseMeasurement p;
  p.t_m_mf = trans;
  p.q_m_f = quat;
  p.sigma2_t_m_mf = getCovPosition().getValue();
  p.sigma2_q_m_f = getCovOrientation().getValue();
  addMeasurement(p, t);
}

void PoseSensor::addMeasurement(const PoseMeasurement& pose, const Timestamp t)
{
  measurements->push_back({t, pose});
}

void PoseSensor::writeConfig(std::ostream& out) const {
  Sensor::writeConfig(out);
  MODULE_WRITE_PARAMETER(targetFrame);
  MODULE_WRITE_PARAMETER(absoluteMeasurements_);
}

PoseSensor::PoseSensor(Model& model, std::string name, sm::value_store::ValueStoreRef config) :
  AbstractPoseSensor(model, name, config),
  covPosition(getMyConfig().getChild("covPosition"), 3),
  covOrientation(getMyConfig().getChild("covOrientation"), 3),
  targetFrame(getModel().getFrame(getMyConfig().getString("targetFrame"))),
  absoluteMeasurements_(getMyConfig().getBool("absoluteMeasurements", true))
{
  if(isUsed()) {
    measurements = std::make_shared<PoseMeasurements>();
    LOG(INFO)
      << getName() << ":covPosition=\n" << covPosition.getValueSqrt() << std::endl
      << "covPosition\n" << covOrientation.getValueSqrt();
  }
}

PoseSensor::~PoseSensor() {
}

void PoseSensor::addMeasurementErrorTerms(CalibratorI& calib, const EstConf & /*ec*/, ErrorTermReceiver & problem, const bool observeOnly) const {
  const std::string errorTermGroupName = getName() + "Pose";
  if(!measurements || measurements->empty()){
    LOG(WARNING) << "No measurements available for " << errorTermGroupName;
    return;
  }

  ErrorTermStatisticsWithProblemAndPredictor es(calib, errorTermGroupName, problem, observeOnly);

  ErrorTermGroupReference etgr(errorTermGroupName);

  auto interval = calib.getCurrentEffectiveBatchInterval();

  auto conditionalLowerBound = interval.start + getDelayUpperBound();
  auto conditionalUpperBound = interval.end + getDelayLowerBound();
  auto certainLowerBound = interval.start + getDelayLowerBound();
  auto certainUpperBound = interval.end + getDelayUpperBound();

  auto & delay = getDelayExpression();
  Timestamp currentDelay = delay.evaluate();
  if(currentDelay < getDelayLowerBound() || currentDelay > getDelayUpperBound()){
    throw std::runtime_error("Delay already out of bounds!");
  }

  const PoseMeasurement * lastPoseMeasurement = nullptr;
  auto lastTimestamp = Timestamp::Zero();
  aslam::backend::TransformationExpression last_T_m_s;
  for (auto & m : *measurements) {
    Timestamp timestamp = m.first;
    auto & poseMeasurement = m.second;
    if(certainLowerBound > timestamp || certainUpperBound < timestamp){
      LOG(WARNING) << "Dropping out of bounds pose measurement at " << calib.secsSinceStart(timestamp) << "!";
      lastPoseMeasurement = nullptr;
      continue;
    }

    if(isOutlier(poseMeasurement)){
      LOG(INFO) << "Outlier removed at " << calib.secsSinceStart(timestamp) << ".";
      lastPoseMeasurement = nullptr;
      continue;
    }

    boost::shared_ptr<ErrorTermPose> e_pose;
    aslam::backend::TransformationExpression T_m_s = getTransformationExpressionToAtMeasurementTimestamp(calib, timestamp, targetFrame, true);
    Timestamp lowerTimestamp;

    if(absoluteMeasurements_){
      e_pose.reset(new ErrorTermPose(T_m_s, poseMeasurement, etgr));
      lowerTimestamp = timestamp;
    } else {
      if(lastPoseMeasurement == nullptr){
//        e_pose.reset(new ErrorTermPose(T_m_s, poseMeasurement, etgr)); // TODO maybe support first absolute measurement? HANDLE outlier right then..
//        if(auto me = getMEstimator()){
//          e_pose->setMEstimatorPolicy(me);
//        }
//        es.add(timestamp, e_pose, false);
      } else {
        sm::kinematics::Transformation deltaT
          = sm::kinematics::Transformation(lastPoseMeasurement->q_m_f, lastPoseMeasurement->t_m_mf).inverse()
          * sm::kinematics::Transformation(poseMeasurement.q_m_f, poseMeasurement.t_m_mf);
        e_pose.reset(new ErrorTermPose(last_T_m_s.inverse() * T_m_s, deltaT.t(), deltaT.q(), poseMeasurement.sigma2_t_m_mf, poseMeasurement.sigma2_t_m_mf, etgr));
      }
      lastPoseMeasurement = &poseMeasurement;
      last_T_m_s = std::move(T_m_s);
      lowerTimestamp = lastTimestamp;
      lastTimestamp = timestamp;
    }

    if(e_pose){
      if(conditionalLowerBound > lowerTimestamp || conditionalUpperBound < timestamp){
        if(!hasDelay()){
          LOG(WARNING) << "Dropping out of bounds pose measurement at " << calib.secsSinceStart(timestamp) << "!";
          continue;
        }
        LOG(INFO) << "Adding conditional PoseErrorTerm for pose measurement at " << calib.secsSinceStart(timestamp) << " because it could go out of bounds!";
        if(conditionalLowerBound > lowerTimestamp){
          e_pose = addConditionShared<ErrorTermPose>(*e_pose, [=](){ return lowerTimestamp - delay.evaluate() >= interval.start; });
        }
        if(conditionalUpperBound < timestamp){
          e_pose = addConditionShared<ErrorTermPose>(*e_pose, [=](){ return timestamp - delay.evaluate() <= interval.end; });
        }
      }
      if(auto me = getMEstimator()){
        e_pose->setMEstimatorPolicy(me);
      }
      es.add(timestamp, e_pose, false);
    }
  }
  es.printInto(LOG(INFO));
}


void PoseSensor::clearMeasurements() {
  measurements.reset();
}

bool PoseSensor::hasMeasurements() const {
  return measurements && !measurements->empty();
}

const PoseMeasurements& PoseSensor::getAllMeasurements() const {
  CHECK(measurements) << "Use hasMeasurements to test for measurements first!";
  return *measurements;
}

static bool isOutlier_(const PoseMeasurement& p) {
  return std::isnan(p.t_m_mf[0]);
}
static PoseMeasurement createOutlier_() {
  PoseMeasurement outlier;
  outlier.t_m_mf[0] = std::numeric_limits<double>::signaling_NaN();
  CHECK(isOutlier_(outlier));
  return outlier;
}

const PoseMeasurement PoseSensor::Outlier = createOutlier_();

bool PoseSensor::isOutlier(const PoseMeasurement& p) const {
  return isOutlier_(p);
}

} /* namespace calibration */
} /* namespace aslam */
