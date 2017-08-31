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

PoseSensor::PoseSensor(Model& model, std::string name, sm::value_store::ValueStoreRef config) :
  AbstractPoseSensor(model, name, config),
  covPosition_(getMyConfig().getChild("covPosition"), 3),
  covOrientation_(getMyConfig().getChild("covOrientation"), 3),
  targetFrame_(getModel().getFrame(getMyConfig().getString("targetFrame"))),
  absoluteMeasurements_(getMyConfig().getBool("absoluteMeasurements", true))
{
  if(isUsed()) {
    LOG(INFO)
      << getName() << ":covPosition=\n" << covPosition_.getValueSqrt() << std::endl
      << "covPosition\n" << covOrientation_.getValueSqrt();
  }
}
void PoseSensor::writeConfig(std::ostream& out) const {
  AbstractPoseSensor::writeConfig(out);
  MODULE_WRITE_PARAMETER(targetFrame_);
  MODULE_WRITE_PARAMETER(absoluteMeasurements_);
}

PoseSensor::~PoseSensor() {
}

void PoseSensor::addInputTo(Timestamp t, const PoseMeasurement& pose, ModuleStorage& storage) const {
  addMeasurement(t, pose, storage);
}

void PoseSensor::addMeasurement(const Timestamp t, const Eigen::Vector4d& quat, const Eigen::Vector3d& trans, ModuleStorage & storage) const {
  PoseMeasurement p;
  if(!isInvertInput()){
    p.t = trans;
    p.q = quat;
  } else {
    auto T = sm::kinematics::Transformation(quat, trans).inverse();
    p.q = T.q();
    p.t = T.t();
  }
  addMeasurement(t, p, storage);
}

void PoseSensor::addMeasurement(const Timestamp t, const PoseMeasurement& pose, ModuleStorage & storage) const
{
  getMeasurementsMutable(storage).emplace_back(t, pose);
}

void PoseSensor::addMeasurementErrorTerms(CalibratorI& calib, const EstConf & /*ec*/, ErrorTermReceiver & problem, const bool observeOnly) const {
  const std::string errorTermGroupName = getName() + "Pose";
  const ModuleStorage & storage = calib.getCurrentStorage();
  if(!hasMeasurements(storage)){
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
  for (auto & m : getAllMeasurements(storage)) {
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
    aslam::backend::TransformationExpression T_m_s = getTransformationExpressionToAtMeasurementTimestamp(calib, timestamp, targetFrame_, true);
    Timestamp lowerTimestamp;

    const auto sigma2_t = getCovPosition().getValue();
    const auto sigma2_q = getCovOrientation().getValue();

    if(absoluteMeasurements_){
      e_pose.reset(new ErrorTermPose(T_m_s, poseMeasurement, sigma2_t, sigma2_q, etgr));
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
          = sm::kinematics::Transformation(lastPoseMeasurement->q, lastPoseMeasurement->t).inverse()
          * sm::kinematics::Transformation(poseMeasurement.q, poseMeasurement.t);
        e_pose.reset(new ErrorTermPose(last_T_m_s.inverse() * T_m_s, deltaT.t(), deltaT.q(), sigma2_t, sigma2_t, etgr));
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

static bool isOutlier_(const PoseMeasurement& p) {
  return std::isnan(p.t[0]);
}
static PoseMeasurement createOutlier_() {
  PoseMeasurement outlier;
  outlier.t[0] = std::numeric_limits<double>::signaling_NaN();
  CHECK(isOutlier_(outlier));
  return outlier;
}

const PoseMeasurement PoseSensor::Outlier = createOutlier_();

bool PoseSensor::isOutlier(const PoseMeasurement& p) const {
  return isOutlier_(p);
}

} /* namespace calibration */
} /* namespace aslam */
