#include <aslam/calibration/model/sensors/MotionCaptureSensor.hpp>

#include <aslam/backend/TransformationExpression.hpp>
#include <glog/logging.h>

#include <aslam/calibration/model/Sensor.hpp>
#include <aslam/calibration/tools/ErrorTermStatistics.h>
#include <boost/make_shared.hpp>
#include "aslam/calibration/error-terms/ErrorTermPose.h"
#include "aslam/calibration/calibrator/CalibratorI.hpp"
#include <aslam/calibration/tools/ErrorTermStatisticsWithProblemAndPredictor.h>
#include <aslam/calibration/model/Model.h>
#include "aslam/calibration/algo/MotionCaptureSource.hpp"
#include "aslam/calibration/data/MeasurementsContainer.h"

namespace aslam {
namespace calibration {

struct MotionCaptureSensorMeasurements{
  PoseMeasurements poses;
};


MotionCaptureSystem::MotionCaptureSystem(Model& model, std::string name, sm::value_store::ValueStoreRef config):
  Module(model, name, config),
  PoseCv(this),
  DelayCv(this),
  CalibratableMinimal(this)
{
}


void MotionCaptureSystem::writeConfig(std::ostream& out) const {
  if(hasTranslation()){ //TODO C deduplicate with sensor
    out << ", hasTrans";
  }
  if(hasRotation()){
    out << ", hasRot";
  }
  if(hasDelay()){
    out << ", hasDelay";
  }
}

void MotionCaptureSystem::registerWithModel() {
  Module::registerWithModel();
  getModel().addCalibrationVariables({rotationVariable, translationVariable, getDelayVariablePtr()});
}

void MotionCaptureSystem::setActive(bool spatial, bool temporal) {
  if(isUsed()){
    PoseCv::setActive(spatial);
    DelayCv::setActive(temporal);
  }
}

MotionCaptureSensor::MotionCaptureSensor(MotionCaptureSystem& motionCaptureSystem, std::string name, sm::value_store::ValueStoreRef config) :
  AbstractPoseSensor(motionCaptureSystem.getModel(), name, config),
  motionCaptureSystem(motionCaptureSystem),
  covPosition(getMyConfig().getChild("covPosition"), 3),
  covOrientation(getMyConfig().getChild("covOrientation"), 3)
{
  if(isUsed()) {
    if(!hasDelay()){
      DelayCv::operator=(motionCaptureSystem);
    }

    LOG(INFO)
      << getName() << ":covPosition=\n" << covPosition.getValueSqrt() << std::endl
      << "covPosition\n" << covOrientation.getValueSqrt();
  }
}

MotionCaptureSensor::~MotionCaptureSensor() {
}

PoseMeasurements MotionCaptureSensor::getMeasurements(Timestamp from, Timestamp till) const
{
  PoseMeasurements poses;
  if(getMotionCaptureSource()){
    Eigen::Matrix3d cov_t = covPosition.getValue();
    Eigen::Matrix3d cov_o = covOrientation.getValue(); //TODO C this should be per model
    auto posesFromSource = getMotionCaptureSource()->getPoses(from, till);
    poses.reserve(posesFromSource.size());
    for(auto & p : posesFromSource){
      poses.emplace_back(p.time, {p.p, cov_t, p.q, cov_o});
    }
  }
  return poses;
}

void MotionCaptureSensor::preProcessNewWindow(CalibratorI& calib) {
  if(getMotionCaptureSource() && !measurements){
    auto currentEffectiveBatchInterval = calib.getCurrentEffectiveBatchInterval();
    measurements = std::make_shared<MotionCaptureSensorMeasurements>();
    if(hasDelay()){
      currentEffectiveBatchInterval.start += getDelayLowerBound();
      currentEffectiveBatchInterval.end += getDelayUpperBound();
    }
    measurements->poses = getMeasurements(currentEffectiveBatchInterval.start, currentEffectiveBatchInterval.end);
    LOG(INFO) << "Found " << measurements->poses.size() << " motion capture measurements for " << getName();
  }
}

Interval MotionCaptureSensor::getSnappedWindow(CalibratorI& calib, const Interval& i) { //TODO C use interval parameter!
  SM_ASSERT_FALSE(std::runtime_error, hasDelay(), "Delay not supported here!");
  preProcessNewWindow(calib);
  SM_ASSERT_FALSE(std::runtime_error, measurements->poses.empty(), "Could not find measurements!");

  Interval res(measurements->poses.front().first, measurements->poses.back().first);
  SM_ASSERT_GE(std::runtime_error, res.start, i.start, "");
  SM_ASSERT_LE(std::runtime_error, res.end, i.end, "");
  return res;
}

void MotionCaptureSensor::addMeasurementErrorTerms(CalibratorI& calib, const EstConf & /*ec*/, ErrorTermReceiver & problem, const bool observeOnly) const {
  const std::string errorTermGroupName = getName() + "Pose";
  if(!measurements || measurements->poses.empty()){
    LOG(WARNING) << "No measurements available for " << errorTermGroupName;
    return;
  }

  ErrorTermStatisticsWithProblemAndPredictor es(calib, errorTermGroupName, problem, observeOnly);

  ErrorTermGroupReference etgr(errorTermGroupName);

  aslam::backend::TransformationExpression mCSFromGlobalTransformation = motionCaptureSystem.getTransformationToParentExpression().inverse();

  auto interval = calib.getCurrentEffectiveBatchInterval();

  auto uLow = interval.start + getDelayUpperBound();
  auto uUpp = interval.end + getDelayLowerBound();

  auto & delay = getDelayExpression();
  Timestamp currentDelay = delay.evaluate();
  if(currentDelay < getDelayLowerBound() || currentDelay > getDelayUpperBound()){
    throw std::runtime_error("Delay already out of bounds!");
  }

  for (auto & m : measurements->poses) {
    const Timestamp timestamp = m.first;
    const bool timestampIsPossiblyOutOfBounds = uLow > timestamp || uUpp < timestamp;
    if(timestampIsPossiblyOutOfBounds && !hasDelay()){
      LOG(INFO) << "Dropping out of bounds pose measurement at " << calib.secsSinceStart(timestamp) << "!";
      continue;
    }

    const auto & pose = m.second;

    aslam::backend::TransformationExpression T_m_s = getTransformationExpressionToAtMeasurementTimestamp(calib, timestamp, motionCaptureSystem.getParentFrame(), true);
    boost::shared_ptr<ErrorTermPose> e_pose(new ErrorTermPose(aslam::backend::TransformationExpression(mCSFromGlobalTransformation * T_m_s), pose, etgr));
    if (timestampIsPossiblyOutOfBounds) {
      LOG(INFO) << "Adding conditional PoseErrorTerm for pose measurement at " << calib.secsSinceStart(timestamp) << " because it could go out of bounds!";
      if(uLow > timestamp){
        e_pose = addConditionShared<ErrorTermPose>(*e_pose, [=](){ return timestamp - delay.evaluate() >= interval.start; });
      }
      if(uUpp < timestamp){
        e_pose = addConditionShared<ErrorTermPose>(*e_pose, [=](){ return timestamp - delay.evaluate() <= interval.end; });
      }
    }

    es.add(timestamp, e_pose, false);
  }
  es.printInto(LOG(INFO));
}

const PoseMeasurements& MotionCaptureSensor::getAllMeasurements() const {
  CHECK(measurements) << "Use hasMeasurements to test for measurements first!";
  return measurements->poses;
}

void MotionCaptureSensor::clearMeasurements() {
  measurements.reset();
}

} /* namespace calibration */
} /* namespace aslam */

