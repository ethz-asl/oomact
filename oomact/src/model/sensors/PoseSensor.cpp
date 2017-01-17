#include <aslam/calibration/model/sensors/PoseSensor.hpp>

#include <aslam/backend/TransformationExpression.hpp>
#include <boost/make_shared.hpp>
#include <glog/logging.h>
#include <memory>

#include "aslam/calibration/CalibratorI.hpp"
#include <aslam/calibration/model/Model.h>
#include <aslam/calibration/model/Sensor.hpp>
#include <aslam/calibration/tools/ErrorTermStatistics.h>
#include <aslam/calibration/data/MeasurementsContainer.h>
#include "aslam/calibration/error-terms/ErrorTermPose.h"
#include <aslam/calibration/tools/ErrorTermStatisticsWithProblemAndPredictor.h>

#include "aslam/calibration/algo/OptimizationProblemSpline.h"

namespace aslam {
namespace calibration {

void PoseSensor::addMeasurement(const PoseMeasurement& pose, const Timestamp t)
{
  measurements->push_back({t, pose});
}

void PoseSensor::writeConfig(std::ostream& out) const {
  Sensor::writeConfig(out);
}

PoseSensor::PoseSensor(Model& model, std::string name, sm::value_store::ValueStoreRef config) :
  AbstractPoseSensor(model, name, config),
  covPosition(getMyConfig().getChild("covPosition"), 3),
  covOrientation(getMyConfig().getChild("covOrientation"), 3),
  targetFrame(getModel().getFrame(getMyConfig().getString("targetFrame")))

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

  auto uLow = interval.start + getDelayUpperBound();
  auto uUpp = interval.end + getDelayLowerBound();

  auto & delay = getDelayExpression();
  Timestamp currentDelay = delay.evaluate();
  if(currentDelay < getDelayLowerBound() || currentDelay > getDelayUpperBound()){
    throw std::runtime_error("Delay already out of bounds!");
  }

  for (auto & m : *measurements) {
    Timestamp timestamp = m.first;

    auto & poseMeasurement = m.second;


    aslam::backend::TransformationExpression T_m_s = getTransformationExpressionToAtMeasurementTimestamp(calib, timestamp, targetFrame, true);
    boost::shared_ptr<ErrorTermPose> e_pose(new ErrorTermPose(T_m_s.inverse(), poseMeasurement, etgr));
    if(uLow > timestamp || uUpp < timestamp){
      if(!hasDelay()){
        LOG(WARNING) << "Dropping out of bounds pose measurement at " << calib.secsSinceStart(timestamp) << "!";
        continue;
      }
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


void PoseSensor::clearMeasurements() {
  measurements.reset();
}

bool PoseSensor::hasMeasurements() const {
  return measurements && !measurements->empty();
}

const PoseMeasurements& PoseSensor::getMeasurements() const {
  CHECK(measurements) << "Use hasMeasurements to test for measurements first!";
  return *measurements;
}

} /* namespace calibration */
} /* namespace aslam */
