#include <aslam/calibration/model/sensors/PositionSensor.hpp>

#include <memory>

#include <aslam/backend/TransformationExpression.hpp>
#include <glog/logging.h>
#include <boost/make_shared.hpp>

#include "aslam/calibration/calibrator/CalibratorI.hpp"
#include "aslam/calibration/data/PositionMeasurement.h"
#include "aslam/calibration/data/MeasurementsContainer.h"
#include "aslam/calibration/error-terms/ErrorTermPosition.h"
#include <aslam/calibration/tools/ErrorTermStatistics.h>
#include <aslam/calibration/tools/ErrorTermStatisticsWithProblemAndPredictor.h>
#include <aslam/calibration/model/Model.h>
#include <aslam/calibration/model/Sensor.hpp>

namespace aslam {
namespace calibration {

void PositionSensor::addMeasurement(const PositionMeasurement& position, const Timestamp t)
{
  measurements->push_back({t, position});
}

void PositionSensor::writeConfig(std::ostream& out) const {
  Sensor::writeConfig(out);
}

PositionSensor::PositionSensor(Model& model, std::string name, sm::value_store::ValueStoreRef config) :
  Sensor(model, name, config),
  covPosition(getMyConfig().getChild("covPosition"), 3),
  targetFrame(getModel().getFrame(getMyConfig().getString("targetFrame")))
{
  if(isUsed()) {
    measurements = std::make_shared<PositionMeasurements>();
    LOG(INFO)
      << getName() << ":covPosition=\n" << covPosition.getValueSqrt() << std::endl;
  }
}

PositionSensor::~PositionSensor() {
}

void PositionSensor::addMeasurementErrorTerms(CalibratorI& calib, const EstConf & /*ec*/, ErrorTermReceiver & problem, const bool observeOnly) const {
  const std::string errorTermGroupName = getName() + "Position";
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

    auto & positionMeasurement = m.second;


    aslam::backend::TransformationExpression T_target_s = getTransformationExpressionToAtMeasurementTimestamp(calib, timestamp, targetFrame, true);
    boost::shared_ptr<ErrorTermPosition> e_position(new ErrorTermPosition(T_target_s.inverse().toEuclideanExpression(), positionMeasurement, etgr));
    if(uLow > timestamp || uUpp < timestamp){
      if(!hasDelay()){
        LOG(WARNING) << "Dropping out of bounds position measurement at " << calib.secsSinceStart(timestamp) << "!";
        continue;
      }
      LOG(INFO) << "Adding conditional PositionErrorTerm for position measurement at " << calib.secsSinceStart(timestamp) << " because it could go out of bounds!";
      if(uLow > timestamp){
        e_position = addConditionShared<ErrorTermPosition>(*e_position, [=](){ return timestamp - delay.evaluate() >= interval.start; });
      }
      if(uUpp < timestamp){
        e_position = addConditionShared<ErrorTermPosition>(*e_position, [=](){ return timestamp - delay.evaluate() <= interval.end; });
      }
    }

    es.add(timestamp, e_position, false);
  }
  es.printInto(LOG(INFO));
}


void PositionSensor::clearMeasurements() {
  measurements.reset();
}

} /* namespace calibration */
} /* namespace aslam */

