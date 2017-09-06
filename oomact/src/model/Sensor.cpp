#include "aslam/calibration/model/Sensor.hpp"

#include <ostream>

#include <boost/make_shared.hpp>
#include <glog/logging.h>

#include <aslam/backend/Vector2RotationQuaternionExpressionAdapter.hpp>

#include <aslam/calibration/model/Model.h>
#include <aslam/calibration/model/ModuleTools.h>
#include <aslam/calibration/calibrator/CalibratorI.hpp> //TODO A remove this and use frames instead!
#include <aslam/calibration/SensorId.hpp>
#include <aslam/calibration/tools/tools.h>
#include <aslam/calibration/tools/TypeName.h>

size_t std::hash < aslam::calibration::SensorId >::operator()(const aslam::calibration::SensorId & sensorId) const
{
  return std::hash<size_t>()(sensorId.getValue());
}

namespace aslam {
namespace calibration {

std::ostream & operator << (std::ostream & o, SensorId id){
  return o << id.getValue();
}

Sensor::Sensor(Model & model, std::string name, ValueStoreRef config) :
    Module(model, name, config),
    PoseCv(this),
    DelayCv(this),
    ObserverMinimal(this),
    CalibratableMinimal(this),
    id(isUsed()? model.createNewSensorId() : NoSensorId),
    maximalExpectedGap(config.getDouble(name + "/maximalExpectedGap", -1.0)), //TODO C RENAME maximalExpectedGap to expectedMaximalGap
    mEstimator(getMestimator(name, getMyConfig().getChild("mestimator"), 1))
{
}

void Sensor::writeConfig(std::ostream& out) const {
  out << ", id=" << id << ", type=" << TypeName(*this) << ", parentFrame=" << getParentFrame();
  if(maximalExpectedGap > 0){
    MODULE_WRITE_PARAMETER(maximalExpectedGap);
  }
  if(hasTranslation()){
    out << ", hasTrans";
  }
  if(hasRotation()){
    out << ", hasRot";
  }
  if(hasDelay()){
    out << ", hasDelay";
  }
}

void Sensor::registerWithModel() {
  Module::registerWithModel();
  getModel().addCalibrationVariables({rotationVariable, translationVariable, getDelayVariablePtr()});
}

void Sensor::setActive(bool spatial, bool temporal){
  if(isUsed()){
    PoseCv::setActive(spatial);
    DelayCv::setActive(temporal);
  }
}

Sensor::~Sensor() {
}

BoundedTimeExpression Sensor::getBoundedTimestampExpression(const CalibratorI& calib, Timestamp t) const {
  auto lBound = t - getDelayUpperBound();
  auto uBound = t - getDelayLowerBound();

  auto & interval = calib.getCurrentEffectiveBatchInterval();

  auto tDelayed = backend::GenericScalarExpression<Timestamp>(t) - getDelayExpression();
  return {tDelayed, std::max(interval.start, lBound), std::min(interval.end, uBound)};
}

std::ostream & operator << (std::ostream & out, const Sensor & sensor){
  return out << "Sensor(" << sensor.getName() << ", " << sensor.getId() << ")";
}

aslam::backend::TransformationExpression Sensor::getTransformationExpressionToAtMeasurementTimestamp(const CalibratorI& calib, Timestamp t, const Frame & to, bool ignoreBounds) const {
  if(hasDelay()){
    if (!ignoreBounds && (!calib.getCurrentEffectiveBatchInterval().contains(t, *this)))
      return aslam::backend::TransformationExpression();
    return getTransformationExpressionTo(calib.getModelAt(getBoundedTimestampExpression(calib, t), 0, {true}), to);
  } else {
    if (!ignoreBounds && !calib.getCurrentEffectiveBatchInterval().contains(t))
      return aslam::backend::TransformationExpression();

    return getTransformationExpressionTo(calib.getModelAt(t, 0, {true}), to);
  }
}

aslam::backend::TransformationExpression Sensor::getTransformationExpressionTo(const ModelAtTime & robotModel, const Frame & to) const {
  return robotModel.getTransformationToFrom(to, this->getParentFrame()) * getTransformationToParentExpression();
}

sm::kinematics::Transformation Sensor::getTransformationTo(const ModelAtTime& robotModel, const Frame& to) const {
  return getTransformationExpressionTo(robotModel, to).toTransformationMatrix();
}

sm::kinematics::Transformation Sensor::getTransformationTo(const CalibratorI & calib, const Frame& to) const {
  return getTransformationTo(calib.getModelAt(Timestamp::Zero(), 0, {false, false}), to);
}

Interval Sensor::getCurrentMeasurementTimestampRange(const CalibratorI & /*calib*/) const {
  throw std::runtime_error("getCurrentMeasurementTimestampRange not implemented for " + getName() + " (" + TypeName(*this) + ")");
}

}
}
