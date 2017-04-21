#include <aslam/calibration/model/Sensor.hpp>

#include <ostream>

#include <boost/make_shared.hpp>
#include <glog/logging.h>

#include <aslam/backend/Vector2RotationQuaternionExpressionAdapter.hpp>

#include <aslam/calibration/CommonTypes.hpp>
#include <aslam/calibration/model/Model.h>
#include <aslam/calibration/model/ModuleTools.h>
#include <aslam/calibration/CalibratorI.hpp> //TODO A remove this and use frames instead!

size_t std::hash < aslam::calibration::SensorType >::operator()(aslam::calibration::SensorType sensorType) const
{
  return std::hash<int>()((int)sensorType);
}

size_t std::hash < aslam::calibration::SensorId >::operator()(const aslam::calibration::SensorId & sensorId) const
{
  return std::hash<size_t>()(sensorId.getValue());
}


namespace aslam {
namespace calibration {
bool isPointCloudSensor(SensorType type) {
  switch(type) {
    case SensorType::POINT_CLOUD_2D:
    case SensorType::POINT_CLOUD_3D:
      return true;
    default:
      return false;
   }
}

std::ostream & operator << (std::ostream & o, SensorId id){
  return o << id.getValue();
}
std::ostream & operator << (std::ostream & o, SensorType type){
  switch(type){
    case SensorType::POINT_CLOUD_2D:
      o << "PointCloud2d";
      break;
    case SensorType::POINT_CLOUD_3D:
      o << "PointCloud3d";
      break;
    case SensorType::POSE:
      o << "Pose";
      break;
    case SensorType::IMU:
      o << "Imu";
      break;
    case SensorType::ODOMETRY:
      o << "Odometry";
      break;
    default:
      o << (int)(type);
  }
  return o;
}

Sensor::Sensor(Model & model, std::string name, ValueStoreRef config) :
    Module(model, name, config),
    PoseCv(this),
    DelayCv(this),
    ObserverMinimal(this),
    CalibratableMinimal(this),
    id(isUsed()? model.createNewSensorId() : NoSensorId),
    maximalExpectedGap(config.getDouble(name + "/maximalExpectedGap", -1.0)) //TODO C RENAME maximalExpectedGap to expectedMaximalGap
{
}

void Sensor::writeConfig(std::ostream& out) const {
  out << ", id=" << id << ", type=" << getType() << ", parentFrame=" << getParentFrame();
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
  return getTransformationTo(calib.getModelAt(Timestamp(0L), 0, {false, false}), to);
}

Interval Sensor::getCurrentMeasurementTimestampRange(const CalibratorI & /*calib*/) const {
  throw std::runtime_error("getCurrentMeasurementTimestampRange not implemented for " + getName() + " (" + typeid(*this).name() + ")");
}

}
}
