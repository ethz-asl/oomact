#include <aslam/calibration/tools/Interval.h>
#include <aslam/calibration/model/BoundedCalibrationVariable.h>
#include <aslam/calibration/model/Sensor.h>


bool aslam::calibration::Interval::contains(Timestamp t, const TimeDesignVariableCv & delay) const{
  return contains(t - delay.getUpperBound()) && contains(t - delay.getLowerBound());
}

bool aslam::calibration::Interval::containsModule(Timestamp t, const Module& module) const {
  if(auto sPtr = module.ptrAs<DelayCv>()){
    return contains(t, *sPtr);
  } else {
    return contains(t);
  }
}

bool aslam::calibration::Interval::contains(Timestamp t, const DelayCv& sensor) const {
  if(sensor.hasDelay()){
    return contains(t, sensor.getDelayVariable());
  } else {
    return contains(t);
  }
}

bool aslam::calibration::Interval::contains(const BoundedTimeExpression& t) const {
  return contains(t.lBound) && contains(t.uBound);
}
