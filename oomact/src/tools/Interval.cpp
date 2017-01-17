#include <aslam/calibration/tools/Interval.hpp>
#include <aslam/calibration/model/BoundedCalibrationVariable.hpp>
#include <aslam/calibration/model/Sensor.hpp>


bool aslam::calibration::Interval::contains(Timestamp t, const TimeDesignVariableCv & delay) const{
  return contains(t - delay.getUpperBound()) && contains(t - delay.getLowerBound());
}

bool aslam::calibration::Interval::contains(Timestamp t, const Sensor& sensor) const {
  if(sensor.hasDelay()){
    return contains(t, sensor.getDelayVariable());
  } else {
    return contains(t);
  }
}

bool aslam::calibration::Interval::contains(const BoundedTimeExpression& t) const {
  return contains(t.lBound) && contains(t.uBound);
}
