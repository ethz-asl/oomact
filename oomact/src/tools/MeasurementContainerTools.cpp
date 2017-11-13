#include <aslam/calibration/tools/MeasurementContainerTools.h>

#include <aslam/calibration/calibrator/CalibratorI.h>
#include <aslam/calibration/model/sensors/PoseSensorI.h>
#include <aslam/calibration/model/Sensor.h>
#include <aslam/calibration/Timestamp.h>


aslam::calibration::PoseMeasurement aslam::calibration::getFirstPoseMeasurement(CalibratorI & calib, Timestamp & startTime, const Sensor & sensor, bool respectDelayLowerBound, const Frame * transformToFramePtr) {
  if(respectDelayLowerBound && sensor.hasDelay()){
    if(startTime - sensor.getDelayUpperBound() < Timestamp(calib.getCurrentEffectiveBatchInterval().start)){
      startTime = Timestamp(calib.getCurrentEffectiveBatchInterval().start) + sensor.getDelayUpperBound(); //TODO C use Timestamp everywhere
    }
  }
  SM_ASSERT_TRUE(std::runtime_error, sensor.isA<PoseSensorI>(), "")

  auto & poseSensor = sensor.as<PoseSensorI>();
  auto & storage = calib.getCurrentStorage();
  CHECK(poseSensor.hasMeasurements(storage));

  auto ps = getMeasurementsSlice(poseSensor.getAllMeasurements(storage), startTime, (startTime + Timestamp(1.0)));
  CHECK(!ps.empty()) << "Did not get any pose measurement!"; //TODO B implement nicer solution

  startTime = ps[0].first;

  PoseMeasurement & pose = ps[0].second;
  if(transformToFramePtr){
    sm::kinematics::Transformation m_f(pose.q, pose.t);

    std::string poseToString(const sm::kinematics::Transformation & trafo);

    m_f = m_f * sm::kinematics::Transformation(sensor.getTransformationExpressionTo(calib.getModelAt(startTime, 0, {false, false}), *transformToFramePtr).toTransformationMatrix()).inverse();

    pose.q = m_f.q();
    pose.t = m_f.t();
  }
  return pose;
}

