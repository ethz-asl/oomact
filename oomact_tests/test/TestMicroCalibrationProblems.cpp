#include <memory>

#include <gtest/gtest.h>
#include <glog/logging.h>

#include <sm/boost/null_deleter.hpp>
#include <eigen-checks/gtest.h>

#include "aslam/calibration/calibrator/CalibratorI.h"
#include <aslam/calibration/model/Model.h>
#include <aslam/calibration/model/sensors/MotionCaptureSensor.h>
#include <aslam/calibration/model/sensors/PoseSensor.h>
#include <aslam/calibration/model/PoseTrajectory.h>
#include <aslam/calibration/model/fragments/So3R3Trajectory.h>
#include <aslam/calibration/model/FrameGraphModel.h>
#include <aslam/calibration/test/MockMotionCaptureSource.h>


using namespace aslam::calibration;
using namespace aslam::calibration::test;

MockMotionCaptureSource mmcsHelix([](Timestamp now, MotionCaptureSource::PoseStamped & p){
  double deltaTimeSecs = now - MockMotionCaptureSource::StartTime;
  const auto angleX = deltaTimeSecs * 3;
  p.q = sm::kinematics::axisAngle2quat({angleX, 0, 0});
  auto rotRQPlusQuaterRotation = sm::kinematics::axisAngle2R({angleX, 0, 0});
  p.p = Eigen::Vector3d::UnitX() * deltaTimeSecs + rotRQPlusQuaterRotation * Eigen::Vector3d::UnitY();
});


TEST(TestCalibration, testEstimatePoseSensorsInit) {
  auto vs = ValueStoreRef::fromString(
      "Gravity{used=false}"
      "frames=body:world,"
      "a{frame=body,targetFrame=world,rotation/used=false,translation{used=true,x=0,y=5,z=0},delay/used=false}"
      "traj{frame=body,referenceFrame=world,McSensor=a,initWithPoseMeasurements=true,splines{knotsPerSecond=5,rotSplineOrder=4,rotFittingLambda=0.001,transSplineOrder=4,transFittingLambda=0.001}}"
    );

  FrameGraphModel m(vs, nullptr);
  PoseSensor mcSensorA(m, "a", vs);
  PoseTrajectory traj(m, "traj", vs);
  m.addModulesAndInit(mcSensorA, traj);

  EXPECT_EQ(1, m.getCalibrationVariables().size());
  EXPECT_DOUBLE_EQ(5.0, mcSensorA.getTranslationToParent()[1]);

  auto vsCalib = ValueStoreRef::fromString(
      "verbose=true\n"
      "acceptConstantErrorTerms=true\n"
      "estimator{optimizer{maxIterations=-1}}\n"
      "timeBaseSensor=a\n"
    );
  auto c = createBatchCalibrator(vsCalib, std::shared_ptr<Model>(&m, sm::null_deleter()));

  const double startTime = 0, endTime = 1.0;
  for (auto& p : test::MmcsStraightLine.getPoses(startTime, endTime)) {
    mcSensorA.addMeasurement(p.time, p.q, p.p, c->getCurrentStorage());
    c->addMeasurementTimestamp(p.time, mcSensorA);
  }
  EXPECT_EQ(1, c->getCurrentStorage().size());

  c->calibrate();

  EXPECT_NEAR(5.0, mcSensorA.getTranslationToParent()[1], 0.0001);
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(
      -mcSensorA.getTranslationToParent(),
      traj.getCurrentTrajectory().getTranslationSpline().template getEvaluatorAt<0>(0).eval(),
      1e-8
    ));
}

TEST(TestCalibration, testEstimateTwoPoseSensors) {
  auto vs = ValueStoreRef::fromString(
      "Gravity{used=false}"
      "frames=body:world,"
      "a{frame=body,targetFrame=world,rotation/used=false,translation/used=false,delay/used=false}"
      "b{frame=body,targetFrame=world,rotation{used=true,yaw=0.1,pitch=0.,roll=0.},translation{used=true,x=0,y=5,z=0},delay/used=false}"
      "traj{frame=body,referenceFrame=world,McSensor=a,initWithPoseMeasurements=true,splines{knotsPerSecond=5,rotSplineOrder=4,rotFittingLambda=0.001,transSplineOrder=4,transFittingLambda=0.001}}"
    );

  FrameGraphModel m(vs);
  PoseSensor mcSensorA(m, "a", vs);
  PoseSensor mcSensorB(m, "b", vs);
  PoseTrajectory traj(m, "traj", vs);
  m.addModulesAndInit(mcSensorA, mcSensorB, traj);

  EXPECT_EQ(2, m.getCalibrationVariables().size());
  EXPECT_DOUBLE_EQ(5.0, mcSensorB.getTranslationToParent()[1]);
  EXPECT_NEAR(0.1, std::abs(sm::kinematics::quat2AxisAngle(mcSensorB.getRotationQuaternionToParent())[2]), 1e-6); // abs for conventional neutrality

  auto vsCalib = ValueStoreRef::fromString(
      "verbose=true\n"
      "acceptConstantErrorTerms=true\n"
      "timeBaseSensor=a\n"
    );
  auto c = createBatchCalibrator(vsCalib, std::shared_ptr<Model>(&m, sm::null_deleter()));

  const double startTime = 0, endTime = 1.0;

  for (auto& p : MmcsRotatingStraightLine.getPoses(startTime, endTime)) {
    mcSensorA.addMeasurement(p.time, p.q, p.p, c->getCurrentStorage());
    c->addMeasurementTimestamp(p.time, mcSensorA);
    mcSensorB.addMeasurement(p.time, p.q, p.p, c->getCurrentStorage());
  }
  c->calibrate();

  EXPECT_NEAR(0, mcSensorB.getTranslationToParent()[1], 0.0001);
  EXPECT_NEAR(0, sm::kinematics::quat2AxisAngle(mcSensorB.getRotationQuaternionToParent())[2], 0.0001);
}


TEST(TestCalibration, testEstimateRelativePoseSensor) {
  auto vs = ValueStoreRef::fromString(
      "Gravity{used=false}"
      "frames=body:world,"
      "a{frame=body,targetFrame=world,absoluteMeasurements=true,rotation/used=false,translation/used=false,delay/used=false}"
      "b{frame=body,targetFrame=world,absoluteMeasurements=false,rotation{used=true,yaw=0.1,pitch=0.,roll=0.},translation{used=true,x=0,y=5,z=0},delay/used=false}"
      "traj{frame=body,referenceFrame=world,McSensor=a,initWithPoseMeasurements=true,splines{knotsPerSecond=30,rotSplineOrder=4,rotFittingLambda=0.0000001,transSplineOrder=4,transFittingLambda=0.00000001}}"
    );

  FrameGraphModel m(vs);
  PoseSensor mcSensorA(m, "a", vs);
  PoseSensor mcSensorB(m, "b", vs);
  PoseTrajectory traj(m, "traj", vs);
  m.addModulesAndInit(mcSensorA, mcSensorB, traj);

  EXPECT_EQ(2, m.getCalibrationVariables().size());
  EXPECT_DOUBLE_EQ(5.0, mcSensorB.getTranslationToParent()[1]);
  EXPECT_NEAR(0.1, std::abs(sm::kinematics::quat2AxisAngle(mcSensorB.getRotationQuaternionToParent())[2]), 1e-6); // abs for conventional neutrality

  auto vsCalib = ValueStoreRef::fromString(
      "verbose=true\n"
      "acceptConstantErrorTerms=true\n"
      "estimator{optimizer{convergenceDeltaX=1e-6,convergenceDeltaError=1e-10,maxIterations=50}}\n"
      "timeBaseSensor=a\n"
    );
  auto c = createBatchCalibrator(vsCalib, std::shared_ptr<Model>(&m, sm::null_deleter()));

  const double startTime = 0, endTime = 1.0;

  for (auto& p : mmcsHelix.getPoses(startTime, endTime)) {
    mcSensorA.addMeasurement(p.time, p.q, p.p, c->getCurrentStorage());
    c->addMeasurementTimestamp(p.time, mcSensorA);
    mcSensorB.addMeasurement(p.time, p.q, p.p, c->getCurrentStorage());
  }
  c->calibrate();

  EXPECT_NEAR(0, mcSensorB.getTranslationToParent()[1], 0.0001);
  EXPECT_NEAR(0, sm::kinematics::quat2AxisAngle(mcSensorB.getRotationQuaternionToParent())[2], 0.0001);
}


TEST(TestCalibration, testEstimateMotionCaptureSensorInit) {
  auto vs = ValueStoreRef::fromString(
      "Gravity{used=false}"
      "frames=body:world,"
      "o{frame=world,rotation/used=false,translation/used=false,delay/used=false}"
      "a{frame=body,rotation/used=false,translation{used=true,estimate=false,x=0,y=5,z=0},delay/used=false}"
      "traj{frame=body,referenceFrame=world,McSensor=a,initWithPoseMeasurements=true,splines{knotsPerSecond=10,rotSplineOrder=4,rotFittingLambda=0.000001,transSplineOrder=4,transFittingLambda=0.0000001}}"
    );

  FrameGraphModel m(vs);
  MotionCaptureSystem observer(m, "o", vs);
  MotionCaptureSensor mcSensorA(observer, "a", vs);
  PoseTrajectory traj(m, "traj", vs);
  m.addModulesAndInit(observer, mcSensorA, traj);

  ASSERT_EQ(1, m.getCalibrationVariables().size());
  EXPECT_DOUBLE_EQ(5.0, mcSensorA.getTranslationToParent()[1]);

  mcSensorA.setMotionCaptureSource(std::shared_ptr<MotionCaptureSource>(&MmcsStraightLine, sm::null_deleter()));

  auto vsCalib = ValueStoreRef::fromString(
      "verbose=true\n"
      "acceptConstantErrorTerms=true\n"
      "timeBaseSensor=a\n"
    );
  std::unique_ptr<BatchCalibratorI> c = createBatchCalibrator(vsCalib, std::shared_ptr<Model>(&m, sm::null_deleter()));

  c->addMeasurementTimestamp(0.0, mcSensorA); // add timestamps to determine the batch interval
  c->addMeasurementTimestamp(1.0, mcSensorA);
  c->calibrate();

  EXPECT_TRUE(EIGEN_MATRIX_NEAR(
      -mcSensorA.getTranslationToParent(),
      traj.getCurrentTrajectory().getTranslationSpline().template getEvaluatorAt<0>(0).eval(),
      1e-8
    ));
  EXPECT_DOUBLE_EQ(5.0, mcSensorA.getTranslationToParent()[1]);
}

TEST(TestCalibration, testEstimateMotionCaptureSensorPose) {
  auto vs = ValueStoreRef::fromString(
      "Gravity{used=false}"
      "frames=body:world,"
      "o{frame=world,rotation/used=false,translation/used=false,delay/used=false}"
      "a{frame=body,rotation/used=false,translation/used=false,delay/used=false}"
      "b{frame=body,rotation/used=false,translation{used=true,x=0,y=5,z=0},delay/used=false}"
      "traj{frame=body,referenceFrame=world,McSensor=a,initWithPoseMeasurements=true,splines{knotsPerSecond=5,rotSplineOrder=4,rotFittingLambda=0.001,transSplineOrder=4,transFittingLambda=0.001}}"
    );
  //TODO Support some validation!
  //TODO find C++ solution to validation

  FrameGraphModel m(vs);
  MotionCaptureSystem observer(m, "o", vs);
  MotionCaptureSensor mcSensorA(observer, "a", vs);
  MotionCaptureSensor mcSensorB(observer, "b", vs);
  PoseTrajectory traj(m, "traj", vs);
  m.addModulesAndInit(observer, mcSensorA, mcSensorB, traj);

  ASSERT_EQ(1, m.getCalibrationVariables().size());


  mcSensorA.setMotionCaptureSource(std::shared_ptr<MotionCaptureSource>(&MmcsStraightLine, sm::null_deleter()));
  mcSensorB.setMotionCaptureSource(std::shared_ptr<MotionCaptureSource>(&MmcsStraightLine, sm::null_deleter()));

  auto vsCalib = ValueStoreRef::fromString(
      "acceptConstantErrorTerms=true\n"
      "timeBaseSensor=a\n"
    );

  EXPECT_DOUBLE_EQ(5.0, mcSensorB.getTranslationToParent()[1]);

  auto c = createBatchCalibrator(vsCalib, std::shared_ptr<Model>(&m, sm::null_deleter()));

  c->addMeasurementTimestamp(0.0, mcSensorA); // add timestamps to determine the batch interval
  c->addMeasurementTimestamp(1.0, mcSensorA);

  c->calibrate();

  EXPECT_NEAR(0, mcSensorB.getTranslationToParent()[1], 0.0001);
}
