#include <cmath>

#include <gtest/gtest.h>

#include "aslam/calibration/calibrator/CalibratorI.h"
#include <aslam/calibration/data/PositionMeasurement.h>
#include <aslam/calibration/model/FrameGraphModel.h>
#include <aslam/calibration/model/PoseTrajectory.h>
#include <aslam/calibration/model/sensors/PoseSensor.h>
#include <aslam/calibration/model/sensors/PositionSensor.h>
#include <aslam/calibration/test/MockMotionCaptureSource.h>
#include <aslam/calibration/test/SimpleModel.h>
#include <aslam/calibration/test/Tools.h>

using namespace aslam::calibration;
using namespace aslam::calibration::test;

TEST(CalibrationTestSuite, testEstimateTwoPoseSensors) {
  auto vs = ValueStoreRef::fromString(
      "Gravity{used=false}"
      "frames=body:world,"
      "a{referenceFrame=body,targetFrame=world,rotation/used=false,translation/used=false,delay/used=false}"
      "b{referenceFrame=body,targetFrame=world,rotation{used=true,yaw=0.1,pitch=0.,roll=0.},translation{used=true,x=0,y=5,z=0},delay/used=false}"
      "traj{frame=body,referenceFrame=world,McSensor=a,initWithPoseMeasurements=true,splines{knotsPerSecond=5,rotSplineOrder=4,rotFittingLambda=0.001,transSplineOrder=4,transFittingLambda=0.001}}"
    );

  FrameGraphModel m(vs);
  PoseSensor mcSensorA(m, "a", vs);
  PoseSensor mcSensorB(m, "b", vs);
  PoseTrajectory traj(m, "traj", vs);
  m.addModulesAndInit(mcSensorA, mcSensorB, traj);

  EXPECT_EQ(2u, m.getCalibrationVariables().size());
  EXPECT_DOUBLE_EQ(5.0, mcSensorB.getTranslationToParent()[1]);
  EXPECT_NEAR(0.1, std::abs(sm::kinematics::quat2AxisAngle(mcSensorB.getRotationQuaternionToParent())[2]), 1e-6); // abs for conventional neutrality

  auto vsCalib = ValueStoreRef::fromString(
      "verbose=true\n"
      "acceptConstantErrorTerms=true\n"
      "timeBaseSensor=a\n"
    );
  auto c = createBatchCalibrator(vsCalib, std::shared_ptr<Model>(&m, sm::null_deleter()));

  const double startTime = 0.0, endTime = 1.0;

  for (auto& p : MmcsRotatingStraightLine.getPoses(startTime, endTime)) {
    mcSensorA.addMeasurement(p.time, p.q, p.p, c->getCurrentStorage());
    c->addMeasurementTimestamp(p.time, mcSensorA);
    mcSensorB.addMeasurement(p.time, p.q, p.p, c->getCurrentStorage());
  }
  c->calibrate();

  EXPECT_NEAR(0, mcSensorB.getTranslationToParent()[1], 0.0001);
  EXPECT_NEAR(0, sm::kinematics::quat2AxisAngle(mcSensorB.getRotationQuaternionToParent())[2], 0.0001);
}



TEST(CalibrationTestSuite, testEstimateOnePoseSensorsAndOnePosition) {
  auto vs = ValueStoreRef::fromString(
      "Gravity{used=false}"
      "frames=body:world,"
      "a{referenceFrame=body,targetFrame=world,rotation/used=false,translation/used=false,delay/used=false}"
      "b{referenceFrame=body,targetFrame=world,rotation/used=false,translation{used=true,x=0,y=5,z=0},delay/used=false}"
      "traj{frame=body,referenceFrame=world,McSensor=a,initWithPoseMeasurements=true,splines{knotsPerSecond=5,rotSplineOrder=4,rotFittingLambda=0.001,transSplineOrder=4,transFittingLambda=0.001}}"
    );

  FrameGraphModel m(vs);
  PoseSensor mcSensorA(m, "a", vs);
  PositionSensor mcSensorB(m, "b", vs);
  PoseTrajectory traj(m, "traj", vs);
  m.addModulesAndInit(mcSensorA, mcSensorB, traj);

  EXPECT_EQ(1u, m.getCalibrationVariables().size());
  EXPECT_DOUBLE_EQ(5.0, mcSensorB.getTranslationToParent()[1]);

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
    mcSensorB.addMeasurement(p.time, PositionMeasurement{p.p});
  }

  c->calibrate();

  EXPECT_NEAR(0, mcSensorB.getTranslationToParent()[1], 0.001);
}
