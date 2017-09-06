#include <memory>

#include <gtest/gtest.h>

#include <aslam/calibration/calibrator/CalibratorI.hpp>
#include <aslam/calibration/data/AccelerometerMeasurement.h>
#include <aslam/calibration/data/GyroscopeMeasurement.h>
#include <aslam/calibration/model/FrameGraphModel.h>
#include <aslam/calibration/model/PoseTrajectory.h>
#include <aslam/calibration/model/sensors/Imu.h>
#include <aslam/calibration/model/sensors/PoseSensor.hpp>
#include <aslam/calibration/test/MockMotionCaptureSource.h>
#include <aslam/calibration/tools/SmartPointerTools.h>
#include <sm/kinematics/Transformation.hpp>


using namespace aslam::calibration;
using namespace aslam::calibration::test;

TEST(CalibrationTestSuite, testImuCalibrationRotateStraight) {
  auto vs = ValueStoreRef::fromFile("acceptance/imu-pose.info");

  FrameGraphModel m(vs.getChild("model"));
  PoseSensor psA(m, "pose");
  Imu imu(m, "imu");
  PoseTrajectory traj(m, "traj");
  m.addModulesAndInit(psA, imu, traj);

  imu.getTranslationVariable().set({0., 1., 0.});
  const double rotUpdate[] = {0., 0.1, 0.};
  imu.getRotationVariable().update(rotUpdate, 3);

  auto spModel = aslam::to_local_shared_ptr(m);
  auto c = createBatchCalibrator(vs.getChild("calibrator"), spModel);

  for (auto& p : MmcsRotatingStraightLine.getPoses(1.0)) {
    psA.addMeasurement(p.time, p.q, p.p, c->getCurrentStorage());
    c->addMeasurementTimestamp(p.time, psA);
    sm::kinematics::Transformation T(p.q, p.p);
    imu.addGyroscopeMeasurement(*c, GyroscopeMeasurement(Eigen::Vector3d::UnitX()), p.time);
    imu.addAccelerometerMeasurement(*c, AccelerometerMeasurement(T.C().transpose() * (Eigen::Vector3d::UnitZ() * 9.81).eval()), p.time);
  }

  EXPECT_NEAR(1, imu.getTranslationToParent()[1], 0.0001);
  EXPECT_NEAR(0.05, imu.getRotationQuaternionToParent()[1], 0.01);
  c->calibrate();
  EXPECT_NEAR(0, imu.getTranslationToParent()[1], 0.0001);
  EXPECT_NEAR(0, imu.getRotationQuaternionToParent()[1], 0.01);
}

TEST(CalibrationTestSuite, testImuCalibrationCircle) {
  auto vs = ValueStoreRef::fromFile("acceptance/imu-pose.info");

  FrameGraphModel m(vs.getChild("model"));
  PoseSensor psA(m, "pose");
  Imu imu(m, "imu");
  PoseTrajectory traj(m, "traj");
  m.addModulesAndInit(psA, imu, traj);

  imu.getTranslationVariable().set({0, 1., 0.0});
  const double rotUpdate[] = {0., 0.1, 0.};
  imu.getRotationVariable().update(rotUpdate, 3);

  auto spModel = aslam::to_local_shared_ptr(m);
  auto c = createBatchCalibrator(vs.getChild("calibrator"), spModel);

  for (auto& p : MmcsCircle.getPoses(1.0)) {
    psA.addMeasurement(p.time, p.q, p.p, c->getCurrentStorage());
    c->addMeasurementTimestamp(p.time, psA);
    sm::kinematics::Transformation T(p.q, p.p);
    imu.addGyroscopeMeasurement(*c, GyroscopeMeasurement(Eigen::Vector3d::UnitZ()), p.time);
    imu.addAccelerometerMeasurement(*c, AccelerometerMeasurement(T.C().transpose() * (Eigen::Vector3d::UnitZ() * 9.81 - p.p).eval()), p.time);
  }

  EXPECT_NEAR(1, imu.getTranslationToParent()[1], 0.0001);
  EXPECT_NEAR(0.05, imu.getRotationQuaternionToParent()[1], 0.01);
  c->calibrate();
  EXPECT_NEAR(0, imu.getTranslationToParent()[1], 0.0001);
  EXPECT_NEAR(0, imu.getRotationQuaternionToParent()[1], 0.01);
}

