#include <aslam/calibration/model/PoseTrajectory.h>

#include <exception>

#include <gtest/gtest.h>
#include <sm/eigen/gtest.hpp>
#include <sm/value_store/ValueStore.hpp>
#include <sm/source_file_pos.hpp>

#include <aslam/calibration/calibrator/AbstractCalibrator.h>
#include <aslam/calibration/calibrator/CalibratorI.h>
#include <aslam/calibration/calibrator/SimpleModuleStorage.h>
#include <aslam/calibration/model/FrameGraphModel.h>
#include <aslam/calibration/model/sensors/PoseSensor.h>
#include <aslam/calibration/test/MockCalibrator.h>
#include <aslam/calibration/test/MockMotionCaptureSource.h>
#include <aslam/calibration/test/Tools.h>

using sm::value_store::ValueStoreRef;

using namespace aslam::calibration;
using namespace aslam::calibration::test;

TEST(PoseTrajectory, frameLinkProperties)
{
  FrameGraphModel m(ValueStoreRef::fromString(
      "frames=body:world,"
      "a{referenceFrame=body,targetFrame=world,rotation/used=false,translation/used=false,delay/used=false}"
      "traj{frame=body,referenceFrame=world,McSensor=a,initWithPoseMeasurements=true,splines{knotsPerSecond=20,rotSplineOrder=4,rotFittingLambda=0.0001,transSplineOrder=4,transFittingLambda=0.001}}"
      "verbose=true"
    ));
  PoseSensor psA(m, "a");
  PoseTrajectory traj(m, "traj");
  m.addModulesAndInit(psA, traj);

  Timestamp endTime = 2 * M_PI, midTime = M_PI;

  MockCalibrator c(m, Interval{0.0, endTime});
  for (auto& p : MmcsCircle.getPoses(endTime)) {
    psA.addMeasurement(p.time, p.q, p.p, c.getCurrentStorage());
  }

  c.initStates();

  const Frame & bodyFrame = m.getFrame("body");
  const Frame & worldFrame = m.getFrame("world");

  auto mAt = m.getAtTime(midTime, 2, { });
  auto R_b_w = sm::kinematics::axisAngle2R(0.0, 0.0, -M_PI_2);
  auto R_w_b = R_b_w.inverse();
  auto t_w_b = -Eigen::Vector3d::UnitX();

  auto T_w_b = mAt.getTransformationToFrom(worldFrame, bodyFrame);
  auto relKin = traj.calcRelativeKinematics(midTime, {}, 2);

  sm::eigen::assertNear(R_w_b, T_w_b.toRotationExpression().toRotationMatrix(), 1e-9,
                        SM_SOURCE_FILE_POS);
  sm::eigen::assertNear(R_w_b, relKin.R.toRotationMatrix(), 1e-9, SM_SOURCE_FILE_POS);
  sm::eigen::assertNear(R_w_b, sm::kinematics::quat2r(MmcsCircle.getPoseAt(midTime).q), 1e-9,
                        SM_SOURCE_FILE_POS);
  sm::eigen::assertNear(t_w_b, MmcsCircle.getPoseAt(midTime).p, 1e-9, SM_SOURCE_FILE_POS);
  sm::eigen::assertNear(t_w_b, relKin.p.evaluate(), 1e-4, SM_SOURCE_FILE_POS);
  sm::eigen::assertNear(t_w_b, T_w_b.toEuclideanExpression().evaluate(), 1e-4,
                        SM_SOURCE_FILE_POS);

  sm::eigen::assertNear(Eigen::Vector3d::UnitZ(), relKin.omega.evaluate(), 1e-4,
                        SM_SOURCE_FILE_POS);
  sm::eigen::assertNear(Eigen::Vector3d::UnitZ(),
                        mAt.getAngularVelocity(bodyFrame, worldFrame).evaluate(),
                        1e-4, SM_SOURCE_FILE_POS);
  sm::eigen::assertNear(-t_w_b,
                        relKin.a.evaluate(),
                        1e-3, SM_SOURCE_FILE_POS);
  sm::eigen::assertNear(-t_w_b,
                        mAt.getAcceleration(bodyFrame, worldFrame).evaluate(),
                        1e-3, SM_SOURCE_FILE_POS);
}
