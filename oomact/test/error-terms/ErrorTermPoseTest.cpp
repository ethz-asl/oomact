#include <gtest/gtest.h>

#include <sm/kinematics/EulerAnglesYawPitchRoll.hpp>
#include <sm/eigen/gtest.hpp>


#include <aslam/backend/test/ErrorTermTester.hpp>
#include <aslam/backend/EuclideanPoint.hpp>
#include <aslam/backend/RotationQuaternion.hpp>
#include <aslam/backend/TransformationExpression.hpp>

#include "aslam/calibration/error-terms/ErrorTermPose.h"

using namespace aslam::backend;
using namespace aslam::calibration;
using namespace sm::kinematics;

TEST(AslamCalibrationTestSuite, testErrorTermPose) {
  EuclideanPoint t(Eigen::Vector3d::Zero());
  RotationQuaternion q(quatIdentity());

  ErrorTermPose e(TransformationExpression(q.toExpression(), t.toExpression()), Eigen::Vector3d::Zero(), quatIdentity(), Eigen::MatrixXd::Identity(6, 6), "TestET");

  // test the error term
  try {
    EXPECT_DOUBLE_EQ(0.0, e.getSquaredError());
    testErrorTerm(e, 1e-4);

    q.update(Eigen::Vector3d(M_PI /2, 0.0, 0.0).data(), 3);
    e.updateRawSquaredError();
    EXPECT_DOUBLE_EQ(std::pow(M_PI/ 2, 2), e.getSquaredError());
    testErrorTerm(e, 1e-4);

    t.set(Eigen::Vector3d::UnitX());
    e.updateRawSquaredError();
    EXPECT_DOUBLE_EQ(std::pow(M_PI/ 2, 2) + 1, e.getSquaredError());
    testErrorTerm(e, 1e-4);

    for(int i = 0 ; i < 100; i ++){
      t.set(Eigen::Vector3d::Random());
      q.set(quatRandom());
      e.updateRawSquaredError();
      testErrorTerm(e, 1e-5);
    }
  }
  catch (const std::exception& e) {
    FAIL() << e.what();
  }
}
