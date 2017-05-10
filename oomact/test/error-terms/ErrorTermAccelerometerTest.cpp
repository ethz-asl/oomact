#include <cmath>

#include <boost/make_shared.hpp>

#include <gtest/gtest.h>

#include <sm/kinematics/EulerAnglesYawPitchRoll.hpp>

#include <aslam/backend/test/ErrorTermTestHarness.hpp>
#include <aslam/backend/EuclideanPoint.hpp>
#include <aslam/backend/RotationQuaternion.hpp>

#include "aslam/calibration/error-terms/ErrorTermAccelerometer.h"

using namespace aslam::backend;
using namespace aslam::calibration;
using namespace sm::kinematics;

TEST(AslamCalibrationTestSuite, testErrorTermAccelerometer) {
  EuclideanPoint a_m_mr(Eigen::Vector3d::Random());
  RotationQuaternion q_i_m(sm::kinematics::quatRandom());
  EuclideanPoint bias(Eigen::Vector3d::Random());
  EuclideanPoint gravity(Eigen::Vector3d::Random());

  ErrorTermAccelerometer aerr(a_m_mr.toExpression(),
                              q_i_m.toExpression(),
                              gravity.toExpression(),
                              bias.toExpression(),
                              Eigen::Vector3d::Random(), Eigen::Matrix3d::Identity());

  // test the error term
  try {
    ErrorTermTestHarness<3> harness(&aerr);
    harness.testAll(1e-5);
  }
  catch (const std::exception& e) {
    FAIL() << e.what();
  }
}
