#include <cmath>

#include <boost/make_shared.hpp>

#include <gtest/gtest.h>

#include <sm/kinematics/EulerAnglesYawPitchRoll.hpp>

#include <aslam/backend/test/ErrorTermTestHarness.hpp>
#include <aslam/backend/EuclideanPoint.hpp>
#include <aslam/backend/RotationQuaternion.hpp>
#include <aslam/backend/EuclideanExpression.hpp>
#include <aslam/backend/RotationExpression.hpp>
#include <aslam/backend/TransformationExpression.hpp>

#include <aslam/calibration/statistics/NormalDistribution.h>

#include "aslam/calibration/error-terms/ErrorTermCloudAssociation.h"

using namespace aslam::backend;
using namespace aslam::calibration;

TEST(AslamCalibrationTestSuite, testErrorTermCloudAssociation) {

  // estimated normals and points
  // Points are choosen to be normal
  const ErrorTermCloudAssociation::Input n_a_e((ErrorTermCloudAssociation::Input() <<
      1, -2, 1.0).finished());
  const ErrorTermCloudAssociation::Input p_a_e((ErrorTermCloudAssociation::Input() <<
      5, 6, 7).finished());
  const ErrorTermCloudAssociation::Input p_b_e((ErrorTermCloudAssociation::Input() <<
      4, 5, 6).finished());

  const double weight = 1.0;

  // covariance matrix
  ErrorTermCloudAssociation::CovarianceInput Q = ErrorTermCloudAssociation::CovarianceInput::Zero();
  Q(0, 0) = 1e-4;
  Q(1, 1) = 1e-4;
  Q(2, 2) = 1e-4;

  // measured normals and points
  const ErrorTermCloudAssociation::Input n_a_m(
      NormalDistribution<3>(n_a_e, Q).getSample());
  const ErrorTermCloudAssociation::Input p_a_m(
      NormalDistribution<3>(p_a_e, Q).getSample());
  const ErrorTermCloudAssociation::Input p_b_m(
      NormalDistribution<3>(p_b_e, Q).getSample());



  // encapsulate the estimated points into Euclidean expression


  auto n_a_a = boost::make_shared<EuclideanPoint>(n_a_e);
  EuclideanExpression n_a_a_e(n_a_a);
  auto p_a_a = boost::make_shared<EuclideanPoint>(p_a_e);
  EuclideanExpression p_a_a_e(p_a_a);
  auto p_b_b = boost::make_shared<EuclideanPoint>(p_b_e);
  EuclideanExpression p_b_b_e(p_b_b);


  auto r_m_a = boost::make_shared<EuclideanPoint>((Eigen::Vector3d() << 100, 150, 200).finished());

  sm::kinematics::EulerAnglesYawPitchRoll conv;
  auto q_m_a = boost::make_shared<RotationQuaternion>(
      conv.parametersToRotationMatrix((Eigen::Vector3d() << 10/ M_PI, 10 / M_PI, 20 / M_PI).finished()));
  TransformationExpression T_m_a_e(RotationExpression(q_m_a.get()),
                             EuclideanExpression(r_m_a.get()));
  TransformationExpression T_m_b_e(T_m_a_e);

  // build the error term
  ErrorTermCloudAssociation e1(T_m_a_e,
                               T_m_b_e,
                               T_m_b_e.toRotationExpression(),
                               n_a_m,
                               p_a_m,
                               p_b_m,
                               weight,
                               Q, Q, Q);

  // test the error term
  try {
    ErrorTermTestHarness<1> harness(&e1);
    harness.testAll();
  }
  catch (const std::exception& e) {
    FAIL() << e.what();
  }
}
