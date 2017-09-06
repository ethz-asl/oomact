#include "aslam/calibration/error-terms/ErrorTermPointToPlain.h"

#include <Eigen/Dense>

#include <sm/kinematics/rotations.hpp>
#include <sm/kinematics/EulerAnglesYawPitchRoll.hpp>

using namespace aslam::backend;

namespace aslam {
namespace calibration {

ErrorTermPointToPlain::ErrorTermPointToPlain(const Input& n_a, const Input& p_a, const aslam::backend::EuclideanExpression& p_A_b,
                                             const double weight,
                                             const CovarianceInput& /* sigma2_n_a */, // TODO use sigma2_n_a
                                             const CovarianceInput& sigma2_p_a, const CovarianceInput& sigma2_p_b,
                                             ErrorTermGroupReference etgr) :
      ErrorTermGroupMember(etgr),
      _n_a(n_a),
      _p_a(p_a)
{
  _p_m_ab = p_A_b - _p_a;

  Eigen::Matrix<double, 1, 1> sigma2;
  sigma2 = n_a.transpose() * (sigma2_p_a + sigma2_p_b) * n_a;
  setInvR(sigma2.inverse() * weight);

  DesignVariable::set_t dv;
  _p_m_ab.getDesignVariables(dv);
  setDesignVariablesIterator(dv.begin(), dv.end());
}

ErrorTermPointToPlain::~ErrorTermPointToPlain() {
}


double ErrorTermPointToPlain::evaluateErrorImplementation() {
  setError(_n_a.transpose() * _p_m_ab.toValue());
  return evaluateChiSquaredError();
}

void ErrorTermPointToPlain::evaluateJacobiansImplementation(aslam::backend::JacobianContainer& jacobians) {
  _p_m_ab.evaluateJacobians(jacobians, _n_a.transpose());
}

}
}
