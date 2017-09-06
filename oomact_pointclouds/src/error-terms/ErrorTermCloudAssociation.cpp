/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 *                                                                            *
 * This program is free software; you can redistribute it and/or modify       *
 * it under the terms of the Lesser GNU General Public License as published by*
 * the Free Software Foundation; either version 3 of the License, or          *
 * (at your option) any later version.                                        *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the              *
 * Lesser GNU General Public License for more details.                        *
 *                                                                            *
 * You should have received a copy of the Lesser GNU General Public License   *
 * along with this program. If not, see <http://www.gnu.org/licenses/>.       *
 ******************************************************************************/

#include "aslam/calibration/error-terms/ErrorTermCloudAssociation.h"

#include <Eigen/Dense>

#include <sm/kinematics/rotations.hpp>
#include <sm/kinematics/EulerAnglesYawPitchRoll.hpp>

using namespace aslam::backend;

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

  ErrorTermCloudAssociation::ErrorTermCloudAssociation(const aslam::backend::TransformationExpression& T_m_v_a,
                                                       const aslam::backend::TransformationExpression& T_m_v_b,
                                                       const aslam::backend::RotationExpression& R_m_v_a,
                                                       const Input& n_a,
                                                       const Input& p_a,
                                                       const Input& p_b,
                                                       const double weight,
                                                       const CovarianceInput& /* sigma2_n_a */, // TODO A use sigma2_n_a
                                                       const CovarianceInput& sigma2_p_a,
                                                       const CovarianceInput& sigma2_p_b) :
                                   _pointToPlane_prediction(0.0),
                                   _n_m_a(NULL),
                                   _p_m_ab(NULL),
                                   _n_a(n_a),
                                   _p_a(p_a),
                                   _p_b(p_b),
                                   w_(weight),
                                   sigma2_()
  {
    //setInvR(sigma2_.inverse());
    //Covariance invR;
    //invR(0,0) = (1.0/sigma2);
    //setInvR(invR);

    // Temporary solution to solve a linker error, it cannot find reference to
    // aslam::backend::TransformationExpression::operator*(aslam::backend::EuclideanExpression const&) const
    //EuclideanExpression p_a_eu(p_a);
    //EuclideanExpression p_b_eu(p_b);

    //auto p_m_a = T_m_v_a * EuclideanExpression(p_a);
    //auto p_m_b = EuclideanExpression(p_b);

    //auto p_m_b = T_m_v_b_copy * p_b_eu;
    auto p_m_a = T_m_v_a * EuclideanExpression(p_a);
    auto p_m_b = T_m_v_b * EuclideanExpression(p_b);

    auto n_m_a = R_m_v_a * EuclideanExpression(n_a);

    _n_m_a = n_m_a;
    _p_m_ab = p_m_b - p_m_a;

    // Calculating the covariance of the error (derived from differentiation)
    // z = [n_a,p_a,p_b] measurement vector
    // TODO: Still to estimate n_a
    // Cov[z] = diag[Cov[n_a],Cov[p_a],Cov[p_b]]

    /*Eigen::Matrix<double, 1, 9> Gradient;
    Eigen::Matrix<double, 9, 9> totCov = Eigen::Matrix<double, 9, 9>::Zero();

    totCov.topLeftCorner<3, 3>() = sigma2_n_a;
    totCov.block<3,3>(3,3) = sigma2_p_a;
    totCov.bottomRightCorner<3, 3>() = sigma2_p_b;

    Gradient.head<3>() = R_m_v_a_copy.toRotationMatrix().inverse() * _p_m_ab.toValue();

    Gradient.block<1,3>(0,3) = - n_a.transpose();

    Gradient.tail<3>() = T_m_v_b.toRotationExpression().toRotationMatrix().inverse() * n_m_a.toValue();
    sigma2_(0,0) = w_ * w_ * Gradient * totCov * Gradient.transpose();*/
    Eigen::Matrix<double, 1, 6> Gradient;
    Eigen::Matrix<double, 6, 6> totCov = Eigen::Matrix<double, 6, 6>::Zero();

    totCov.topLeftCorner<3, 3>() = sigma2_p_a;
    totCov.bottomRightCorner<3, 3>() = sigma2_p_b;

    Gradient.head<3>() = - n_a.transpose();

    Gradient.tail<3>() = T_m_v_b.toRotationExpression().toRotationMatrix().inverse() * n_m_a.toValue();
    // TODO: B Check if it is w_ or 1/w_.. ??
    sigma2_(0,0) = Gradient * totCov * Gradient.transpose(); //*w_ * w_;
    // sigma2_(0,0) = w_ * w_ * Gradient.dot(totCov * Gradient.transpose());
    setInvR(sigma2_.inverse());


    DesignVariable::set_t dv;

    //_pointToPlane_prediction.getDesignVariables(dv);
    _n_m_a.getDesignVariables(dv);
    _p_m_ab.getDesignVariables(dv);
    setDesignVariablesIterator(dv.begin(), dv.end());
  }

    ErrorTermCloudAssociation::~ErrorTermCloudAssociation() {
    }

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

    // TODO: Setup these accessors
    const ErrorTermCloudAssociation::Input& ErrorTermCloudAssociation::getInput() const {
      return _n_a;
    }

    ErrorTermCloudAssociation::Input& ErrorTermCloudAssociation::getInput() {
      return _n_a;
    }

    void ErrorTermCloudAssociation::setInput(const Input& n_a) {
      _n_a = n_a;
    }

    const ErrorTermCloudAssociation::Covariance& ErrorTermCloudAssociation::getCovariance() const {
      return sigma2_;
    }

    ErrorTermCloudAssociation::Covariance& ErrorTermCloudAssociation::getCovariance() {
      return sigma2_;
    }

    void ErrorTermCloudAssociation::setCovariance(const Covariance& sigma2) {
      sigma2_ = sigma2;
    }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    double ErrorTermCloudAssociation::evaluateErrorImplementation() {
      error_t error;
      //error(0) = _pointToPlane_prediction.toValue();
      error(0) = w_ * _n_m_a.toValue().dot(_p_m_ab.toValue());
      setError(error);
      return evaluateChiSquaredError();
    }

    void ErrorTermCloudAssociation::evaluateJacobiansImplementation(
        aslam::backend::JacobianContainer& jacobians) {

      auto n_m_a = _n_m_a.toValue();
      auto p_m_ab = _p_m_ab.toValue();

      Eigen::Matrix<double, 1, 3> Jn = Eigen::Matrix<double, 1, 3>::Zero();
      Eigen::Matrix<double, 1, 3> Jp = Eigen::Matrix<double, 1, 3>::Zero();

      // No weight on the jacobian errors, no sense
      Jn = p_m_ab.transpose();//* w_;
      Jp = n_m_a.transpose();//* w_;

      _n_m_a.evaluateJacobians(jacobians, Jn);
      _p_m_ab.evaluateJacobians(jacobians,Jp);
      //_pointToPlane_prediction.evaluateJacobians(jacobians);
    }

  }
}
