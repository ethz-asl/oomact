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

/** \file ErrorTermWheelTest.cpp
    \brief This file tests the ErrorTermWheel class.
  */

#include <cmath>

#include <boost/make_shared.hpp>

#include <gtest/gtest.h>

#include <aslam/backend/test/ErrorTermTestHarness.hpp>
#include <aslam/backend/EuclideanPoint.hpp>

#include "aslam/calibration/error-terms/ErrorTermGyroscope.h"

using namespace aslam::backend;
using namespace aslam::calibration;

TEST(AslamCalibrationTestSuite, testErrorTermGyroscope) {
  /*ErrorTermGyroscope(const aslam::backend::EuclideanExpression& _i_w_mr,
                           const aslam::backend::EuclideanExpression& _bias,
                           const Input& wm, const Covariance& sigma2);*/
  EuclideanPoint i_w_mr(Eigen::Vector3d::Random());
  EuclideanPoint bias(Eigen::Vector3d::Random());

  ErrorTermGyroscope gerr(i_w_mr.toExpression(),
                          bias.toExpression(),
                          Eigen::Vector3d::Random(), Eigen::Matrix3d::Identity());

  // test the error terms
  try {
    ErrorTermTestHarness<1> harness(&gerr);
    harness.testAll(1e-5);
  }
  catch (const std::exception& e) {
    FAIL() << e.what();
  }

}
