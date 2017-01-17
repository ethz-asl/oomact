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
#include <aslam/backend/EuclideanExpression.hpp>
#include <aslam/backend/Scalar.hpp>
#include <aslam/backend/ScalarExpression.hpp>

#include "aslam/calibration/error-terms/ErrorTermWheel.h"

using namespace aslam::backend;
using namespace aslam::calibration;

TEST(AslamCalibrationTestSuite, testErrorTermWheel) {

  /*ErrorTermWheel(const aslam::backend::EuclideanExpression& w_v_mw, const
          aslam::backend::ScalarExpression& r, double measurement, const
          Covariance& sigma2Wheel);*/
  // linear velocity
  const Eigen::Matrix<double, 3, 1> v(1.5, 1.7, 2.8);
  auto v_dv = boost::make_shared<EuclideanPoint>(v);
  EuclideanExpression v_exp(v_dv);

  // radius
  auto r_dv = boost::make_shared<Scalar>(1.6);
  ScalarExpression r_exp(r_dv);

  // error term wheel
  ErrorTermWheel ew(v_exp, r_exp, 1.5, 1);

  // test the error terms
  try {
    ErrorTermTestHarness<2> harness(&ew);
    harness.testAll(1e-5);
  }
  catch (const std::exception& e) {
    FAIL() << e.what();
  }

}
