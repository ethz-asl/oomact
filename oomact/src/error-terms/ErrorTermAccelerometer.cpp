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

#include "aslam/calibration/error-terms/ErrorTermAccelerometer.h"

#include <Eigen/Dense>

#include <sm/kinematics/rotations.hpp>
#include <sm/kinematics/EulerAnglesYawPitchRoll.hpp>

using namespace aslam::backend;

namespace aslam {
  namespace calibration {
    ErrorTermAccelerometer::ErrorTermAccelerometer(const EuclideanExpression& a_m_mi, const RotationExpression& R_i_m, const EuclideanExpression& g_m, const EuclideanExpression& bias, const Input& am, const Covariance& sigma2,
                                                   const ErrorTermGroupReference & etgr) :
        Parent(RotationExpression(R_i_m) * (a_m_mi + g_m) + bias, am, sigma2, etgr)
    {}
  }
}
