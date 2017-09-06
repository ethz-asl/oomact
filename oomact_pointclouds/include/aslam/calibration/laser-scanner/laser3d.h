/******************************************************************************
 * Copyright (C) 2014 by Ignazio Indovina                                     *
 * ignazio.indovina@gmail.com                                                 *
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

/** \file laser3d.h
    \brief This file contains a utility to build a pose using calibrated or uncalibrated.
  */

#ifndef ASLAM_CALIBRATION_LASER_3D_H
#define ASLAM_CALIBRATION_LASER_3D_H

#include <Eigen/Core>
#include <sm/kinematics/Transformation.hpp>

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    // Polar coordinates represented by r, th, phi
    Eigen::Vector3d cart2pol(const Eigen::Vector3d& p_c);

    Eigen::Vector3d pol2cart(const Eigen::Vector3d& p_p);

    const Eigen::Matrix3d covPol2covCart(const double& r, const double& th, const double& phi,
                                   const double& dr, const double& dth, const double& dphi);

    const Eigen::Matrix3d jacobianPol(const double& r, const double& th, const double& phi);

    const Eigen::Matrix3d rotFramePol(const double th, const double phi);

    // Approximated covariance, It should be better to linearize
    const Eigen::Matrix3d covPointSurface(bool useSurfaceNormal, const Eigen::Vector3d p_v_cart,const Eigen::Vector3d n_v_p,
                                           const double& dr, const double& dth, const double& dphi, const double& diverAngle,
                                           const double& gainX, const double& gainY, const double& gainZ);
  }
}

#endif // ASLAM_CALIBRATION_LASER_3D_H
