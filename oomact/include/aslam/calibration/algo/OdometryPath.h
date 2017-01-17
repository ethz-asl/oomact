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

/** \file OdometryPath.h
    \brief This file contains a utility to build a pose using calibrated or uncalibrated.
  */

#ifndef ASLAM_CALIBRATION_ODOMETRY_PATH_H
#define ASLAM_CALIBRATION_ODOMETRY_PATH_H

#include <Eigen/Core>
#include <sm/kinematics/Transformation.hpp>

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/
    /// Functions to calculate the Odometry Integration in time dt using kinematicFunctions
    //Eigen::Vector2d wheels2robotSpeeds
    Eigen::Vector3d directKinematics(const Eigen::Vector4d& rotPose,
                                     const Eigen::Vector3d& angVel,
                                     const double dt,
                                     const double Wl,
                                     const double Wr,
                                     const double Rl,
                                     const double Rr,
                                     const double L);

    /// Gives the kinematic model, the velocities in the robot frame (different robots have different kinematic models)
    std::pair<Eigen::Vector3d, Eigen::Vector3d> odomKinemModel(const double Wl,
                                           const double Wr,
                                           const double Rl,
                                           const double Rr,
                                           const double L);
    /// Gives back a transformation in whatever frame (3d integration)
    sm::kinematics::Transformation integrateMotionModel(const Eigen::Vector3d& t_m_f_km1,
                                                        const Eigen::Vector4d& q_m_f_km1,
                                                        const Eigen::Vector3d& v_f_mf_km1,
                                                        const Eigen::Vector3d& w_f_mf_km1,
                                                        double dt);

  }
}

#endif // ASLAM_CALIBRATION_ODOMETRY_PATH_H
