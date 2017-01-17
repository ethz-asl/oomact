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

/** \file geodetic.h
    \brief This file contains utilities for geodetic datum.
  */

#ifndef ASLAM_CALIBRATION_CAR_GEODETIC_H
#define ASLAM_CALIBRATION_CAR_GEODETIC_H

#include <sm/kinematics/Transformation.hpp>

namespace aslam {
  namespace calibration {

    /** \name Methods
      @{
      */
    /// Returns the transformation from ECEF to ENU
    sm::kinematics::Transformation ecef2enu(double x, double y, double z, double
      latitude, double longitude);
    /// Returns the transformation from ENU to ECEF
    sm::kinematics::Transformation enu2ecef(double x, double y, double z, double
      latitude, double longitude);
    /// Returns the transformation from ECEF to NED
    sm::kinematics::Transformation ecef2ned(double x, double y, double z, double
      latitude, double longitude);
    /// Returns the transformation from NED to ECEF
    sm::kinematics::Transformation ned2ecef(double x, double y, double z, double
      latitude, double longitude);
    /// Returns the ECEF coordinates from WGS84
    void wgs84ToEcef(double latitude, double longitude, double altitude,
      double& x, double& y, double& z);
    /** @}
      */

  }
}

#endif // ASLAM_CALIBRATION_CAR_GEODETIC_H
