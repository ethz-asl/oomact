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

#include "aslam/calibration/geo/geodetic.h"

#include <cmath>

#include <sm/kinematics/quaternion_algebra.hpp>

using namespace sm::kinematics;

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    sm::kinematics::Transformation ecef2enu(double x, double y, double z, double
        latitude, double longitude) {
      return enu2ecef(x, y, z, latitude, longitude).inverse();
    }

    sm::kinematics::Transformation enu2ecef(double x, double y, double z, double
        latitude, double longitude) {

      const double slat = sin(latitude);
      const double clat = cos(latitude);
      const double slong = sin(longitude);
      const double clong = cos(longitude);

      Eigen::Matrix3d enu_R_ecef;
      enu_R_ecef << -slong, clong, 0,
        -slat * clong, -slat * slong, clat,
        clat * clong, clat * slong, slat;

      return Transformation(
        r2quat(enu_R_ecef.transpose()), Eigen::Vector3d(x, y, z));
    }

    sm::kinematics::Transformation ecef2ned(double x, double y, double z, double
        latitude, double longitude) {
      return ned2ecef(x, y, z, latitude, longitude).inverse();
    }

    sm::kinematics::Transformation ned2ecef(double x, double y, double z, double
        latitude, double longitude) {

      const double slat = sin(latitude);
      const double clat = cos(latitude);
      const double slong = sin(longitude);
      const double clong = cos(longitude);

      Eigen::Matrix3d ned_R_ecef;
      ned_R_ecef << -slat * clong, -slat * slong, clat,
        -slong, clong, 0,
        -clat * clong, -clat * slong, -slat;

      return Transformation(
        r2quat(ned_R_ecef.transpose()), Eigen::Vector3d(x, y, z));
    }

    void wgs84ToEcef(double latitude, double longitude, double altitude,
        double& x, double& y, double& z) {
      const double slat = sin(latitude);
      const double clat = cos(latitude);
      const double slong = sin(longitude);
      const double clong = cos(longitude);
      const double a = 6378137;
      const double e2 = 0.006694380004260827;
      const double R = a / std::sqrt(1 - e2 * slat * slat);
      x = (R + altitude) * clat * clong;
      y = (R + altitude) * clat * slong;
      z = (R * (1 - e2) + altitude) * slat;
    }

  }
}
