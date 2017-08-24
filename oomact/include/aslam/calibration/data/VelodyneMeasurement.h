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

/** \file VelodyneMeasurement.h
    \brief This file defines the VelodyneMeasurement structure which represents a
           measurement of a pose sensor.
  */

#ifndef ASLAM_CALIBRATION_VELODYNE_MEASUREMENT_H
#define ASLAM_CALIBRATION_VELODYNE_MEASUREMENT_H

#include <Eigen/Core>
#include <sm/timing/NsecTimeUtilities.hpp>

namespace aslam {
  namespace calibration {

    /** The structure VelodyneMeasurement represents a measurement returned by the Map building system
        \brief Velodyne measurement
      */
    struct VelodyneMeasurement {
      /** \name Public members
        @{
        */
      Timestamp timestamp_k;
      Timestamp timestamp_km1;
      /// Relative position of the reference frame of velodyne (Fv) w.r. to the mapping frame
      Eigen::Vector3d r_km1_k;
      /// Covariance matrix for r_m_mv
      Eigen::Matrix3d sigma2_r_km1_k;

      /// Relative orientation of the reference frame w.r. to the mapping frame
      Eigen::Vector3d R_km1_k; // [yaw, pitch, roll]
      /// Covariance matrix for R_m_v
      Eigen::Matrix3d sigma2_R_km1_k;
      /** @}
        */

    };

  }
}

#endif // ASLAM_CALIBRATION_VELODYNE_MEASUREMENT_H
