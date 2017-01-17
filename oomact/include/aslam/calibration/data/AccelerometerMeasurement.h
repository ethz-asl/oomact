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

/** \file DMIMeasurement.h
    \brief This file defines the AccelerometerMeasurement structure which represents an
           Applanix DMI measurement.
  */

#ifndef ASLAM_CALIBRATION_ACCELEROMETER_MEASUREMENT_H
#define ASLAM_CALIBRATION_ACCELEROMETER_MEASUREMENT_H

#include <Eigen/Core>

namespace aslam {
  namespace calibration {

    /** The structure AccelerometerMeasurement represents an Applanix DMI
        measurement.
        \brief Applanix DMI measurement.
      */
    struct AccelerometerMeasurement {
      /** \name Public members
        @{
        */
      /// Accleration of the IMU reference frame w.r. to the mapping frame, represented in the robot frame
      Eigen::Vector3d a_i_mi;
      /// Covariance matrix for r_a_mr
      Eigen::Matrix3d sigma2_a_i_mi;
      /** @}
       */

    };

  }
}

#endif // ASLAM_CALIBRATION_ACCELEROMETER_MEASUREMENT_H
