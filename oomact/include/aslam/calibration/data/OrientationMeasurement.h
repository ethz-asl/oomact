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

/** \file OrientationMeasurement.h
    \brief This file defines the OrientationMeasurement structure which represents an
           Applanix Orientation measurement.
  */

#ifndef ASLAM_CALIBRATION_ORIENTATION_MEASUREMENT_H
#define ASLAM_CALIBRATION_ORIENTATION_MEASUREMENT_H

#include <Eigen/Core>

namespace aslam {
  namespace calibration {

    /** The structure OrientationMeasurement represents an Orientation
        measurement from an IMU (quaternion), or from another device
        \brief Orientation measurement.
      */
    struct OrientationMeasurement {
      /** \name Public members
        @{
        */
      /// Orientation of the IMU reference frame w.r. to the mapping frame, represented in the mapping frame
      Eigen::Vector4d q_m_f;
      /// Covariance matrix for r_q_mr ??
      Eigen::Matrix3d sigma2_q_m_f;
      /** @}
        */

    };

  }
}

#endif // ASLAM_CALIBRATION_ORIENTATION_MEASUREMENT_H
