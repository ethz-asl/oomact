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

/** \file PoseMeasurement.h
    \brief This file defines the PoseMeasurement structure which represents a
           measurement of a pose sensor.
  */

#ifndef ASLAM_CALIBRATION_POSE_MEASUREMENT_H
#define ASLAM_CALIBRATION_POSE_MEASUREMENT_H

#include <Eigen/Core>

namespace aslam {
  namespace calibration {

    /** The structure PoseMeasurement represents a measurement returned by a
        pose sensor such as Applanix.

        It can be used though also for initial guess and tests
        \brief Pose measurement
      */
    struct PoseMeasurement {
      /** \name Public members
        @{
        */
      /// Relative position of the reference frame w.r. to the mapping frame
      Eigen::Vector3d t_m_mf;

      // Quaternions in JPL convention
      Eigen::Vector4d q_m_f; //TODO B move to quaternion class
      /** @}
        */

    };

  }
}

#endif // ASLAM_CALIBRATION_POSE_MEASUREMENT_H
