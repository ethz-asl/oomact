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

/** \file PositionMeasurement.h
    \brief This file defines the PositionMeasurement structure which represents a
           measurement of a position sensor.
  */

#ifndef ASLAM_CALIBRATION_POSITION_MEASUREMENT_H
#define ASLAM_CALIBRATION_POSITION_MEASUREMENT_H

#include <Eigen/Core>

namespace aslam {
  namespace calibration {

    /** The structure PositionMeasurement represents a measurement returned by a
        position sensor such as Leica Total Station or GPS coordinates converted to a local frame.

        It can be used though also for initial guess and tests
        \brief Position measurement
      */
    struct PositionMeasurement {
      /** \name Public members
        @{
        */
      /// Relative position of the reference frame w.r. to the mapping frame
      Eigen::Vector3d t_m_mf;
      /// Covariance matrix for m_r_mr
      Eigen::Matrix3d sigma2_t_m_mf;

    };

  }
}

#endif // ASLAM_CALIBRATION_POSITION_MEASUREMENT_H
