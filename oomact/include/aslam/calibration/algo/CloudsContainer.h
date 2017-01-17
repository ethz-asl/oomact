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

/** \file OptimizationProblemSpline.h
    \brief This file defines the OptimizationProblemSpline class, which is
           a specialization of OptimizationProblem for handling splines.
  */

#ifndef ASLAM_CALIBRATION_CLOUDS_CONTAINER_H
#define ASLAM_CALIBRATION_CLOUDS_CONTAINER_H

#include <vector>
#include <string>


#include <unordered_set>

#include <memory>

#include <Eigen/Core>

#include <aslam/calibration/CommonTypes.hpp>

#include <aslam/calibration/CloudBatch.h>

#include <functional>
#include <atomic>

namespace bsplines {

  struct NsecTimePolicy;

}

namespace aslam {
  namespace calibration {
    class Calibrator;
    class Model;
    class Parallelizer;
    class Sensor;
    class PointCloudSensor;
    class EstimationConfiguration;

    struct CloudBatches : private std::vector<CloudBatch> {
      CloudBatches(Sensor & sensor) : sensor(sensor) {}
      Sensor & sensor;

      void finishCurrentCloud(const Calibrator & calib);
      CloudBatch & createNewCloud(const Calibrator & calib, const PointCloudSensor & sensor);
      bool isAcceptingData() const;
      CloudBatch & getCurrentCloud(); // requires an open cloud
      const CloudBatch & getCurrentCloud() const; // requires an open cloud
      void dropCurrentCloud();

      using std::vector<CloudBatch>::size;
      using std::vector<CloudBatch>::empty;
      using std::vector<CloudBatch>::operator [];
      using std::vector<CloudBatch>::begin;
      using std::vector<CloudBatch>::end;
      using std::vector<CloudBatch>::front;
      using std::vector<CloudBatch>::back;
      using std::vector<CloudBatch>::clear;
    };

    /** The class CloudsContainer is a class to handle the correspondences and the cloud stuff.
        \brief .
      */
    class CloudsContainer{
    public:
      /** \name Constructors/destructor
        @{
        */
      /// Constructor
      CloudsContainer(Calibrator & calibrator);
      /// Destructor
      virtual ~CloudsContainer();
      /** @}
        */

      /** \name Methods
        @{
        */
      size_t associationsIntersection(const Sensor & sensorA, const Sensor & sensorB);

      size_t computeAssociations(const EstimationConfiguration & ec);

      size_t associationsRandomSubsample(double prob, const Sensor & sensorA, const Sensor & sensorB);

      /** @}
        */

      /** \name Accessors
        @{
        */

      const Eigen::VectorXi& getIndices(const int& readIter, const int& refIter) const;

      const Eigen::MatrixXi& getMatches(const int& readIter, const int& refIter) const;

      /// Save associations
      void saveAssociations(const std::string& folderName) const;

      /// Save associations
      void saveReducedAssociations(const std::string& folderName) const;

      /// Save reduced set of associations for 2d lasers
      void saveReducedAssociations2d(const size_t& laserId, const std::string& folderName) const;

      /// Save aligned clouds
      void saveAlignedClouds(const std::string& folderName, size_t version) const;

      /// Save filtered clouds
      void saveFilteredClouds(const SensorId& sensorId, const std::string& folderName, size_t version, Parallelizer & parallelizer) const;
      void saveTransformations(const std::string& folderName) const;
      void clearClouds();

      void clearAssociations();
      size_t countAssociations(const Sensor & from, const Sensor & to) const;


      CloudBatches & getCloudsFor(SensorId id) {
        return _cloudBatchesMap.at(id);
      }
      const CloudBatches & getCloudsFor(SensorId id) const {
        return _cloudBatchesMap.at(id);
      }

      /** @}
        */

      /** \name Public members
              @{
              */
      /// Point cloud source id to cloud batches map
      std::unordered_map<SensorId, CloudBatches> _cloudBatchesMap;

      /// Current CloudBatch starting timestamp
      sm::timing::NsecTime _currentCloudBatchStartTimestamp;
      /// Last timestamp
      sm::timing::NsecTime _lastCloudBatchTimestamp;

      std::shared_ptr<PM::Transformation> _rigidTrans;
    protected:
      /** \name Protected members
        @{
        */
      const Calibrator & _calibrator;
      const Model & _model;


      /// Transform a cloud and save it in a specified file
      void transformAndSave(const std::string& fileName, const TP& T,const DP& inCloud) const;

      /** @}
        */
    };
  }
}

#endif // ASLAM_CALIBRATION_CLOUD_CONTAINER_H
