#ifndef ASLAM_CALIBRATION_CLOUDS_CONTAINER_H
#define ASLAM_CALIBRATION_CLOUDS_CONTAINER_H

#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include <aslam/calibration/SensorId.hpp>
#include <Eigen/Core>

#include <aslam/calibration/clouds/CloudBatch.h>

namespace aslam {
namespace calibration {
class CalibrationConfI;
class CalibratorI;
class Model;
class Parallelizer;
class PointCloudSensor;
class PointCloudsPlugin;
class Sensor;

struct CloudBatches : private std::vector<CloudBatch> {
  CloudBatches(const Sensor& sensor)
      : sensor(sensor) {
  }
  const Sensor& sensor;

  void finishCurrentCloud(const PointCloudsPlugin& pcp);
  CloudBatch& createNewCloud(const PointCloudsPlugin& pcp, const PointCloudSensor& sensor);
  bool isAcceptingData() const;
  CloudBatch& getCurrentCloud();  // requires an open cloud
  const CloudBatch& getCurrentCloud() const;  // requires an open cloud
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

/**
 * The CloudsContainer maintains the
 */
class CloudsContainer {
 public:
  CloudsContainer(PointCloudsPlugin& pcp);
  virtual ~CloudsContainer();

  size_t associationsIntersection(const Sensor& sensorA, const Sensor& sensorB);

  size_t computeAssociations(const CalibrationConfI& cc);

  size_t associationsRandomSubsample(double prob, const Sensor& sensorA, const Sensor& sensorB);

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
  void saveFilteredClouds(const SensorId& sensorId, const std::string& folderName, size_t version, Parallelizer& parallelizer) const;
  void saveTransformations(const std::string& folderName) const;
  void clearClouds();

  void clearAssociations();
  size_t countAssociations(const Sensor& from, const Sensor& to) const;

  CloudBatches& getCloudsFor(SensorId id) {
    return cloudBatchesMap_.at(id);
  }
  const CloudBatches& getCloudsFor(SensorId id) const {
    return cloudBatchesMap_.at(id);
  }

  template<typename Sensor, typename Data, typename TimestampVector>
  void addCloudScan(const Sensor& pcSensor, const Data& data, const TimestampVector& timestamps) {
    applyCloudDataFunctor(
        pcSensor,
        [&](CloudBatch& toCloud) {
          return toCloud.getMeasurements<typename Sensor::CloudMeasurementsImpl>().addData(timestamps, data, timestamps.size());
        },
        timestamps
      );
  }

  template<typename TimestampVector>
  void applyCloudDataFunctor(const PointCloudSensor& sensor, std::function<int(CloudBatch& toCloud)> addDataFuctor, const TimestampVector& timestamps);

 protected:
  friend PointCloudsPlugin;

  /// Transform a cloud and save it in a specified file
  void transformAndSave(const std::string& fileName, const TP& T, const DP& inCloud) const;

  PointCloudsPlugin& pcp_;
  const Model& model_;

  /// Current CloudBatch's starting timestamp
  Timestamp currentCloudBatchStartTimestamp_;
  /// Last timestamp
  Timestamp lastCloudBatchTimestamp_;

  /// Point cloud source id to cloud batches map
  std::unordered_map<SensorId, CloudBatches> cloudBatchesMap_;

  std::shared_ptr<PM::Transformation> rigidTrans_;
};
}
}

#endif // ASLAM_CALIBRATION_CLOUD_CONTAINER_H
