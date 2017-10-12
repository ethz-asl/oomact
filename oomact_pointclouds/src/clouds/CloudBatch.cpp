#include "aslam/calibration/clouds/CloudBatch.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <type_traits>
#include <utility>
#include <vector>

#include <glog/logging.h>
#include <pointmatcher/IO.h>
#include <pointmatcher/PointMatcher.h>
#include <sm/kinematics/EulerAnglesYawPitchRoll.hpp>
#include <sm/kinematics/quaternion_algebra.hpp>
#include <sm/kinematics/Transformation.hpp>

#include <aslam/calibration/clouds/PointCloudsPlugin.h>
#include <aslam/calibration/model/sensors/PointCloudSensor.h>

using aslam::calibration::transform;

namespace aslam {
namespace calibration {

CloudMeasurements::CloudMeasurements(NanPolicy nanPolicy) : nanPolicy_(nanPolicy) {
}

CloudBatch::CloudBatch(const PointCloudSensor& sensor, CloudId id):
    sensor(sensor),
    id(id),
    measurements(sensor.createCloudMeasurements(*this))
{
}

CloudBatch::~CloudBatch() {}

void EuclideanCloudMeasurements::deleteUpToStartTimestamp(Timestamp start){
  if(measurements.empty()) return;
  if(measurements.back().t < start){
    LOG(WARNING) << "Throwing away a complete cloud [" << std::fixed << measurements.front().t << ", " << std::fixed << measurements.back().t << "] because it is before start " << std::fixed << start << "!";
    measurements.clear();
    return;
  }
  for(auto it = measurements.begin(); it!= measurements.end(); it++){
    if (start <= it->t){
      measurements.erase(measurements.begin(), it);
      break;
    }
  }
  if(measurements.size()>1){
    assert(measurements.front().t >= start);
#ifndef NDEBUG
    auto f = std::min_element(measurements.begin(), measurements.end(), [](const PointMeasurement& a, const PointMeasurement& b){
      return a.t < b.t;
    });
    assert(f->t >= start);
#endif
  }
}

void EuclideanCloudMeasurements::deleteFromEndTimestamp(Timestamp end){
  if(measurements.empty()) return;
  if(measurements.front().t > end){
    LOG(WARNING) << "Throwing away a complete cloud [" << std::fixed << measurements.front().t << ", " << std::fixed << measurements.back().t << "] because it is after end " << std::fixed << end << "!";
    measurements.clear();
    return;
  }
  for(auto it = measurements.rbegin(); it!= measurements.rend(); it++){
    if (end >= it->t){
//      it--; TODO why is that wrong?
      measurements.erase(it.base(), measurements.end());
      break;
    }
  }
  if(measurements.size()>1){
    assert(measurements.back().t <= end);
#ifndef NDEBUG
    auto f = std::max_element(measurements.begin(), measurements.end(), [](const PointMeasurement& a, const PointMeasurement& b){
      return a.t < b.t;
    });
    assert(f->t <= end);
#endif
  }
}

void EuclideanCloudMeasurements::fillMeasurementsInto(const MeasurementTransformer& transformer, PointCloudView cloud, PointDescriptor::RowXpr indices) const {
  assert(size_t(PmScalar(measurements.size())) == measurements.size());
  // TODO O Check if point matcher allows descriptors to be whatever type,

  int cloudIndex = 0;
  for(auto& m : measurements) {
    cloud.col(cloudIndex).head<3>() = transform(transformer, m.t, m.p);
    indices(cloudIndex) = (PmScalar)cloudIndex;
    cloudIndex++;
  }
}


void CloudBatch::clearAssociations(){
  indicesMeasCloud.clear();
  matches.clear();
  cloudScan = DP();// Reinitialize the point cloud
  updateCloudInfo();
  created = false;
}

void CloudBatch::createCloud(){ //TODO C improve name
  SM_ASSERT_TRUE(std::runtime_error, isClosed(), "Cloud wasn't closed before preparing it for ICP.")

  DP::Labels featLabels;
  featLabels.push_back(DP::Label("x",1));
  featLabels.push_back(DP::Label("y",1));
  featLabels.push_back(DP::Label("z",1));
  featLabels.push_back(DP::Label("pad",1));

  DP::Labels descriptorLabels;
  descriptorLabels.push_back(DP::Label("index", 1));

  cloudScan = DP(featLabels, descriptorLabels, getMeasurements().getSize());

  cloudScan.features.row(3).setOnes();

  updateCloudInfo();
  created = true;
}

void CloudBatch::updateIndices(){
  CHECK(indexRow >= 0) << "Index is missing!";
  auto indexVector = cloudScan.descriptors.row(indexRow);
  const int numPoints = cloudScan.features.cols();

  indicesMeasCloud.clear();
  indicesMeasCloud.reserve(numPoints);

  for(int index = 0; index < numPoints; index++){
    indicesMeasCloud.push_back(indexVector(index));
  }
}

template <typename TimeFunctor, typename Scan>
int EuclideanCloudMeasurements::addDataInternal(TimeFunctor timestamps, const Scan& scan, size_t size){
  const int dim = scan.rows();
  assert(maximalDuration > 0);

  assert(dim == 4 || dim == 3);

  Timestamp endTime = std::numeric_limits<Timestamp::Integer>::max();

  for(int pointIndex = 0; pointIndex < size; ++pointIndex){
    auto t = timestamps(pointIndex);
    if(t > endTime){
      return pointIndex;
    }
    if(measurements.empty()){
      minTimestamp = maxTimestamp = t;
      endTime = minTimestamp + maximalDuration;
    }
    else{
      minTimestamp = std::min(minTimestamp, t);
      maxTimestamp = std::max(maxTimestamp, t);
    }

    if(dim == 3)
      measurements.emplace_back(PointMeasurement{t, scan.col(pointIndex)});
    else
      measurements.emplace_back(PointMeasurement{t, scan.col(pointIndex).template head<3>()});

    if((nanPolicy_.checkForNans || nanPolicy_.nansAreFine) && measurements.back().p.hasNaN()){
      measurements.pop_back();
      if(!nanPolicy_.nansAreFine){
        LOG(WARNING) <<"Removed NANs from scan!";
      }
    }
  }
  return size;
}

// TODO X Handle the intensity measurements
int EuclideanCloudMeasurements::addData(const std::vector<Timestamp>& timestamps,const PointCloud& scan, const size_t useFirstN){
  assert(size_t(scan.cols()) >= useFirstN);
  assert(timestamps.size() >= useFirstN);
  return addDataInternal([&](int index){return timestamps[index];}, scan, useFirstN);
}

int EuclideanCloudMeasurements::addData(const Eigen::Matrix<Timestamp, Eigen::Dynamic, 1>& timestamps, const PointCloud& scan, const size_t useFirstN){
  assert(size_t(scan.cols()) >= useFirstN);
  assert(size_t(timestamps.size()) >= useFirstN);
  return addDataInternal([&](int index){return timestamps(index);}, scan, useFirstN);
}

void CloudBatch::saveFilteredCloud(const std::string& fileName) const{
  VLOG(1) << "Writing cloudScan to cloud file " << fileName;
  if(boost::filesystem::exists(fileName)){
    LOG(WARNING) << "Overwriting!" << fileName;
  }
  cloudScan.save(fileName, true);
}

void CloudBatch::saveAssociations(const std::string& /*fileName*/) const{
  // TODO X Basic idea, we run into the associations and we save the corresponding
  // It is better to do that at higher level, structuring the calibration matcher better

  // We can set up here a DP creator with the two vectors of correspondences
}

const PointMeasurement& EuclideanCloudMeasurements::getMeasurement(const size_t index) const {
  return measurements.at(index);
}

Timestamp CloudBatch::getMeasurementTimestamp(const size_t indexFiltered) const{
    return getMeasurements().getTimestamp(getMeasurementIndexFromFilteredIndex(indexFiltered));
}

size_t CloudBatch::getMeasurementIndexFromFilteredIndex(const size_t indexFiltered) const{
  return indicesMeasCloud.at(indexFiltered);
}

const PmVector3&CloudBatch::getMeasurementFeature(const size_t indexFiltered) const{
  return getMeasurementByFilteredIndex(indexFiltered).p;
}

const PointMeasurement& CloudBatch::getMeasurementByFilteredIndex(const size_t indexFiltered) const{
  return getMeasurements().getMeasurement(getMeasurementIndexFromFilteredIndex(indexFiltered));
}


PmVector3 CloudBatch::getNormal(const size_t index) const{
  CHECK(hasNormals()) << "Normals not set for this cloud";
  return cloudScan.descriptors.block(normalsRow, index, 3, 1);
}

PmVector3 CloudBatch::getEigenValues(const size_t index) const{
  CHECK(eigenValuesRow >= 0) << "EigenValues not set for this cloud";
  return cloudScan.descriptors.block(eigenValuesRow, index, 3, 1);
}

bool CloudBatch::hasNormals() const {
  return normalsRow >= 0;
}

unsigned CloudBatch::getDescriptorIndexIfExists(const std::string& descName) const {
  return cloudScan.descriptorExists(descName) ? cloudScan.getDescriptorStartingRow(descName) : -1;
}

void CloudBatch::updateCloudInfo() {
  indexRow = getDescriptorIndexIfExists("index");
  normalsRow = getDescriptorIndexIfExists("normals");
  eigenValuesRow = getDescriptorIndexIfExists("eigValues");
}

void CloudBatch::transformMeasurementsIntoCloud(const MeasurementTransformer& transformer) {
  if(getFilteredSize() == getMeasurements().getSize()){ // is not filtered?
    CHECK(indexRow >= 0) << "Index is missing!";
    getMeasurements().fillMeasurementsInto(transformer, cloudScan.features.topRows(3), cloudScan.descriptors.row(indexRow));
  } else {
    for(size_t i = 0; i < getFilteredSize(); i ++){
      const PointMeasurement& m = getMeasurementByFilteredIndex(i);
      cloudScan.features.col(i).head<3>() = transform(transformer, m.t, m.p);
    }
  }
}

CloudBatch::AssociationsMapAndTransformation& CloudBatch::AssociationsMapAndTransformation::operator =(const CloudBatch::AssociationsAndTransformation& other) {
  matchingPair2DataMap.clear();
  T_ref_read = other.T_ref_read;
  matchingPair2DataMap.reserve(other.associations.size() * 2);
  for(const CloudBatch::Association& asso : other.associations){
    matchingPair2DataMap.emplace(asso.matchIndixPair, asso.data);
  }
  assert(size() == other.size());
  return *this;
}

}
}
