#include <aslam/calibration/model/sensors/PointCloudSensor.h>

#include <aslam/calibration/CalibrationConfI.h>
#include <aslam/calibration/calibrator/CalibratorI.hpp>
#include <aslam/calibration/model/Model.h>
#include <aslam/calibration/model/ModuleTools.h>
#include <aslam/calibration/tools/tools.h>

#include <aslam/calibration/clouds/CloudBatch.h>
#include <aslam/calibration/clouds/CloudsContainer.h>
#include <aslam/calibration/clouds/PointCloudsPlugin.h>

namespace aslam {
namespace calibration {

PointCloudSensor::PointCloudSensor(Model& model, const std::string& name, sm::value_store::ValueStoreRef config)
: Sensor(model, name, config),
  filter_(isUsed() ? model.resolveConfigPath(config.getString(name + "/pcFilterConfig")) : std::string()),
  nanPolicy_({true, false}),
  clouds_(this, [this](const Module*, ModuleStorage& storage){ return new CloudBatches(*this, storage.getCalibrator());})
{
}

const CloudBatches& PointCloudSensor::getClouds(const ModuleStorage& storage) const {
  return clouds_.getDataFrom(storage);
}

CloudBatches& PointCloudSensor::getClouds(ModuleStorage& storage) const {
  return clouds_.getDataFrom(storage);
}

void PointCloudSensor::writeConfig(std::ostream& out) const {
  MODULE_WRITE_PARAM(nanPolicy_.checkForNans);
  MODULE_WRITE_PARAM(nanPolicy_.nansAreFine);
}

PointCloudSensor::~PointCloudSensor(){
}

void PointCloudSensor::filterPointCloud(CloudBatch& data) {
  filter_.filter(data);
}

size_t PointCloudSensor::findPointsInFieldOfView(const DP& /* pointCloud */, const sm::kinematics::Transformation& /* T_sensor_pointCloud */, std::vector<bool>& /* goodPoints */) const {
  throw(std::runtime_error("Not implemented!"));
}

Eigen::Matrix3d PointCloudSensor::covPoint(bool /* useSurfaceNormal */, const Eigen::Vector3d& /* pInSensorFrame */, const Eigen::Vector3d& /* nInSensorFrame */) const {
  throw std::runtime_error("Not implemented");
}

std::ostream& operator <<(std::ostream& out, const CloudBatch& c) {
  out
    << "Cloud(sensor=" << c.getSensor().getName()
    << ", size=" << c.getMeasurements().getSize()
    << ", filteredSize=" << c.getFilteredSize()
    << ", dur=" << double(Duration(c.getDuration()))
    << ", closed="<< c.isClosed()
    << ", maximalDuration=" << double(Duration(c.getMaximalDuration()))
    << ")";
  return out;
}

void PointCloudPolicy::prepareForEstimation(const PointCloudsPlugin& pcp, CloudBatches& clouds) const {
  if(clouds.isAcceptingData()){
    clouds.finishCurrentCloud(pcp);
  }
}

SimplePointCloudPolicy::SimplePointCloudPolicy(std::string name, ValueStoreRef config) :
    NamedMinimal(name),
    maximalDuration_(config.getDouble("maximalDuration")),
    minimalDuration_(config.getDouble("minimalDuration", double(maximalDuration_) / 2.0)),
    minimalGap_(config.getDouble("minimalGap")),
    startPadding_(config.getDouble("startPadding", 0.01))
{
  SM_ASSERT_GE(std::runtime_error, minimalDuration_, Duration(0.), "");
  SM_ASSERT_GE(std::runtime_error, minimalGap_, Duration(0.), "");
  SM_ASSERT_LT(std::runtime_error, minimalDuration_, maximalDuration_, "");
}
void SimplePointCloudPolicy::print(std::ostream& out) const {
  out << "SimplePointCloudPolicy( "<< getName();
  MODULE_WRITE_PARAM(minimalDuration_);
  MODULE_WRITE_PARAM(maximalDuration_);
  MODULE_WRITE_PARAM(minimalGap_);
  MODULE_WRITE_PARAM(startPadding_);
  out << ")";
}

bool SimplePointCloudPolicy::isReady(CloudBatch& b) const {
  return minimalDuration_ <= Duration(b.getDuration());
}
bool SimplePointCloudPolicy::wouldStillFit(CloudBatch& b, Timestamp t) const {
  return ! b.isClosed() && (b.isEmpty() || t - Timestamp(b.getMinTimestamp()) <= maximalDuration_);
}

void SimplePointCloudPolicy::prepareForNewData(const PointCloudsPlugin& pcp, CloudBatches& clouds, Timestamp t, const PointCloudSensor& sensor) const {
  if(clouds.isAcceptingData()){
    auto& currentCloud = clouds.getCurrentCloud();
    if(!wouldStillFit(currentCloud, t)){
      if(!isReady(currentCloud)){
        LOG(WARNING) << "Dropping a intermediate point cloud "<< currentCloud << " because it isn't ready but new data doesn't fit in.";
        clouds.dropCurrentCloud();
      } else {
        clouds.finishCurrentCloud(pcp);
      }
    }
  }
  auto& calib = pcp.getCalibrator();
  if(!clouds.isAcceptingData() && calib.isNextWindowScheduled()) {
    Timestamp startTime;
    if(clouds.empty()){
      startTime = calib.getNextTimeWindowStartTimestamp() + startPadding_;
    } else {
      auto& lastCloud = clouds.back();
      startTime = Timestamp(lastCloud.getMaxTimestamp()) + minimalGap_;
    }
    if(startTime <= t){
      clouds.createNewCloud(pcp, sensor);
      clouds.getCurrentCloud().setMaximalDuration(maximalDuration_);
    }
  }
}

void SimplePointCloudPolicy::prepareForEstimation(const PointCloudsPlugin& pcp, CloudBatches& clouds) const {
  if(clouds.isAcceptingData()){
    if(isReady(clouds.getCurrentCloud())){
      clouds.finishCurrentCloud(pcp);
    } else {
      clouds.dropCurrentCloud();
    }
  }
}

Interval PointCloudSensor::getCurrentMeasurementTimestampRange(const CalibratorI& calib) const {
  auto& clouds = getClouds(calib.getCurrentStorage());
  CHECK(clouds.hasData());
  return Interval(clouds.front().getMinTimestamp(), (clouds.back().isEmpty() ? *(clouds.end()-2) : clouds.back()).getMaxTimestamp());
}

bool PointCloudSensor::isProvidingCorrection() const {
  return false;
}

aslam::backend::EuclideanExpression PointCloudSensor::calcMeasurementExpressionByUnfilteredIndex(const CloudBatch&/*cloud*/, size_t /*pointIndex*/) const {
  throw(std::runtime_error("Not implemented!"));
}

aslam::backend::EuclideanExpression PointCloudSensor::calcMeasurementExpressionByFilteredIndex(const CloudBatch& cloud, size_t filteredPointIndex) const {
  SM_ASSERT_EQ_DBG(std::runtime_error, cloud.getSensor(), *this, "");
  return calcMeasurementExpressionByUnfilteredIndex(cloud, cloud.getMeasurementIndexFromFilteredIndex(filteredPointIndex));
}

std::unique_ptr<CloudMeasurements> PointCloudSensor::createCloudMeasurements(CloudBatch& /*cloudBatch*/) const {
  return std::unique_ptr<CloudMeasurements>(new EuclideanCloudMeasurements(nanPolicy_));
}

std::shared_ptr<const PointCloudPolicy> PointCloudSensor::getDefaultPointCloudPolicy(const PointCloudsPlugin& pcp) const {
  return pcp.getDefaultPointCloudPolicy();
}

bool PointCloudSensor::hasClouds(const ModuleStorage& storage) const {
  return !getClouds(storage).empty();
}

} /* namespace calibration */
} /* namespace aslam */
