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

std::unique_ptr<CloudMeasurements> createCloudMeasurements(const PointCloudSensor& pcs, CloudBatch& cloud){
  return pcs.createCloudMeasurements(cloud);
}

PointCloudSensor::PointCloudSensor(Model& model, const std::string& name, sm::value_store::ValueStoreRef config)
: Sensor(model, name, config),
  filter(isUsed() ? model.resolveConfigPath(config.getString(name + "/pcFilterConfig")) : std::string()),
  nanPolicy({true, false})
{
}

PointCloudSensor::~PointCloudSensor(){
}

void PointCloudSensor::filterPointCloud(CloudBatch& data) {
  filter.filter(data);
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
    maximalDuration(config.getDouble("maximalDuration")),
    minimalDuration(config.getDouble("minimalDuration", double(maximalDuration) / 2.0)),
    minimalGap(config.getDouble("minimalGap")),
    startPadding(config.getDouble("startPadding", 0.01))
{
  SM_ASSERT_GE(std::runtime_error, minimalDuration, Duration(0.), "");
  SM_ASSERT_GE(std::runtime_error, minimalGap, Duration(0.), "");
  SM_ASSERT_LT(std::runtime_error, minimalDuration, maximalDuration, "");
}
void SimplePointCloudPolicy::print(std::ostream& out) const {
  out << "SimplePointCloudPolicy( "<< getName();
  MODULE_WRITE_PARAM(minimalDuration);
  MODULE_WRITE_PARAM(maximalDuration);
  MODULE_WRITE_PARAM(minimalGap);
  MODULE_WRITE_PARAM(startPadding);
  out << ")";
}

bool SimplePointCloudPolicy::isReady(CloudBatch& b) const {
  return minimalDuration <= Duration(b.getDuration());
}
bool SimplePointCloudPolicy::wouldStillFit(CloudBatch& b, Timestamp t) const {
  return ! b.isClosed() && (b.isEmpty() || t - Timestamp(b.getMinTimestamp()) <= maximalDuration);
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
      startTime = calib.getNextTimeWindowStartTimestamp() + startPadding;
    } else {
      auto& lastCloud = clouds.back();
      startTime = Timestamp(lastCloud.getMaxTimestamp()) + minimalGap;
    }
    if(startTime <= t){
      clouds.createNewCloud(pcp, sensor);
      clouds.getCurrentCloud().setMaximalDuration(maximalDuration);
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

const PointCloudPolicy& PointCloudSensor::getPointCloudPolicy(const PointCloudsPlugin& pc) const {
  if(!pointCloudPolicy){
    pointCloudPolicy = pc.getPointCloudPolicy(*this);
  }
  return *pointCloudPolicy;
}

Interval PointCloudSensor::getCurrentMeasurementTimestampRange(const CalibratorI& calib) const {
  auto& clouds = calib.getPlugin<PointCloudsPlugin>().getCloudsContainer().getCloudsFor(id);
  CHECK(!clouds.empty() && ! clouds.front().isEmpty());
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
  return std::unique_ptr<CloudMeasurements>(new EuclideanCloudMeasurements(nanPolicy));
}

void PointCloudSensor::writeConfig(std::ostream& out) const {
  MODULE_WRITE_PARAM(nanPolicy.checkForNans);
  MODULE_WRITE_PARAM(nanPolicy.nansAreFine);
}

} /* namespace calibration */
} /* namespace aslam */
