#include <aslam/backend/ErrorTerm.hpp>
#include <aslam/backend/EuclideanExpression.hpp>
#include "aslam/calibration/clouds/PointCloudsPlugin.h"

#include <aslam/backend/OptimizationProblemBase.hpp>
#include <aslam/backend/TransformationExpression.hpp>
#include <aslam/calibration/calibrator/CalibratorI.h>

#include <aslam/calibration/clouds/CalibrationMatcher.h>
#include <aslam/calibration/clouds/CloudsContainer.h>
#include <aslam/calibration/error-terms/ErrorTermCloudAssociation.h>
#include <aslam/calibration/error-terms/ErrorTermPointToPlain.h>
#include <aslam/calibration/model/sensors/Lidar3d.h>
#include <aslam/calibration/model/sensors/PointCloudSensor.h>
#include <aslam/calibration/tools/ErrorTermStatistics.h>
#include <aslam/calibration/tools/Parallelizer.h>
#include <aslam/calibration/tools/PointCloudTools.h>
#include <sm/kinematics/Transformation.hpp>
#include <sm/timing/Timer.hpp>

using aslam::backend::ErrorTerm;
using aslam::backend::EuclideanExpression;
using aslam::backend::TransformationExpression;
using sm::timing::Timer;

namespace aslam {
namespace calibration {

struct CloudMatcher {
  CloudMatcher(std::string name, ValueStoreRef& c) : name(name), calibMatcher(name, c.getString("")) {
  }
  std::string name;
  CalibrationMatcher calibMatcher;
};

PointCloudsPlugin::PointCloudsPlugin(CalibratorI& calib, ValueStoreRef config) :
  CalibratorPlugin(calib, config),
  useSymmetricLidar3dAssociations_(config.getBool("useSymmetricLidar3dAssociations")),
  usePointToPlainErrorTerm_(config.getBool("usePointToPlainErrorTerm")),
  mapFrame_(calib.getModel().getFrame(config.getString("mapFrame"))),
  cloudsContainer_(std::make_shared<CloudsContainer>(*this))
{
  for(auto child : config.getChild("cloudMatchers").getChildren()){
    LOG(INFO) << "Loading cloud matcher " << child.getKey();
    cloudMatchers_.push_back(std::make_shared<CloudMatcher>(child.getKey(), child));
  }

  for(auto child : config.getChild("cloudPolicies").getChildren()){
    pointCloudPolicies_.push_back(std::make_shared<SimplePointCloudPolicy>(child.getKey(), child));
    LOG(INFO) << "Loaded point cloud policy " << *pointCloudPolicies_.back();
  }
  CHECK(!pointCloudPolicies_.empty()) << "Require at least one point cloud policy";
  defaultPointCloudPolicy_ = pointCloudPolicies_.back();
  LOG(INFO) << "Using DefaultPointCloudPolicy=" << *defaultPointCloudPolicy_;
}

PointCloudsPlugin::~PointCloudsPlugin() = default;

const CalibrationMatcher& PointCloudsPlugin::getCloudMatcher(const Sensor& sensorRegister, const Sensor& sensorReference) const {
  CHECK(!cloudMatchers_.empty()) << "Require at least one cloud matcher!";

  std::shared_ptr<CloudMatcher> found;

  for(std::string testName : {sensorRegister.getName() + "2" + sensorReference.getName(), sensorRegister.getName() + "2Any", std::string("Any2") + sensorReference.getName(), std::string("default")} ){
    for(auto p : cloudMatchers_){
      if (p->name == testName) {
        found = p;
        break;
      } else {
        VLOG(1) << "Name mismatch:" << p->name << " != " << testName;
      }
    }
    if(found) break;
  }
  if(!found){
   found = cloudMatchers_.back();
  }

  LOG(INFO) << "Using cloud matcher " << found->name << " for " << sensorRegister.getName() << " to " << sensorReference.getName() << " matching.";
  return found->calibMatcher;
}

bool PointCloudsPlugin::getUseSymmetricLidar3dAssociations() {
  return useSymmetricLidar3dAssociations_;
}

std::shared_ptr<const PointCloudPolicy> PointCloudsPlugin::getDefaultPointCloudPolicy() const {
  return std::static_pointer_cast<const PointCloudPolicy>(defaultPointCloudPolicy_);
}

std::shared_ptr<const PointCloudPolicy> PointCloudsPlugin::getPointCloudPolicy(const PointCloudSensor& sensor) const {
  CHECK(!pointCloudPolicies_.empty()) << "This requires at least one point cloud policy";
  std::shared_ptr<const PointCloudPolicy> found;

  for(std::string testName : {sensor.getName()} ){
    for(auto p : pointCloudPolicies_){
      if (p->getName() == testName) {
        found = std::static_pointer_cast<const PointCloudPolicy>(p);
        break;
      } else {
        VLOG(1) << "Name mismatch:" << p->getName() << " != " << testName;
      }
    }
  }
  if(!found){
   found = sensor.getDefaultPointCloudPolicy(*this);
  }

  LOG(INFO) << "Using " << *found << " for " << sensor.getName() << ".";
  return found;
}

const Frame& PointCloudsPlugin::getMapFrame() const {
  return mapFrame_;
}


//TODO A check those two weight functions
double getDistanceWeight(const CloudBatch& cloud, const CloudBatch& cloudRef, const int readIndex, const int refIndex) {
  //if(associations.indicesHash.find(read) != refMatchingPairs.end()
  //return associations.weights(associations.indicesHash.)

  auto& matches = cloud.getMatchesTo(cloudRef).matchingPair2DataMap;
  auto iter = matches.find({readIndex, refIndex});
  if(iter != matches.end()){
    return iter->second.weight;
  }
  LOG(WARNING) << "Could not find match.";
  return 0.0;
}


// Describes the planarity of the surface
PM::ScalarType getNormalWeight(const CloudBatch& cloud, const int readIndex) {
  auto eigV = cloud.getEigenValues(readIndex);
  PM::ScalarType p = 2*(eigV.sum() - 2*eigV.minCoeff() - eigV.maxCoeff())/eigV.sum();
  assert(p >= 0.0 && p <= 1.0);
  return p;
}

void PointCloudsPlugin::addCloudAssociationsErrorTerms(const Sensor& referenceSensor, const std::function<bool(const Sensor &)> sensorFilter, ErrorTermReceiver& errorTermReceiver, const CalibrationConfI& ec) {
  CHECK(referenceSensor.isA<PointCloudSensor>()) << "Sensor " << referenceSensor << " is not a PointCloudSensor!";

  const std::string taskName =  referenceSensor.getName() + " to sensor selection error terms.";
  if(!ec.getErrorTermActivator().isActive(referenceSensor)){
    LOG(INFO) << "Skipping " << taskName << " because " << referenceSensor.getName() << "'s error terms are deactivated.";
    return;
  }
  LOG(INFO) << "Adding " << taskName;

  for (const Sensor& s : getModel().getSensors()) {
    if(sensorFilter(s)){
      CHECK(s.isA<PointCloudSensor>()) << "Sensor " << s << " is not a PointCloudSensor!";
      addCloudAssociationsErrorTerms(referenceSensor.as<PointCloudSensor>(), s.as<PointCloudSensor>(), errorTermReceiver, ec);
    }
  }
}

void PointCloudsPlugin::addCloudAssociationsErrorTerms(const PointCloudSensor& sensorRef, const PointCloudSensor& sensorRead, ErrorTermReceiver& errorTermReceiver, const CalibrationConfI& ec) {
  ErrorTermCloudAssociation::CovarianceInput Qn;
  if(auto lidar3dPtr = sensorRef.ptrAs<Lidar3d>()){
    // Covariance of the normal needs further investigation (Not used)
    // TODO: A Use normals covariance!
    Qn = lidar3dPtr->getNormalsCovariance().getValue();
  }
  else {
    LOG(ERROR) << "Using non Lidar3d sensor (" << sensorRef.getName() << ") as reference is not supported, yet!"; // Current issue is the missing normal covariance.
    return;
  }

  const auto& storage = getCalibrator().getCurrentStorage();

  const std::string taskGroupName = sensorRead.getName() + " to " + sensorRef.getName() + " association";
  if(!sensorRead.hasClouds(storage)){
    LOG(INFO) << "Skipping " << taskGroupName << " error terms because " << sensorRead.getName() << " has no clouds.";
    return;
  }
  if(!sensorRef.hasClouds(storage)){
    LOG(INFO) << "Skipping " << taskGroupName << " error terms because " << sensorRef.getName() << " has no clouds.";
    return;
  }
  if(!ec.getErrorTermActivator().isActive(sensorRead)){
    LOG(INFO) << "Skipping " << taskGroupName << " error terms because " << sensorRead.getName() << "'s error terms are deactivated.";
    return;
  }
  if(!ec.shouldSensorsBeRegistered(sensorRead, sensorRef)){
    LOG(INFO) << "Skipping " << taskGroupName << " error terms because non of them gets calibrated and the trajectory is deactivated.";
    return;
  }
  LOG(INFO) << "Adding " << taskGroupName << " error terms.";
  ErrorTermStatistics totalToRefSensorStat(taskGroupName);

  const auto& cloudBatchesRef = sensorRef.getClouds(storage);
  const auto& cloudBatchesRead = sensorRead.getClouds(storage);

  auto& calib = getCalibrator();
  const Interval& validRange = calib.getCurrentEffectiveBatchInterval();

  const bool observeOnly = sensorRef.shouldObserveOnly(ec) || sensorRead.shouldObserveOnly(ec);

  const bool readProvidesCorrection = sensorRead.isProvidingCorrection();
  const bool refProvidesCorrection = sensorRef.isProvidingCorrection();

  for (const auto& cloudRef : cloudBatchesRef){
    for (const auto& cloudRead : cloudBatchesRead) {
      if (&cloudRef == &cloudRead) // identical clouds
        continue;

      const std::string taskName = sensorRead.getName() + "_" + cloudRead.getId() + "->" + sensorRef.getName() + "_" + cloudRef.getId() + (observeOnly ? "(OBSERVER)" : "");
      if (!cloudRead.hasMatchesTo(cloudRef)){
        VLOG(1) << "Nothing to match for : " << taskName;
        continue;
      }

      ErrorTermStatistics taskStat(taskName);
      ErrorTermGroupReference etgr(taskName);

      const bool readCloudHasNormals = cloudRead.hasNormals();
      try {
        const auto& readMatchingPairs = cloudRead.getMatchesTo(cloudRef).matchingPair2DataMap;

        if(readMatchingPairs.empty())
          continue;

        for(auto it = readMatchingPairs.begin(); it != readMatchingPairs.end(); it++)
        {
          const int refIndex = it->first.second;
          const int readIndex = it->first.first;

          auto& refMeasurement = cloudRef.getMeasurementByFilteredIndex(refIndex);
          auto& refTimestamp = refMeasurement.t;
          auto refPoint = refMeasurement.p.cast<double>().eval();
          auto& readMeasurement = cloudRead.getMeasurementByFilteredIndex(readIndex);
          auto& readTimestamp = readMeasurement.t;
          auto readPoint = readMeasurement.p.cast<double>().eval();

          if (!validRange.contains(readTimestamp, sensorRead) ||!validRange.contains(refTimestamp, sensorRef)){
            taskStat.skip();
            continue;
          }

          auto robotRead = calib.getModelAt(sensorRead, readTimestamp, 0, ModelSimplification{true});
          auto robotRef = calib.getModelAt(sensorRef, refTimestamp, 0, ModelSimplification{true});

          TransformationExpression T_m_read = sensorRead.getTransformationExpressionTo(robotRead, getMapFrame());
          TransformationExpression T_m_ref = sensorRef.getTransformationExpressionTo(robotRef, getMapFrame());
          const auto R_m_read = T_m_read.toRotationExpression();
          const auto R_m_ref = T_m_ref.toRotationExpression();

          // TODO A check all this!
          //These normals don't depend on the R_m_v_read expression! not its current value, because they have been transformed to map frame using the same transformation!
          Eigen::Vector3d refNormal = R_m_ref.toRotationMatrix().inverse() * cloudRef.getNormal(refIndex).cast<double>();
          Eigen::Vector3d readNormal = Eigen::Vector3d::Zero();
          if(readCloudHasNormals){
            readNormal = R_m_read.toRotationMatrix().inverse() * cloudRead.getNormal(readIndex).cast<double>();
          }

          // Error term: e = <n_a,(p_a-p_b)>
          // TODO: A Setup the covariances, possibly using the variance of the velodyne
          // TODO: B Check for the covariance of the normal vector

          ErrorTermCloudAssociation::CovarianceInput Qp_read = sensorRead.covPoint(readCloudHasNormals, readPoint, readNormal);
          ErrorTermCloudAssociation::CovarianceInput Qp_ref = sensorRef.covPoint(true, refPoint, refNormal);

          // TODO: A Get the outlier weight

          const auto weight = getDistanceWeight(cloudRead, cloudRef, readIndex, refIndex) * getNormalWeight(cloudRef, refIndex);
          // TODO B cleanup the weighting!
          //weight = it->second.weight;

          boost::shared_ptr<ErrorTerm> e_cloud;
          if(!usePointToPlainErrorTerm_){
            e_cloud.reset(new ErrorTermCloudAssociation(
              T_m_ref,
              T_m_read,
              R_m_ref,
              refNormal, refPoint, readPoint,
              weight,
              Qn, Qp_ref, Qp_read));
          }else{
            Eigen::Matrix3d R_read_ref = (R_m_read.toRotationMatrix()).transpose() * R_m_ref.toRotationMatrix();

            EuclideanExpression readExpression = readProvidesCorrection ? sensorRead.calcMeasurementExpressionByFilteredIndex(cloudRead, readIndex) : EuclideanExpression(readPoint);
            EuclideanExpression readTransformed = T_m_ref.inverse() * T_m_read * readExpression;
            if(refProvidesCorrection){
              readTransformed = readTransformed - sensorRef.calcMeasurementExpressionByFilteredIndex(cloudRef, refIndex);
              refPoint.setZero();
            }

            e_cloud.reset(new ErrorTermPointToPlain(
                refNormal, refPoint,
                readTransformed,
                weight,
                Qn, Qp_ref, R_read_ref.transpose() * Qp_read * R_read_ref,
                etgr));
          }

          e_cloud->setMEstimatorPolicy(sensorRead.getMEstimator());

          totalToRefSensorStat.add(e_cloud);
          taskStat.add(e_cloud);
          if (!observeOnly) {
            errorTermReceiver.addErrorTerm(e_cloud);
          }
        }

      } catch (std::exception& e){
        LOG(ERROR) << "Matching failed :" << e.what();
      }

      taskStat.printInto(LOG(INFO));
    }
  }
  totalToRefSensorStat.printInto(LOG(INFO));
}

void PointCloudsPlugin::postprocessAssociations() {
  auto pointCloudSensors = getModel().getSensors<PointCloudSensor>();
  for (const PointCloudSensor& fromSensor : pointCloudSensors) {
    for(const PointCloudSensor& toSensor : pointCloudSensors) {
      const std::string taskName = fromSensor.getName() + "->" + toSensor.getName() + " associations.";
      LOG(INFO) << "Post processing " << taskName;
      size_t nAssociations = cloudsContainer_->countAssociations(fromSensor, toSensor);
      VLOG(1) << "Counted " << nAssociations << " " << taskName;
      if(nAssociations == 0){
        continue;
      }
      if(fromSensor == toSensor) { // B TODO what about symetric associations for two different 3d sensors?
        if (getUseSymmetricLidar3dAssociations()) {
          LOG(INFO)<< "Running symmetric 3d cloud association filer: " << taskName;
          // Verify all the matches and get the intersection ones
          Timer intersection_points_timer("Filter symmetric associations");
          nAssociations = cloudsContainer_->associationsIntersection(fromSensor, toSensor);
          intersection_points_timer.stop();

          LOG(INFO) << "Found symmetric associations : " << nAssociations  << " " << taskName;
        }
      }

      if (nAssociations > 0) {
        // Limit on the maximum number of associations (Should be set up in the configuration file)
        size_t maxNumberAssociationsErrorTerms = maxNumCloudAssociations_;
        if(is2dPointCloudSensor(fromSensor) || is2dPointCloudSensor(toSensor)){
          maxNumberAssociationsErrorTerms /= 10; //TODO B find a good maxNumCloudAssociations policy
        }
        if (nAssociations > maxNumberAssociationsErrorTerms) {
          LOG(INFO)<< "Subsampling to get about maxNumberAssociationsErrorTerms=" << maxNumberAssociationsErrorTerms << " " << taskName;
          // We send a ratio that gives us the desired amount of associations (more or less)
          nAssociations = cloudsContainer_->associationsRandomSubsample(maxNumberAssociationsErrorTerms/double(nAssociations), fromSensor, toSensor);
          LOG(INFO) << "Associations after: " << nAssociations  << " " << taskName;
        } else {
          LOG(INFO) << "Too few; keeping all for " << taskName;
        }
      }
    }
  }
}

void PointCloudsPlugin::closeCurrentCloud(const PointCloudSensor& sensor) {
  LOG(INFO) << "Closing current cloud for " << sensor.getName();
  auto& clouds = sensor.getClouds(getCalibrator().getCurrentStorage());
  clouds.getPointCloudPolicy().prepareForEstimation(*this, clouds);
}

void PointCloudsPlugin::closeAllCurrentClouds() {
  LOG(INFO) << "Closing all remaining open clouds.";
  for (const PointCloudSensor& sensor : getModel().getSensors<PointCloudSensor>()){
    closeCurrentCloud(sensor);
  }
}

void PointCloudsPlugin::preProcessNewWindow(CalibratorI &) {
  LOG(INFO) << "Preprocessing current window's clouds.";
  closeAllCurrentClouds();

  auto& storage = getCalibrator().getCurrentStorage();
  for (const PointCloudSensor& sensor : getModel().getSensors<PointCloudSensor>()){
    auto& clouds = sensor.getClouds(storage);
    size_t s = 0;
    for (auto& cloud : clouds) {
      Timer deleting_points_timer("Deleting out of bounds points");
      auto& measurements = cloud.getMeasurements();
      const Interval & interval = getCalibrator().getCurrentEffectiveBatchInterval();
      measurements.deleteUpToStartTimestamp(interval.start);
      measurements.deleteFromEndTimestamp(interval.end);
      deleting_points_timer.stop();

      s+= measurements.getSize();

      if (measurements.getSize() < 1000) { // TODO C Make magic number a named constant or parameter
        LOG(WARNING) << "Got too small point cloud, containing only " << measurements.getSize()<< " points!";
      }
    }
    LOG(INFO)<< "Point cloud sensor " << sensor.getName() << " has " << clouds.size() << " clouds with a total of " << s << " points.";
  }
}


void PointCloudsPlugin::buildCloudProblem(CloudsContainerSP& cloudsContainer, const CalibrationConfI& config){
  LOG(INFO) << "Processing point clouds";

  Parallelizer parallelizer(getCalibrator().getOptions().getNumThreads());

  auto& storage = getCalibrator().getCurrentStorage();

  for(PointCloudSensor& sensor : getModel().getSensors<PointCloudSensor>()){
    auto& clouds = sensor.getClouds(storage);
    if(!clouds.hasData()) continue;

    size_t cloudCount = 0;
    const auto T_r_s = sensor.calcTransformationToParent();

    for (auto& cloud : clouds) {
      {
        Timer creating_points_timer("Creating cloud points");
        cloud.createCloud();
        assert(cloud.getMeasurements().getSize() == size_t(cloud.getFilteredSize()));
      }
      {
        Timer predicting_points_timer("Predicting points"); //TODO B optimize : don't predict point that get filtered out?
        updateCloudUsingCurrentTrajectory(cloud, T_r_s, sensor); //TODO C why isn't this thread safe? (cannot be moved into the parallel part below. If it was one could probably move everything there
        assert(cloud.getMeasurements().getSize() == size_t(cloud.getFilteredSize()));
      }

      parallelizer.add([this, &sensor, &cloud, &T_r_s, cloudCount, cloudsContainer](){
        const std::string task = sensor.getName() + "(" + std::to_string(cloudCount) + ")"; // TODO D support batches again ( batchId_ + ", ")
        if(cloud.getMeasurements().getSize() == 0){
          LOG(WARNING) << "Skipping empty point cloud " << task << ", " << cloudCount << "!";
          return;
        }
        LOG(INFO) << "Num points before filtering " << task << " : " << cloud.getMeasurements().getSize();
        {
          Timer filtering_points_timer("Filtering points");
          sensor.as<PointCloudSensor>().filterPointCloud(cloud);
        }
        {
          Timer updating_points_timer("Updating point indices");
          cloud.updateIndices();
        }
        LOG(INFO) << "Num points after filtering  " << task << " : " << cloud.getFilteredSize();
      });
      cloudCount++;
    }
  }

  parallelizer.doAndWait();

  LOG(INFO) << "Computing associations: ";
  Timer associating_points_timer("Associating points");
  size_t nAssociations = cloudsContainer_->computeAssociations(config);
  associating_points_timer.stop();
  LOG(INFO) << "Associations found: " << nAssociations;

  postprocessAssociations();
}

void clampTimestamp(NsecTime timestamp, NsecTime min, NsecTime max, bool * clamped = nullptr){
  *clamped = false;
  if(timestamp < min){
    if(clamped) *clamped = true;
    timestamp = min;
  }
  if(timestamp > max){
    if(clamped) *clamped = true;
    timestamp = max;
  }
}

struct MeasurementTransformer {
  const sm::kinematics::Transformation& operator () (Timestamp timestamp) const {
    if(timestamp != last_timestamp){
      last_timestamp = timestamp;
      T_m_s = calib_.getModelAt(timestamp, 0, {}).getTransformationToFrom(pcp_.getMapFrame(), parentFrame).toTransformation() * T_r_s;
    }
    return T_m_s;
  }

  friend Eigen::Vector3d transform(const MeasurementTransformer& transformer, const Timestamp timestamp, const Eigen::Vector3d& p);

  MeasurementTransformer(const PointCloudsPlugin& pcp, const sm::kinematics::Transformation& T_r_s, const PointCloudSensor& sensor) :
    pcp_(pcp),
    calib_(pcp.getCalibrator()),
    T_r_s(T_r_s),
    parentFrame(sensor.getParentFrame()),
    pcs(sensor)
  {}

  const PointCloudsPlugin& pcp_;
  const CalibratorI& calib_;
  sm::kinematics::Transformation T_r_s;
  mutable Timestamp last_timestamp = InvalidTimestamp();
  mutable sm::kinematics::Transformation T_m_s;
  const Frame& parentFrame;
  const PointCloudSensor& pcs;
};

Eigen::Vector3d transform(const MeasurementTransformer& transformer, const Timestamp timestamp, const Eigen::Vector3d& p) {
  return transformer(timestamp) * p;
}
Eigen::Vector3f transform(const MeasurementTransformer& transformer, const Timestamp timestamp, const Eigen::Vector3f& p) {
  // TODO O support float transformation
  return transform(transformer, timestamp, p.cast<double>().eval()).cast<float>();
}

void PointCloudsPlugin::updateCloudUsingCurrentTrajectory(CloudBatch& cloud, const sm::kinematics::Transformation& T_r_s, const Sensor& sensor){
  CHECK(sensor.isA<PointCloudSensor>());
  const PointCloudSensor& pcs = sensor.as<PointCloudSensor>();
  MeasurementTransformer transformer(*this, T_r_s, pcs);
  cloud.transformMeasurementsIntoCloud(transformer);
}

void PointCloudsPlugin::clearCloudsAssociations() {
  sm::timing::Timer timer("Calibrator: clearCloudsAssociations");
  cloudVersionCounter_ = 0; //TODO C move resetting output counters to a proper event.
  if(cloudsContainer_){
    cloudsContainer_->clearAssociations();
  }
}

void PointCloudsPlugin::writeSnapshot(const CalibrationConfI&, const std::string& path, bool updateFirstBasedOnCurrentSplines){
  Parallelizer parallelizer(getCalibrator().getOptions().getNumThreads());
  auto& storage = getCalibrator().getCurrentStorage();

  cloudVersionCounter_ ++;

  if(updateFirstBasedOnCurrentSplines){
    // Update clouds with newest trajectory
    for(const PointCloudSensor& sensor : getModel().getSensors<PointCloudSensor>()){
      auto T_p_s = sensor.calcTransformationToParent(); // TODO A support more comples models.
      for (auto& cloud : sensor.getClouds(storage)){
        parallelizer.add([this, &cloud, T_p_s, &sensor](){ updateCloudUsingCurrentTrajectory(cloud, T_p_s, sensor); });
      }
    }
    parallelizer.doAndWait();
    LOG(INFO) << "Fished parallel updating of the point clouds." <<std::endl;
  }

  // Write clouds
  for(const PointCloudSensor& s : getModel().getSensors<PointCloudSensor>()){
    if(s.hasClouds(storage)){
      cloudsContainer_->saveFilteredClouds(s, path, cloudVersionCounter_, parallelizer);
    }
  }
  parallelizer.doAndWait();
  LOG(INFO) << "Fished parallel writing of the point clouds." <<std::endl;

// TODO D support aligned
//  LOG(INFO) << "Writing aligned point clouds." <<std::endl;
//  cloudsContainer_->saveAlignedClouds(resultsFolder, version);
//  LOG(INFO) << "Fished writing of aligned point clouds." <<std::endl;

  // TODO C save associations again
//  cloudsContainer_->saveTransformations(resultsFolder);
//  cloudsContainer_->saveReducedAssociations(resultsFolder);
//  cloudsContainer_->saveAssociations(resultsFolder);
}

} /* namespace calibration */
} /* namespace aslam */
