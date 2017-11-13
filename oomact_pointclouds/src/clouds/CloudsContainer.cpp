#include "aslam/calibration/clouds/CloudsContainer.h"

#include <cmath>
#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/make_shared.hpp>
#include <Eigen/Core>
#include <glog/logging.h>
#include <sm/boost/null_deleter.hpp>
#include <sm/kinematics/EulerAnglesYawPitchRoll.hpp>
#include <sm/kinematics/quaternion_algebra.hpp>
#include <sm/kinematics/Transformation.hpp>

#include <aslam/calibration/calibrator/CalibrationConfI.h>
#include <aslam/calibration/calibrator/CalibratorI.h>
#include <aslam/calibration/clouds/CalibrationMatcher.h>
#include <aslam/calibration/clouds/PointCloudsPlugin.h>
#include <aslam/calibration/model/Model.h>
#include <aslam/calibration/model/sensors/GroundPlanePseudoSensor.h>
#include <aslam/calibration/model/sensors/PointCloudSensor.h>
#include <aslam/calibration/model/sensors/StereoCamera.h>
#include <aslam/calibration/model/sensors/Velodyne.h>
#include <aslam/calibration/tools/Parallelizer.h>
#include <aslam/calibration/tools/PointCloudTools.h>
#include <aslam/calibration/tools/tools.h>

namespace aslam {
namespace calibration {


//std::string CloudFileSuffix = "csv";
std::string CloudFileSuffix = "vtk";

CloudsContainer::CloudsContainer(PointCloudsPlugin& pcp):
    pcp_(pcp),
    model_(pcp.getCalibrator().getModel()),
    currentCloudBatchStartTimestamp_(InvalidTimestamp()),
    lastCloudBatchTimestamp_(InvalidTimestamp())
{
}

CloudsContainer::~CloudsContainer() {
}

/*
/// Save associations
void CloudsContainer::saveAssociations(const std::string& folderName) const{
  auto subFolderName = folderName + "associations_batch_" + pcp_.getCalibrator().batchId_;
  if (!boost::filesystem::exists(subFolderName))
    boost::filesystem::create_directory(subFolderName);
  int i=0;
  for (auto it = lidar3dCloudBatches_.begin(); it != lidar3dCloudBatches_.cend(); ++it, i++){
    int j=0;
    for(auto jt = it->cloudAssociations.begin(); jt != it->cloudAssociations.end(); jt++, j++){
      if(i==j)
        continue;
      std::ofstream associations(subFolderName + "/" + "associations" + i + "_" + j + ".dat");
      associations << std::fixed << std::setprecision(18);
      associations << "indices " << "matches " << "dists " << "weights " << std::endl;
      for(int k=0; k<jt->indices.size(); ++k){
        associations << jt->indices(k) << " " << jt->matches(k)
                        << " " << jt->dists(k) << " " << jt->weights(k) << std::endl;

      }
    }
  }
}

void CloudsContainer::saveReducedAssociations(const std::string& folderName) const{
  auto subFolderName = folderName + "reduced_associations_batch_" + pcp_.getCalibrator().batchId_;
  if (!boost::filesystem::exists(subFolderName))
    boost::filesystem::create_directory(subFolderName);
  int i=0;
  for (auto it = lidar3dCloudBatches_.begin(); it != lidar3dCloudBatches_.cend(); ++it, i++){
    int j=0;

    for(auto jt = it->matchingPairs.begin(); jt != it->matchingPairs.end(); jt++, j++){
      if(i>=j)
        continue;


      std::ofstream associations(subFolderName + "/" + "reducedAssociations" + i + "_" + j + ".dat");
      associations << std::fixed << std::setprecision(18);
      associations << "indices " << "matches "
          << "dists(" << i << "," << j << ") dists(" << j << "," << i
          << ") weights(" << i << "," << j << ") weigths(" << j << "," << i << ")" <<  std::endl;

      std::for_each(jt->cbegin(), jt->cend(), [&](decltype(*jt->cbegin()) x) {
        associations << x.first.first << " " << x.first.second;
        // Retrieve the distances and the weights
        auto iter_ji = lidar3dCloudBatches_[j].matchingPairs[i].find(std::make_pair(x.first.second,x.first.first));
        if(iter_ji != lidar3dCloudBatches_[j].matchingPairs[i].end()){
          associations << " " << x.second.dist << " " << iter_ji->second.dist <<
                      " " << x.second.weight << " " << iter_ji->second.weight << std::endl;
        }
        else
          //associations << " " << 0.0 << " " << 0.0 << " " << 0.0 << " " << 0.0 << std::endl;
          associations << "NA NA NA NA" << std::endl;
      }
      );
    }
  }
}

void CloudsContainer::saveReducedAssociations2d(const size_t& laserId, const std::string& folderName) const{
  assert(laserId < pcp_.getCalibrator().getOptions().getNum2dLidars());
  const auto& laserBatches = lidars2dVector_[laserId];
  auto subFolderName = folderName + "reduced_associations2d_lidar_" + laserId + "_batch_" + pcp_.getCalibrator().batchId_;
  if (!boost::filesystem::exists(subFolderName))
    boost::filesystem::create_directory(subFolderName);
  int i=0;
  for (auto it = laserBatches.begin(); it != laserBatches.cend(); ++it, i++){
    int j=0;

    auto& jt = it->matchingPairs.at(i);

    std::ofstream associations(subFolderName + "/" + "reducedAssociations" + i + ".dat");
    associations << std::fixed << std::setprecision(18);
    associations << "indices " << "matches "
        << "dists(" << i << "," << j << ") dists(" << j << "," << i
        << ") weights(" << i << "," << j << ") weigths(" << j << "," << i << ")" <<  std::endl;

    std::for_each(jt.cbegin(), jt.cend(), [&](decltype(*jt.cbegin()) x) {
      associations << x.first.first << " " << x.first.second;SensorType::POINT_CLOUD_2D
      associations << std::endl;
      // Retrieve the distances and the weights
//      auto iter_ji = lidar3dCloudBatches_[j].matchingPairs[i].find(std::make_pair(x.first.second,x.first.first));
//      if(iter_ji != lidar3dCloudBatches_[j].matchingPairs[i].end()){
//        associations << " " << x.second.dist << " " << iter_ji->second.dist <<
//            " " << x.second.weight << " " << iter_ji->second.weight << std::endl;
//      }
//      else
//        //associations << " " << 0.0 << " " << 0.0 << " " << 0.0 << " " << 0.0 << std::endl;
//        associations << "NA NA NA NA" << std::endl;
    }
    );


  }
}



*/

// TODO D Support multiple batches again (Before: "_" + pcp_.getCalibrator().batchId_ + "_" + i + "_")

/// Save aligned clouds
/// Transform all the cloud into the RF of the first cloud
void CloudsContainer::saveAlignedClouds(const std::string& folderName, size_t version) const{
  if (!boost::filesystem::exists(folderName)){
     boost::filesystem::create_directory(folderName);
  }
  for (const PointCloudSensor& sensor : model_.getSensors<PointCloudSensor>()){
    int i=0;
    const CloudBatches& clouds = sensor.getClouds(pcp_.getCalibrator().getCurrentStorage());
    if(clouds.hasData()){
      for (auto& cloud : clouds) {
        transformAndSave(folderName + "aligned_" + sensor.getName() + "_" + "_" + i + "_" + version + "." + CloudFileSuffix,
                                              i == 0 ? TP::Identity(4, 4) : cloud.getMatchesTo(clouds.front()).T_ref_read, cloud.getCloud());
        i++;
      }
    }
  }
}

void CloudsContainer::transformAndSave(const std::string& fileName, const TP& T, const DP& inCloud) const {
  auto newCloud = rigidTrans_->compute(inCloud,T);
  newCloud.save(fileName);
}

/// Save filtered clouds
void CloudsContainer::saveFilteredClouds(const PointCloudSensor& sensor, const std::string& folderName, size_t version, Parallelizer& parallelizer) const{
  if (!boost::filesystem::exists(folderName))
    boost::filesystem::create_directory(folderName);
  int i=0;
  for (auto& cloud : getCloudsFor(sensor)){
    parallelizer.add([i, version, sensor, &cloud, &folderName, this](){
      cloud.saveFilteredCloud(folderName + sensor.getName() + "_" + i + "_" + version + "." + CloudFileSuffix);
    });
    i++;
  }
}

/*
void CloudsContainer::saveTransformations(const std::string& folderName) const{
  auto subFolderName = folderName + "transformations_batch_" + pcp_.getCalibrator().batchId_;
  if (!boost::filesystem::exists(subFolderName))
    boost::filesystem::create_directory(subFolderName);
  int i=0;
  for (auto it = lidar3dCloudBatches_.begin(); it != lidar3dCloudBatches_.cend(); ++it, i++){
    int j=0;
    for(auto jt = it->cloudAssociations.begin(); jt != it->cloudAssociations.end(); jt++, j++){
      if(i>=j)
        continue;
      std::ofstream transformations(subFolderName + "/" + "transformations" + i + "_" + j + ".dat");
      transformations << std::fixed << std::setprecision(18);
      transformations << "Transformation (" << i << "," << j << ")" << std::endl;
      transformations << lidar3dCloudBatches_[i].cloudAssociations[j].T_ref_read << std::endl;
      transformations << "Transformation (" << j << "," << i << ")" << std::endl;
      transformations << lidar3dCloudBatches_[j].cloudAssociations[i].T_ref_read.inverse() << std::endl;
    }
  }
}
*/

void CloudsContainer::clearAssociations(){
  auto& storage = pcp_.getCalibrator().getCurrentStorage();
  for (const PointCloudSensor& sensor : model_.getSensors<PointCloudSensor>()){
    for (auto& cloud : sensor.getClouds(storage)){
      cloud.clearAssociations();
    }
  }
}

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

size_t CloudsContainer::computeAssociations(const CalibrationConfI& cc){
  std::atomic<size_t> numberReducedAssociations(0);

  Parallelizer parallelizer(pcp_.getCalibrator().getOptions().getNumThreads());

  auto& storage = pcp_.getCalibrator().getCurrentStorage();

  for(const PointCloudSensor& sensorReference : model_.getSensors<PointCloudSensor>()){
    auto& cloudBatchesReference = sensorReference.getClouds(storage);
    if(!cloudBatchesReference.hasData()) continue;

    LOG(INFO) << "New reference sensor: " << sensorReference.getName()  << " with " << cloudBatchesReference.size() << " clouds.";

    if(!cc.shouldAnySensorBeRegisteredTo(sensorReference)){
      LOG(INFO) << "Skipping associations to point cloud sensor " << sensorReference.getName() << " because no other should be registered to it.";
      continue;
    }

    for (auto& cloudReference : cloudBatchesReference) {
      if(!cloudReference.hasNormals()){
        LOG(INFO) << "Skipping " << sensorReference.getName() << " as reference because it lacks normals";
        continue;
      }

      //TODO B make the cloud matching logic more flexible
      //TODO C optimize: select only some 3d clouds depending on the 2d laser.
      //TODO B optimize: prepare point matcher with reference
      for(const PointCloudSensor& sensorRegister : model_.getSensors<PointCloudSensor>()){
        if(!sensorRegister.isUsed() || !sensorRegister.isA<PointCloudSensor>()){
          continue;
        }
        if(sensorReference.isA<StereoCamera>() && sensorRegister.isA<Velodyne>()){
          LOG(INFO) << "Skipping Velodyne to stereo camera matching.";
          continue;
        }
        if(sensorRegister.isA<GroundPlanePseudoSensor>()){
          LOG(INFO) << "Skipping ground plain to anything matching.";
          continue;
        }
        if (!cc.shouldSensorsBeRegistered(sensorRegister, sensorReference)) {
          LOG(INFO) << "Skipping associations of " << sensorRegister.getName() << " to " << sensorReference.getName() << " because of negative shouldSensorsBeRegistered.";
          continue;
        }

        auto& cloudBatchesRegister = sensorRegister.getClouds(storage);
        if(!cloudBatchesRegister.hasData()) {
          LOG(WARNING) << "Skipping associations of " << sensorRegister.getName() << " because it has no clouds!";
          continue;
        }

        const CalibrationMatcher& cloudMatcher = pcp_.getCloudMatcher(sensorRegister, sensorReference);
        if(!cloudMatcher.isActive()){
          LOG(INFO) << "Skipping associations of " << sensorRegister.getName() << " to " << sensorReference.getName() << " because the cloud matcher is inactive.";
          continue;
        }

        LOG(INFO) << "New register sensor: " << sensorRegister.getName()  << " with " << cloudBatchesRegister.size() << " clouds.";

        for (auto& cloudRegister : cloudBatchesRegister) {
          if (sensorRegister == sensorReference && cloudReference.getId() == cloudRegister.getId()){
            continue;
          }

          auto& matches = cloudRegister.getOrCreateMatchesTo(cloudReference);

          parallelizer.add([this, &cloudMatcher, &numberReducedAssociations, &sensorRegister, &matches, &cloudRegister, &sensorReference, &cloudReference](){
            const std::string taskName = sensorRegister.getName() + "(" + cloudRegister.getId() + ") to " + sensorReference.getName() + "(" + cloudReference.getId() + ")";
            matches = CloudBatch::AssociationsMapAndTransformation(); // reset matches
            LOG(INFO) << "Associating " << taskName;
            std::vector<bool> goodPoints;

            if(cloudRegister.getFilteredSize() <= 0){
              LOG(WARNING) << "Skipping empty first point cloud " << taskName;
            }
            else if(cloudReference.getFilteredSize() <= 0){
              LOG(WARNING) << "Skipping empty second point cloud " << taskName;
            }
            else {
              bool enoughGoodPoints = true;
              if(sensorReference.isA<PointCloudSensor>() && (is2dPointCloudSensor(sensorRegister) || sensorReference.isA<GroundPlanePseudoSensor>())){
                goodPoints.reserve(cloudRegister.getFilteredSize());
                size_t countGood = sensorReference.as<PointCloudSensor>().findPointsInFieldOfView(
                  cloudRegister.getCloud(),
                  sensorReference.getTransformationExpressionToAtMeasurementTimestamp(pcp_.getCalibrator(), cloudReference.getCenterTimestamp(), pcp_.getMapFrame()).inverse().toTransformationMatrix(),
                  goodPoints
                );
                enoughGoodPoints = countGood > 1000;
                if(!enoughGoodPoints){
                  LOG(INFO) << "Skipping association " << taskName << " with too little expected overlap: " << countGood;
                }
              }

              if(enoughGoodPoints) {
                matches = cloudMatcher.getAssociations(cloudRegister, cloudReference, taskName, goodPoints.empty() ? nullptr: &goodPoints);
                VLOG(1) << "Transformation of " << taskName << ":\n" << matches.T_ref_read;
              }

              if(matches.size() == 0){
                if(enoughGoodPoints)
                  LOG(INFO) << "No associations for " << taskName << "!";
              } else {
                numberReducedAssociations += matches.size();
                LOG(INFO) << "Num associations " << taskName << ": " << matches.size();
              }
            }
          });
        }
      }
    }
  }

  parallelizer.doAndWait();
  return numberReducedAssociations;
}

size_t CloudsContainer::associationsIntersection(const PointCloudSensor& sensorA, const PointCloudSensor& sensorB){
  std::atomic<size_t> numberIntersectedAssociations(0);

    //TODO A support cross sensor
  SM_ASSERT_EQ(std::runtime_error, sensorA, sensorB, "");
  auto& cloudBatchesA = getCloudsFor(sensorA); //TODO A rename
  for (CloudBatch& readCloud : cloudBatchesA) {
    for (CloudBatch& refCloud : cloudBatchesA){
      if(readCloud.getId() >= refCloud.getId()) continue;

      const std::string taskName = sensorA.getName() + "(" + refCloud.getId() + ") <-> " + sensorA.getName() + "(" + readCloud.getId() + ")";

      // Reference to the unordered set
      auto& readMatching = readCloud.getMatchesTo(refCloud);
      auto& readMatchingPairs = readMatching.matchingPair2DataMap;
      auto& refMatching = refCloud.getMatchesTo(readCloud);
      auto& refMatchingPairs = refMatching.matchingPair2DataMap;

      if(readMatchingPairs.empty()){
        // Clear the other
        refMatchingPairs.clear();
        continue;
      }

      if(refMatchingPairs.empty()){
        // Clear the other
        readMatchingPairs.clear();
        continue;
      }

      // Checking for transformation convergence: It should take away the bad transformations, hoping that later it is better
      if(!readMatching.T_ref_read.isApprox(refMatching.T_ref_read.inverse(), 1E-1)){
        LOG(INFO) << "Skipping "<< taskName << " because transformations are not comparable:\n" <<  readMatching.T_ref_read << "vs.:\n" << refMatching.T_ref_read;
        readMatchingPairs.clear();
        refMatchingPairs.clear();
        continue;
      }

      const int pointsCount = readMatchingPairs.size();

      //assert(matches.rows() == 1);

      // At maximum there will be pointsCount
      Eigen::VectorXi keptIndices(pointsCount);
      Eigen::VectorXi keptMatches(pointsCount);

      // Reading distances and weights
      Eigen::VectorXd keptDists(pointsCount);
      Eigen::VectorXd keptWeights(pointsCount);

      // Reference distances and weights
      Eigen::VectorXd keptDistsRef(pointsCount);
      Eigen::VectorXd keptWeightsRef(pointsCount);

      int k = 0;
      int i = 0;
      for(auto it = readMatchingPairs.begin(); it != readMatchingPairs.end(); it++, i++)
      {
        auto& readIndex = it->first.first;
        auto& refIndex = it->first.second;
        bool kept = false;


        // We could simply remove the elements, instead of creating a vecotr
        // Inverting the couple (i,j) to (j,i) to find it

        auto refElem = refMatchingPairs.find(std::make_pair(refIndex,readIndex));
        if(refElem != refMatchingPairs.end()){
          kept = true;
        }

        if(kept){
          keptIndices(k) = readIndex;
          keptMatches(k) = refIndex;

          keptDists(k) = it->second.dist;
          keptWeights(k) = it->second.weight;

          keptDistsRef(k) = refElem->second.dist;
          keptWeightsRef(k) = refElem->second.weight;
          ++k;
        }
      }

      // k is the effective number of indices, so we take that
      // TODO: D Check if there is a more efficient way

      LOG(INFO) << "Number of symmetric associations for " << taskName << ": " << k;


      // Rebuild the unordered_set
      /// This operation could be potentially expensive, O(N), and in these loop there are a lot of O(N)

      readMatchingPairs.clear();
      refMatchingPairs.clear();
      for(int iter = 0; iter < k; iter++){
        readMatchingPairs.emplace(std::make_pair(keptIndices(iter),keptMatches(iter)), CloudBatch::MatchData(keptDists(iter),keptWeights(iter)));
        refMatchingPairs.emplace(std::make_pair(keptMatches(iter), keptIndices(iter)), CloudBatch::MatchData(keptDistsRef(iter),keptWeightsRef(iter)));
      }

      numberIntersectedAssociations += 2*k;
    }
  }

  return numberIntersectedAssociations;
}

bool isReferenceSensor(const PointCloudSensor& s){
  if(auto sPtr = s.ptrAs<PointCloudSensor>()){
    return sPtr->isA<Velodyne>(); // TODO B introduce the configurable notion of a reference sensor
  } else {
    return false;
  }
}

size_t CloudsContainer::associationsRandomSubsample(double prob, const PointCloudSensor& sensorA, const PointCloudSensor& sensorB){
  if(prob >= 1){
    throw (std::runtime_error("Filtering with prob >= 1"));
  }
  auto& cloudBatchesA = getCloudsFor(sensorA);
  auto& cloudBatchesB = getCloudsFor(sensorB);

  const bool bothReferenceSensors = isReferenceSensor(sensorA) && isReferenceSensor(sensorB);
  const bool symmetricMatchesRequired = bothReferenceSensors && pcp_.getCalibrator().getPlugin<PointCloudsPlugin>().getUseSymmetricLidar3dAssociations() && sensorA == sensorB;

  std::atomic<size_t> numberReducedAssociations(0);
  for (auto& cloudA : cloudBatchesA) {
    for (auto& cloudB : cloudBatchesB){
      const std::string taskName = sensorA.getName() + "(" + cloudA.getId() + ") with " + sensorB.getName() + "(" + cloudB.getId() + ")";

      if (sensorA == sensorB && cloudA.getId() == cloudB.getId())
        continue;

      if (symmetricMatchesRequired && cloudA.getId() <= cloudB.getId())
        continue;

      LOG(INFO) << "Subsampling " << taskName;

      auto& matchingPairsAtoB = cloudA.getMatchesTo(cloudB).matchingPair2DataMap;
      CloudBatch::MatchingPair2DataMap * matchingPairsBtoAPtr = nullptr;

      if(symmetricMatchesRequired){
        matchingPairsBtoAPtr = &cloudB.getMatchesTo(cloudA).matchingPair2DataMap;
        CloudBatch::MatchingPair2DataMap& matchingPairsBtoA = *matchingPairsBtoAPtr;

        if(matchingPairsAtoB.empty()){
          matchingPairsBtoA.clear();
          continue;
        }
        if(matchingPairsBtoA.empty()){
          matchingPairsAtoB.clear();
          continue;
        }
      }

      const size_t before = matchingPairsAtoB.size();
      CloudBatch::MatchingPair2DataMap oldAtoB, oldBtoA;

      std::swap(matchingPairsAtoB, oldAtoB);
      assert(matchingPairsAtoB.size() == 0);
      assert(oldAtoB.size() == before);

      if(symmetricMatchesRequired){
        CloudBatch::MatchingPair2DataMap& matchingPairsBtoA = *matchingPairsBtoAPtr;
        std::swap(matchingPairsBtoA, oldBtoA);
        assert(matchingPairsBtoA.size() == 0);
      }

      for(auto& pair : oldAtoB)
      {
        const double r = (double)std::rand()/(double)RAND_MAX;
        if (r < prob)
        {
          matchingPairsAtoB.emplace(pair);

          if(symmetricMatchesRequired){
            CloudBatch::MatchingPair2DataMap& matchingPairsBtoA = *matchingPairsBtoAPtr;
            auto it = oldBtoA.find(std::make_pair(pair.first.second, pair.first.first));
            if(it != oldBtoA.end()){
              matchingPairsBtoA.emplace(*it);
            }
          }
        }
      }

      const size_t after = matchingPairsAtoB.size();
      LOG(INFO) << "Subsampled " << taskName << " from " << before << " to " << after;
      numberReducedAssociations += (symmetricMatchesRequired ? 2 : 1) * after;
    }
  }

  return numberReducedAssociations;
}

size_t CloudsContainer::countAssociations(const PointCloudSensor& from, const PointCloudSensor& to) const {
  size_t n = 0;
  for(const CloudBatch& c : from.getClouds(pcp_.getCalibrator().getCurrentStorage())){
    for(const auto& m : c.getSensorCloud2AssociationsMap()){
      if(m.first.get().getSensor() == to){
        n+=m.second.size();
      }
    }
  }
  return n;
}

CloudBatches::CloudBatches(const PointCloudSensor& sensor, CalibratorI & calib)
    : sensor_(sensor),
      pcp_(calib.getPlugin<PointCloudsPlugin>())
{
  pointCloudPolicy_ = pcp_.getPointCloudPolicy(sensor);
}

template <typename TimestampVector>
void CloudBatches::applyCloudDataFunctor(std::function<int(CloudBatch& toCloud)> addDataFuctor, const TimestampVector& timestamps) {
  if(!getSensor().isUsed()){
    LOG(ERROR) << "Got scan input for " << getSensor().getName() <<  " in spite of it being disabled.";
    return;
  }
  if(timestamps.size() == 0){
    LOG(WARNING) << "Got empty cloud from " << getSensor().getName();
    return;
  }

  pcp_.getCalibrator().addMeasurementTimestamp(timestamps[0], getSensor());

  if(!pcp_.getCalibrator().isMeasurementRelevant(getSensor(), timestamps[0])){ //TODO B support dropping only beginning of cloud
    VLOG(1) << "Skipping irrelevant scan data from " << pcp_.getCalibrator().secsSinceStart(Interval{timestamps[0], timestamps[timestamps.size() - 1]});
    return;
  }

  getPointCloudPolicy().prepareForNewData(pcp_, *this, timestamps[0], getSensor());

  if(isAcceptingData()){
    const int i = addDataFuctor(getCurrentCloud());
    if(i > 0) {
      pcp_.getCalibrator().addMeasurementTimestamp(timestamps[0], getSensor());
      pcp_.getCalibrator().addMeasurementTimestamp(timestamps[i - 1], getSensor());
    }

    if(i < static_cast<int>(timestamps.size())){
      VLOG(1) << "Dropping " << (timestamps.size() - i) << " points from " << getSensor().getName();
    }
  }
}
// request instantiation
template void CloudBatches::applyCloudDataFunctor(std::function<int(CloudBatch& toCloud)> addDataFuctor, const Eigen::Matrix<Timestamp, Eigen::Dynamic, 1>& timestamps);
template void CloudBatches::applyCloudDataFunctor(std::function<int(CloudBatch& toCloud)> addDataFuctor, const std::vector<Timestamp>& timestamps);


void CloudBatches::dropCurrentCloud() {
  if(!empty()){
    SM_ASSERT_FALSE(std::runtime_error, back().isClosed(), "Asked to drop a closed cloud!");
    LOG(INFO) << "Dropping unfinished cloud " << back() << ".";
    pop_back();
  }
}

void CloudBatches::finishCurrentCloud(const PointCloudsPlugin& pcp) {
  if(!empty()){
    if(back().isEmpty()){
      dropCurrentCloud();
    }
    if(!back().isClosed()){
      back().close();
      auto& calib = pcp.getCalibrator();
      LOG(INFO) << "New full cloud : " << getSensor().getName() << "(" << size() - 1 << ")[" << calib.secsSinceStart(back().getMinTimestamp()) << ", " << calib.secsSinceStart(back().getMaxTimestamp()) << "](#=" << back().getMeasurements().getSize() << ")";
    }
  }
}

CloudBatch& CloudBatches::createNewCloud(const PointCloudsPlugin& pcp, const PointCloudSensor& sensor) {
  if(isAcceptingData()){
    finishCurrentCloud(pcp);
  }
  for(auto& c : *this){
    if(!c.isClosed()){
      assert(c.isClosed());
    }
  }
  emplace_back(sensor, CloudId(size()));
  return back();
}

CloudBatch& CloudBatches::getCurrentCloud() {
  SM_ASSERT_TRUE(std::runtime_error, isAcceptingData(), "Asked for the current cloud while not accepting new data!");
  return back();
}
const CloudBatch& CloudBatches::getCurrentCloud() const {
  SM_ASSERT_TRUE(std::runtime_error, isAcceptingData(), "Asked for the current cloud while not accepting new data!");
  return back();
}

bool CloudBatches::isAcceptingData() const {
  return !empty() && !back().isClosed();
}

bool CloudBatches::hasData() const {
  return !empty() && !front().isEmpty();
}

const CloudBatches& CloudsContainer::getCloudsFor(const PointCloudSensor& s) const {
  return s.getClouds(pcp_.getCalibrator().getCurrentStorage());
}

CloudBatches& CloudsContainer::getCloudsFor(const PointCloudSensor& s) {
  return s.getClouds(pcp_.getCalibrator().getCurrentStorage());
}

const aslam::calibration::PointCloudPolicy& CloudBatches::getPointCloudPolicy() const {
  return *pointCloudPolicy_;
}

}
}
