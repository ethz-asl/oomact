#include "aslam/calibration/clouds/CalibrationMatcher.h"

#include <cmath>
#include <fstream>
#include <fstream>
#include <iostream>
#include <sstream>
#include <unordered_set>
#include <utility>
#include <vector>

#include <boost/algorithm/string.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/make_shared.hpp>
#include <glog/logging.h>
#include <pointmatcher/IO.h>
#include <pointmatcher/PointMatcher.h>
#include <tbb/concurrent_queue.h>

// TODO: Read configuration for ICP, filters and map dimension as a Yaml file or a sm::ConstPropertyTree
//  (Is it better to create an Option class? The options are few)
using namespace PointMatcherSupport;

using namespace sm::kinematics;
using namespace std;

namespace aslam {
namespace calibration {

template <typename T>
class ObjectPool
{
public:
  typedef std::shared_ptr<T> Ptr;
  ObjectPool()
  : pool_() {}

  void push(Ptr p){
    pool_.push(p);
  }

  Ptr get()
  {
        Ptr obj;
        if(factory){
          if(!pool_.try_pop(obj)){
            obj = factory();
          }
        } else {
          pool_.pop(obj);
        }
        return Ptr(obj.get(), [this, obj](T*){push(obj);});
  }

  const tbb::concurrent_bounded_queue<Ptr>&getPool() { return pool_; }

  void setFactory(const std::function<Ptr()>& factory) {
    this->factory = factory;
  }

private:
  tbb::concurrent_bounded_queue<Ptr> pool_;
  std::function<Ptr()> factory;
};


struct IICP : PM::ICP {
  bool inspectionEnabled = false;

  bool isInspectionEnabled() const;
  void setInspectionEnabled(bool inspectionEnabled, const std::string& baseFileName);
};


//TODO C Rename CalibrationMatcher to CloudMatcher
CalibrationMatcher::CalibrationMatcher(const std::string& icpConfigName, const std::string& icpConfigData) :
  icps_(make_shared<ObjectPool<IICP>>())
{
  if(icpConfigData.empty()) {
    auto& icpConfigFile = icpConfigName;
    ifstream ifs(icpConfigFile);

    if (ifs.good()){
      std::ostringstream configSStream;
      configSStream << ifs.rdbuf();
      configString_ = configSStream.str();
      LOG(INFO)<< "Loaded ICP configuration from file " << icpConfigFile << ".";
    }else{
      LOG(ERROR)<< "Cannot open ICP configuration file " << icpConfigFile << "! Using default configuration.";
    }
    ifs.close();
  } else {
    configString_ = icpConfigData;
    LOG(INFO)<< "Loaded ICP configuration " << icpConfigName << ".";
  }

  active_ = configString_ != "None";
  if(active_ && configString_.find("None") != std::string::npos){
    LOG(WARNING)<< "Found None in configString but it is considered active '" << configString_ << "'";
  }
  if(!active_) {
    LOG(INFO) << "CloudMatcher " << icpConfigName << " is inactive.";
    return;
  }

  icps_->setFactory([this, icpConfigName](){
    auto icp = make_shared<IICP>();
    if(!configString_.empty()){
      LOG(INFO)<< "Creating ICPs from configuration " << icpConfigName << ".";
      std::istringstream configSStream;
      configSStream.str(configString_);
      icp->loadFromYaml(configSStream);
    } else {
      LOG(INFO)<< "Creating default ICP because configuration " << icpConfigName << " is missing.";
      icp->setDefault();
    }
    return icp;
  });

  rigidTrans_.reset(PM::get().REG(Transformation).create("RigidTransformation"));
}

std::vector<CloudBatch::Index> filterInto(
    const PM::DataPoints& srcPts,
    PM::DataPoints& destPts,
    const std::vector<bool>&acceptBits
    )
{
  const int pointsCount = std::count(acceptBits.cbegin(), acceptBits.cend(), true);

  destPts.featureLabels = srcPts.featureLabels;
  destPts.descriptorLabels = srcPts.descriptorLabels;

  destPts.features.resize(srcPts.features.rows(), pointsCount);
  destPts.descriptors.resize(srcPts.descriptors.rows(), pointsCount);

  std::vector<CloudBatch::Index> originalIndices;
  originalIndices.reserve(pointsCount);
  int j = 0;
  for (int i = 0, inputSize = srcPts.features.cols(); i < inputSize; ++i)
  {
    if(acceptBits[i])
    {
      destPts.features.col(j) = srcPts.features.col(i);
      destPts.descriptors.col(j) << srcPts.descriptors.col(i);
      originalIndices.push_back(i);
      ++j;
    }
  }
  CHECK_EQ(j, pointsCount);

  return originalIndices;
}


CloudBatch::AssociationsAndTransformation CalibrationMatcher::getAssociations(const CloudBatch& readCloud,const CloudBatch& refCloud, const std::string& name, const std::vector<bool> * usePoints) const {
  if(!isActive()){
    LOG(ERROR) << "Inactive CalibrationMatcher was used!";
    return CloudBatch::AssociationsAndTransformation();
  }

  DP newCloud;
  std::vector<int> originalIndices;
  if(usePoints){
    originalIndices = filterInto(readCloud.getCloud(), newCloud, *usePoints);
  }
  else{
    newCloud = readCloud.getCloud();
  }

  CloudBatch::AssociationsAndTransformation associationsAndTrafo;
  auto icp = icps_->get();
  icp->setInspectionEnabled(inspectionEnabled, inspectionOutputFolder + "inspector-" + name);

  try{
    associationsAndTrafo.T_ref_read = (*icp)(newCloud, refCloud.getCloud());
    if(!associationsAndTrafo.T_ref_read.isApprox(TP::Identity(4, 4), 1)){ //TODO A make  isAprox threshold a parameter
      LOG(WARNING) << "Ignoring associations (" << name << ") because transformation is too far off the expected identity : T_m1_m_icp=" << std::endl << associationsAndTrafo.T_ref_read << std::endl;
      return associationsAndTrafo;
    }
  }
  catch (const PM::ConvergenceError& error)
  {
    LOG(WARNING) << "ICP failed to converge when aligning " << name << ": " << error.what() << endl;
    return associationsAndTrafo;
  }

  // It sets up the reference and should return the matches with kdTreeMatcher
  newCloud = rigidTrans_->compute(newCloud,associationsAndTrafo.T_ref_read);

  // Match to closest point in Reference
  icp->matcher->init(refCloud.getCloud());
  const PM::Matches matches = icp->matcher->findClosests(newCloud);

  // Detect outliers
  const PM::OutlierWeights outlierWeights = icp->outlierFilters.compute(newCloud, refCloud.getCloud(), matches);
  LOG(INFO) << "mean(outlierWeights, for " << name << "):" << outlierWeights.mean() << std::endl;

  getMatchedPoints(newCloud, refCloud.getCloud(), matches, outlierWeights, associationsAndTrafo.associations);
  if(usePoints){ // translate first indices back to original values
    for(auto& a : associationsAndTrafo.associations){
      a.matchIndixPair.first = originalIndices.at(a.matchIndixPair.first);
    }
  }
  return associationsAndTrafo;
}

/// TODO:If for now it is ok, it should be better to change the Pointmatcher getMatchedPoints to public,
/// Or  set to publick the lastErrorElements, so we can access these
// TODO optimize: copy less data after returning from this function (maybe output parameters?)
void CalibrationMatcher::getMatchedPoints(
    const PM::DataPoints& requestedPts,
    const PM::DataPoints& sourcePts,
    const PM::Matches& matches,
    const PM::OutlierWeights& outlierWeights,
    CloudBatch::Associations& associations) const
{
  CHECK(matches.ids.rows() > 0);
  CHECK(matches.ids.cols() > 0);
  CHECK_EQ(matches.ids.cols(), requestedPts.features.cols()); //nbpts
  CHECK_EQ(outlierWeights.rows(), matches.ids.rows());  // knn

  const int knn = outlierWeights.rows();
  const int dimFeat = requestedPts.features.rows();
  CHECK_EQ(dimFeat, sourcePts.features.rows());

  // Count points with no weights
  size_t pCount = 0;
  for(int i = 0; i < outlierWeights.cols(); i++)
    for(int j= 0; j < outlierWeights.rows(); j++)
      if(outlierWeights(j,i) != 0.0)
        pCount++;
  const size_t pointsCount = pCount;
  if (pointsCount == 0)
    throw PM::ConvergenceError("ErrorMnimizer: no point to minimize");

  associations.reserve(pointsCount);
  for(int k = 0; k < knn; k++) // knn
  {
    for (int i = 0; i < requestedPts.features.cols(); ++i) //nb pts
    {
      if (outlierWeights(k,i) != 0.0)
      {
        associations.emplace_back(
            CloudBatch::Association{
              {i, matches.ids(k, i)},
              {matches.dists(k, i), outlierWeights(k,i)}
            });
      }
    }
  }
  CHECK_EQ(associations.size(), pointsCount);
}

void CalibrationMatcher::enableIcpInspection(bool active, const std::string& inspectionOutputFolder) {
  this->inspectionEnabled = active;
  this->inspectionOutputFolder = inspectionOutputFolder;
  if(active){
    boost::filesystem::create_directories(boost::filesystem::path(inspectionOutputFolder));
  }
}

bool IICP::isInspectionEnabled() const {
  return inspectionEnabled;
}

void IICP::setInspectionEnabled(bool active, const std::string& baseFileName) {

  Parametrizable::Parameters param;
  if(active){
    PointMatcherSupport::setLogger(PM::get().LoggerRegistrar.create("FileLogger"));
    boost::assign::insert(param) ("baseFileName", baseFileName);
    boost::assign::insert(param) ("dumpDataLinks", "1");
    boost::assign::insert(param) ("dumpReading", "1");
    boost::assign::insert(param) ("dumpReference", "1");
    boost::assign::insert(param) ("writeBinary", "1");

    inspector.reset(PM::get().REG(Inspector).create("VTKFileInspector", param));
  }

  if(inspectionEnabled != active){
    inspectionEnabled = active;
    if(!active){
      inspector.reset(PM::get().REG(Inspector).create("NullInspector", param));
      PointMatcherSupport::setLogger(PM::get().LoggerRegistrar.create("NullLogger"));
    }
  }
}


}
}
