#include "aslam/calibration/tools/tools.h"

#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/make_shared.hpp>
#include <boost/algorithm/string.hpp>
#include <Eigen/Core>
#include <glog/logging.h>

#include <sm/kinematics/Transformation.hpp>
#include <aslam/backend/MEstimatorPolicies.hpp>

namespace aslam {
namespace calibration {

void writeToFile(const std::string & fileName, std::function<void(std::ostream &o)> writer){
  std::ofstream file(fileName);
  writer(file);
  file.close();
}

void writeStringToFile(const std::string & fileName, const std::string & content){
  writeToFile(fileName, [&](std::ostream & o){ o << content; });
}

void createDirs(const std::string & path, bool ignoreErrors){
  try{
    boost::filesystem::create_directories(boost::filesystem::path(path).parent_path());
  }catch(boost::filesystem::filesystem_error & e){
    if(!ignoreErrors){
      LOG(ERROR) << "error when creating all directories for path='" << path << "' :" << e.what();
      throw e;
    }
  }
}

std::vector<std::string> splitString(const std::string & s, const std::string & byAnyOf) {
  std::vector<std::string> parts;
  boost::split(parts, s, boost::is_any_of(byAnyOf));
  return parts;
}

void openStream(std::ofstream & outputFile, std::string path) {
  path += ".dat";
  createDirs(path);
  outputFile.open(path, std::ofstream::out | std::ofstream::trunc);
  if(!outputFile.is_open())
    throw std::runtime_error(std::string("could not open ") + path);
  VLOG(1) << "Writing data to " << path << ".";
  outputFile << std::fixed << std::setprecision(18);
}



backend::MEstimator::Ptr getMestimator(const std::string & name, const sm::value_store::ValueStoreRef config, int dim) {
  std::string mEstimatorName = boost::algorithm::to_lower_copy(config.getString("name", std::string("none")).get());
  if (mEstimatorName.empty() || mEstimatorName == "none"){
    LOG(INFO) << "Using no M-estimator for " << name << ".";
    return boost::make_shared<backend::NoMEstimator>();
  }
  else if (mEstimatorName == "cauchy"){
    double cauchySigma2 = config.getDouble("cauchySigma2", 10);
    LOG(INFO) << "Using Cauchy M-estimator(sigma^2 = " << cauchySigma2 << ") for " << name << ".";
    return boost::make_shared<backend::CauchyMEstimator>(cauchySigma2);
  }
  else if (mEstimatorName == "blakezisserman"){
    double pCut = config.getDouble("bzPCut", 0.99);
    CHECK(dim > 0);
    LOG(INFO) << "Using Blake Zisserman M-estimator(cut = " << pCut << ", dim = " << dim << " ) for " << name << ".";
    return boost::make_shared<backend::BlakeZissermanMEstimator>(dim, pCut);
  }
  else
    throw std::runtime_error("unknown M-estimator " + mEstimatorName);
}

std::string poseToString(const Eigen::Vector3d & trans, const Eigen::Vector4d & rotQuat){
  std::stringstream ss;
  ss << "P(t= " << trans.transpose() << " , r=" << rotQuat.transpose() << ")";
  return ss.str();
}

std::string poseToString(const sm::kinematics::Transformation & trafo){
  return poseToString(trafo.t(), trafo.q());
}

}
}
