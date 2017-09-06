#include <aslam/calibration/clouds/PmFilter.h>

#include <fstream>

#include <glog/logging.h>

#include "aslam/calibration/clouds/CloudBatch.h"

namespace aslam {
namespace calibration {

PmFilter::PmFilter(const std::string& filterConfigFile) {
  if(!filterConfigFile.empty()){
    std::ifstream ifsF(filterConfigFile.c_str());
    if (!ifsF.good())
    {
      throw std::runtime_error(std::string("Cannot open filter config file ") + filterConfigFile);
    }
    else {
      LOG(INFO) << "Loading filter chain from configuration file '" << filterConfigFile<< "'";
      pmFilters = std::make_shared<PM::DataPointsFilters>(ifsF);
    }
  }
}

void PmFilter::filter(DP& data) const {
  if(pmFilters && !pmFilters->empty()) {
    pmFilters->apply(data);
  }
}

void PmFilter::filter(CloudBatch& cloud) const {
   cloud.applyFilter([this](DP& data){filter(data);});
}

} /* namespace calibration */
} /* namespace aslam */
