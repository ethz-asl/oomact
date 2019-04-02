#include <aslam/calibration/tools/DeprecationAlerter.h>
#include <glog/logging.h>

namespace aslam {
namespace calibration {

void parameterDeprecationAlerter(std::string path, std::string hint){
  LOG(WARNING) << "Use of deprecated parameter: " << path << "! " << hint;
}

} /* namespace calibration */
} /* namespace aslam */
