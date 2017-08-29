#include "aslam/calibration/tools/CheckNotNull.h"

#include <glog/logging.h>

namespace aslam {
namespace calibration {

void checkNotNull(void *v){
  CHECK(v);
}

} /* namespace calibration */
} /* namespace aslam */
