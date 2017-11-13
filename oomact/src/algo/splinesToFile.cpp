#include "aslam/calibration/algo/splinesToFile.h"

#include <string>
#include <vector>

#include <glog/logging.h>

#include "aslam/calibration/calibrator/CalibratorI.h"

using namespace sm::timing;
using namespace sm::kinematics;

namespace aslam {
  namespace calibration {
    template <typename Spline>
    void writeSplines(const std::vector<Spline> & splines, double dt, std::ofstream & stream) {
      if(stream.is_open()){
        for (auto & spl : splines) {
          writeSpline(spl, dt, stream);
        }
      }
    }
  }
}
