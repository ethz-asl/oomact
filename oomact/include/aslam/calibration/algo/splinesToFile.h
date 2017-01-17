#ifndef ASLAM_CALIBRATION_SPLINES_TO_FILE_H
#define ASLAM_CALIBRATION_SPLINES_TO_FILE_H

#include <string>

namespace aslam {
  namespace calibration {

    class CalibratorI;

    void writeSplines(const CalibratorI& cal, double dt, const std::string & pathPrefix, bool theCurrentSplines);

  }
}

#endif // ASLAM_CALIBRATION_SPLINES_TO_FILE_H
