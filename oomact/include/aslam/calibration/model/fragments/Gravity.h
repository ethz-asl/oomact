#ifndef H5D8032BB_3274_46C6_B264_B4CECCC53D72
#define H5D8032BB_3274_46C6_B264_B4CECCC53D72
#include <aslam/backend/EuclideanExpression.hpp>
#include <aslam/calibration/calibrator/CalibrationConfI.h>
#include <aslam/calibration/model/Module.h>

namespace aslam {
namespace calibration {

class Gravity : public Activatable {
 public:
  virtual ~Gravity() = default;
  virtual aslam::backend::EuclideanExpression & getVectorExpression() = 0;
};

} /* namespace calibration */
} /* namespace aslam */

#endif /* H5D8032BB_3274_46C6_B264_B4CECCC53D72 */
