#ifndef LIDAR2D_HPP_
#define LIDAR2D_HPP_

#include <aslam/calibration/model/sensors/Lidar.h>

namespace aslam {
namespace calibration {

class Lidar2d : public Lidar {
 public:
  Lidar2d(Model& model, std::string name, sm::value_store::ValueStoreRef config);

  double minimalAngle;
  double maximalAngle;
  double angularResolution;

  double measurementTimeIncrement;

  virtual ~Lidar2d();
 protected:
  void writeConfig(std::ostream& out) const override;
};

} /* namespace calibration */
} /* namespace aslam */

#endif /* LIDAR2D_HPP_ */
