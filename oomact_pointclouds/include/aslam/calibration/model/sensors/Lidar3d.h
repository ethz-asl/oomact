#ifndef LIDAR3D_HPP_
#define LIDAR3D_HPP_

#include <aslam/calibration/tools/Covariance.h>
#include <aslam/calibration/model/sensors/Lidar.h>

namespace aslam {
namespace calibration {

class Lidar3d : public Lidar {
 public:
  Lidar3d(Model& model, const std::string& name, sm::value_store::ValueStoreRef config);

  virtual ~Lidar3d();

  const Covariance& getNormalsCovariance() const {
    return normalsCovariance_;
  }

 protected:
  void writeConfig(std::ostream& out) const override;
 private:
  Covariance normalsCovariance_;
};

} /* namespace calibration */
} /* namespace aslam */

#endif /* LIDAR3D_HPP_ */
