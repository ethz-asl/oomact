#ifndef STEREO_CAMERA_HPP_
#define STEREO_CAMERA_HPP_

#include <aslam/calibration/model/sensors/PointCloudSensor.h>

namespace aslam {
namespace calibration {

class StereoCamera : public PointCloudSensor {
 public:
  StereoCamera(Model& model, std::string name, sm::value_store::ValueStoreRef config);

  virtual ~StereoCamera();

  Eigen::Matrix3d covPoint(bool useSurfaceNormal, const Eigen::Vector3d& pInSensorFrame, const Eigen::Vector3d& nInSensorFrame) const override;

  double getBaseLine() const {
    return baseLine;
  }

  double getFocalLength() const {
    return focalLength;
  }

  bool isUseSemiGlobalMatcher() const {
    return useSemiGlobalMatcher;
  }

  double getXyVarianceFactor() const {
    return xyVarianceFactor;
  }

  double getVarianceFactor() const {
    return zVarianceFactor;
  }

  double getMinimalX() const {
    return minimalX;
  }

  int getNumDisparities() const {
    return numDisparities;
  }

  double getMaximalX() const {
    return maximalX;
  }

  double getPreScaleFactor() const {
    return preScaleFactor;
  }

 protected:
  virtual void writeConfig(std::ostream& out) const;
 private:
  bool useSemiGlobalMatcher;
  int numDisparities;
  double minimalX, maximalX;
  double xyVarianceFactor, zVarianceFactor;
  double focalLength;
  double baseLine;
  double preScaleFactor;
};

} /* namespace calibration */
} /* namespace aslam */

#endif /* STEREO_CAMERA_HPP_ */
