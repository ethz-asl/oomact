#ifndef HEEC34BD0_6221_49B3_A5D7_00810E96CF8A
#define HEEC34BD0_6221_49B3_A5D7_00810E96CF8A

#include <aslam/calibration/model/sensors/PointCloudSensor.h>

namespace aslam {
namespace calibration {

class Lidar : public PointCloudSensor {
 public:
  Lidar(Model& model, const std::string& name, sm::value_store::ValueStoreRef config);

  Eigen::Matrix3d covPoint(bool useSurfaceNormal, const Eigen::Vector3d& pInSensorFrame, const Eigen::Vector3d& nInSensorFrame) const override;

  virtual ~Lidar() {}

  double vertAngleVariance;
  double horizAngleVariance;
  double rangeVariance;
  double beamDiverAngle;
  double noiseGainX;
  double noiseGainY;
  double noiseGainZ;

  double minimalDistance;
  double maximalDistance;
 protected:
  void writeConfig(std::ostream& out) const override;

  ValueStoreRef lidarConfig;
};

} /* namespace calibration */
} /* namespace aslam */

#endif /* HEEC34BD0_6221_49B3_A5D7_00810E96CF8A */
