#ifndef HD99F880C_876B_4FC1_A6D7_CBC4D83CB182
#define HD99F880C_876B_4FC1_A6D7_CBC4D83CB182

#include <aslam/backend/ScalarExpression.hpp>
#include <aslam/calibration/model/sensors/Lidar3d.hpp>
#include <vector>

using aslam::backend::ScalarExpression;

namespace ethz {
namespace velodyne {
class Velodyne32Calibration;
}
}

namespace aslam {
namespace calibration {

class Velodyne : public Lidar3d {
 public:
  Velodyne(Model& model, std::string name, sm::value_store::ValueStoreRef config);
  virtual ~Velodyne();

  void registerWithModel() override;

  size_t findPointsInFieldOfView(const DP& pointCloud, const sm::kinematics::Transformation& T_sensor_pointCloud, std::vector<bool>&goodPoints) const override;

  bool isProvidingCorrection() const override;

  aslam::backend::EuclideanExpression calcMeasurementExpressionByUnfilteredIndex(const CloudBatch& cloud, size_t pointIndex) const override;

  void writeSnapshot(const CalibrationConfI& cc, bool stateWasUpdatedSinceLastTime) const override;

  void estimatesUpdated(CalibratorI& calib) const override;

  const ethz::velodyne::Velodyne32Calibration& getCalibration() const { return *calibration; }

  void addNewPackage(CalibratorI& calibrator, const std::string& data, const Timestamp& t) const;

  float getBeamDistantanceCorrection(int laserIndex) const;
  ScalarExpression getBeamDistantanceCorrectionExpression(int laserIndex) const;
  float getBeamVerticalAngleCorrection(int laserIndex) const;
  ScalarExpression getBeamVerticalAngleCorrectionExpression(int laserIndex) const;

  virtual std::unique_ptr<CloudMeasurements> createCloudMeasurements(CloudBatch& cloudBatch) const;
 protected:
  void writeConfig(std::ostream& out) const override;
  void setActive(bool spatial, bool temporal) override;
 private:
  bool doIntrinsicCalibration;
  bool doBeamAngleCalibration;

  double filterMinVerticalAngleDeg;
  double filterMaxVerticalAngleDeg;

  std::shared_ptr<ethz::velodyne::Velodyne32Calibration> calibration;
  std::vector<ScalarCvSp> beamDistanceOffsets;
  std::vector<ScalarExpression> beamDistanceOffsetExpressions;
  std::vector<ScalarCvSp> beamVerticalAngle;
  std::vector<ScalarExpression> beamVerticalAngleExpressions;
};

} /* namespace calibration */
} /* namespace aslam */

#endif /* HD99F880C_876B_4FC1_A6D7_CBC4D83CB182 */
