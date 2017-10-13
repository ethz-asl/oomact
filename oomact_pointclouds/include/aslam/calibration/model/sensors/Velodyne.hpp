#ifndef HD99F880C_876B_4FC1_A6D7_CBC4D83CB182
#define HD99F880C_876B_4FC1_A6D7_CBC4D83CB182

#include <array>
#include <vector>
#include <functional>

#include <aslam/backend/ScalarExpression.hpp>
#include <aslam/calibration/input/InputReceiverI.h>
#include <aslam/calibration/model/sensors/Lidar3d.hpp>

namespace ethz {
namespace velodyne {
class Velodyne32Calibration;
}
}

namespace aslam {
namespace calibration {

struct VelodynePackageRef {
  const char * data;
  size_t length;
};

struct VelodynePointsFunctor {
  std::function<void(Eigen::MatrixXf &)> fillCloud;
  size_t length;
};

class Velodyne : public Lidar3d, public InputReceiverIT<VelodynePackageRef>, public InputReceiverIT<VelodynePointsFunctor> {
 public:
  Velodyne(Model& model, std::string name, sm::value_store::ValueStoreRef config = sm::value_store::ValueStoreRef());
  virtual ~Velodyne();

  void registerWithModel() override;

  size_t findPointsInFieldOfView(const DP& pointCloud, const sm::kinematics::Transformation& T_sensor_pointCloud, std::vector<bool>&goodPoints) const override;

  bool isProvidingCorrection() const override;

  aslam::backend::EuclideanExpression calcMeasurementExpressionByUnfilteredIndex(const CloudBatch& cloud, size_t pointIndex) const override;

  void writeSnapshot(const CalibrationConfI& cc, bool stateWasUpdatedSinceLastTime) const override;

  void estimatesUpdated(CalibratorI& calib) const override;

  const ethz::velodyne::Velodyne32Calibration& getCalibration() const { return *calibration; }

  void addNewPackage(const Timestamp& t, const std::string& data, ModuleStorage& storage) const;

  void addInputTo(Timestamp t, const VelodynePackageRef & input, ModuleStorage & s) const override;
  void addInputTo(Timestamp t, const VelodynePointsFunctor & input, ModuleStorage & s) const override;

  float getBeamDistantanceCorrection(int laserIndex) const;
  aslam::backend::ScalarExpression getBeamDistantanceCorrectionExpression(int laserIndex) const;
  float getBeamVerticalAngleCorrection(int laserIndex) const;
  aslam::backend::ScalarExpression getBeamVerticalAngleCorrectionExpression(int laserIndex) const;

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
  std::vector<aslam::backend::ScalarExpression> beamDistanceOffsetExpressions;
  std::vector<ScalarCvSp> beamVerticalAngle;
  std::vector<aslam::backend::ScalarExpression> beamVerticalAngleExpressions;
};

} /* namespace calibration */
} /* namespace aslam */

#endif /* HD99F880C_876B_4FC1_A6D7_CBC4D83CB182 */
