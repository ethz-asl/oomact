#ifndef HD99F880C_876B_4FC1_A6D7_CBC4D83CB183
#define HD99F880C_876B_4FC1_A6D7_CBC4D83CB183

#include <aslam/calibration/model/sensors/Lidar3d.h>
#include <aslam/calibration/model/PoseTrajectory.h>

namespace aslam {
namespace calibration {
class CalibratorI;
class CalibrationConfI;
class ErrorTermReceiver;

class GroundPlanePseudoSensor : public Lidar3d {
 public:
  GroundPlanePseudoSensor(Model& model, std::string name, sm::value_store::ValueStoreRef config);
  virtual ~GroundPlanePseudoSensor();

  size_t findPointsInFieldOfView(const DP& pointCloud, const sm::kinematics::Transformation& T_sensor_pointCloud, std::vector<bool>&goodPoints) const override;

  void preProcessNewWindow(CalibratorI& calib) override;
  void addMeasurementErrorTerms(CalibratorI& calib, const CalibrationConfI& cc, ErrorTermReceiver& problem, bool observeOnly) const override;

  std::shared_ptr<const PointCloudPolicy> getDefaultPointCloudPolicy(const PointCloudsPlugin & pcp) const override;
 protected:
  void writeConfig(std::ostream& out) const override;
 private:
  double cutOffDistance_;
  double hightSigma_, attitudeSigma_;
  double gridDelta;
  int gridNx, gridNy;

  const Frame& baseFrame_;
  ModuleLink<PoseTrajectory> baseTrajectory_;
};

} /* namespace calibration */
} /* namespace aslam */

#endif /* HD99F880C_876B_4FC1_A6D7_CBC4D83CB183 */
