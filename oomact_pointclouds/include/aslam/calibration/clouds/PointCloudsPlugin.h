#ifndef HAAC551F8_790E_45CF_8D33_2EFD64AF92F8
#define HAAC551F8_790E_45CF_8D33_2EFD64AF92F8

#include <functional>
#include <memory>
#include <vector>

#include <aslam/calibration/calibrator/CalibratorPlugin.h>
#include <aslam/calibration/CalibrationPhaseClient.h>
#include <aslam/calibration/Timestamp.hpp>
#include <Eigen/Dense>
#include <sm/value_store/ValueStore.hpp>

namespace sm {
namespace kinematics {
  class Transformation;
}
}
namespace aslam {
namespace backend {
class ErrorTermReceiver;
}
namespace calibration {
class CalibrationMatcher;
class CloudsContainer;
class CalibrationConfI;
class CloudBatch;
class Frame;
class Model;
class PointCloudPolicy;
class PointCloudSensor;
class Sensor;

//TODO C hide these internals;
struct CloudMatcher;
class MeasurementTransformer;

class PointCloudsPlugin : public CalibratorPlugin, public CalibrationPhaseClientI {
 public:
  typedef std::shared_ptr<CloudsContainer> CloudsContainerSP;

  PointCloudsPlugin(CalibratorI& calib, sm::ValueStoreRef config);
  virtual ~PointCloudsPlugin();

  const CloudsContainer& getCloudsContainer() const {
    return *cloudsContainer_;
  }
  CloudsContainer& getCloudsContainer() {
    return *cloudsContainer_;
  }

  const CalibrationMatcher& getCloudMatcher(const Sensor& sensorRegister, const Sensor& sensorReference) const;

  std::shared_ptr<const PointCloudPolicy> getDefaultPointCloudPolicy() const;
  std::shared_ptr<const PointCloudPolicy> getPointCloudPolicy(const PointCloudSensor& sensor) const;

  const Frame& getMapFrame() const;

  bool getUseSymmetricLidar3dAssociations();

 private:
//  friend CloudsContainer;
  void addCloudAssociationsErrorTerms(const PointCloudSensor& sensorRef, const PointCloudSensor& sensorRead, backend::ErrorTermReceiver& errorTermReceiver, const CalibrationConfI& ec);
  void addCloudAssociationsErrorTerms(const Sensor& referenceSensor, const std::function<bool(const Sensor &)> sensorFilter, backend::ErrorTermReceiver& errorTermReceiver, const CalibrationConfI& ec);
  void buildCloudProblem(CloudsContainerSP& batches, const CalibrationConfI & config);
  void clearCloudsAssociations();
  void closeAllCurrentClouds();
  void closeCurrentCloud(const PointCloudSensor& pcs);
  void postprocessAssociations();
  void preProcessNewWindow(CalibratorI &) override;
  void updateCloudUsingCurrentTrajectory(CloudBatch& cloud, const sm::kinematics::Transformation& T_r_laser, const Sensor& sensor);
  void writeSnapshot(const CalibrationConfI& config, const std::string& path, bool updateFirstBasedOnCurrentSplines);

  bool useSymmetricLidar3dAssociations_, usePointToPlainErrorTerm_;
  int maxNumCloudAssociations_;
  const Frame& mapFrame_;

  int cloudVersionCounter_ = 0;

  /// CloudsContainer
  CloudsContainerSP cloudsContainer_;

  std::vector<std::shared_ptr<CloudMatcher> > cloudMatchers_;
  std::shared_ptr<PointCloudPolicy> defaultPointCloudPolicy_;
  std::vector<std::shared_ptr<PointCloudPolicy> > pointCloudPolicies_;
};

Eigen::Vector3f transform(const MeasurementTransformer& transformer, Timestamp timestamp, const Eigen::Vector3f& p);
Eigen::Vector3d transform(const MeasurementTransformer& transformer, Timestamp timestamp, const Eigen::Vector3d& p);

} /* namespace calibration */
} /* namespace aslam */

#endif /* HAAC551F8_790E_45CF_8D33_2EFD64AF92F8 */
