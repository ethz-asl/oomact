#ifndef H22B0B098_C827_4C3A_ACE2_10734530AF87
#define H22B0B098_C827_4C3A_ACE2_10734530AF87

#include <aslam/calibration/model/Module.h>
#include <aslam/calibration/model/StateCarrier.h>
#include <aslam/calibration/model/fragments/So3R3TrajectoryCarrier.h>

namespace aslam {
namespace calibration {

class BaseTrajectoryBatchState;
class So3R3Trajectory;
class PoseSensorI;
class WheelOdometry;

class PoseTrajectory : public Module, public StateCarrier, public Activatable, public So3R3TrajectoryCarrier {
 public:
  PoseTrajectory(Model & model, const std::string & name, sm::value_store::ValueStoreRef config = sm::value_store::ValueStoreRef());

  bool initState(CalibratorI & calib) override;
  void addToBatch(const Activator & stateActivator, BatchStateReceiver & batchStateReceiver, DesignVariableReceiver & problem) override;
  void addErrorTerms(CalibratorI & calib, const EstConf & ec, ErrorTermReceiver & problem) const override;


  const So3R3Trajectory & getCurrentTrajectory() const;
  So3R3Trajectory & getCurrentTrajectory();

  virtual ~PoseTrajectory();

  bool isUseTangentialConstraint() const {
    return useTanConstraint;
  }

  bool isAssumeStatic() const {
    return assumeStatic;
  }

  const Frame & getReferenceFrame() const { return referenceFrame_; }
 protected:
  void writeConfig(std::ostream & out) const override;
 private:
  std::shared_ptr<BaseTrajectoryBatchState> state_;
  bool estimate = true;
  bool useTanConstraint;
  double tanConstraintVariance;
  bool initWithPoseMeasurements;
  ModuleLink<PoseSensorI> poseSensor;
  ModuleLink<WheelOdometry> odometrySensor;
  bool assumeStatic;
  const Frame & referenceFrame_;
};

} /* namespace calibration */
} /* namespace aslam */

#endif /* H22B0B098_C827_4C3A_ACE2_10734530AF87 */
