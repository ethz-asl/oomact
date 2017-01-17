#ifndef H023C6573_77C9_4AB4_8E41_2607C128B3F5
#define H023C6573_77C9_4AB4_8E41_2607C128B3F5

#include <memory>
#include <string>

#ifndef CALIBRATOR_HIGH_LEVEL
#include <bsplines/NsecTimePolicy.hpp>
#include <bsplines/EuclideanBSpline.hpp>
#include <aslam/splines/OPTBSpline.hpp>
#endif

#include <aslam/calibration/model/Module.h>
#include <aslam/calibration/model/Joint.h>
#include <aslam/calibration/model/fragments/PoseCv.h>
#include <aslam/calibration/model/fragments/TrajectoryCarrier.h>
#include <aslam/calibration/data/MeasurementsContainer.h>

namespace aslam {
namespace calibration {
class EuropaModel;
class CalibratorI;

class NoseJoint : public Module, public PoseCv, public Joint, public Activatable, public TrajectoryCarrier, public CalibratableMinimal {
 public:
#ifdef CALIBRATOR_HIGH_LEVEL
  class NoseJointBatchState;
#else
  typedef typename aslam::splines::OPTBSpline<typename bsplines::EuclideanBSpline<Eigen::Dynamic, 1, bsplines::NsecTimePolicy>::CONF>::BSpline NoseJointSpline;

  class NoseJointBatchState : public BatchState {
   public:
    NoseJointBatchState();
    void writeToFile(const CalibratorI & calib, const std::string & pathPrefix) const override;
   public:
    NoseJointSpline tiltAngle;
    friend class NoseJoint;
  };
#endif

  NoseJoint(EuropaModel & model, const std::string & name, sm::value_store::ValueStoreRef config);
  void registerWithModel() override;

  void clearMeasurements() override;
  bool initState(CalibratorI & calib) override;
  void addToBatch(const Activator & stateActivator, BatchStateReceiver & batchStateReceiver, DesignVariableReceiver & problem) override;

  void addTiltMeasurement(Timestamp t, double tiltAngle) const;

  virtual ~NoseJoint();

  bool isAvailable() const {
    return isUsed() && state_;
  }

  const NoseJointBatchState& getState() const {
    assert(state_);
    return *state_;
  }

 protected:
  void setActive(bool spatial, bool temporal) override;
  void writeConfig(std::ostream& out) const override;
 private:
  void addMeasurementErrorTerms(CalibratorI & calib, const EstConf & ec, ErrorTermReceiver & problem, bool observeOnly) const override;

  TimeDesignVariableSp dt_r_s;

  std::shared_ptr<NoseJointBatchState> state_;

  mutable MeasurementsContainer<double> tiltAngleMeasurements;
  double angleSigma;
};

} /* namespace calibration */
} /* namespace aslam */

#endif /* H023C6573_77C9_4AB4_8E41_2607C128B3F5 */
