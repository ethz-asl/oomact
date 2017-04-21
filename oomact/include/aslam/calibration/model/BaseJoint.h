#ifndef HB260A70E_C889_4F10_B2E0_F43E806E3C5D
#define HB260A70E_C889_4F10_B2E0_F43E806E3C5D

#ifndef CALIBRATOR_HIGH_LEVEL
#include <bsplines/NsecTimePolicy.hpp>
#include <bsplines/UnitQuaternionBSpline.hpp>
#include <aslam/splines/OPTBSpline.hpp>
#include <aslam/splines/OPTUnitQuaternionBSpline.hpp>
#endif

#include <aslam/calibration/model/Module.h>
#include <aslam/calibration/model/StateCarrier.h>
#include <aslam/calibration/model/fragments/TrajectoryCarrier.h>
#include <aslam/calibration/model/Sensor.hpp>

namespace aslam {
namespace calibration {

class Model;

class BaseJoint : public Module, public StateCarrier, public ObserverMinimal, public CalibratableMinimal, public Activatable, public TrajectoryCarrier {
 public:
#ifdef CALIBRATOR_HIGH_LEVEL
  class BaseJointBatchState;
#else
  typedef aslam::splines::OPTBSpline<typename bsplines::UnitQuaternionBSpline<Eigen::Dynamic, bsplines::NsecTimePolicy>::CONF>::BSpline BaseJointSpline;
  class BaseJointBatchState : public BatchState {
   public:
    BaseJointBatchState(BaseJoint & baseJoint);

    void writeToFile(const CalibratorI & calib, const std::string & pathPrefix) const override;

    BaseJointSpline baseJointSpline;
  };

  BaseJointSpline& getBaseJointSpline() {
    return state_->baseJointSpline;
  }

  const BaseJointSpline& getBaseJointSpline() const {
    return state_->baseJointSpline;
  }
#endif
  BaseJoint(Model & model, const std::string & name, sm::value_store::ValueStoreRef config);


  aslam::backend::EuclideanExpression getTranslationExpression() const;

  bool initState(CalibratorI & calib) override;
  void addToBatch(const Activator & stateActivator, BatchStateReceiver & batchStateReceiver, DesignVariableReceiver & problem) override;

  void setActive(bool active);

  void addErrorTerms(CalibratorI & calib, const EstConf & ec, ErrorTermReceiver & problem) const override;

  virtual ~BaseJoint();

 protected:
  void writeConfig(std::ostream& out) const override;
  void setActive(bool spatial, bool temporal) override;
  void registerWithModel() override;

 private:
  const bool estimatePosition = false;

  ScalarCvSp baseJointX;
  ScalarCvSp baseJointZ;

  ScalarCvSp dampingLambda;
  ScalarCvSp dampingOmega0Squared;

  double randomWalk;
  double couplingFactor;

  std::shared_ptr<BaseJointBatchState> state_;
  friend struct OscillatorIntegrationErrorExpressionFactory;
};

} /* namespace calibration */
} /* namespace aslam */

#endif /* HB260A70E_C889_4F10_B2E0_F43E806E3C5D */
