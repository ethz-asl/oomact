#ifndef HCB4890AB_0CD7_40A3_9243_CE0E2A54F8FC
#define HCB4890AB_0CD7_40A3_9243_CE0E2A54F8FC

#include <boost/shared_ptr.hpp>

#include <aslam/backend/FixedPointNumber.hpp>
#include <aslam/backend/GenericScalarExpression.hpp>
#include <aslam/backend/GenericScalar.hpp>
#include <sm/value_store/ValueStore.hpp>

#include <aslam/calibration/model/BoundedCalibrationVariable.h>
#include <aslam/calibration/model/Module.h>
#include <aslam/calibration/Timestamp.h>

namespace aslam {
namespace calibration {

/// Time design variable
typedef aslam::backend::GenericScalar<Timestamp> TimeDesignVariable;
/// Time design variable
typedef aslam::backend::GenericScalarExpression<Timestamp> TimeExpression;
/// Time calibration design variable
typedef BoundedCalibrationVariable<TimeDesignVariable, Timestamp> TimeDesignVariableCv;
/// Shared pointer to TimeDesignVariable
typedef boost::shared_ptr<TimeDesignVariableCv> TimeDesignVariableSp;

class DelayCv {
 public:
  DelayCv(const Module * module);

  bool hasDelay() const { return bool(dt_r_s); }
  Timestamp getDelay() const { return hasDelay() ? dt_r_s->toScalar() : Timestamp::Zero(); }
  Timestamp getDelayLowerBound() const { return hasDelay() ? dt_r_s->getLowerBound() : Timestamp::Zero(); }
  Timestamp getDelayUpperBound() const { return hasDelay() ? dt_r_s->getUpperBound() : Timestamp::Zero(); }

  const TimeExpression & getDelayExpression() const { return delayExp; }
  const TimeDesignVariableCv& getDelayVariable() const {
    assert(dt_r_s);
    return *dt_r_s;
  }
  TimeDesignVariableCv& getDelayVariable() {
    assert(dt_r_s);
    return *dt_r_s;
  }

  TimeDesignVariableSp getDelayVariablePtr() {
    return dt_r_s;
  }

  virtual ~DelayCv() = default;
 protected:
  void setActive(bool active) {
    if(dt_r_s) dt_r_s->setActive(active && dt_r_s->isToBeEstimated());
  }

 private:
  TimeDesignVariableSp dt_r_s;
  TimeExpression delayExp;
};

} /* namespace calibration */
} /* namespace aslam */

#endif /* HCB4890AB_0CD7_40A3_9243_CE0E2A54F8FC */
