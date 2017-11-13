#ifndef INCLUDE_ASLAM_CALIBRATION_BOUNDEDCALIBRATIONVARIABLE_HPP_
#define INCLUDE_ASLAM_CALIBRATION_BOUNDEDCALIBRATIONVARIABLE_HPP_

#include <aslam/backend/GenericScalarExpression.hpp>
#include <aslam/backend/Scalar.hpp>
#include <glog/logging.h>
#include "CalibrationVariable.h"

namespace aslam {
namespace calibration {

enum class BoundsEnforceEvent {
  NOP,
  CLAMPED_TO_LOWER,
  CLAMPED_TO_UPPER
};
namespace internal {

template <typename BoundedCalibrationVariable> 
struct BoundsEnforcer;

}

template <typename DesignVariable_, typename Bound = double>
class BoundedCalibrationVariable : public CalibrationDesignVariable<DesignVariable_> {
 public:
  BoundedCalibrationVariable(const std::string & name, ValueStoreRef valueStore, std::function<void(BoundedCalibrationVariable &, BoundsEnforceEvent event)> boundsEventHandler = std::function<void(BoundedCalibrationVariable &, BoundsEnforceEvent event)>()) :
    DesignVariable_(internal::DVLoadTraits<DesignVariable_>::load(valueStore)),
    CalibrationDesignVariable<DesignVariable_>(name, valueStore),
    upperBound_(valueStore.getDouble("upperBound")),
    lowerBound_(valueStore.getDouble("lowerBound")),
    boundEventHandler_(boundsEventHandler)
  {
    assert(DesignVariable_::minimalDimensions() == 1) ; //TODO C support multi dim calibration variables with bounds
    auto v = this->getParams()(0, 0);

    if(valueStore.getBool("relativeBounds", false)){
      lowerBound_ += v;
      upperBound_ += v;
    }

    LOG(INFO) << name << "(v=" << v << ", lB=" << double(lowerBound_) << ", uB=" << double(upperBound_) << ")";
    if(!isWithinBounds()){
      throw std::runtime_error("Bounded variable " + name + " is out of bounds!");
    }
  }

  Bound getLowerBound() const {
    return lowerBound_;
  }

  Bound getUpperBound() const {
    return upperBound_;
  }

  bool isWithinBounds() const {
    Bound v = this->getParams()(0, 0);
    return getLowerBound() <= v && v <= getUpperBound();
  }

  /// \brief Update the design variable.
  virtual void updateImplementation(const double* dp, int size) override {
    DesignVariable_::updateImplementation(dp, size);
    BoundsEnforceEvent result = internal::BoundsEnforcer<BoundedCalibrationVariable>::apply(*this);
//    if(result != BoundsEnforceEvent::NOP){
      boundEventHandler_(*this, result);
//    }
  }

  virtual ~BoundedCalibrationVariable() {}
 private:
  Bound upperBound_, lowerBound_;
  std::function<void(BoundedCalibrationVariable &, BoundsEnforceEvent event)> boundEventHandler_;
};

namespace internal {
  template <typename Scalar>
  struct BoundsEnforcer<BoundedCalibrationVariable<aslam::backend::GenericScalar<Scalar>, Scalar>> {
    static BoundsEnforceEvent apply(BoundedCalibrationVariable<aslam::backend::GenericScalar<Scalar>, Scalar> & bdv){
      auto & v = bdv.getValue();
      if(bdv.getLowerBound() > v){
        bdv.setValue(bdv.getLowerBound());
        return BoundsEnforceEvent::CLAMPED_TO_LOWER;
      }
      else if(bdv.getUpperBound() < v){
        bdv.setValue(bdv.getUpperBound());
        return BoundsEnforceEvent::CLAMPED_TO_UPPER;
      }
      return BoundsEnforceEvent::NOP;
    }
  };

  template <>
  struct BoundsEnforcer<BoundedCalibrationVariable<aslam::backend::Scalar, double>> {
    static BoundsEnforceEvent apply(BoundedCalibrationVariable<aslam::backend::Scalar, double> & bdv){
      auto v = bdv.getValue();
      if(bdv.getLowerBound() > v){
        bdv.setValue(bdv.getLowerBound());
        return BoundsEnforceEvent::CLAMPED_TO_LOWER;
      }
      else if(bdv.getUpperBound() < v){
        bdv.setValue(bdv.getUpperBound());
        return BoundsEnforceEvent::CLAMPED_TO_UPPER;
      }
      return BoundsEnforceEvent::NOP;
    }
  };
}
}
}


#endif /* INCLUDE_ASLAM_CALIBRATION_BOUNDEDCALIBRATIONVARIABLE_HPP_ */
