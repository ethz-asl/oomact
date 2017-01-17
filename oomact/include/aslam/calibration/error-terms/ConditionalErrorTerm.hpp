#ifndef H06EA16DA_4084_49B3_AF57_064BCFB2CF8B
#define H06EA16DA_4084_49B3_AF57_064BCFB2CF8B


#include <aslam/backend/ErrorTerm.hpp>

namespace aslam {
namespace calibration {


class ConditionalErrorTermBase {
 public:
  typedef std::function<bool()> Condition;
  ConditionalErrorTermBase(Condition condition) : condition(condition) {}

  bool isActive() const {
    return active;
  }
  virtual ~ConditionalErrorTermBase() = default;
 protected:
  bool active = true;
  Condition condition;
};


template <typename ToConditionErrorTerm, typename = void>
struct Predictor : public ToConditionErrorTerm {
  Predictor(const ToConditionErrorTerm & toConditionErrorTerm) : ToConditionErrorTerm(toConditionErrorTerm) {}
};

template <typename ToConditionErrorTerm>
class ConditionalErrorTerm : public Predictor<ToConditionErrorTerm>, public ConditionalErrorTermBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ConditionalErrorTerm(const ToConditionErrorTerm & toConditionErrorTerm, ConditionalErrorTermBase::Condition condition) :
    Predictor<ToConditionErrorTerm>(toConditionErrorTerm),
    ConditionalErrorTermBase{condition}
  {
    static_assert(std::is_base_of<backend::ErrorTerm, ToConditionErrorTerm>::value, "This is only for real ErrorTerms!");
  }

  virtual ~ConditionalErrorTerm() = default;
 protected:
  virtual double evaluateErrorImplementation() override {
    if(this->condition()){
      this->active = true;
      return ToConditionErrorTerm::evaluateErrorImplementation();
    } else {
      this->active = false;
      this->setError(ToConditionErrorTerm::error_t::Zero());
      return 0;
    }
  }

  /// \brief evaluate the Jacobians
  virtual void evaluateJacobiansImplementation(backend::JacobianContainer & outJacobians) override {
    if(this->isActive()){
      ToConditionErrorTerm::evaluateJacobiansImplementation(outJacobians);
    } else {
      for(backend::DesignVariable * dv : this->designVariables()){
        outJacobians.add(dv, Eigen::MatrixXd::Zero(outJacobians.expectedRows(), dv->minimalDimensions()));
      }
    }
  }
};


template <typename ToConditionErrorTerm>
ConditionalErrorTerm<ToConditionErrorTerm> addCondition(const ToConditionErrorTerm & et, ConditionalErrorTermBase::Condition condition) {
  return ConditionalErrorTerm<ToConditionErrorTerm>(et, condition);
}

template <typename ToConditionErrorTerm>
boost::shared_ptr<ToConditionErrorTerm> addConditionShared(const ToConditionErrorTerm & et, ConditionalErrorTermBase::Condition condition) {
  return boost::shared_ptr<ToConditionErrorTerm>(new ConditionalErrorTerm<ToConditionErrorTerm>(et, condition));
}

inline bool errorTermIsActive(const backend::ErrorTerm& e) {
  const ConditionalErrorTermBase* p = dynamic_cast<const ConditionalErrorTermBase*>(&e);
  return !p || p->isActive();
}


template <typename ToConditionErrorTerm>
struct Predictor<ToConditionErrorTerm, typename std::enable_if<std::is_same<Eigen::VectorXd, decltype(static_cast<ToConditionErrorTerm*>(nullptr)->getPrediction())>::value>::type> : public ToConditionErrorTerm {
  Predictor(const ToConditionErrorTerm & toConditionErrorTerm) : ToConditionErrorTerm(toConditionErrorTerm) {}
  virtual Eigen::VectorXd getPrediction() const override {
    if(errorTermIsActive(*this))
      return ToConditionErrorTerm::getPrediction();
    else{
      return ToConditionErrorTerm::getMeasurement();
    }
  }
  virtual ~Predictor() = default;
};


}
}

#endif /* H06EA16DA_4084_49B3_AF57_064BCFB2CF8B */
