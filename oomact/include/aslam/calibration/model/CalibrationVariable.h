#ifndef SRC_DESIGN_VARIABLES_CALIBRATIONVARIABLE_H_
#define SRC_DESIGN_VARIABLES_CALIBRATIONVARIABLE_H_

#include <string>
#include <ostream>
#include <vector>

#include <Eigen/Core>

#include <sm/boost/null_deleter.hpp>
#include <boost/optional.hpp>
#include <boost/make_shared.hpp>

#include <aslam/backend/DesignVariable.hpp>
#include <aslam/backend/ExpressionErrorTerm.hpp>

#include <sm/value_store/ValueStore.hpp>
#include <aslam/calibration/error-terms/ErrorTermGroup.h>
#include <aslam/calibration/error-terms/MeasurementErrorTerm.h>
#include <aslam/calibration/tools/Covariance.h>

namespace aslam {
namespace calibration {

using namespace ::sm::value_store;

class CalibrationVariable {
 public:
  virtual ~CalibrationVariable();

  virtual const std::string & getName() const = 0;

  virtual void updateStore() = 0;
  virtual void resetToStore() = 0;
  virtual boost::shared_ptr<backend::ErrorTerm> createPriorErrorTerm() = 0;

  void printFunctorInto(std::ostream& out, std::function<void(int)> f, int limit) const;

  virtual void printValuesNiceInto(std::ostream& out) const;
  virtual backend::DesignVariable & getDesignVariable() = 0;
  virtual const backend::DesignVariable & getDesignVariable() const = 0;

  virtual const char * getTangentComponentName(int i) const = 0;
  int getDimension() const { return getDesignVariable().minimalDimensions(); }
  int getNumParams() const { return getParams().size(); }
  void setIndex(int i) { _index = i; }
  int getIndex() const { return _index; }
  Eigen::MatrixXd getParams() const;
  virtual Eigen::VectorXd getMinimalComponents() const = 0;
  virtual void setMinimalComponents(const Eigen::VectorXd & v) = 0;
  virtual Eigen::VectorXd getDisplacementToLastUpdateValue() const = 0;
  virtual double getDistanceToLastUpdateValue() const { return getDisplacementToLastUpdateValue().norm(); }
  virtual bool isUpdateable() const { return false; };
  virtual bool isToBeEstimated() const = 0;
  virtual bool isActivated() const = 0;
  static int const NameWidth;
 private:
  int _index = -1;
};

namespace internal {
typedef std::vector<const char *> ComponentNames;
extern const ComponentNames SingleComponent;

template <typename DesignVariable_>
struct DVComponentNames {
  static constexpr const ComponentNames & value = SingleComponent;
};

void getPTVector(const std::vector<const char*>& componentNames, std::vector<ValueHandle<double> >& v, ValueStore& pt);

template <typename DesignVariable_>
void getPTVector(std::vector<ValueHandle<double>> & v, ValueStore & pt){
  return getPTVector(DVComponentNames<DesignVariable_>::value, v, pt);
}

template <typename DesignVariable>
struct ParamsPackTraits {
  static Eigen::VectorXd pack(const Eigen::VectorXd &v) { return v; }
  static Eigen::VectorXd unpack(const Eigen::VectorXd &v) { return v; }
};

Eigen::VectorXd loadPacked(std::vector<ValueHandle<double>> &vhs);
void storePacked(std::vector<ValueHandle<double>> &vhs, const Eigen::VectorXd & vPacked);

inline Eigen::MatrixXd toMatrixXd(double v) {
  Eigen::MatrixXd m(1, 1);
  m << v;
  return m;
}
inline Eigen::MatrixXd toMatrixXd(Eigen::MatrixXd m){
  return m;
}

class DVLoadTraitsBase {
 public:
  bool isUpdateable() const;
 protected:
  std::vector<ValueHandle<double>> vhs;
};

template <typename DesignVariable_>
class DVLoadTraits : public internal::DVLoadTraitsBase {
 public:
  decltype (ParamsPackTraits<DesignVariable_>::unpack(Eigen::VectorXd())) load(ValueStoreRef & vs){
    assert(vhs.size() == 0);
    getPTVector<DesignVariable_>(vhs, vs);
    return ParamsPackTraits<DesignVariable_>::unpack(internal::loadPacked(vhs));
  }
  void store(const Eigen::VectorXd & v){
    internal::storePacked(vhs, ParamsPackTraits<DesignVariable_>::pack(v));
  }
};

}

template <typename DesignVariable_>
class CalibrationDesignVariable : virtual protected internal::DVLoadTraits<DesignVariable_>, virtual public DesignVariable_, virtual public CalibrationVariable {
 public:
  CalibrationDesignVariable(const std::string & name, ValueStoreRef valueStore) :
    internal::DVLoadTraits<DesignVariable_>(), DesignVariable_(internal::DVLoadTraits<DesignVariable_>::load(valueStore)),  name_(name), covariance_(valueStore, getDimension()),
    upstreamValue_(getParams()),
    upstreamValueStore_(valueStore)
  {
    estimateVh_ = valueStore.getBool("estimate", true);
  }

  virtual ~CalibrationDesignVariable(){}

  const std::string & getName() const override { return name_; }

  boost::shared_ptr<backend::ErrorTerm> createPriorErrorTerm() override;

  const char * getTangentComponentName(int i) const override {
    return internal::DVComponentNames<DesignVariable_>::value[i];
  }
  Eigen::VectorXd getMinimalComponents() const override {
    return internal::ParamsPackTraits<DesignVariable_>::pack(getParams());
  }

  void setMinimalComponents(const Eigen::VectorXd & v) override {
    Eigen::MatrixXd val = internal::toMatrixXd(internal::ParamsPackTraits<DesignVariable_>::unpack(v));
    DesignVariable_::setParameters(val);
  }

  void updateStore() override {
    auto v = getParams();
    internal::DVLoadTraits<DesignVariable_>::store(v);
    upstreamValue_ = v;
  }

  virtual void resetToStore() override {
    Eigen::MatrixXd params;
    DesignVariable_(internal::DVLoadTraits<DesignVariable_>::load(upstreamValueStore_)).aslam::backend::DesignVariable::getParameters(params);
    DesignVariable_::setParameters(params);
    upstreamValue_ = params;
    covariance_ = Covariance(upstreamValueStore_, getDimension());
  }

  Eigen::VectorXd getDisplacementToLastUpdateValue() const override {
    Eigen::VectorXd v;
    DesignVariable_::minimalDifference(upstreamValue_, v);
    return v;
  }

  bool isUpdateable() const override {
    return internal::DVLoadTraits<DesignVariable_>::isUpdateable();
  }

  bool isToBeEstimated() const override { return estimateVh_.get(); }
  bool isActivated() const override { return DesignVariable_::isActive(); }

  virtual Eigen::MatrixXd getPriorCovarianceSqrt() const {
    return covariance_.getValueSqrt();
  }

 private:
  backend::DesignVariable & getDesignVariable() override { return *this; };
  const backend::DesignVariable & getDesignVariable() const override { return *this; };
  std::string name_;
  Covariance covariance_;
  Eigen::MatrixXd upstreamValue_;
  sm::value_store::ValueHandle<bool> estimateVh_;
  ValueStoreRef upstreamValueStore_;
};

extern const ErrorTermGroupReference CvPriorGroup;

}

namespace backend{
class Scalar;
template<typename Scalar_> class GenericScalar;
class EuclideanPoint;
class RotationQuaternion;
}

namespace calibration {
namespace internal {
template <> struct DVComponentNames<backend::EuclideanPoint> {
  static const ComponentNames value;
};
template <> struct DVComponentNames<backend::RotationQuaternion> {
  static const ComponentNames value;
};

class RotationQuaternionLoadImpl;
template<> class DVLoadTraits<backend::RotationQuaternion> {
 public:
  DVLoadTraits();
  ~DVLoadTraits();
  Eigen::Vector4d load(ValueStoreRef & vs);
  void store(const Eigen::Vector4d & v);
  bool isUpdateable() const;
 private:
  std::unique_ptr<RotationQuaternionLoadImpl> impl;
};

template <>
struct ParamsPackTraits<backend::RotationQuaternion> {
  static Eigen::VectorXd pack(const Eigen::VectorXd &v);
  static Eigen::Vector4d unpack(const Eigen::VectorXd &v);
};
template <>
struct ParamsPackTraits<backend::Scalar> {
  static const Eigen::VectorXd & pack(const Eigen::VectorXd &v) {  return v; }
  static double unpack(const Eigen::VectorXd &v) { return v[0];}
};

template <typename Scalar>
struct ParamsPackTraits<backend::GenericScalar<Scalar>> {
  static const Eigen::VectorXd & pack(const Eigen::VectorXd &v) { return v; }
  static double unpack(const Eigen::VectorXd &v) { return v[0]; }
};

Eigen::MatrixXd getMinimalComponents(const CalibrationDesignVariable<backend::RotationQuaternion> & cv);


template<typename DesignVariable_>
struct PriorErrorTermCreater {
  static boost::shared_ptr<backend::ErrorTerm> createPriorErrorTerm(CalibrationDesignVariable<DesignVariable_> &dv, Eigen::MatrixXd covSqrt) {
    return boost::make_shared<MeasurementErrorTerm<decltype(dv.toExpression())::Dimension, decltype(dv.toExpression())>>(dv.toExpression(), dv.getMinimalComponents(), covSqrt, CvPriorGroup, true);
  }
};

template<>
struct PriorErrorTermCreater<backend::RotationQuaternion> {
  static boost::shared_ptr<backend::ErrorTerm> createPriorErrorTerm(CalibrationDesignVariable<backend::RotationQuaternion> &dv, Eigen::MatrixXd covSqrt);
};

template<>
struct PriorErrorTermCreater<backend::Scalar> {
  static boost::shared_ptr<backend::ErrorTerm> createPriorErrorTerm(CalibrationDesignVariable<backend::Scalar> &dv, Eigen::MatrixXd covSqrt);
};

template<typename Scalar_>
struct PriorErrorTermCreater<backend::GenericScalar<Scalar_>> {
  static boost::shared_ptr<backend::ErrorTerm> createPriorErrorTerm(CalibrationDesignVariable<backend::GenericScalar<Scalar_>> &dv, Eigen::MatrixXd covSqrt){
    return boost::make_shared<MeasurementErrorTerm<1, aslam::backend::GenericScalarExpression<Scalar_>>>(dv.toExpression(), dv.getParams()(0, 0), covSqrt, CvPriorGroup, true);
  }
};
}

template<typename DesignVariable_>
boost::shared_ptr<backend::ErrorTerm> CalibrationDesignVariable<DesignVariable_>::createPriorErrorTerm() {
  return internal::PriorErrorTermCreater<DesignVariable_>::createPriorErrorTerm(*this, getPriorCovarianceSqrt());
}


/**
 * Globally enable JPL quaternion convention for value store input/output of the CalibrationDesignVariable<backend::RotationQuaternion>.
 * (Default is the traditional Hamilton convention.)

 * Technically this conjugates (negates i,j,k components) for all Quaternion-input and -output compared
 * to when this is not called before. Call it (once) before loading any
 * CalibrationDesignVariable<backend::RotationQuaternion> as input output will be inconsistent for those
 * loaded before.
 */
void useJPLQuaternionConventionForInputOutput();

}
}

#endif /* SRC_DESIGN_VARIABLES_CALIBRATIONVARIABLE_H_ */
