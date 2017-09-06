#ifndef MEASUREMENTERRORTERMFS_H_
#define MEASUREMENTERRORTERMFS_H_

#include <aslam/backend/ErrorTerm.hpp>
#include <aslam/backend/FixedPointNumber.hpp>
#include <aslam/backend/VectorExpression.hpp>

#include "ErrorTermGroup.h"

namespace aslam {
namespace calibration {
namespace internal {
inline Eigen::Matrix<double, 1, 1> toMatrix(double s) {
  Eigen::Matrix<double, 1, 1> m;
  m(0, 0) = s;
  return m;
}

template<typename T, std::uintmax_t D>
inline Eigen::Matrix<double, 1, 1> toMatrix(aslam::backend::FixedPointNumber<T, D> t) {
  return toMatrix((double) t);
}
template<typename T>
const T & toMatrix(const T & m) {
  return m;
}
}

template<int D, typename PredictionExpression = backend::VectorExpression<D>>
class MeasurementErrorTerm : public aslam::backend::ErrorTermFs<D>, public ErrorTermGroupMember {
 public:
  typedef aslam::backend::ErrorTermFs<D> Parent;
  // Required by Eigen for fixed-size matrices members
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef Eigen::Matrix<double, D, D> Covariance;
  typedef decltype(static_cast<PredictionExpression*>(nullptr)->evaluate()) Input;
  typedef Input Output;

  MeasurementErrorTerm(const PredictionExpression & measurementPredictionExpression, const Input& measurement, const Covariance& cov, const ErrorTermGroupReference & etgr = ErrorTermGroupReference(), bool covarinaceIsSqrt = false)
      : ErrorTermGroupMember(etgr),
        _measurement(measurement),
        _cov(cov),
        _measurementPredictionExpression(measurementPredictionExpression) {
    if (covarinaceIsSqrt) {
      this->setSqrtInvR(_cov.inverse().eval());
    } else {
      this->setInvR(_cov.inverse());
    }
    backend::DesignVariable::set_t dv;
    _measurementPredictionExpression.getDesignVariables(dv);
    this->setDesignVariablesIterator(dv.begin(), dv.end());
  }

  MeasurementErrorTerm(const MeasurementErrorTerm& other) = default;

  virtual ~MeasurementErrorTerm() = default;

  /// Returns the input measurement
  const Input& getMeasurement() const {
    return _measurement;
  }
  /// Returns the covariance of the measurement
  const Covariance& getCovariance() const {
    return _cov;
  }
  /// Returns the prediction corresponding to the measurement
  Output getPrediction() const {
    return _measurementPredictionExpression.toValue();
  }

 protected:
  /// Evaluate the error term and return the weighted squared error
  virtual double evaluateErrorImplementation() override {
    typename Parent::error_t error;
    error = internal::toMatrix(_measurementPredictionExpression.toValue() - _measurement);
    this->setError(error);
    return this->evaluateChiSquaredError();
  }
  /// Evaluate the Jacobians
  virtual void evaluateJacobiansImplementation(aslam::backend::JacobianContainer& jacobians) override {
    _measurementPredictionExpression.evaluateJacobians(jacobians);
  }

  const Input _measurement;
  const Covariance _cov;
  PredictionExpression _measurementPredictionExpression;
};

}
}

#endif /* MEASUREMENTERRORTERMFS_H_ */
