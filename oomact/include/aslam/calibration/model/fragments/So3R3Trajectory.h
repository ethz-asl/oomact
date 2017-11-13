#ifndef H8E4AC88D_C8F7_417C_8732_BFF3DB9C79DE
#define H8E4AC88D_C8F7_417C_8732_BFF3DB9C79DE

#include <aslam/splines/OPTBSpline.hpp>
#include <aslam/splines/OPTUnitQuaternionBSpline.hpp>
#include <bsplines/EuclideanBSpline.hpp>
#include <bsplines/NsecTimePolicy.hpp>
#include <bsplines/UnitQuaternionBSpline.hpp>
#include <sm/timing/NsecTimeUtilities.hpp>

#include "../../SensorId.h"
#include "../../tools/Interval.h"

namespace aslam {
namespace backend {
class ErrorTermReceiver;
}
namespace calibration {
class CalibratorI;
class DesignVariableReceiver;
class So3R3TrajectoryCarrier;

typedef aslam::splines::OPTBSpline<typename bsplines::UnitQuaternionBSpline<Eigen::Dynamic, bsplines::NsecTimePolicy>::CONF>::BSpline RotationSpline;
typedef aslam::splines::OPTBSpline<typename bsplines::EuclideanBSpline<Eigen::Dynamic, 3, bsplines::NsecTimePolicy>::CONF>::BSpline TranslationSpline;

template <typename RotationFactory, typename TranslationFactory>
struct ExpressionFactoryPair {
  RotationFactory rot;
  TranslationFactory trans;
};

template <int MaxDerivative, typename Spline>
auto getEF(Spline & s, BoundedTimeExpression t) -> decltype(s.template getExpressionFactoryAt<MaxDerivative>(t.timestampExpresion, t.lBound, t.uBound)) {
  return s.template getExpressionFactoryAt<MaxDerivative>(t.timestampExpresion, t.lBound, t.uBound);
}
template <int MaxDerivative, typename Spline>
auto getEF(Spline & s, Timestamp t) -> decltype(s.template getExpressionFactoryAt<MaxDerivative>(t)) {
  return s.template getExpressionFactoryAt<MaxDerivative>(t);
}

class So3R3Trajectory {
 public:
  So3R3Trajectory(const So3R3TrajectoryCarrier & carrier);

  RotationSpline & getRotationSpline() {
    return rotationSpline;
  }

  const RotationSpline & getRotationSpline() const {
    return rotationSpline;
  }

  TranslationSpline & getTranslationSpline() {
    return translationSpline;
  }

  const TranslationSpline & getTranslationSpline() const {
    return translationSpline;
  }

  void writeToFile(const CalibratorI & calib, const std::string & pathPrefix) const;
  void addToProblem(const bool stateActive, DesignVariableReceiver & designVariableReceiver);
  void addWhiteNoiseModelErrorTerms(backend::ErrorTermReceiver & errorTermReceiver, std::string name, const double invSigma) const;

  void fitSplines(const Interval& effectiveBatchInterval, const size_t numMeasurements, const std::vector<sm::timing::NsecTime> & timestamps, const std::vector<Eigen::Vector3d> & transPoses, const std::vector<Eigen::Vector4d> & rotPoses);

  void initSplinesConstant(const Interval& effectiveBatchInterval, const size_t numMeasurements, const Eigen::Vector3d & transPose = Eigen::Vector3d::Zero(), const Eigen::Vector4d & rotPose = sm::kinematics::quatIdentity());

  const So3R3TrajectoryCarrier& getCarrier() const {
    return carrier;
  }

  template <int RotationMaxDerivative, int TranslationMaxDerivative = RotationMaxDerivative, typename Time>
  auto getExpressionFactoryPair(Time t) const -> ExpressionFactoryPair<decltype(getEF<RotationMaxDerivative>(getRotationSpline(), t)), decltype(getEF<TranslationMaxDerivative>(getTranslationSpline(), t))> {
    return {getEF<RotationMaxDerivative>(getRotationSpline(), t), getEF<TranslationMaxDerivative>(getTranslationSpline(), t)};
  }

 private:
  RotationSpline rotationSpline;
  TranslationSpline translationSpline;
  const So3R3TrajectoryCarrier & carrier;
};

} /* namespace calibration */
} /* namespace aslam */

#endif /* H8E4AC88D_C8F7_417C_8732_BFF3DB9C79DE */
