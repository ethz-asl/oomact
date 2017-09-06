#ifndef HC7392DE6_1111_462C_AE18_438D86FE435B
#define HC7392DE6_1111_462C_AE18_438D86FE435B

#include <aslam/backend/EuclideanExpression.hpp>
#include <aslam/backend/TransformationExpression.hpp>
#include <aslam/backend/EuclideanPoint.hpp>
#include <aslam/backend/EuclideanExpression.hpp>
#include <aslam/backend/RotationQuaternion.hpp>
#include <aslam/backend/RotationExpression.hpp>
#include <sm/kinematics/Transformation.hpp>

#include <aslam/calibration/model/Module.h>
#include <aslam/calibration/model/CalibrationVariable.h>
#include <boost/optional.hpp>

namespace aslam {
namespace calibration {

class Frame;

/// Euclidean point calibration variable
typedef CalibrationDesignVariable<aslam::backend::EuclideanPoint> EuclideanPointCv;
/// Shared pointer to Euclidean point calibration variable
typedef boost::shared_ptr<EuclideanPointCv> EuclideanPointCvSp;

/// Rotation quaternion calibration variable
typedef CalibrationDesignVariable<aslam::backend::RotationQuaternion> RotationQuaternionCv;
/// Shared pointer to rotation quaternion calibration variable
typedef boost::shared_ptr<RotationQuaternionCv> RotationQuaternionCvSp;


class PoseCv {
 public:
  PoseCv(Module * module, boost::optional<std::string> defaultFrameName = boost::optional<std::string>());

  const Eigen::Vector3d & getTranslationToParent() const { if(translationVariable) return translationVariable->getValue(); else return NoTranslation; }
  const Eigen::Vector4d & getRotationQuaternionToParent() const { if(rotationVariable) return rotationVariable->getQuaternion(); else return NoRotation; }
  Eigen::Matrix3d calcRotationToParentMatrix() const { assert(rotationVariable); return rotationVariable->toRotationMatrix(); }
  sm::kinematics::Transformation calcTransformationToParent() const { return sm::kinematics::Transformation(getRotationQuaternionToParent(), getTranslationToParent()); }

  const aslam::backend::EuclideanExpression & getTranslationToParentExpression() const { return transExp; }
  const aslam::backend::RotationExpression & getRotationToParentExpression() const { return rotExp; }
  const aslam::backend::TransformationExpression & getTransformationToParentExpression() const { return trafoExp; }

  virtual ~PoseCv() {}

  bool hasRotation() const {
    return bool(rotationVariable);
  }
  bool hasTranslation() const {
    return bool(translationVariable);
  }
  bool hasAny() const {
    return hasRotation() || hasTranslation();
  }

  const RotationQuaternionCv& getRotationVariable() const {
    assert(rotationVariable);
    return *rotationVariable;
  }
  RotationQuaternionCv& getRotationVariable() {
    assert(rotationVariable);
    return *rotationVariable;
  }

  const EuclideanPointCv& getTranslationVariable() const {
    assert(translationVariable);
    return *translationVariable;
  }
  EuclideanPointCv& getTranslationVariable() {
    assert(translationVariable);
    return *translationVariable;
  }

  const Frame& getParentFrame() const {
    return parentFrame_;
  }

  const Frame& getFrame() const {
    return frame_;
  }
 protected:
  void setActive(bool active) {
    if(rotationVariable) rotationVariable->setActive(active && rotationVariable->isToBeEstimated());
    if(translationVariable) translationVariable->setActive(active && translationVariable->isToBeEstimated());
  }
  const Frame & parentFrame_, & frame_;

  RotationQuaternionCvSp rotationVariable;
  EuclideanPointCvSp translationVariable;
  aslam::backend::EuclideanExpression transExp;
  aslam::backend::RotationExpression rotExp;
  aslam::backend::TransformationExpression trafoExp;

 private:
  static Eigen::Vector3d NoTranslation;
  static Eigen::Vector4d NoRotation;
};

} /* namespace calibration */
} /* namespace aslam */

#endif /* HC7392DE6_1111_462C_AE18_438D86FE435B */
