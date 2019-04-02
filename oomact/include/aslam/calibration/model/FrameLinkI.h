#ifndef H94A17E9A_126B_4B3A_BCCB_F82E8EEFA016
#define H94A17E9A_126B_4B3A_BCCB_F82E8EEFA016

#include <aslam/backend/EuclideanExpression.hpp>
#include <aslam/backend/RotationExpression.hpp>

#include <aslam/calibration/model/Model.h>

namespace aslam {
namespace calibration {

class RelativeKinematicExpression {
 public:
  RelativeKinematicExpression(
      aslam::backend::RotationExpression R,
      aslam::backend::EuclideanExpression p,
      aslam::backend::EuclideanExpression omega = aslam::backend::EuclideanExpression(),
      aslam::backend::EuclideanExpression v = aslam::backend::EuclideanExpression(),
      aslam::backend::EuclideanExpression alpha = aslam::backend::EuclideanExpression(),
      aslam::backend::EuclideanExpression a = aslam::backend::EuclideanExpression()
  ) : R(R),p(p),v(v),a(a),omega(omega),alpha(alpha) {}

  const aslam::backend::RotationExpression R;
  const aslam::backend::EuclideanExpression p, v, a, omega, alpha;
};

class FrameLinkI
{
 public:
  virtual RelativeKinematicExpression calcRelativeKinematics(
      Timestamp at, const ModelSimplification& simplification,
      const size_t maximalDerivativeOrder) const = 0;

  virtual RelativeKinematicExpression calcRelativeKinematics(
      const BoundedTimeExpression & at, const ModelSimplification& simplification,
      const size_t maximalDerivativeOrder) const = 0;

  virtual ~FrameLinkI();

  virtual const Frame& getReferenceFrame() const = 0;

  virtual const Frame& getFrame() const = 0;
};

class AbstractStaticFrameLink : public FrameLinkI
{
 public:
  virtual RelativeKinematicExpression calcRelativeKinematics() const = 0;

  virtual ~AbstractStaticFrameLink();

 private:
  RelativeKinematicExpression calcRelativeKinematics(
      Timestamp at, const ModelSimplification& simplification,
      const size_t maximalDerivativeOrder) const override final;

  RelativeKinematicExpression calcRelativeKinematics(
      const BoundedTimeExpression & at, const ModelSimplification& simplification,
      const size_t maximalDerivativeOrder) const override final;
};

} /* namespace calibration */
} /* namespace aslam */

#endif /* H94A17E9A_126B_4B3A_BCCB_F82E8EEFA016 */
