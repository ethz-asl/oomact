#include <aslam/calibration/model/FrameLinkI.h>

namespace aslam {
namespace calibration {


FrameLinkI::~FrameLinkI()
{
}

RelativeKinematicExpression AbstractStaticFrameLink::calcRelativeKinematics(
    Timestamp /*at*/, const ModelSimplification& /*simplification*/,
    const size_t /*maximalDerivativeOrder*/) const
{
  return calcRelativeKinematics();
}

RelativeKinematicExpression AbstractStaticFrameLink::calcRelativeKinematics(
    const BoundedTimeExpression& /*at*/, const ModelSimplification& /*simplification*/,
    const size_t /*maximalDerivativeOrder*/) const
{
  return calcRelativeKinematics();
}

AbstractStaticFrameLink::~AbstractStaticFrameLink()
{
}

}
}
