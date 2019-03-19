#include <aslam/calibration/model/fragments/PoseCv.h>

#include <boost/make_shared.hpp>

#include <aslam/calibration/model/Model.h>
#include <aslam/calibration/tools/DeprecationAlerter.h>

namespace aslam {
namespace calibration {

PoseCv::PoseCv(Module* module, boost::optional<std::string> defaultReferenceFrameName):
        referenceFrame_(module->getModel().getFrame(module->getMyConfig().get(
            {"referenceFrame",
            "!frame", "Use referenceFrame instead."},
            defaultReferenceFrameName, parameterDeprecationAlerter))),
        frame_(module->getModel().getOrCreateFrame(module->getName())), //TODO D make sure there is only one PoseCv in a module
        rotationVariable(module->createCVIfUsed<RotationQuaternionCv>("rotation", "R")),
        translationVariable(module->createCVIfUsed<EuclideanPointCv>("translation", "T")),
        transExp(translationVariable),
        rotExp(rotationVariable),
        trafoExp(rotExp, transExp)
{
}

Eigen::Vector3d PoseCv::NoTranslation = Eigen::Vector3d::Zero();
Eigen::Vector4d PoseCv::NoRotation = Eigen::Vector4d::Unit(3);

aslam::calibration::RelativeKinematicExpression PoseCv::calcRelativeKinematics() const
{
  return RelativeKinematicExpression(getRotationToParentExpression(),
                                     getTranslationToParentExpression());
}

} /* namespace calibration */
} /* namespace aslam */

