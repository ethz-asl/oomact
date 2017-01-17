#include <aslam/calibration/model/fragments/PoseCv.h>

#include <boost/make_shared.hpp>

#include <aslam/calibration/model/Model.h>

namespace aslam {
namespace calibration {

PoseCv::PoseCv(const Module* module, boost::optional<std::string> defaultFrameName):
        frame(module->getModel().getFrame(module->getMyConfig().getString("frame", defaultFrameName))),
        rotationVariable(module->createCVIfUsed<RotationQuaternionCv>("rotation", "R")),
        translationVariable(module->createCVIfUsed<EuclideanPointCv>("translation", "T")),
        transExp(translationVariable),
        rotExp(rotationVariable),
        trafoExp(rotExp, transExp)
{
}

Eigen::Vector3d PoseCv::NoTranslation = Eigen::Vector3d::Zero();
Eigen::Vector4d PoseCv::NoRotation = Eigen::Vector4d::Unit(3);

} /* namespace calibration */
} /* namespace aslam */

