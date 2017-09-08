#include <aslam/calibration/model/priors/CrossPoseCvPrior.h>
#include <glog/logging.h>

#include <aslam/calibration/tools/ErrorTermStatisticsWithProblemAndPredictor.h>
#include "aslam/calibration/error-terms/ErrorTermPose.h"

namespace aslam {
namespace calibration {

CrossPoseCvPrior::CrossPoseCvPrior(Model& model, std::string name, sm::value_store::ValueStoreRef config, PoseCv & from, PoseCv & to) :
  Module(model, name, config, false),
  PoseCv(this, std::string("None")),
  from(from),
  to(to)
{
}

Eigen::Matrix3d squaredMatrix(const Eigen::MatrixXd & m){
  return m.transpose() * m;
}

void CrossPoseCvPrior::addMeasurementErrorTerms(CalibratorI& calib, const CalibrationConfI & /*ec*/, ErrorTermReceiver & problem, bool observeOnly) const {
  if(!from.hasAny()){
    LOG(INFO) << "Not adding error term because " << getObjectName(from) << " has no variables.";
  }
  else if(!to.hasAny()){
    LOG(INFO) << "Not adding error term because " << getObjectName(to) << " has no variables.";
  } else {
    ErrorTermStatisticsWithProblemAndPredictor dest(calib, getName(), problem, observeOnly);
    PoseMeasurement pose{
      getTranslationToParent(),
      getRotationQuaternionToParent()
    };

    auto e = boost::make_shared<ErrorTermPose>(
        to.getTransformationToParentExpression().inverse() * from.getTransformationToParentExpression(),
        pose,
        squaredMatrix(getTranslationVariable().getPriorCovarianceSqrt()),
        squaredMatrix(getRotationVariable().getPriorCovarianceSqrt()),
        getName()
      );

    LOG(INFO) << getName() << " initial prediction="<< e->getPrediction().transpose() << ", measurement=" << e->getMeasurement().transpose();
    dest.add(Timestamp::Zero(), e);
    dest.printInto(LOG(INFO));
  }
}

CrossPoseCvPrior::~CrossPoseCvPrior() {
}

void CrossPoseCvPrior::writeConfig(std::ostream& out) const {
  out << ", from=" << getObjectName(from) << ", to=" << getObjectName(to) << ", dp=" << getTranslationToParent().transpose() << ", dq="<<  getRotationQuaternionToParent().transpose() ;
}

} /* namespace calibration */
} /* namespace aslam */
