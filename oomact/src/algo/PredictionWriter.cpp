#include "aslam/calibration/algo/PredictionWriter.h"
#include <aslam/backend/ScalarExpression.hpp>
#include <aslam/calibration/error-terms/MeasurementErrorTerm.h>

#include <glog/logging.h>

#include "aslam/calibration/tools/tools.h"

#include "aslam/calibration/error-terms/ErrorTermPose.h"

namespace aslam {
namespace calibration {

namespace internal {
void outMeasurementsAndPredictions(Timestamp /* timestamp */, const ErrorTermTangency & /* e */, std::ostream &/* outPred */, std::ostream &/* outMeasure */){
  //TODO D implement
}

void outMeasurementsAndPredictions(Timestamp timestamp, const ErrorTermPose & e, std::ostream & outPred, std::ostream & outMeasure){
  outPred << timestamp << " " << e.getPrediction().transpose(); outPred << std::endl;
  outMeasure << timestamp << " " << e.getMeasurement().transpose(); outMeasure << std::endl;
}
void outMeasurementsAndPredictions(Timestamp /* timestamp */, const backend::ErrorTerm & /* e */, std::ostream &/* outPred */, std::ostream &/* outMeasure */){
}
void outMeasurementsAndPredictions(Timestamp timestamp, const MeasurementErrorTerm<1, aslam::backend::ScalarExpression> & e, std::ostream &outPred, std::ostream &outMeasure){
  outPred << timestamp << " " << e.getPrediction(); outPred << std::endl;
  outMeasure << timestamp << " " << e.getMeasurement(); outMeasure << std::endl;
}
}


void PredictionFunctorWriter::write(const std::string & filePrefix) const {
  if(functors.empty()) return;

  std::ofstream outPred, outErr, outMeasure, outNormalizedErr;

  auto open = [&](std::ofstream & out, std::string postFix){
    auto s = filePrefix + name + postFix;
    VLOG(1) << "Writing " << name << postFix  << "(" << functors.size()<< ") to " << s;
    openStream(out, s);
  };

  open(outPred, "Pred");
  open(outMeasure, "Measure");
  open(outErr, "Err");
  open(outNormalizedErr, "ErrNormalized");

  for(const auto &f: functors) f(outPred, outMeasure, outErr, outNormalizedErr);

  outPred.close();
  outMeasure.close();
  outErr.close();
  outNormalizedErr.close();
}

}
}
