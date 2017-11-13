#ifndef PREDICTIONWRITER_H_
#define PREDICTIONWRITER_H_

#include <fstream>
#include <functional>
#include <iostream>
#include <vector>
#include <string>

#include <boost/shared_ptr.hpp>

#include "aslam/calibration/error-terms/ConditionalErrorTerm.h"
#include "aslam/calibration/Timestamp.h"

namespace aslam {
namespace backend {
class ErrorTerm;
class ScalarExpression;
}
namespace calibration {

template <int D, typename PredictionExpression> class MeasurementErrorTerm;
class ErrorTermTangency;
class ErrorTermPose;

namespace internal {
template <int D, typename PredictionExpression>
void outMeasurementsAndPredictions(Timestamp timestamp, const MeasurementErrorTerm<D, PredictionExpression> & e, std::ostream &outPred, std::ostream &outMeasure){
  outPred << timestamp << " " << e.getPrediction().transpose(); outPred << std::endl;
  outMeasure << timestamp << " " << e.getMeasurement().transpose(); outMeasure << std::endl;
}

void outMeasurementsAndPredictions(Timestamp timestamp, const MeasurementErrorTerm<1, aslam::backend::ScalarExpression> & e, std::ostream &outPred, std::ostream &outMeasure);
void outMeasurementsAndPredictions(Timestamp timestamp, const ErrorTermTangency & e, std::ostream &outPred, std::ostream &outMeasure);
void outMeasurementsAndPredictions(Timestamp timestamp, const ErrorTermPose & e, std::ostream &outPred, std::ostream &outMeasure);
void outMeasurementsAndPredictions(Timestamp timestamp, const backend::ErrorTerm & e, std::ostream &outPred, std::ostream &outMeasure);
}

class PredictionWriter {
 public:
  inline virtual ~PredictionWriter();
  virtual void write(const std::string & filePrefix) const = 0;
  virtual const std::string & getName() const = 0;
};

PredictionWriter::~PredictionWriter(){}

class PredictionFunctorWriter : public PredictionWriter {
 public:
  PredictionFunctorWriter (std::string name) : name(name){}
  ~PredictionFunctorWriter(){}
  typedef std::function<void(std::ostream &outPred, std::ostream &outMeasure, std::ostream &outErr, std::ostream & outNormalizedErr)> Writer;

  const std::string & getName() const {
    return name;
  }

  void add(const Writer & c){
    functors.push_back(c);
  }

  void write(const std::string & filePrefix) const;


  template <typename ErrorTerm>
  void add(Timestamp timestamp, boost::shared_ptr<ErrorTerm> e){
    add([=](std::ostream &outPred, std::ostream &outMeasure, std::ostream &outErr, std::ostream & outNormalizedErr){
      e->evaluateError();
      internal::outMeasurementsAndPredictions(timestamp, *e, outPred, outMeasure);
      outErr << timestamp << " " << e->error().transpose(); outErr << std::endl;
      outNormalizedErr << timestamp << " " << (e->sqrtInvR() * e->error()).transpose(); outNormalizedErr << std::endl;
    });
  }

 private:
  const std::string name;
  std::vector<Writer> functors;
};

}
}

#endif /* PREDICTIONWRITER_H_ */
