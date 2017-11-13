#ifndef H67E40F4F_4DF6_4A43_8F6A_7E41450092A7
#define H67E40F4F_4DF6_4A43_8F6A_7E41450092A7

#include <aslam/backend/FixedPointNumber.hpp>
#include <aslam/backend/OptimizationProblemBase.hpp>

#include <aslam/calibration/tools/ErrorTermStatistics.h>
#include <aslam/calibration/algo/PredictionWriter.h>
#include <aslam/calibration/SensorId.h>

namespace aslam {
namespace calibration {
class CalibratorI;

template <typename ErrorTerm>
Timestamp getTimestampFor(size_t counter, ErrorTerm & /* errorTerm */){
  return Timestamp(Timestamp::Numerator(counter));
}

class ErrorTermStatisticsWithProblemAndPredictor : public ErrorTermStatistics {
 public:
  ErrorTermStatisticsWithProblemAndPredictor(CalibratorI & calib, std::string name, aslam::backend::ErrorTermReceiver & passToETReceiver, bool observeOnly);

  template <typename ErrorTerm>
  void add(Timestamp timestamp, const boost::shared_ptr<ErrorTerm> & e, bool ignoreInactive = true){
    ErrorTermStatistics::add(*e, ignoreInactive);
    if(!observeOnly) passToETReceiver_.addErrorTerm(e);
    predictionWriterPtr->add(timestamp, e);
  }

  template <typename ErrorTerm>
  void addErrorTerm(const boost::shared_ptr<ErrorTerm> & e) {
    Timestamp t = getTimestampFor(getCounter(), *e);
    add(t, e);
  }

  virtual ~ErrorTermStatisticsWithProblemAndPredictor() {
  }

 private:
  aslam::backend::ErrorTermReceiver & passToETReceiver_;
  std::shared_ptr<PredictionFunctorWriter> predictionWriterPtr;
  bool observeOnly;
};

} /* namespace calibration */
} /* namespace aslam */

#endif /* H67E40F4F_4DF6_4A43_8F6A_7E41450092A7 */
