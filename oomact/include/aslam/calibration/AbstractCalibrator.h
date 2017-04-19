#ifndef H2E153EC9_9902_41A1_A522_E4E0A04D6F76
#define H2E153EC9_9902_41A1_A522_E4E0A04D6F76

#include "CalibratorI.hpp"
#include <aslam/calibration/CommonTypes.hpp>
#include <aslam/calibration/algo/PredictionWriter.h>

namespace aslam {
namespace backend {
class ErrorTermReceiver;
namespace callback {
class Registry;
}
}
namespace calibration {
class EstConf;
class CalibrationProblem;


class AbstractCalibratorOptions : public CalibratorOptionsI {
 public:
  AbstractCalibratorOptions(const sm::value_store::ValueStoreRef& config);
  virtual ~AbstractCalibratorOptions();

  double getSplineOutputSamplePeriod() const override {
    return splineOutputSamplePeriod;
  }
  bool getPredictResults() const override {
    return predictResults;
  }

  bool getVerbose() const override {
    return verbose;
  }
  void setVerbose(bool verbose) override {
    this->verbose = verbose;
  }

  bool getAcceptConstantErrorTerms() const override {
    return acceptConstantErrorTerms;
  }
  void setAcceptConstantErrorTerms(bool acceptConstantErrorTerms) {
    this->acceptConstantErrorTerms = acceptConstantErrorTerms;
  }
 private:
  bool predictResults;
  bool verbose;
  bool acceptConstantErrorTerms;
  double splineOutputSamplePeriod;
};

class AbstractCalibrator : public virtual CalibratorI {
 public:
  AbstractCalibrator(ValueStoreRef config, std::shared_ptr<Model> model);

  virtual ~AbstractCalibrator() {}

  void setUpdateHandler(StatusUpdateHandler statusUpdateHandler, CalibrationUpdateHandler calibrationUpdateHandler) override;

  virtual void addToArchive(sm::MatrixArchive & ma, bool append = false) const override;
  virtual void loadFromArchive(sm::MatrixArchive & ma, int index = -1) override;

  virtual void setLowestTimestamp(Timestamp lowestTimeStamp) override;

  virtual std::shared_ptr<PredictionFunctorWriter> createPredictionCollector (const std::string & name) override;

  const Model & getModel() const override { return _model; }
  Model & getModel() override { return _model; }

  double secsSinceStart(Timestamp timestamp) const override;
  std::string secsSinceStart(const Interval & interval) const override;

  virtual const Interval& getCurrentEffectiveBatchInterval() const override {
    return _currentEffectiveBatchInterval;
  }

  void addMeasurementTimestamp(Timestamp t, const Sensor & sensor) override;

protected:
  bool initStates();
  void estimate(const EstConf & estimationConfig, CalibrationProblem & calibrationProblem, BatchStateReceiver & batchStateReceiver, std::function<void()> optimize);
  void printBatchErrorTermStatistics(const CalibrationProblem& batch, bool updateError, std::ostream& out);
  void updateOptimizerInspector(const CalibrationProblem &  currentBatch, bool printRegessionErrorStatistics, std::function<void(std::ostream & o)> printOptimizationState, backend::callback::Registry & callbackRegistry);
  virtual void addFactors(const EstConf& estimationConfig, backend::ErrorTermReceiver & problem, std::function<void()> statusCallback);

  Timestamp _lastTimestamp = -1L;
  Timestamp _lowestTimestamp = -1L;
  bool _lowesTimestampProvided = false;

  Interval _currentEffectiveBatchInterval;

  ModuleLink<Sensor> _timeBaseSensor;

  std::vector<std::shared_ptr<PredictionWriter>> _predictionData;

  StatusUpdateHandler _statusUpdateHandler;
  CalibrationUpdateHandler _calibrationUpdateHandler;


  const std::shared_ptr<Model> _modelSP;
  Model & _model;
 private:
  void addMeasurementTimestamp(Timestamp lowerBound, Timestamp upperBound = -1L);

  void setCalibrationVariablesActivity(const EstConf& ec);

  virtual bool handleNewTimeBaseTimestamp(Timestamp t) = 0;
  virtual void clearAfterEstimation();
};

extern template class ModuleLink<Sensor>;

} /* namespace calibration */
} /* namespace aslam */

#endif /* H2E153EC9_9902_41A1_A522_E4E0A04D6F76 */
