#include "aslam/calibration/calibrator/AbstractCalibrator.h"

#include <chrono>
#include <functional>
#include <map>

#include <glog/logging.h>
#include <sm/MatrixArchive.hpp>
#include <aslam/backend/OptimizerCallback.hpp>
#include <aslam/backend/OptimizerCallbackManager.hpp>
#include <aslam/calibration/calibrator/CalibrationConfI.h>

#include <aslam/calibration/calibrator/CalibrationProblem.h>
#include <aslam/calibration/DesignVariableReceiver.h>
#include <aslam/calibration/model/Model.h>
#include <aslam/calibration/calibrator/StateCarrier.h>
#include <aslam/calibration/error-terms/ErrorTermGroup.h>
#include <aslam/calibration/tools/ErrorTermStatistics.h>

using std::chrono::system_clock;

namespace aslam {
namespace calibration {

template class ModuleLink<Sensor>;

// TODO C move CalibratorI::getModelAt to a more suitable place
aslam::calibration::ModelAtTime CalibratorI::getModelAt(const Sensor& sensor, Timestamp time, int maximalDerivativeOrder, const ModelSimplification& simplification) const {
  return sensor.hasDelay() ? getModelAt(sensor.getBoundedTimestampExpression(*this, time), maximalDerivativeOrder, simplification) : getModelAt(time, maximalDerivativeOrder, simplification);
}

AbstractCalibratorOptions::AbstractCalibratorOptions(const sm::value_store::ValueStoreRef& config) :
    predictResults(config.getBool("predictResults", true)),
    verbose(config.getBool("verbose", false)),
    acceptConstantErrorTerms(config.getBool("acceptConstantErrorTerms", true)),
    splineOutputSamplePeriod(config.getDouble("splineOutputSamplePeriod", 0.01)),
    numThreads(config.getInt("numThreads", 1))
{
  if(acceptConstantErrorTerms){
    LOG(INFO)<< "Using acceptConstantErrorTerms = true";
  }
}

AbstractCalibratorOptions::~AbstractCalibratorOptions(){
}

AbstractCalibrator::AbstractCalibrator(ValueStoreRef config, std::shared_ptr<Model> model,
                                       bool timeBaseSensorRequired) :
  _timeBaseSensor("Calibrator", config, "timeBaseSensor", timeBaseSensorRequired),
  _modelSP(model),
  _model(*model),
  _config(config)
{
  _timeBaseSensor.resolve(_model);

  // Sanity checks
  if(_timeBaseSensor.isResolved()){
    CHECK(_timeBaseSensor.get().isUsed()) << "Time base sensor (" << _timeBaseSensor.get() << ") is not used!";
    CHECK(!_timeBaseSensor.get().hasDelay()) << "Time base sensor (" << _timeBaseSensor.get() << ") with delay isn't supported!";
  }
}


bool AbstractCalibrator::initStates(){
  for(Module & m : getModel().getModules()){
    LOG(INFO) << "Initializing module " << m.getName() << "'s state.";
    if(!m.initState(*this)){
      LOG(ERROR) << "Module " << m.getName() << " failed to initialize its state. Going to abort this window.";
      return false;
    }
  }
  return true;
}

void AbstractCalibrator::setUpdateHandler(StatusUpdateHandler statusUpdateHandler, CalibrationUpdateHandler calibrationUpdateHandler) {
  _statusUpdateHandler = statusUpdateHandler;
  _calibrationUpdateHandler = calibrationUpdateHandler;
}

void AbstractCalibrator::setLowestTimestamp(Timestamp lowestTimestamp) {
  if(!(!_lowesTimestampProvided || lowestTimestamp == _lowestTimestamp)){
    LOG(WARNING) << "Lowest timestamp provided twice!";
  }
  _lowestTimestamp = lowestTimestamp;
  _lowesTimestampProvided = true;
}

std::shared_ptr<PredictionFunctorWriter> AbstractCalibrator::createPredictionCollector(const std::string & name){
  auto pd = std::make_shared<PredictionFunctorWriter>(name);
  _predictionData.push_back(pd);
  return pd;
}

void AbstractCalibrator::addToArchive(sm::MatrixArchive & archive, bool append) const {
  for(const auto & c: getModel().getCalibrationVariables()){
    Eigen::VectorXd v = c->getMinimalComponents();
    if(!append){
      sm::MatrixArchive m;
      archive.setMatrix(c->getName(), v);
    }else{
      Eigen::MatrixXd & old = archive.getMatrix(c->getName());
      Eigen::MatrixXd M = old;
      old.resize(v.rows(), M.cols() + 1);
      old << M, v;
    }
  }
}

void AbstractCalibrator::loadFromArchive(sm::MatrixArchive & archive, int columnIndex) {
  for(const auto & c: getModel().getCalibrationVariables()){
    auto m = archive.find(c->getName());
    if(m != archive.end()){
      assert(m->second.cols() > 0);
      assert(columnIndex < m->second.cols());
      if(columnIndex < 0){
        columnIndex = m->second.cols() - 1;
      }
      Eigen::MatrixXd M = m->second.col(columnIndex);
      LOG(INFO) << "Loading into c->getName(): " << M.transpose().eval();
      c->setMinimalComponents(M);
    }
  }
}

void AbstractCalibrator::setCalibrationVariablesActivity(const CalibrationConfI& ec) {
  using sm::timing::Timer;
  Timer timerSetActive("Calibrator: SetActive");
  for (Module& m : getModel().getModules()) {
    m.setCalibrationActive(ec);
  }
}


struct ValueObserver {
  friend std::ostream & operator << (std::ostream & out, const ValueObserver & obs){
    out << obs.currentValue;
    if (obs.diff) out << " (" << (obs.diff > 0 ? "+": "") << obs.diff << ")";
    return out;
  }

  ValueObserver & update(int newValue){
    diff = newValue - currentValue;
    currentValue = newValue;
    return *this;
  }

  int getCurrentValue() const {
    return currentValue;
  }

  int getDiff() const {
    return diff;
  }

 private:
  int currentValue = 0, diff = 0;
};


double AbstractCalibrator::secsSinceStart(Timestamp timestamp) const {
  return timestamp - Timestamp(_lowestTimestamp);
}
std::string AbstractCalibrator::secsSinceStart(const Interval & interval) const {
  std::stringstream s;
  s << "[" << std::setprecision(8) << secsSinceStart(interval.start) << ", " << std::setprecision(8) << secsSinceStart(interval.end) << "]";
  return s.str();
}

void AbstractCalibrator::addMeasurementTimestamp(const Timestamp lowerBound, Timestamp upperBound) {
  if(upperBound.getNumerator() < 0L) { upperBound = lowerBound; }

  _lastTimestamp = lowerBound;
  if (_lowestTimestamp == InvalidTimestamp() || _lowestTimestamp > lowerBound){
    _lowestTimestamp = lowerBound;
    {
      SM_ASSERT_FALSE(std::runtime_error, _lowesTimestampProvided, "Found even lower timestamp!");
      std::time_t now;
      now = std::chrono::system_clock::to_time_t(sm::timing::nsecToChrono(lowerBound));
      LOG(INFO) << "Start collecting measurements from " << std::ctime(&now) << " (" << std::fixed << static_cast<double>(lowerBound) << ") on.";
    }
  }
}

void AbstractCalibrator::addMeasurementTimestamp(Timestamp t, const Sensor & sensor) {
  addMeasurementTimestamp(t - sensor.getDelayUpperBound(), t - sensor.getDelayLowerBound());
  if(_timeBaseSensor.isResolved() && _timeBaseSensor.get() == sensor){
    handleNewTimeBaseTimestamp(t);
  }
}

void AbstractCalibrator::addFactors(const CalibrationConfI& estimationConfig, ErrorTermReceiver & problem, std::function<void()> statusCallback) {
  for(Module & m : getModel().getModules()){
    LOG(INFO) << "Adding module " << m.getName() << "'s error terms.";
    m.addErrorTerms(*this, getCurrentStorage(), estimationConfig, problem);
    statusCallback();
  }
}

ValueStoreRef AbstractCalibrator::getValueStore() const {
  return _config;
}

void AbstractCalibrator::clearAfterEstimation() {
}


const char* getActivityPrefix(const CalibrationVariable& cv);

void AbstractCalibrator::printBatchErrorTermStatistics(const CalibrationProblem& batch, bool updateError, std::ostream& out) {
  sm::timing::Timer t("printBatchErrorTermStatistics");
  //TODO B order by error!
  out << "Error term statistics:" << std::endl;
  std::map<std::reference_wrapper<const ErrorTermGroup>, ErrorTermStatistics> etgs;
  for (auto et : batch.getErrorTerms()) {
    const aslam::calibration::ErrorTermGroup& errorTermGroup = getErrorTermGroup(*et);
    auto i = etgs.emplace(std::reference_wrapper<const ErrorTermGroup>(errorTermGroup), ErrorTermStatistics{errorTermGroup.getName(), updateError});
    i.first->second.add(*et);
  }
  double total = 0;
  for (auto etg : etgs) {
    total += etg.second.getCost();
    out << etg.second << std::endl;
  }

  out << "Overall:" << total << std::endl;

  out << "Cv-wise error term statistics:" << std::endl;
  std::set<aslam::backend::ErrorTerm*> ets;
  for (auto cv : _model.getCalibrationVariables()) {
    ets.clear();
    batch.getErrors(&cv->getDesignVariable(), ets);
    if(ets.empty()) continue;
    etgs.clear();
    out << getActivityPrefix(*cv) << std::setfill(' ') << std::setw(CalibrationVariable::NameWidth) << cv->getName();
    out << " : #errors:" << ets.size();
    for (auto et : ets) {
      const aslam::calibration::ErrorTermGroup& errorTermGroup = getErrorTermGroup(*et);
      auto i = etgs.emplace(std::reference_wrapper<const ErrorTermGroup>(errorTermGroup), ErrorTermStatistics{errorTermGroup.getName(), updateError});
      i.first->second.add(*et);
    }
    for (auto etg : etgs) {
      out << " " << etg.second;
    }
    out << std::endl;
  }
}

void AbstractCalibrator::updateOptimizerInspector(const CalibrationProblem &  currentBatch, bool printRegessionErrorStatistics, std::function<void(std::ostream & o)> printOptimizationState, backend::callback::Registry & callbackRegistry) {
  callbackRegistry.clear();
  callbackRegistry.add<aslam::backend::callback::event::LINEAR_SYSTEM_SOLVED>([this, printOptimizationState]() {
      if(getOptions().getVerbose()){
        printOptimizationState(LOG(INFO) << "Optimizer: Linear system solved:\n");
      }
    });
  callbackRegistry.add<aslam::backend::callback::event::COST_UPDATED>([this, &currentBatch, printRegessionErrorStatistics](const aslam::backend::callback::event::COST_UPDATED & a)
    {
      const bool wasRegression = a.previousLowestCost > 0  && a.previousLowestCost < a.currentCost;
      if(wasRegression) {
        LOG(WARNING) << "Last update was a regression: " <<  a.previousLowestCost << " -> " << a.currentCost;
      }
      if(getOptions().getVerbose() && (printRegessionErrorStatistics || !wasRegression)){
        if(a.previousLowestCost < 0) // is initial update
        {
          printBatchErrorTermStatistics(currentBatch, false, LOG(INFO) << "Optimizer: initial cost = " << a.currentCost << ":\n");
        } else {
          printBatchErrorTermStatistics(currentBatch, false, LOG(INFO) << "Optimizer: cost and residuals updated. Current cost: " << a.currentCost << " (decreased by " << (a.previousLowestCost - a.currentCost) << (wasRegression ? " REGRESSION!" : "") << "):\n");
        }
      }
    });
  callbackRegistry.add<aslam::backend::callback::event::DESIGN_VARIABLES_UPDATED>([this]() {
      if(getOptions().getVerbose()){
        _model.printCalibrationVariables(LOG(INFO) << "Optimizer: Variables updated:" << std::endl);
      }
      if(_calibrationUpdateHandler){
        _calibrationUpdateHandler();
      }
    });
}

void AbstractCalibrator::estimate(const CalibrationConfI & estimationConfig, CalibrationProblem & problem, BatchStateReceiver & batchStateReceiver, std::function<void()> optimize) {
  using sm::timing::Timer;

  {
    Timer timer("Calibrator: clearPredictionData");
    _predictionData.clear();
  }

  estimationConfig.print(LOG(INFO) << "Optimizing");
  if(_statusUpdateHandler){
    std::stringstream ss;
    estimationConfig.print(ss);
    _statusUpdateHandler(ss.str());
  }

  setCalibrationVariablesActivity(estimationConfig);

  ValueObserver dimCalibObserver, dimStateObserver, numErrorTermsObserver;

  auto logGroupDimsAndErrorNum = [&](){
    dimCalibObserver.update(problem.getDimCalibrationVariables());
    dimStateObserver.update(problem.getDimStateVariables());
    numErrorTermsObserver.update(problem.getNumErrorTerms());

    if(dimCalibObserver.getDiff() || dimStateObserver.getDiff() || numErrorTermsObserver.getDiff()){
      LOG(INFO)
          << "\ndim(Calib)="<< dimCalibObserver
          << "\ndim(State)="<< dimStateObserver
          << "\n#errorTerms="<< numErrorTermsObserver;
    }
  };

  logGroupDimsAndErrorNum();

  {
    Timer timer("Calibrator: AddDvsToProblem");

    LOG(INFO) << "adding new batch.";
    getModel().addToBatch([&](CalibrationVariable * c){
      LOG(INFO) << "Adding calibration variable " << c->getName() << " (dim=" << c->getDimension() << ", active="<< c->isActivated() << ")";
      problem.addCalibrationVariable(c);
    });

    logGroupDimsAndErrorNum();

    if(estimationConfig.getUseCalibPriors()){
      getModel().addCalibPriors(problem);
      logGroupDimsAndErrorNum();
    }

    auto stateVariableReceiver = createFunctorDesignVariableReceiver([&](backend::DesignVariable* dv) {
          problem.addStateVariable(dv);
        });

    for(Module & m : getModel().getModules()){
      if(m.isUsed()){
        LOG(INFO) << "Adding module " << m.getName() << "'s state.";
        m.addToBatch(estimationConfig.getStateActivator(), batchStateReceiver, stateVariableReceiver);
        logGroupDimsAndErrorNum();
      }
    }

    getModel().updateCVIndices();
  }

  {
    Timer timer("Calibrator: Create factors");
    addFactors(estimationConfig, problem, logGroupDimsAndErrorNum);
  }

  if(dimCalibObserver.getCurrentValue() + dimStateObserver.getCurrentValue() == 0){
    LOG(WARNING) << "Not estimating because there are no variables to optimize!";
  } else if (numErrorTermsObserver.getCurrentValue() == 0){
    LOG(WARNING) << "Not estimating because there are no error terms!";
  } else {
    optimize();
  }

  clearAfterEstimation();
}

} /* namespace calibration */
} /* namespace aslam */
