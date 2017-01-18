#ifndef H11CD0922_188C_41DE_97F7_CE130E1CD440
#define H11CD0922_188C_41DE_97F7_CE130E1CD440

#include <memory>
#include <string>

#include <sm/value_store/ValueStore.hpp>

#include <aslam/calibration/CommonTypes.hpp>
#include <aslam/calibration/model/Model.h>
#include <aslam/calibration/tools/Interval.hpp>

namespace sm {
  class MatrixArchive;
}

namespace aslam {
namespace calibration {
using sm::value_store::ValueStoreRef;

class Model;
class CalibrationVariable;
class CalibratorOptions;
class MotionCaptureSource;
class ModuleList;
class PredictionFunctorWriter;

typedef std::function<void()> CalibrationUpdateHandler;
typedef std::function<void(std::string)> StatusUpdateHandler;

class CalibratorOptionsI {
 public:
  virtual ~CalibratorOptionsI(){}
  virtual double getSplineOutputSamplePeriod() const = 0;
  virtual bool getPredictResults() const = 0;
  virtual bool getAcceptConstantErrorTerms() const = 0;
};

class CalibratorI {
 public:
  virtual ~CalibratorI(){}

  virtual Model & getModel() = 0;
  virtual const Model & getModel() const = 0;

  virtual void setUpdateHandler(StatusUpdateHandler statusUpdateHandler, CalibrationUpdateHandler calibrationUpdateHandler) = 0;

  virtual void addToArchive(sm::MatrixArchive & ma, bool append = false) const = 0;
  virtual void loadFromArchive(sm::MatrixArchive & ma, int index = -1) = 0;

  virtual void setLowestTimestamp(Timestamp lowestTimeStamp) = 0;

  virtual const CalibratorOptionsI & getOptions() const = 0;

  //TODO C sort functions into other interfaces, such as calibrator state

  virtual void addMeasurementTimestamp(Timestamp t, const Sensor & sensor) = 0;

  virtual std::shared_ptr<PredictionFunctorWriter> createPredictionCollector(const std::string & name) = 0; //TODO make private again and only expose via special interface available during addMeasurements ..

  virtual double secsSinceStart(Timestamp timestamp) const = 0;
  virtual std::string secsSinceStart(const Interval & interval) const = 0;

  virtual const Interval& getCurrentEffectiveBatchInterval() const = 0;

  template <typename Time>
  ModelAtTime getModelAt(Time time, int maximalDerivativeOrder, const ModelSimplification & simplification) const{
    return getModel().getAtTime(time, maximalDerivativeOrder, simplification);
  }

  ModelAtTime getModelAt(const BoundedTimeExpression & time, int maximalDerivativeOrder, const ModelSimplification & simplification) const{
    return getModelAt<const BoundedTimeExpression &>(time, maximalDerivativeOrder, simplification);
  }

  ModelAtTime getModelAt(const Sensor& sensor, Timestamp time, int maximalDerivativeOrder, const ModelSimplification& simplification) const;
};

class BatchCalibratorI :public virtual CalibratorI {
 public:
  virtual ~BatchCalibratorI(){}

  virtual void calibrate() = 0;
};

class IncrementalCalibratorI : public virtual CalibratorI {
 public:
  typedef std::function<void(IncrementalCalibratorI & calibrator)> WindowFullHandler;

  virtual ~IncrementalCalibratorI(){}
  virtual void setWindowFullHandler(WindowFullHandler handler) = 0;

  virtual void addMeasurementsAsNewBatch() = 0;

  virtual void skipLastWindow() = 0;

  /// invoke window-full handler if window is full. Returns true iff it it was full.
  virtual bool invokeWindowFullHanlderIfNecessary(bool acceptShortWindow = false) = 0;

  virtual void startCollectingDataFrom(const ModuleList & moduleList) = 0;
  virtual void stopCollectingDataFrom(const ModuleList & moduleList) = 0;
  virtual size_t getNumModulesDataCollecting() const = 0;

  virtual void enableIcpInspection(bool active, const std::string & inspectionOutputFolder) = 0; //TODO Move to ICP related Module!
};

std::unique_ptr<IncrementalCalibratorI> createIncrementalCalibrator(ValueStoreRef vs, std::shared_ptr<Model> model);
std::unique_ptr<BatchCalibratorI> createBatchCalibrator(ValueStoreRef vs, std::shared_ptr<Model> model);
}
}

#endif /* H11CD0922_188C_41DE_97F7_CE130E1CD440 */
