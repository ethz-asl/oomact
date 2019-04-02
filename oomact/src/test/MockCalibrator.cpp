#include <aslam/calibration/calibrator/AbstractCalibrator.h>
#include <aslam/calibration/calibrator/CalibratorI.h>
#include <aslam/calibration/calibrator/SimpleModuleStorage.h>
#include <aslam/calibration/test/MockCalibrator.h>

using aslam::calibration::MockCalibrator;


namespace aslam {
namespace calibration {

MockCalibrator::MockCalibrator(Model& model, Interval initialInverval)
    : AbstractCalibrator(model.getConfig(), std::shared_ptr<Model>(&model, sm::null_deleter()),
                         false),
      storage_(*this)
{
  _currentEffectiveBatchInterval = initialInverval;
  LOG(INFO) << "Initialized mock calibrator with batch interval = ["
      << static_cast<double>(_currentEffectiveBatchInterval.start) << " s, "
      << static_cast<double>(_currentEffectiveBatchInterval.end) << " s]";

}

ModuleStorage& MockCalibrator::getCurrentStorage()
{
  return storage_;
}
const ModuleStorage & MockCalibrator::getCurrentStorage() const
{
  return storage_;
 }
bool MockCalibrator::isNextWindowScheduled() const
{
  throw std::runtime_error(std::string(__func__) + " not implemented!");
}
Timestamp MockCalibrator::getNextTimeWindowStartTimestamp() const
{
  throw std::runtime_error(std::string(__func__) + " not implemented!");
}
bool MockCalibrator::isMeasurementRelevant(const Sensor &, Timestamp) const
{
  throw std::runtime_error(std::string(__func__) + " not implemented!");
}
const CalibratorOptionsI & MockCalibrator::getOptions() const
{
  throw std::runtime_error(std::string(__func__) + " not implemented!");
}
bool MockCalibrator::handleNewTimeBaseTimestamp(Timestamp)
{
  throw std::runtime_error(std::string(__func__) + " not implemented!");
}

} /* namespace calibration */
} /* namespace aslam */
