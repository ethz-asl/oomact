#ifndef H0D4C3CB3_0E76_475B_B6FB_ABF47A12DED4
#define H0D4C3CB3_0E76_475B_B6FB_ABF47A12DED4
#include <aslam/calibration/calibrator/AbstractCalibrator.h>
#include <aslam/calibration/calibrator/SimpleModuleStorage.h>


namespace aslam {
namespace calibration {

class MockCalibrator: public AbstractCalibrator
{
 public:
  MockCalibrator(Model& model, Interval initialInverval = {0.0, 1.0});

  ModuleStorage& getCurrentStorage() override;

  const ModuleStorage & getCurrentStorage() const override;
  bool isNextWindowScheduled() const override;
  Timestamp getNextTimeWindowStartTimestamp() const override;
  bool isMeasurementRelevant(const Sensor &, Timestamp) const override;
  const CalibratorOptionsI & getOptions() const override;
  bool handleNewTimeBaseTimestamp(Timestamp) override;

  using AbstractCalibrator::initStates;

  SimpleModuleStorage storage_;
};

} /* namespace calibration */
} /* namespace aslam */


#endif /* H0D4C3CB3_0E76_475B_B6FB_ABF47A12DED4 */
