#ifndef H5CC08D7E_8450_473B_B779_89023C21D91E
#define H5CC08D7E_8450_473B_B779_89023C21D91E

#include <aslam/calibration/calibrator/CalibrationConfI.h>
#include <aslam/calibration/model/fragments/PoseCv.h>
#include <aslam/calibration/model/Module.h>

namespace aslam {
namespace calibration {

class CrossPoseCvPrior : public Module, public PoseCv, public Activatable {
 public:
  CrossPoseCvPrior(Model & model, std::string name, sm::value_store::ValueStoreRef config, PoseCv & from, PoseCv & to);

  void addMeasurementErrorTerms(CalibratorI & calib, const CalibrationConfI & ec, ErrorTermReceiver & problem, bool observeOnly) const override;

  virtual ~CrossPoseCvPrior();
 protected:
  void writeConfig(std::ostream & out) const override;
 private:
  PoseCv & from, & to;
};

} /* namespace calibration */
} /* namespace aslam */

#endif /* H5CC08D7E_8450_473B_B779_89023C21D91E */
