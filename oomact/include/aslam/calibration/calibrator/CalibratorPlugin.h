#ifndef HA0283384_8D04_4319_BE62_1F66D8BADFBF
#define HA0283384_8D04_4319_BE62_1F66D8BADFBF
#include <aslam/calibration/tools/PluginI.h>

namespace aslam {
namespace calibration {
class CalibratorI;
class Model;

class CalibratorPlugin : public PluginI {
 public:
  CalibratorPlugin(CalibratorI & calibrator);
  virtual ~CalibratorPlugin();

  CalibratorI& getCalibrator() {
    return calibrator_;
  }
  const CalibratorI& getCalibrator() const {
    return calibrator_;
  }

  Model& getModel();
  const Model& getModel() const;

 private:
  CalibratorI & calibrator_;
};

} /* namespace calibration */
} /* namespace aslam */

#endif /* HA0283384_8D04_4319_BE62_1F66D8BADFBF */
