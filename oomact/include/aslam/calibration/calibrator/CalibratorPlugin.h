#ifndef HA0283384_8D04_4319_BE62_1F66D8BADFBF
#define HA0283384_8D04_4319_BE62_1F66D8BADFBF
#include <sm/value_store/ValueStore.hpp>

#include <aslam/calibration/tools/PluginI.h>

using sm::value_store::ValueStoreRef;

namespace aslam {
namespace calibration {
class CalibratorI;
class Model;

class CalibratorPlugin : public PluginI {
 public:
  CalibratorPlugin(CalibratorI & calibrator, sm::ValueStoreRef config);
  virtual ~CalibratorPlugin();

  CalibratorI& getCalibrator() {
    return calibrator_;
  }
  const CalibratorI& getCalibrator() const {
    return calibrator_;
  }

  Model& getModel();
  const Model& getModel() const;

 protected:
  const sm::ValueStoreRef & getConfig() {
    return config_;
  }

 private:
  CalibratorI & calibrator_;
  sm::ValueStoreRef config_;
};

} /* namespace calibration */
} /* namespace aslam */

#endif /* HA0283384_8D04_4319_BE62_1F66D8BADFBF */
