#ifndef H224E4723_D72A_4715_947A_F81E0A1714EB
#define H224E4723_D72A_4715_947A_F81E0A1714EB

#include <aslam/calibration/input/InputReceiverI.h>
#include "../../data/StorageI.h"

namespace aslam {
namespace calibration {

template <typename MeasurementT>
class GenericInputReceiverT : public InputReceiverIT<MeasurementT>{
 public:
  typedef MeasurementContainerI<MeasurementT> Measurements;
  virtual ~GenericInputReceiverT() = default;

  virtual bool hasMeasurements(const ModuleStorage & storage) const override {
    return storageConnector_.hasData(storage);
  }
  virtual const Measurements & getAllMeasurements(const ModuleStorage & storage) const override {
    return storageConnector_.getDataFrom(storage);
  }

 protected:
  Measurements & getMeasurementsMutable(ModuleStorage & storage) const {
    return storageConnector_.getDataFrom(storage);
  }

 private:
  ModuleStorage::Connector<MeasurementT> storageConnector_;
};

} /* namespace calibration */
} /* namespace aslam */

#endif /* H224E4723_D72A_4715_947A_F81E0A1714EB */
