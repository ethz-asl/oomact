#ifndef H224E4723_D72A_4715_947A_F81E0A1714EB
#define H224E4723_D72A_4715_947A_F81E0A1714EB

#include "../../data/StorageI.h"
#include "../Sensor.hpp"
#include "PoseSensorI.hpp"

namespace aslam {
namespace calibration {

class AbstractPoseSensor : public PoseSensorI, public Sensor {
 public:
  AbstractPoseSensor(Model& model, std::string name, sm::value_store::ValueStoreRef config);
  virtual ~AbstractPoseSensor();

  AbstractPoseSensor & getSensor() override {
    return *this;
  }
  const AbstractPoseSensor & getSensor() const override {
    return *this;
  }

  virtual bool hasMeasurements(ConstStorage & storage) const override;
  virtual const PoseMeasurements & getAllMeasurements(ConstStorage & storage) const override;

 protected:
  PoseMeasurements & getAllMeasurements(Storage & storage) const;
 private:
  Storage::Connector<PoseMeasurements> storageConnector_;
};

} /* namespace calibration */
} /* namespace aslam */

#endif /* H224E4723_D72A_4715_947A_F81E0A1714EB */
