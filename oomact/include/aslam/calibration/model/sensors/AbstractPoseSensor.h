#ifndef H224E4723_D72A_4715_947A_F81E0A1714EB
#define H224E4723_D72A_4715_947A_F81E0A1714EB

#include "../../data/StorageI.h"
#include "../Sensor.h"
#include "PoseSensorI.h"

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

  virtual bool hasMeasurements(const ModuleStorage & storage) const override;
  virtual const PoseMeasurements & getAllMeasurements(const ModuleStorage & storage) const override;

  bool isInvertInput() const {
    return invertInput_;
  }

  const Covariance& getCovOrientation() const {
    return covOrientation_;
  }

  const Covariance& getCovPosition() const {
    return covPosition_;
  }
 protected:
  PoseMeasurements & getMeasurementsMutable(ModuleStorage & storage) const;

  void writeConfig(std::ostream & out) const override;
 private:
  ModuleStorage::Connector<PoseMeasurements> storageConnector_;
  const bool invertInput_;
  Covariance covPosition_, covOrientation_; // One noise model for all measurements that do not provide individual noise model
};

} /* namespace calibration */
} /* namespace aslam */

#endif /* H224E4723_D72A_4715_947A_F81E0A1714EB */
