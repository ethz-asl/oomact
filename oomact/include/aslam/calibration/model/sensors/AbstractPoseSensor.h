#ifndef H224E4723_D72A_4715_947A_F81E0A1714EB
#define H224E4723_D72A_4715_947A_F81E0A1714EB

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

  virtual PoseMeasurements getMeasurements(Timestamp from, Timestamp till) const override;
};

} /* namespace calibration */
} /* namespace aslam */

#endif /* H224E4723_D72A_4715_947A_F81E0A1714EB */
