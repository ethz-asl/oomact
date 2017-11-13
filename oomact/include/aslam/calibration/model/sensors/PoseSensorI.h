#ifndef H3B9CD3B6_B657_40D3_8741_16FE63E05C0F
#define H3B9CD3B6_B657_40D3_8741_16FE63E05C0F

#include <aslam/calibration/model/Model.h>
#include <aslam/calibration/model/Module.h>
#include "../../Timestamp.h"

namespace aslam {
namespace calibration {

struct PoseMeasurement;
template <typename T> struct MeasurementsContainer;
typedef MeasurementsContainer<PoseMeasurement> PoseMeasurements;

class Sensor;

class PoseSensorI {
 public:
  virtual ~PoseSensorI(){}

  virtual bool hasMeasurements(const ModuleStorage & storage) const = 0;
  virtual const PoseMeasurements & getAllMeasurements(const ModuleStorage & storage) const = 0;

  virtual const Frame & getTargetFrame() const = 0;

  virtual Sensor & getSensor() = 0;
  virtual const Sensor & getSensor() const = 0;
};

}
}

#endif /* H3B9CD3B6_B657_40D3_8741_16FE63E05C0F */
