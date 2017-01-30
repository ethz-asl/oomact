#ifndef H3B9CD3B6_B657_40D3_8741_16FE63E05C0F
#define H3B9CD3B6_B657_40D3_8741_16FE63E05C0F

#include <aslam/calibration/CommonTypes.hpp>

namespace aslam {
namespace calibration {

struct PoseMeasurement;
template <typename T> struct MeasurementsContainer;
typedef MeasurementsContainer<PoseMeasurement> PoseMeasurements;

class Sensor;

class PoseSensorI {
 public:
  virtual ~PoseSensorI(){}

  virtual bool hasMeasurements() const = 0;
  virtual const PoseMeasurements & getMeasurements() const = 0;
  virtual PoseMeasurements getMeasurements(Timestamp from, Timestamp till) const = 0;

  virtual Sensor & getSensor() = 0;
  virtual const Sensor & getSensor() const = 0;
};

}
}

#endif /* H3B9CD3B6_B657_40D3_8741_16FE63E05C0F */
