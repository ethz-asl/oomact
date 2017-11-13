#ifndef H579F5188_F15B_47A4_B00E_B16F5109EE17
#define H579F5188_F15B_47A4_B00E_B16F5109EE17

#include <string>
#include <aslam/calibration/model/Module.h>

#include "../Timestamp.h"

namespace aslam {
namespace calibration {

struct Interval;
class Sensor;

class ObservationManagerI {
 public:
  ObservationManagerI();
  virtual ~ObservationManagerI();

  virtual ModuleStorage & getCurrentStorage() = 0;
  virtual const ModuleStorage & getCurrentStorage() const = 0;

  virtual void setLowestTimestamp(Timestamp lowestTimeStamp) = 0;
  virtual void addMeasurementTimestamp(Timestamp t, const Sensor & sensor) = 0;

  virtual bool isNextWindowScheduled() const = 0;
  virtual Timestamp getNextTimeWindowStartTimestamp() const = 0;
  virtual bool isMeasurementRelevant(const Sensor & s, Timestamp t) const = 0;

  virtual double secsSinceStart(Timestamp timestamp) const = 0;
  virtual std::string secsSinceStart(const Interval & interval) const = 0;

  virtual const Interval& getCurrentEffectiveBatchInterval() const = 0;
};

} /* namespace calibration */
} /* namespace aslam */

#endif /* H579F5188_F15B_47A4_B00E_B16F5109EE17 */
