#ifndef H7DDF1E87_4711_4319_AFFE_235B2CB0DF76
#define H7DDF1E87_4711_4319_AFFE_235B2CB0DF76

#include <aslam/calibration/data/ObservationManagerI.h>
#include <rosbag/message_instance.h>

#include "internal/Tools.h"
namespace aslam {
namespace calibration {

class Sensor;

namespace ros {


class InputFeederI {
 public:
  InputFeederI();
  virtual ~InputFeederI();

  virtual void feed(const ::rosbag::MessageInstance & m, ObservationManagerI & obsManager) const = 0;
};

template <typename Msg, typename Receiver, typename Derived>
class InputFeederImpl : public InputFeederI {
 public:
  InputFeederImpl(const Receiver & receiver) : receiver_(receiver) {}
  virtual ~InputFeederImpl() = default;

  virtual void feed(const ::rosbag::MessageInstance & m, ObservationManagerI & obsManager) const override {
    auto mInst = m.instantiate<Msg>();
    if(mInst){ //TODO O : gain efficiency by check out the type of a topic in a bag first and then don't even instantiate the other feeders
      const Timestamp t = static_cast<const Derived*>(this)->feed(*mInst, receiver_, obsManager);
      if(t != InvalidTimestamp()){
        const Sensor & sensor = getSensorFromReceiver(receiver_);
        VLOG(3) << "Feeding measurement to " << getNameFromSensor(sensor) << " at t=" << obsManager.secsSinceStart(t) << " secs.";
        obsManager.addMeasurementTimestamp(t, sensor);
      }
    }
  }
 private:
  const Receiver & receiver_;
};

} /* namespace ros */
} /* namespace calibration */
} /* namespace aslam */

#endif /* H7DDF1E87_4711_4319_AFFE_235B2CB0DF76 */
