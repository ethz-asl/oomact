#ifndef HB94494AD_043E_41B9_B842_1BDA520FD393
#define HB94494AD_043E_41B9_B842_1BDA520FD393

#include <stdexcept>
#include "InputFeederFactoryI.h"

namespace aslam {
namespace calibration {
namespace ros {

template <typename Msg, typename Receiver, typename Derived>
class InputFeederImpl : public InputFeederI {
 public:
  InputFeederImpl(const Receiver & receiver) : receiver_(receiver) {}
  virtual ~InputFeederImpl() = default;

  virtual void feed(const ::rosbag::MessageInstance & m, ObservationManagerI & obsManager) const override final {
    auto mInst = m.instantiate<Msg>();
    if(mInst){ //TODO O : gain efficiency by check out the type of a topic in a bag first and then don't even instantiate the other feeders
      const Timestamp t = getDerived().feedMeasurementAndReturnTimestamp(*mInst, getReceiver(), obsManager);
      if(t != InvalidTimestamp()){
        const Sensor & sensor = getSensorFromReceiver(getReceiver());
        VLOG(3) << "Feeding measurement to " << getNameFromSensor(sensor) << " at t=" << obsManager.secsSinceStart(t) << " secs.";
        obsManager.addMeasurementTimestamp(t, sensor);
      }
    }
  }

  /**
   * Feed the measurement from the message to the receiver.
   * @param m Measurement message
   * @param receiver
   * @param obsManager
   * @return timestamp used for measurement (In receivers time frame). If InvalidTimestamp() is returned the message is assumed skipped.
   */
  Timestamp feedMeasurementAndReturnTimestamp(const Msg & m, const Receiver & receiver, ObservationManagerI & obsManager) const {
    throw std::runtime_error(std::string("Class ") + typeid(Derived).name() + " must implement (shadow) feedMeasurementAndReturnTimestamp.");
    return InvalidTimestamp();
  }


  const Receiver& getReceiver() const {
    return receiver_;
  }
 protected:
  const Derived& getDerived() const {
    return static_cast<const Derived&>(*this);
  }
 private:
  const Receiver & receiver_;
};

template <typename Msg, typename Measurement>
class InputFeederForMeassagesWithHeader : public InputFeederImpl<Msg, InputReceiverIT<Measurement>, InputFeederForMeassagesWithHeader<Msg, Measurement>> {
 public:
  using InputFeederImpl<Msg, InputReceiverIT<Measurement>, InputFeederForMeassagesWithHeader<Msg, Measurement>>::InputFeederImpl;
  virtual ~InputFeederForMeassagesWithHeader() = default;

  template <typename Receiver>
  Timestamp feedMeasurementAndReturnTimestamp(const Msg & m, const Receiver & receiver, ObservationManagerI & obsManager) const {
    Measurement measurement;
    bool msg2Measurement(const Msg & m, Measurement&);
    if(!msg2Measurement(m, measurement)){
      return InvalidTimestamp();
    }
    const auto t = getTimestampFromHeader(m);
    receiver.addInputTo(t, measurement, obsManager.getCurrentStorage());
    return t;
  }

  virtual void print(std::ostream & o) const {
    o << "RosInputFeeder(" << typeid(Msg).name() << " -> "<< this->getReceiver().getModule().getName() << "." << typeid(Measurement).name() << ")";
  }
};

template <typename Msg, typename Measurement>
class InputFeederFactoryForMessageWithHeader : public InputFeederFactoryIT<InputReceiverIT<Measurement>, InputFeederForMeassagesWithHeader<Msg, Measurement>, InputFeederFactoryForMessageWithHeader<Msg, Measurement>> {
};

} /* namespace ros */
} /* namespace calibration */
} /* namespace aslam */


#endif /* HB94494AD_043E_41B9_B842_1BDA520FD393 */
