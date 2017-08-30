#ifndef HB94494AD_043E_41B9_B842_1BDA520FD393
#define HB94494AD_043E_41B9_B842_1BDA520FD393

#include "InputFeederFactoryI.h"

namespace aslam {
namespace calibration {
namespace ros {

template <typename Msg, typename Measurement>
class InputFeederForMeassagesWithHeader : public InputFeederImpl<Msg, InputReceiverIT<Measurement>, InputFeederForMeassagesWithHeader<Msg, Measurement>> {
 public:
  using InputFeederImpl<Msg, InputReceiverIT<Measurement>, InputFeederForMeassagesWithHeader<Msg, Measurement>>::InputFeederImpl;
  virtual ~InputFeederForMeassagesWithHeader() = default;

  Timestamp feed(const Msg & m, const InputReceiverIT<Measurement> & receiver, ObservationManagerI & obsManager) const {
    Measurement measurement;
    bool msg2Measurement(const Msg & m, Measurement&);
    if(!msg2Measurement(m, measurement)){
      return InvalidTimestamp();
    }
    const auto t = getTimestampFromHeader(m);
    receiver.addInputTo(t, measurement, obsManager.getCurrentStorage());
    return t;
  }
};

template <typename Msg, typename Measurement>
class InputFeederFactoryForMessageWithHeader : public InputFeederFactoryIT<InputReceiverIT<Measurement>, InputFeederForMeassagesWithHeader<Msg, Measurement>, InputFeederFactoryForMessageWithHeader<Msg, Measurement>> {
};

} /* namespace ros */
} /* namespace calibration */
} /* namespace aslam */


#endif /* HB94494AD_043E_41B9_B842_1BDA520FD393 */
