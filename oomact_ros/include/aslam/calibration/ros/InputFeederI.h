#ifndef H7DDF1E87_4711_4319_AFFE_235B2CB0DF76
#define H7DDF1E87_4711_4319_AFFE_235B2CB0DF76

#include <aslam/calibration/data/ObservationManagerI.h>
#include <rosbag/message_instance.h>

namespace aslam {
namespace calibration {
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
    static_cast<const Derived*>(this)->feed(*m.instantiate<Msg>(), receiver_, obsManager);
  }
 private:
  const Receiver & receiver_;
};

} /* namespace ros */
} /* namespace calibration */
} /* namespace aslam */

#endif /* H7DDF1E87_4711_4319_AFFE_235B2CB0DF76 */
