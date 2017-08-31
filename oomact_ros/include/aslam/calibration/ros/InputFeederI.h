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

} /* namespace ros */
} /* namespace calibration */
} /* namespace aslam */

#endif /* H7DDF1E87_4711_4319_AFFE_235B2CB0DF76 */
