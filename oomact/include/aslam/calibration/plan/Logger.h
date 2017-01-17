#ifndef HDF2DD22F_BF8D_400A_9069_4D5E79D575C4
#define HDF2DD22F_BF8D_400A_9069_4D5E79D575C4

#include <string>

namespace aslam {
namespace calibration {
namespace plan {

class Logger {
 public:
  Logger() {}

  virtual void start(const std::string & destinationFolder) = 0;
  virtual bool waitForLoggerToBecomeReady(long milliSeconds) const = 0;
  virtual void stop() = 0;

  virtual ~Logger() {}
};

} /* namespace plan */
} /* namespace calibration */
} /* namespace aslam */

#endif /* HDF2DD22F_BF8D_400A_9069_4D5E79D575C4 */
