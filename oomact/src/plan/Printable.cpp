#include <aslam/calibration/tools/Printable.h>

#include <sstream>

namespace aslam {
namespace calibration {
std::string aslam::calibration::Printable::toString() const {
  std::stringstream ss;
  ss << *this;
  return ss.str();
}

} /* namespace calibration */
} /* namespace aslam */
