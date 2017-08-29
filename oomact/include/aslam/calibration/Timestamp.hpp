#ifndef H976A3F3F_F06D_477A_B9BA_5BA9EE04EC2F
#define H976A3F3F_F06D_477A_B9BA_5BA9EE04EC2F

#include <aslam/backend/FixedPointNumber.hpp>
#include <cstdint>

namespace aslam {
namespace calibration {
  /// Time type
  typedef std::int64_t NsecTime;
  typedef aslam::backend::FixedPointNumber<NsecTime, std::uintmax_t(1e9)> Timestamp;
  typedef Timestamp Duration;
  inline constexpr Timestamp InvalidTimestamp() { return Timestamp::Numerator(-1); }
}
}

#endif /* H976A3F3F_F06D_477A_B9BA_5BA9EE04EC2F */
