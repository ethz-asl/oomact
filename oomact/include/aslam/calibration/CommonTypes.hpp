#ifndef EUROPA_CALIBRATION_COMMON_TYPES_H_
#define EUROPA_CALIBRATION_COMMON_TYPES_H_

#include <iosfwd>

#include <aslam/backend/FixedPointNumber.hpp>
#include <sm/timing/NsecTimeUtilities.hpp>

namespace aslam {
namespace backend {
template <typename Integer_, std::uintmax_t Divider>
class FixedPointNumber;

}

namespace calibration {
  /// Time type
  typedef aslam::backend::FixedPointNumber<sm::timing::NsecTime, std::uintmax_t(1e9)> Timestamp;
  typedef Timestamp Duration;

  struct SensorId{
    explicit constexpr SensorId(size_t id) : id(id) {};
    SensorId(const SensorId& other) = default;
    SensorId & operator = (const SensorId& other) = default;

    bool operator == (const SensorId& other) const { return id == other.id; }

    bool operator < (const SensorId & other) const {
      return id < other.id;
    }

    size_t getValue() const { return id; }
   private:
    size_t id;
  };

  constexpr SensorId NoSensorId(std::numeric_limits<size_t>::max());

  std::ostream & operator << (std::ostream & o, SensorId id);
}
}

namespace std {
template <typename> struct hash;

template <>
struct hash < aslam::calibration::SensorId >{
public :
    size_t operator()(const aslam::calibration::SensorId & sensorId) const;
};
}


#endif /* EUROPA_CALIBRATION_COMMON_TYPES_H_ */
