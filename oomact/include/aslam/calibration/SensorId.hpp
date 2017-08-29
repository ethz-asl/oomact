#ifndef EUROPA_CALIBRATION_COMMON_TYPES_H_
#define EUROPA_CALIBRATION_COMMON_TYPES_H_

#include <iosfwd>
#include <functional>
#include <limits>

namespace aslam {
namespace calibration {
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

template <>
struct hash < aslam::calibration::SensorId >{
public :
    size_t operator()(const aslam::calibration::SensorId & sensorId) const;
};

}

#endif /* EUROPA_CALIBRATION_COMMON_TYPES_H_ */
