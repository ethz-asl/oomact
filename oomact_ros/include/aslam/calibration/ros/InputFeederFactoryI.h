#ifndef H7CEFE265_0E5D_400C_AA63_FBB5AB8F1C21
#define H7CEFE265_0E5D_400C_AA63_FBB5AB8F1C21

#include <memory>

#include <aslam/calibration/model/Sensor.h>

#include "InputFeederI.h"

namespace aslam {
namespace calibration {
namespace ros {

class InputFeederFactoryI {
 public:
  InputFeederFactoryI();
  virtual ~InputFeederFactoryI();

  virtual bool matches(const Sensor & s) const = 0;

  virtual std::unique_ptr<InputFeederI> createFeeder(const Sensor & s) const = 0;
};

template <typename SensorT, typename Feeder, typename Derived>
class InputFeederFactoryIT : public InputFeederFactoryI {
 public:
  InputFeederFactoryIT() {
    static_assert(std::is_convertible<Derived, InputFeederFactoryIT>::value, "");
  }
  virtual ~InputFeederFactoryIT() = default;

  virtual bool matches(const Sensor & s) const override {
    auto ptr = s.ptrAs<SensorT>();
    return ptr && getDerived().matchesT(*ptr);
  }

  bool matchesT(const SensorT &) const {
    return true;
  }

  virtual std::unique_ptr<InputFeederI> createFeeder(const Sensor & s) const override {
    return getDerived().createFeederT(s.as<const SensorT>());
  }

  std::unique_ptr<InputFeederI> createFeederT(const SensorT & s) const {
    return std::unique_ptr<InputFeederI>(new Feeder(s));
  }
 private:
  const Derived & getDerived() const {
    return static_cast<const Derived &>(*this);
  }
};

} /* namespace ros */
} /* namespace calibration */
} /* namespace aslam */

#endif /* H7CEFE265_0E5D_400C_AA63_FBB5AB8F1C21 */
