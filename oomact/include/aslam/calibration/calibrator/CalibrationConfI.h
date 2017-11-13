#ifndef H2FC5F3AD_D310_4A93_B2E5_55925AA5F4DF
#define H2FC5F3AD_D310_4A93_B2E5_55925AA5F4DF
#include <aslam/calibration/model/Module.h>
#include "../tools/Printable.h"

namespace aslam {
namespace calibration {

class Activatable : public virtual Named {
 public:
  bool operator == (const Activatable & other) const { return this == & other; }
  virtual ~Activatable(){}
};
class Activator {
 public:
  virtual bool isActive(const Activatable & a) const = 0;
  virtual ~Activator() {}
};

extern const Activator & AllActiveActivator;


class CalibrationConfI : public virtual Printable {
 public:
  virtual const Activator& getCalibrationActivator() const = 0;

  virtual const Activator& getStateActivator() const = 0;

  virtual const Activator& getErrorTermActivator() const = 0;

  virtual bool isSpatialActive() const = 0;
  virtual bool isTemporalActive() const = 0;

  virtual std::string getOutputFolder(size_t segmentIndex = 0) const = 0;

  virtual bool shouldAnySensorBeRegisteredTo(const Sensor & to) const = 0;
  virtual bool shouldSensorsBeRegistered(const Sensor & from, const Sensor & to) const = 0;

  virtual bool getUseCalibPriors() const = 0;

  virtual CalibratorI & getCalibrator() = 0;
  virtual const CalibratorI & getCalibrator() const = 0;

  virtual ~CalibrationConfI();
};

} /* namespace calibration */
} /* namespace aslam */

namespace std {
template <>
struct hash<std::reference_wrapper<aslam::calibration::Activatable> > {
 public :
  size_t operator()(const std::reference_wrapper<aslam::calibration::Activatable> & ref) const {
    return hash<size_t>()(reinterpret_cast<size_t>(&ref));
  }
};
};


#endif /* H2FC5F3AD_D310_4A93_B2E5_55925AA5F4DF */
