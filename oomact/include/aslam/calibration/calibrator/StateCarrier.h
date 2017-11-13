#ifndef HCD72CF91_0489_4EEA_BFFB_E7E21C9B4799
#define HCD72CF91_0489_4EEA_BFFB_E7E21C9B4799

#include <functional>
#include <memory>
#include <string>

#include <aslam/calibration/model/Sensor.h>

namespace aslam {
namespace calibration {
class CalibratorI;

class BatchState {
 public:
  virtual void writeToFile(const CalibratorI & calib, const std::string & pathPrefix) const = 0;
  virtual ~BatchState(){}
};

class StateCarrier {
 public:
  virtual void writeState(const CalibratorI & calib, const std::string & pathPrefix) const;

  virtual ~StateCarrier() = default;
};

inline bool operator == (const StateCarrier & a, const StateCarrier & b){
  return &a == &b;
}

typedef std::shared_ptr<BatchState> BatchStateSP;

class BatchStateReceiver {
 public:
  virtual ~BatchStateReceiver(){}
  virtual void addBatchState(StateCarrier & stateCarrier, const BatchStateSP& batchState) = 0;

};


} /* namespace calibration */
} /* namespace aslam */


namespace std {
  template <>
  struct hash<std::reference_wrapper<aslam::calibration::StateCarrier> > {
   public :
       size_t operator()(const std::reference_wrapper<aslam::calibration::StateCarrier> & ref) const {
         return hash<size_t>()(reinterpret_cast<size_t>(&ref));
       }
  };
};

#endif /* HCD72CF91_0489_4EEA_BFFB_E7E21C9B4799 */
