#ifndef HA31B1AEE_82E7_43F6_9638_2A533C65E724
#define HA31B1AEE_82E7_43F6_9638_2A533C65E724

namespace aslam {
namespace calibration {
class CalibratorI;

class CalibrationPhaseClientI {
 public:
  virtual ~CalibrationPhaseClientI();
  virtual void preProcessNewWindow(CalibratorI & calib);
};

} /* namespace calibration */
} /* namespace aslam */

#endif /* HA31B1AEE_82E7_43F6_9638_2A533C65E724 */
