#ifndef HAE93B7A9_307E_4E62_9F9D_A7CB4394EB59
#define HAE93B7A9_307E_4E62_9F9D_A7CB4394EB59
#include <aslam/calibration/plan/CalibrationServer.h>
#include <aslam/calibration/plan/Driver.h>
#include <aslam/calibration/plan/Logger.h>
#include <aslam/calibration/plan/PlanFragment.h>
#include <aslam/calibration/plan/SmartDriver.h>

namespace aslam {
namespace calibration {
namespace plan {

class HomingDriver : public SmartDriver {
 public:
  HomingDriver(const HomingDriver&) = delete;

  explicit HomingDriver(const PlanFragmentWithHome& planFrag, Driver& d, bool useHome = true);
  explicit HomingDriver(const PlanFragmentWithHome& planFrag, CalibrationServer& cs, Driver& d, Logger& l, bool useHome = true);
  explicit HomingDriver(const PlanFragmentWithHome& planFrag, CalibrationServer* cs, Driver& d, Logger* l = nullptr, bool useHome = true);
  ~HomingDriver();
 private:
  Logger * l;
  CalibrationServer * cs;
  bool useHome = true;
};

} /* namespace plan */
} /* namespace calibration */
} /* namespace aslam */

#endif /* HAE93B7A9_307E_4E62_9F9D_A7CB4394EB59 */
