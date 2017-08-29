#ifndef HFE630FBC_8B0C_4982_A216_8EC5040E1BB3
#define HFE630FBC_8B0C_4982_A216_8EC5040E1BB3

#include "../tools/Printable.h"
#include "Driver.h"
#include "Logger.h"
#include "CalibrationServer.h"


namespace aslam {
namespace calibration {
namespace plan {

class PlanFragment : public Printable {
 public:
  PlanFragment();
  virtual void execute(CalibrationServer &, Driver &, Logger &) const = 0;
  virtual ~PlanFragment();
};

} /* namespace plan */
} /* namespace calibration */
} /* namespace aslam */

#endif /* HFE630FBC_8B0C_4982_A216_8EC5040E1BB3 */
