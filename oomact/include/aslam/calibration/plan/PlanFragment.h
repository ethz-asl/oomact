#ifndef HFE630FBC_8B0C_4982_A216_8EC5040E1BB3
#define HFE630FBC_8B0C_4982_A216_8EC5040E1BB3

#include <string>

#include "../tools/Printable.h"
#include "Driver.h"
#include "Logger.h"
#include "CalibrationServer.h"


namespace aslam {
namespace calibration {
namespace plan {

class PlanFragment : public Printable {
 public:
  PlanFragment(std::string name);
  virtual void execute(CalibrationServer &, Driver &, Logger &) const = 0;
  virtual ~PlanFragment();

  const std::string & getName() const;

  void print(std::ostream& into) const override;
private:
  std::string name_;
};


class PlanFragmentWithHome : public PlanFragment{
 public:
  PlanFragmentWithHome(const sm::value_store::ValueStoreRef& vs, std::string name);

  bool hasHome() const;
  const std::string& getHome() const;

 private:
  std::string homeFileName_;
  std::string home_;
};

void startLogging(const PlanFragment & frag, Logger& l);

class SmartDriver;

namespace fragment {
void takeStaticPointClouds(CalibrationServer &cs, SmartDriver & d);
void takeDynamicPointClouds(CalibrationServer &cs, SmartDriver & d);
}

} /* namespace plan */
} /* namespace calibration */
} /* namespace aslam */

#endif /* HFE630FBC_8B0C_4982_A216_8EC5040E1BB3 */
