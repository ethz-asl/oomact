#include <aslam/calibration/plan/HomingDriver.h>
#include <aslam/calibration/plan/PlanFragment.h>


namespace aslam {
namespace calibration {
namespace plan {

HomingDriver::HomingDriver(const PlanFragmentWithHome& planFrag, Driver& d, bool useHome)
    : HomingDriver(planFrag, nullptr, d, nullptr, useHome) {
}

HomingDriver::HomingDriver(const PlanFragmentWithHome& planFrag, CalibrationServer& cs, Driver& d, Logger& l, bool useHome)
    : HomingDriver(planFrag, &cs, d, &l, useHome) {
}

HomingDriver::HomingDriver(const PlanFragmentWithHome& planFrag, CalibrationServer* cs, Driver& d, Logger* l, bool useHome)
    : SmartDriver(d),
      l(l),
      cs(cs),
      useHome(useHome) {
  if (useHome && planFrag.hasHome()) {
    d.setHome(planFrag.getHome());
    goHome();
  } else {
    d.markHome();
  }
  if (l)
    startLogging(planFrag, *l);
}

HomingDriver::~HomingDriver() {
  stop();
  if (cs)
    cs->startCalibration();

  sleep(0.1);
  if (useHome) {
    LOG(INFO)<< "Going home";
    if(l) goHomeAndStopLogging(*l);
    else goHome();
  } else {
    if(l) l->stop();
  }
  if (cs)
    cs->waitForCalibration();
}

} /* namespace plan */
} /* namespace calibration */
} /* namespace aslam */
