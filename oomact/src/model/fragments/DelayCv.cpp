#include <aslam/calibration/model/fragments/DelayCv.h>

#include <glog/logging.h>

#include <aslam/calibration/model/Model.h>

namespace aslam {
namespace calibration {


void boundsEventHandler(TimeDesignVariableCv & cv, BoundsEnforceEvent event){
  switch(event){
    case BoundsEnforceEvent::CLAMPED_TO_LOWER:
      LOG(WARNING) << "Delay " << cv.getName() << " had to be clamped to its lower bound " << static_cast<double>(cv.getLowerBound()) << " s!";
      break;
    case BoundsEnforceEvent::CLAMPED_TO_UPPER:
      LOG(WARNING) << "Delay " << cv.getName() << " had to be clamped to its upper bound " << static_cast<double>(cv.getUpperBound()) << " s!";
      break;
    case BoundsEnforceEvent::NOP:
      LOG(INFO) << "Delay " << cv.getName() << " has been updated to " << static_cast<double>(cv.getValue()) << " s.";
      break;
  }
}

DelayCv::DelayCv(const Module* module) :
  dt_r_s(module->createCVIfUsed<TimeDesignVariableCv>("delay", "d", boundsEventHandler)),
  delayExp(dt_r_s)
{
}

} /* namespace calibration */
} /* namespace aslam */

