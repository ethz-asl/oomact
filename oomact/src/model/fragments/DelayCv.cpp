#include <aslam/calibration/model/fragments/DelayCv.h>

#include <glog/logging.h>

#include <aslam/calibration/model/Model.h>

namespace aslam {
namespace calibration {


void boundsEventHandler(TimeDesignVariableCv & cv, BoundsEnforceEvent event){
  switch(event){
    case BoundsEnforceEvent::CLAMPED_TO_LOWER:
      LOG(WARNING) << "Delay " << cv.getName() << " had to be clamped to its lower bound " << cv.getLowerBound().getNumerator() << "!";
      break;
    case BoundsEnforceEvent::CLAMPED_TO_UPPER:
      LOG(WARNING) << "Delay " << cv.getName() << " had to be clamped to its upper bound " << cv.getUpperBound().getNumerator() << "!";
      break;
    case BoundsEnforceEvent::NOP:
      LOG(INFO) << "Delay " << cv.getName() << " has been updated to " << cv.getValue().getNumerator() << ".";
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

