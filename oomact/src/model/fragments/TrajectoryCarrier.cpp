#include <aslam/calibration/model/fragments/TrajectoryCarrier.h>

#include <ostream>

#include "aslam/calibration/model/Model.h"
#include <aslam/calibration/model/fragments/So3R3TrajectoryCarrier.h>

namespace aslam {
namespace calibration {

So3R3TrajectoryCarrier::So3R3TrajectoryCarrier(sm::value_store::ValueStoreRef config, const Frame & frame) :
  frame(frame),
  knotsPerSecond(config.getDouble("knotsPerSecond")),
  rotSplineOrder(config.getInt("rotSplineOrder")),
  transSplineOrder(config.getInt("transSplineOrder")),
  rotFittingLambda(config.getDouble("rotFittingLambda")),
  transFittingLambda(config.getDouble("transFittingLambda"))
{
}

void So3R3TrajectoryCarrier::writeConfig(std::ostream& out) const {
  MODULE_WRITE_PARAMETER(frame);
  MODULE_WRITE_PARAMETER(knotsPerSecond);
  MODULE_WRITE_PARAMETER(rotSplineOrder);
  MODULE_WRITE_PARAMETER(rotFittingLambda);
  MODULE_WRITE_PARAMETER(transSplineOrder);
  MODULE_WRITE_PARAMETER(transFittingLambda);
}

TrajectoryCarrier::TrajectoryCarrier(sm::value_store::ValueStoreRef config) :
  knotsPerSecond(config.getDouble("knotsPerSecond")),
  splineOrder(config.getInt("splineOrder")),
  fittingLambda(config.getDouble("fittingLambda"))
{
}
void TrajectoryCarrier::writeConfig(std::ostream& out, const std::string & /* namePrefix */) const {
  //TODO C use prefix

  MODULE_WRITE_PARAMETER(knotsPerSecond);
  MODULE_WRITE_PARAMETER(splineOrder);
  MODULE_WRITE_PARAMETER(fittingLambda);
}

} /* namespace calibration */
} /* namespace aslam */

