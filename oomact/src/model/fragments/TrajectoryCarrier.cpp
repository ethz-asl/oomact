#include <aslam/calibration/model/fragments/TrajectoryCarrier.h>

#include <ostream>

#include "aslam/calibration/model/Model.h"
#include <aslam/calibration/model/ModuleTools.h>
#include <aslam/calibration/model/fragments/So3R3TrajectoryCarrier.h>

namespace aslam {
namespace calibration {

So3R3TrajectoryCarrier::So3R3TrajectoryCarrier(sm::value_store::ValueStoreRef config) :
  knotsPerSecond(config.getDouble("knotsPerSecond")),
  rotSplineOrder(config.getInt("rotSplineOrder")),
  transSplineOrder(config.getInt("transSplineOrder")),
  rotFittingLambda(config.getDouble("rotFittingLambda")),
  transFittingLambda(config.getDouble("transFittingLambda"))
{
}

void So3R3TrajectoryCarrier::writeConfig(std::ostream& out) const {
  MODULE_WRITE_PARAM(knotsPerSecond);
  MODULE_WRITE_PARAM(rotSplineOrder);
  MODULE_WRITE_PARAM(rotFittingLambda);
  MODULE_WRITE_PARAM(transSplineOrder);
  MODULE_WRITE_PARAM(transFittingLambda);
}

TrajectoryCarrier::TrajectoryCarrier(sm::value_store::ValueStoreRef config) :
  knotsPerSecond(config.getDouble("knotsPerSecond")),
  splineOrder(config.getInt("splineOrder")),
  fittingLambda(config.getDouble("fittingLambda"))
{
}
void TrajectoryCarrier::writeConfig(std::ostream& out, const std::string & /* namePrefix */) const {
  //TODO C use prefix

  MODULE_WRITE_PARAM(knotsPerSecond);
  MODULE_WRITE_PARAM(splineOrder);
  MODULE_WRITE_PARAM(fittingLambda);
}

} /* namespace calibration */
} /* namespace aslam */

