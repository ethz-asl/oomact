#ifndef H8E4AC88D_C8F7_417C_8732_BFF3DC9C79DF
#define H8E4AC88D_C8F7_417C_8732_BFF3DC9C79DF
#include <aslam/calibration/model/Module.h>


namespace aslam {
namespace calibration {

class TrajectoryCarrier {
 public:
  TrajectoryCarrier(sm::value_store::ValueStoreRef config);

  double getKnotsPerSecond() const {
    return knotsPerSecond;
  }

  double getFittingLambda() const {
    return fittingLambda;
  }

  int getSplineOrder() const {
    return splineOrder;
  }

  void writeConfig(std::ostream& out, const std::string & namePrefix = std::string()) const;
 protected:
  double knotsPerSecond;
  int splineOrder;
  double fittingLambda;
};

class NamedTrajectoryCarrier :  public TrajectoryCarrier, public virtual Named {
  using TrajectoryCarrier::TrajectoryCarrier;
};

} /* namespace calibration */
} /* namespace aslam */

#endif /* H8E4AC88D_C8F7_417C_8732_BFF3DC9C79DF */
