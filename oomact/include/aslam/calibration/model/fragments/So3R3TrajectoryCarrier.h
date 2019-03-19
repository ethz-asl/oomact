#ifndef H8E4AC88D_C8F7_417C_8732_BFF3DB9C79DF
#define H8E4AC88D_C8F7_417C_8732_BFF3DB9C79DF
#include <aslam/calibration/model/Module.h>


namespace aslam {
namespace calibration {

class Frame;

class So3R3TrajectoryCarrier : public virtual Named {
 public:
  So3R3TrajectoryCarrier(sm::value_store::ValueStoreRef config);

  double getKnotsPerSecond() const {
    return knotsPerSecond;
  }

  void setKnotsPerSecond(double knotsPerSecond) {
    this->knotsPerSecond = knotsPerSecond;
  }

  int getRotSplineOrder() const {
    return rotSplineOrder;
  }

  int getTransSplineOrder() const {
    return transSplineOrder;
  }

  double getRotFittingLambda() const {
    return rotFittingLambda;
  }

  double getTransFittingLambda() const {
    return transFittingLambda;
  }

 protected:
  void writeConfig(std::ostream & out) const;

  double knotsPerSecond;
  int rotSplineOrder, transSplineOrder;
  double rotFittingLambda, transFittingLambda;
};

} /* namespace calibration */
} /* namespace aslam */

#endif /* H8E4AC88D_C8F7_417C_8732_BFF3DB9C79DF */
