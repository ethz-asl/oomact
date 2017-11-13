#ifndef H189CC15E_474C_43FD_8CD9_7B96A85B7F70
#define H189CC15E_474C_43FD_8CD9_7B96A85B7F70

#include <aslam/backend/DesignVariable.hpp>
#include <sm/boost/null_deleter.hpp>

namespace aslam {
namespace calibration {

class DesignVariableReceiver {
 public:
  virtual ~DesignVariableReceiver(){}
  virtual void addDesignVariable(backend::DesignVariable * dv) = 0;

  template <typename Spline> void addSplineDesignVariables(Spline & spline, bool active){
    const size_t numDV = spline.numDesignVariables();
    for (size_t i = 0; i < numDV; ++i) {
      spline.designVariable(i)->setActive(active);
      addDesignVariable(spline.designVariable(i));
    }
  }
};

template <typename F>
class FunctorDesignVariableReceiver : public DesignVariableReceiver {
 public:
  FunctorDesignVariableReceiver(F f) : f_(f) {}
  virtual ~FunctorDesignVariableReceiver(){}
  virtual void addDesignVariable(backend::DesignVariable * dv){
    f_(dv);
  }

 private:
  F f_;
};
template <typename F>
FunctorDesignVariableReceiver<F> createFunctorDesignVariableReceiver(F f){
  return FunctorDesignVariableReceiver<F>(f);
}

}
}


#endif /* H189CC15E_474C_43FD_8CD9_7B96A85B7F70 */
