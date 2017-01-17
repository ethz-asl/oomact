#ifndef H4FCBA143_7A1D_4792_8B1C_825C9B311CE5
#define H4FCBA143_7A1D_4792_8B1C_825C9B311CE5

#include <sm/timing/NsecTimeUtilities.hpp>
#include <string>
#include "aslam/calibration/tools/tools.h"

namespace aslam {
namespace calibration {

template <typename Spline>
void writeSpline(const Spline& spline, double dt, std::ofstream & stream) {
  using sm::timing::secToNsec;
  if(stream.is_open()){
    auto t = spline.getMinTime();
    auto T = spline.getMaxTime();
    sm::timing::NsecTime dtNSec = secToNsec(dt);
    while (t <= T) {
      auto expFactory = spline.template getExpressionFactoryAt<0>(t);
      stream << t << " " << expFactory.getValueExpression().toValue().transpose() << std::endl;
      t += dtNSec;
    }
  }
}

template <typename Spline>
void writeSpline(const Spline& spline, double dt, const std::string & path) {
  std::ofstream stream;
  openStream(stream, path);
  if(stream.is_open()){
    writeSpline(spline, dt, stream);
  }
}

}
}


#endif /* H4FCBA143_7A1D_4792_8B1C_825C9B311CE5 */
