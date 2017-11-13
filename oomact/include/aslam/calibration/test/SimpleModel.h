#ifndef H2C92550D_9A2D_4436_A900_7F53EDF85E01
#define H2C92550D_9A2D_4436_A900_7F53EDF85E01

#include <aslam/calibration/model/Model.h>
#include <aslam/calibration/model/sensors/WheelOdometry.h>

#include "Tools.h"

namespace aslam {
namespace calibration {
namespace test {

class SimpleModel : public Model {
 public:
  SimpleModel(ValueStoreRef config, std::shared_ptr<ConfigPathResolver> configPathResolver = std::make_shared<SimpleConfigPathResolver>());
  virtual ~SimpleModel() = default;

 private:
  ValueStoreRef config_;
  ValueStoreRef sensorsConfig_;
  WheelOdometry wheelOdometry_;
};

}
}
}

#endif /* H2C92550D_9A2D_4436_A900_7F53EDF85E01 */
