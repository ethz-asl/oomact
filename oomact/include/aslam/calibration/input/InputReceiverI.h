#ifndef HCD22AC9B_8438_43D6_BD69_2D38100CFD14
#define HCD22AC9B_8438_43D6_BD69_2D38100CFD14
#include <aslam/calibration/model/Module.h>
#include <aslam/calibration/Timestamp.h>

namespace aslam {
namespace calibration {

/**
 * The InputReceiverIT is an templated interface for input receiving modules.
 * The template parameter determines the type of input data.
 */

template <typename Input>
class InputReceiverIT : public virtual ModuleBase {
 public:
  virtual ~InputReceiverIT() = default;

  virtual void addInputTo(Timestamp t, const Input & input, ModuleStorage & s) const = 0;
};

} /* namespace calibration */
} /* namespace aslam */



#endif /* HCD22AC9B_8438_43D6_BD69_2D38100CFD14 */
