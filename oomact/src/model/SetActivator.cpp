#include <aslam/calibration/calibrator/SetActivator.h>

#include <ostream>

namespace aslam {
namespace calibration {


bool SetActivator::setActive(const Activatable & a, bool active) {
  if(auto used = dynamic_cast<const Used*>(&a)){
    active &= used->isUsed();
  }
  if(active){
    activeSet.insert(&a);
  }else{
    activeSet.erase(&a);
  }
  return active;
}

bool SetActivator::isActive(const Activatable & a) const {
  return parent.isActive(a) && activeSet.count(&a) > 0;
}

void SetActivator::writeActiveList(std::ostream& o) const {
  int i = 0;
  for(const Activatable* a : activeSet){
    if(a){
      if(i++) {
        o << ", ";
      }
      o << a->getName();
    }
  }
}

} /* namespace calibration */
} /* namespace aslam */
