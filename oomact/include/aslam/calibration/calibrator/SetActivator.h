#ifndef H47608F63_8183_4C91_8EF0_A465A5EB37BB
#define H47608F63_8183_4C91_8EF0_A465A5EB37BB

#include <iosfwd>
#include <unordered_set>

#include <aslam/calibration/calibrator/CalibrationConfI.h>

namespace aslam {
namespace calibration {

class SetActivator : public Activator {
 public:
  SetActivator(const Activator & parent) : parent(parent) {}

  bool isActive(const Activatable & a) const override;

  bool setActive(const Activatable & a, bool active = true);

  virtual ~SetActivator() {}

  const std::unordered_set<const Activatable*>& getActiveSet() const {
    return activeSet;
  }

  void writeActiveList(std::ostream & o) const;

 private:
  std::unordered_set<const Activatable *> activeSet;
  const Activator & parent;
};

} /* namespace calibration */
} /* namespace aslam */

#endif /* H47608F63_8183_4C91_8EF0_A465A5EB37BB */
