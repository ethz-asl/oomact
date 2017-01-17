#ifndef HFE630FBC_8B0C_4982_A216_8EC5040E1BB2
#define HFE630FBC_8B0C_4982_A216_8EC5040E1BB2
#include <string>
#include <sm/value_store/ValueStore.hpp>

#include "Printable.h"
#include "PlanFragment.h"

namespace aslam {
namespace calibration {
namespace plan {

class Plan : public Printable {
 public:
  Plan();
  void loadFromString(std::string content);
  void loadFromFile(std::string fileName);
  void load(sm::value_store::ValueStoreRef in);

  bool step();

  PlanFragment & getCurrentFragment();

  bool isFinished() const;

  virtual ~Plan();

  const std::string& getId() const {
    return id;
  }

  virtual void print(std::ostream & into) const;
 private:
  std::string id;
  bool finished = true;
  size_t fragmentIndex = 0;
  std::vector<std::shared_ptr<PlanFragment> > fragments;
};

} /* namespace plan */
} /* namespace calibration */
} /* namespace aslam */

#endif /* HFE630FBC_8B0C_4982_A216_8EC5040E1BB2 */
