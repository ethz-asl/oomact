#ifndef H20231A9A_85A2_4AA6_AF7E_089F9112BF21
#define H20231A9A_85A2_4AA6_AF7E_089F9112BF21

#include <memory>
#include <future>

#include "Plan.h"

namespace aslam {
namespace calibration {
namespace plan {

class Executor {
 public:
  Executor();

  bool loadNewPlan(sm::value_store::ValueStoreRef planSource);

  void step();


  bool isFinished() {
    return !planPtr_ || planPtr_->isFinished();
  }

  virtual ~Executor();
  bool isPlanReplaceable();

  bool hasPlan(){
    return bool(planPtr_);
  }
  const Plan & getPlan(){
    assert(planPtr_);
    return *planPtr_;
  }

 private:
  virtual void exec(const PlanFragment & pf) = 0;

  std::future<void> execFuture;
  std::shared_ptr<Plan> planPtr_;
};

} /* namespace plan */
} /* namespace calibration */
} /* namespace aslam */

#endif /* H20231A9A_85A2_4AA6_AF7E_089F9112BF21 */
