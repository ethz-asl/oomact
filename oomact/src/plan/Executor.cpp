#include <aslam/calibration/plan/Executor.h>

#include <glog/logging.h>

namespace aslam {
namespace calibration {
namespace plan {

Executor::Executor() {

}

bool Executor::isPlanReplaceable() {
  return ! planPtr_ || planPtr_->isFinished();
}

bool Executor::loadNewPlan(sm::value_store::ValueStoreRef planSource) {
  if(!isPlanReplaceable()) {
    LOG(WARNING) << "Old plan wasn't finished, yet!";
  }
  planPtr_.reset(new Plan());
  planPtr_->load(planSource);
  LOG(INFO) << "Loaded plan " << planPtr_->getId();
  return true;
}

void Executor::step() {
  if(planPtr_){
    if(execFuture.valid()){
      auto status = execFuture.wait_for(std::chrono::microseconds(10));
      if(status == std::future_status::ready){
        execFuture.get();
      }
    } else if(!planPtr_->isFinished()){
      LOG(INFO)<< "Starting fragment " << planPtr_->getId() << ":" << planPtr_->getCurrentFragment();
      auto plan = planPtr_;
      execFuture = std::async(std::launch::async, [=]() {
        exec(plan->getCurrentFragment());
        plan->step();
      });
    }
  }
}

Executor::~Executor() {
}

} /* namespace plan */
} /* namespace calibration */
} /* namespace aslam */
