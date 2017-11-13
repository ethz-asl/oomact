#include <aslam/calibration/tools/Parallelizer.h>

#include <glog/logging.h>
#include <assert.h>

namespace aslam {
namespace calibration {
void Parallelizer::doAndWait(){
  for(auto && f : futures_) {
    try {
      f.get();
    } catch(const std::exception & e){
      LOG(ERROR)<< "Exception in async task : " << e.what();
    }
    catch (...) {
      LOG(ERROR)<< "Unknown exception in async task";
    }
  }
  futures_.clear();

  LOG_IF(ERROR, currentThreads_ > 0) << "released too few tickets!";
}

Parallelizer::~Parallelizer(){
  doAndWait();
}

namespace internal {

struct ParallelTicket{
  ParallelTicket(Parallelizer & p) : p(p) {}
  ~ParallelTicket() {
    p.releaseTicket(*this);
  }
 private:
  Parallelizer & p;
};

}

std::shared_ptr<internal::ParallelTicket> Parallelizer::waitForAndTakeOneTicket() {
  auto p = std::make_shared<internal::ParallelTicket>(*this);
  std::unique_lock<std::mutex> lk(m_);
  cv_.wait(lk, [this]{return currentThreads_ < maxThreads_;});
  currentThreads_ ++;
  return p;
}

void Parallelizer::releaseTicket(internal::ParallelTicket& /*t*/) {
  std::unique_lock<std::mutex> lk(m_);

  LOG_IF(ERROR, currentThreads_ <= 0) << "released too many tickets!";
  currentThreads_ --;
  lk.unlock();
  cv_.notify_one();
}

}
}
