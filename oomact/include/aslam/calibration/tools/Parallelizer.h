#ifndef PARALLELIZER_HPP_
#define PARALLELIZER_HPP_

#include <future>
#include <memory>
#include <vector>

namespace aslam {
namespace calibration {

namespace internal {
  struct ParallelTicket;
}
class Parallelizer {
 public:
  Parallelizer(size_t maxThreads) : maxThreads_(maxThreads), currentThreads_(0) {
  }

  template <typename Functor>
  void add(Functor f) {
    if(maxThreads_ == 0){
      f();
    }
    else{
      futures_.push_back(std::async(maxThreads_ > 1 ? std::launch::async : std::launch::deferred, [this, f](){
        auto ticket = waitForAndTakeOneTicket();
        f();
      }));
    }
  };

  void doAndWait();

  ~Parallelizer();
 private:
  std::shared_ptr<internal::ParallelTicket> waitForAndTakeOneTicket();
  void releaseTicket(internal::ParallelTicket & t);
  friend struct internal::ParallelTicket;

  std::vector<std::future<void> > futures_;
  size_t maxThreads_;
  std::mutex m_;
  std::condition_variable cv_;
  size_t currentThreads_;
};

}
}


#endif /* PARALLELIZER_HPP_ */
