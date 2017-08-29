#ifndef H4155AC23_BEA3_4657_90C6_F2945AF23E80
#define H4155AC23_BEA3_4657_90C6_F2945AF23E80

#include <memory>

#include <glog/logging.h>
#include <sm/boost/null_deleter.hpp>

namespace aslam {

template <typename T>
class local_shared_ptr : public std::shared_ptr<T> {
 public:
  local_shared_ptr(T & o) : std::shared_ptr<T>(&o, sm::null_deleter()) {}
  local_shared_ptr(std::unique_ptr<T> & up) : std::shared_ptr<T>(std::move(up)) {}
  ~local_shared_ptr(){
    CHECK_EQ(1, this->use_count());
  }
};

template <typename T>
local_shared_ptr<T> to_local_shared_ptr(T & o) {
  return local_shared_ptr<T>(o);
}

template <typename T>
std::shared_ptr<T> unique_to_shared_ptr(std::unique_ptr<T> up) {
  return  std::shared_ptr<T>(std::move(up));
}

}

#endif /* H4155AC23_BEA3_4657_90C6_F2945AF23E80 */
