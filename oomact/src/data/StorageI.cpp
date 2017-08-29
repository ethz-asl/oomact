#include <aslam/calibration/data/MapStorage.h>

namespace aslam {
namespace calibration {

StorageElement::StorageElement() : ptr_(nullptr) {
}

StorageElement::~StorageElement() {
  del();
}

void StorageElement::del() {
  if(ptr_) {
    deleter_(ptr_);
    ptr_ = nullptr;
  }
}

StorageElement & StorageElement::operator= (StorageElement && other) {
  del();
  ptr_ = other.ptr_;
  other.ptr_ = nullptr;
  deleter_ = std::move(other.deleter_);
  return *this;
}

} /* namespace calibration */
} /* namespace aslam */
