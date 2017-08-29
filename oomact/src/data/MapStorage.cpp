#include <aslam/calibration/data/MapStorage.h>

namespace aslam {
namespace calibration {

void* internal::MapStorageUnsafe::get(size_t key) const {
  auto it = data_.find(key);
  if (it == data_.end()){
    return nullptr;
  } else {
    return it->second.get();
  }
}

void internal::MapStorageUnsafe::add(size_t key, StorageElement && data) {
  data_[key] = std::move(data);
}

void internal::MapStorageUnsafe::clear() {
  data_.clear();
}

void internal::MapStorageUnsafe::remove(size_t key) {
  data_.erase(key);
}

} /* namespace calibration */
} /* namespace aslam */

