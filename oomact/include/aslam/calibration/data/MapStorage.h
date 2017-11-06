#ifndef HFEA7051B_5790_422C_9B0F_F23CB3F5C546
#define HFEA7051B_5790_422C_9B0F_F23CB3F5C546
#include <aslam/calibration/data/StorageI.h>
#include <unordered_map>
#include <memory>

namespace aslam {
namespace calibration {
namespace internal {


/**
 * The MapStorageUnsafe class is a helper for Storage below doing the heavy lifting but not typesafe.
 */
class MapStorageUnsafe {
 public:
  void clear();
  void * get(size_t key) const;
  void add(size_t key, StorageElement && se);
  void remove(size_t key);

  size_t size() const { return data_.size(); }
 private:
  std::unordered_map<size_t, StorageElement> data_;
};
}

/**
 * The MapStorage class provides a simple implementation of the StorageI interface based on a std::unordered_map.
 * The template layer only requires the Key type. The actual work is done by internal::MapStorageUnsafe.
 */
template <typename Key>
class MapStorage : public StorageI<Key> {
 public:
  virtual ~MapStorage() = default;

  void remove(Key key) override {
    impl.remove(convertKey(key));
  }
  void clear() {
    impl.clear();
  }

  void * get(Key key) const override {
    return impl.get(convertKey(key));
  }

  void add(Key key, StorageElement && data) override {
    impl.add(convertKey(key), std::move(data));
  }

  size_t size() const override {
    return impl.size();
  }
 private:
  static size_t convertKey(Key key) {
    return reinterpret_cast<size_t>(key);
  }

  internal::MapStorageUnsafe impl;
};

} /* namespace calibration */
} /* namespace aslam */

#endif /* HFEA7051B_5790_422C_9B0F_F23CB3F5C546 */
