#ifndef HA8673F24_CDA4_4E2F_AA65_E4BA90C9F80F
#define HA8673F24_CDA4_4E2F_AA65_E4BA90C9F80F

#include <type_traits>
#include <functional>

#include <glog/logging.h>

#include "../tools/CheckNotNull.h"

namespace aslam {
namespace calibration {

class StorageElement {
 public:
  StorageElement();

  template <typename T>
  StorageElement(T * ptr) :
    ptr_(ptr),
    deleter_([](void * ptr){ delete reinterpret_cast<T*>(ptr);})
    {
  }
  ~StorageElement();

  StorageElement & operator= (StorageElement && other);

  void * get() const { return ptr_; }
 private:
  void del();

  void * ptr_;
  std::function<void(void *ptr)> deleter_;
};

/**
 * The Storage class provides typesafe storage service.
 * Storage is accessed through a key of type Key.
 * For example it allows a calibrator to store the measurement data for all sensors of the model without introducing state for the sensors.
 * This way it can control the life time and architecture (one /  multiple batch, sliding windows ..)
 */

namespace internal {
template <typename Value, typename Key, typename Storage>
auto valueFactory(Key key, Storage& storage) -> decltype(new Value(key, storage)){
  return new Value(key, storage);
};

template <typename Value, typename Key, typename Storage>
auto valueFactory(Key, Storage& storage, ...) -> decltype(new Value(storage)){
  return new Value(storage);
};

template <typename Value, typename Key, typename Storage>
Value * valueFactory(Key, Storage&, ...) {
  return new Value();
}

template <typename Value, typename Key, typename Storage>
Value * defaultFactory(Key key, Storage& storage) {
  return valueFactory<Value>(key, storage);
}
}

template <typename Value, typename Storage, typename Key = typename Storage::Key>
class StorageConnector;

template <typename Key_>
class StorageI {
 public:
  typedef Key_ Key;

  StorageI(){
    static_assert(std::is_same<std::size_t, decltype(reinterpret_cast<std::size_t>(*static_cast<Key*>(nullptr)))>::value, "Key must be reinterpretable as size_t.");
  }
  virtual ~StorageI() = default;


  bool has(Key key) const {
    return get(key) != nullptr;
  }

  virtual void remove(Key key) = 0;

  virtual size_t size() const = 0;

  template <typename Value> using Connector = StorageConnector<Value, StorageI>;
 private:
  virtual void * get(Key key) const = 0;
  virtual void add(Key key, StorageElement && data) = 0;
  template <typename, typename, typename> friend class StorageConnector;
};

template <typename Value, typename Storage, typename Key>
class StorageConnector {
 public:
  StorageConnector(Key key) : key_(key), factory_(&internal::defaultFactory<Value, Key, Storage &>) { };
  StorageConnector(Key key, std::function<Value *(Key, Storage &)> factory) : key_(key), factory_(factory) { };

  Value* getDataPtrFrom(Storage& storage, bool createIfMissing = true) const;

  const Value* getDataPtrFrom(const Storage & storage) const {
    return static_cast<const Value*>(storage.get(key_));
  }

  Value & getDataFrom(Storage & storage) const {
    return *getDataPtrFrom(storage, true);
  }

  const Value & getDataFrom(const Storage & storage) const {
    return *checkNotNull(getDataPtrFrom(storage));
  }

  bool hasData(const Storage & storage) const {
    return storage.get(key_);
  }
 private:
  Key key_;
  std::function<Value *(Key, Storage &)> factory_;
};

template<typename Value, typename Storage, typename Key>
inline Value* StorageConnector<Value, Storage, Key>::getDataPtrFrom(Storage& storage, bool createIfMissing) const {
  auto ptr = static_cast<Value*>(storage.get(key_));
  if (createIfMissing && !ptr) {
    ptr = factory_(key_, storage);
    storage.add(key_, ptr);
  }
  return ptr;
}

} /* namespace calibration */
} /* namespace aslam */

#endif /* HA8673F24_CDA4_4E2F_AA65_E4BA90C9F80F */
