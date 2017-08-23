#ifndef HA8673F24_CDA4_4E2F_AA65_E4BA90C9F80F
#define HA8673F24_CDA4_4E2F_AA65_E4BA90C9F80F

#include <type_traits>
#include <functional>

#include <glog/logging.h>

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

template <typename Key>
class StorageI {
 public:
  StorageI(){
    static_assert(std::is_same<std::size_t, decltype(reinterpret_cast<std::size_t>(*static_cast<Key*>(nullptr)))>::value, "Key must be reinterpretable as size_t.");
  }
  virtual ~StorageI() = default;


  bool has(Key key) const {
    return get(key) != nullptr;
  }

  virtual void remove(Key key) = 0;

  template <typename Value>
  class Connector {
   public:
    Connector(Key key) : key_(key){ };

    Value* getDataPtrFrom(StorageI<Key> & storage, bool createIfMissing = true) const {
      auto ptr = static_cast<Value*>(storage.get(key_));
      if (createIfMissing && !ptr) {
        ptr = createNewStorage();
        storage.add(key_, ptr);
      }
      return ptr;
    }

    Value & getDataFrom(StorageI<Key> & storage) const {
      const auto ptr = getDataPtrFrom(storage, false);
      CHECK(ptr);
      return *ptr;
    }

   private:
    Value * createNewStorage() const {
      return new Value;
    }
    Key key_;
  };
 private:
  virtual void * get(Key key) const = 0;
  virtual void add(Key key, StorageElement && data) = 0;
  template <typename> friend class Connector;
};

} /* namespace calibration */
} /* namespace aslam */

#endif /* HA8673F24_CDA4_4E2F_AA65_E4BA90C9F80F */
