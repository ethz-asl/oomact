#ifndef H10C446C7_DFEC_4215_B08D_A0A5EBAF6577
#define H10C446C7_DFEC_4215_B08D_A0A5EBAF6577

namespace aslam {
namespace calibration {

template <typename Derived>
class IsA {
 public:
  template <typename T>
  bool isA() const {
    return dynamic_cast<const T*>(getDerived()) != nullptr;
  }
  template <typename T>
  const T& as() const {
    return *dynamic_cast<const T*>(getDerived());
  }
  template <typename T>
  T& as() {
    return *dynamic_cast<T*>(getDerived());
  }
  template <typename T>
  const T* ptrAs() const {
    return dynamic_cast<const T*>(getDerived());
  }
  template <typename T>
  T* ptrAs() {
    return dynamic_cast<T*>(getDerived());
  }
 private:
  Derived * getDerived() {
    return static_cast<Derived*>(this);
  }
  const Derived * getDerived() const {
    return static_cast<const Derived*>(this);
  }
};

} /* namespace calibration */
} /* namespace aslam */


#endif /* H10C446C7_DFEC_4215_B08D_A0A5EBAF6577 */
