#ifndef HF975EBFA_EF55_4EB2_977F_0AE7C2615F31
#define HF975EBFA_EF55_4EB2_977F_0AE7C2615F31
#include <iosfwd>
#include <memory>
#include <typeinfo>

namespace aslam {
namespace calibration {

class Typename {
 public:
  template <typename T>
  Typename(const T & o) : Typename(typeid(o)) {
    static_assert(std::is_class<T>::value, "Only type names for classes are supported");
  }

  ~Typename();

  friend std::ostream & operator << (std::ostream &, const Typename & tname);
 private:
  class Impl;
  Typename(const std::type_info & info);
  std::unique_ptr<Impl> impl_;
};

} /* namespace calibration */
} /* namespace aslam */

#endif /* HF975EBFA_EF55_4EB2_977F_0AE7C2615F31 */
