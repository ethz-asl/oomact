#ifndef HF975EBFA_EF55_4EB2_977F_0AE7C2615F31
#define HF975EBFA_EF55_4EB2_977F_0AE7C2615F31
#include <iosfwd>
#include <memory>
#include <iosfwd>
#include <string>
#include <typeinfo>
#include <typeindex>

namespace aslam {
namespace calibration {

class TypeName {
 public:
  template <typename T>
  TypeName(const T & o) : TypeName(typeid(o)) {
    static_assert(std::is_class<T>::value, "Only type names for classes are supported");
  }

  TypeName(const std::type_index& typeIndex);
  TypeName(const std::type_info& typeInfo);

  ~TypeName();

  std::string toString() const;
  std::string operator () () const { return toString(); }
  friend std::ostream & operator << (std::ostream &, const TypeName & tname);
 private:
  std::type_index typeIndex_;
};

std::string toPrettyTypename(const std::type_index& t);

inline std::string operator+(const std::string& s, const TypeName& tname) {
  return s + tname.toString();
}

inline std::ostream& operator<<(std::ostream& o, const std::type_info& type_info) {
  o << TypeName(type_info);
  return o;
}
inline std::ostream& operator<<(std::ostream& o, const std::type_index& type_index) {
  o << TypeName(type_index);
  return o;
}


} /* namespace calibration */
} /* namespace aslam */

#endif /* HF975EBFA_EF55_4EB2_977F_0AE7C2615F31 */
