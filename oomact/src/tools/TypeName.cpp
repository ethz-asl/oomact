#include <aslam/calibration/tools/TypeName.h>
#include <ostream>

#ifdef __GNUG__
#include <cstdlib>
#include <memory>
#include <cxxabi.h>

std::string demangle(const char* name) {
    int status;
    char * demangled_name = abi::__cxa_demangle(name, NULL, NULL, &status);
    std::string ret =(status==0) ? demangled_name : name;
    if(status == 0){
      free(demangled_name);
    }
    return ret;
}

#else

// does nothing if not g++
std::string demangle(const char* name) {
    return name;
}

#endif

namespace aslam {
namespace calibration {

std::string toPrettyTypename(const std::type_index& t) {
  return demangle(t.name());
}

TypeName::TypeName(const std::type_index& typeIndex)
    : typeIndex_(typeIndex) {
}
TypeName::TypeName(const std::type_info& typeInfo)
    : typeIndex_(typeInfo) {
}

TypeName::~TypeName() = default;

std::string TypeName::toString() const {
  return toPrettyTypename(typeIndex_);
}

std::ostream& operator << (std::ostream& o, const TypeName& tname) {
  o << toPrettyTypename(tname.typeIndex_);
  return o;
}

} /* namespace calibration */
} /* namespace aslam */
