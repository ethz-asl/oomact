#include <aslam/calibration/tools/Typename.h>
#include <ostream>

namespace aslam {
namespace calibration {

//TODO D support typename demangling

class Typename::Impl : public std::string {
  using std::string::string;
};

Typename::Typename(const std::type_info & to) : impl_(new Impl(to.name())) {
}

Typename::~Typename() = default;

std::ostream & operator << (std::ostream & o, const Typename & tname){
  o << *tname.impl_;
  return o;
}

} /* namespace calibration */
} /* namespace aslam */

