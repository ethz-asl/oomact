#include <aslam/calibration/tools/Named.h>

#include <ostream>

#include <aslam/calibration/tools/TypeName.h>

namespace aslam {
namespace calibration {

Named::~Named() = default;

void Named::print(std::ostream & o) const {
  o << getName();
}

std::string getUnnamedObjectName(const std::type_info & typeInfo, const void* o) {
  return TypeName(typeInfo).toString() + "@" + std::to_string(reinterpret_cast<size_t>(o));
}

NamedMinimal::NamedMinimal(const std::string & name) : name_(name) {
}

NamedMinimal::~NamedMinimal() = default;

const std::string& NamedMinimal::getName() const {
  return name_;
}

} /* namespace calibration */
} /* namespace aslam */
