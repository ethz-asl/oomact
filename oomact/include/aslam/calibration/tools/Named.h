#ifndef H018BA56E_D419_4E45_8341_B57F9764C3AA
#define H018BA56E_D419_4E45_8341_B57F9764C3AA
#include <string>
#include <typeinfo>

#include "Printable.h"

namespace aslam {
namespace calibration {


class Named : public virtual Printable {
 public:
  virtual ~Named();

  virtual const std::string& getName() const = 0;
  virtual void print(std::ostream & o) const override;
};

class NamedMinimal : public virtual Named {
 public:
  NamedMinimal(const std::string & name);
  virtual ~NamedMinimal();

  const std::string& getName() const override;

 private:
  const std::string name_;
};

std::string getUnnamedObjectName(const std::type_info & typeInfo, const void *o);

template <typename T>
std::string getObjectName(const T & o) {
  if(auto p = dynamic_cast<const Named*>(&o)){
    return p->getName();
  } else {
    return getUnnamedObjectName(typeid(T), &o);
  }
}

} /* namespace calibration */
} /* namespace aslam */

#endif /* H018BA56E_D419_4E45_8341_B57F9764C3AA */
