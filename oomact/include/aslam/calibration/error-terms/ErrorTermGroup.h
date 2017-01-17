#ifndef H02D83F26_4F1F_4F29_885D_49AEE1264845
#define H02D83F26_4F1F_4F29_885D_49AEE1264845

#include <aslam/backend/ErrorTerm.hpp>
#include <string>

namespace aslam {
namespace calibration {

struct ErrorTermGroup {
  const std::string & getName() const {
    return name;
  }

  static const ErrorTermGroup & getByName(const std::string & name);
  const std::string name;
};

extern const ErrorTermGroup & defaultErrorTermGroup;


struct ErrorTermGroupReference {
  const ErrorTermGroup & group;
  ErrorTermGroupReference() : group(defaultErrorTermGroup) {}
  ErrorTermGroupReference(const char * name) : group(ErrorTermGroup::getByName(name)) {}
  ErrorTermGroupReference(const std::string & name) : group(ErrorTermGroup::getByName(name)) {}
  ErrorTermGroupReference(const ErrorTermGroup & group) : group(group) {}
  const std::string & getName() const {
    return group.name;
  }
};

struct ErrorTermGroupMember : public ErrorTermGroupReference {
  using ErrorTermGroupReference::ErrorTermGroupReference;
  ErrorTermGroupMember() = default;
  ErrorTermGroupMember(const ErrorTermGroupMember &) = default;
  ErrorTermGroupMember(const ErrorTermGroupReference &ref) :ErrorTermGroupReference(ref) {}
};

inline bool operator<(const ErrorTermGroup &a, const ErrorTermGroup &b){
  return a.getName() < b.getName();
}

inline const ErrorTermGroup & getErrorTermGroup(const char * name){
  return ErrorTermGroup::getByName(name);
}
inline const ErrorTermGroup & getErrorTermGroup(const std::string & name){
  return ErrorTermGroup::getByName(name);
}

inline const ErrorTermGroup & getErrorTermGroup(const ErrorTermGroupMember & member){
  return member.group;
}
const ErrorTermGroup & getErrorTermGroup(const aslam::backend::ErrorTerm & member);

}
}



#endif /* H02D83F26_4F1F_4F29_885D_49AEE1264845 */
