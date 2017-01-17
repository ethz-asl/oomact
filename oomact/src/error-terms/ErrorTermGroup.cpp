#include <aslam/calibration/error-terms/ErrorTermGroup.h>
namespace aslam {
namespace calibration {

const ErrorTermGroup & ErrorTermGroup::getByName(const std::string & name){
  static std::unordered_map<std::string, ErrorTermGroup> m;

  auto i = m.find(name);
  if(i != m.end()){
    return i->second;
  } else {
    return m.emplace(name, ErrorTermGroup{name}).first->second;
  }
}
const ErrorTermGroup & defaultErrorTermGroup = ErrorTermGroup::getByName("Default");


const ErrorTermGroup & getErrorTermGroup(const aslam::backend::ErrorTerm & member){
  if(auto p = dynamic_cast<const ErrorTermGroupMember*>(&member)){
    return p->group;
  }

  return defaultErrorTermGroup;
}

}
}
