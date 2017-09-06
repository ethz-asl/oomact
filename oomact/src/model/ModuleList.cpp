#include <aslam/calibration/model/ModuleList.h>
#include <aslam/calibration/model/Model.h>

#include <glog/logging.h>

namespace aslam {
namespace calibration {

ModuleList::ModuleList(const std::vector<std::string>& moduleIds)
    : moduleIds(moduleIds) {
}

ModuleList::ModuleList(std::initializer_list<std::string> moduleIds)
    : moduleIds(moduleIds) {
}  //TODO B early check names!

ModuleList::ModuleList(std::initializer_list<std::reference_wrapper<Module> > modules) {
  for (const Module& m : modules) {
    moduleIds.emplace_back(m.getUid());
  }
}

std::vector<std::reference_wrapper<Module>> ModuleList::resolveModuleList(Model& model) const
{
  std::vector<std::reference_wrapper<Module> > modules;
  for (std::string moduleName: getModuleIds()) {
    if(Module* mp = model.getModulePtr(moduleName)){
      modules.push_back(*mp);
    } else {
      LOG(WARNING) << "Could not find module named '" << moduleName << "'. Going to ignore it!";
    }
  }
  return modules;
}

}
}
