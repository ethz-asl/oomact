#ifndef H4F16F278_43CD_4074_B9C1_33C1583E55CB
#define H4F16F278_43CD_4074_B9C1_33C1583E55CB
#include <functional>
#include <initializer_list>
#include <vector>

#include <aslam/calibration/model/Module.h>

namespace aslam {
namespace calibration {

class Model;

class ModuleList {
 public:

  ModuleList(std::initializer_list<std::reference_wrapper<Module>> modules) {
    for(const Module & m : modules){
      moduleIds.emplace_back(m.getUid());
    }
  }
  ModuleList(std::initializer_list<std::string> moduleIds) : moduleIds(moduleIds) {} //TODO B early check names!
  ModuleList(const std::vector<std::string> & moduleIds) : moduleIds(moduleIds) {}

  std::vector<std::reference_wrapper<Module> > resolveModuleList(Model& model) const;

  const std::vector<std::string>& getModuleIds() const {
    return moduleIds;
  }
 private:

  std::vector<std::string> moduleIds;
};

}
}

#endif /* H4F16F278_43CD_4074_B9C1_33C1583E55CB */
