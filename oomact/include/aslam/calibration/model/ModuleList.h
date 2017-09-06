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
  ModuleList() = default;
  ModuleList(std::initializer_list<std::reference_wrapper<Module> > modules);
  ModuleList(std::initializer_list<std::string> moduleIds);
  ModuleList(const std::vector<std::string>& moduleIds);

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
