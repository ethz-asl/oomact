#include <aslam/calibration/plan/PlanFragment.h>

#include <fstream>

namespace aslam {
namespace calibration {
namespace plan {

PlanFragment::PlanFragment(std::string name) : name_(name) {
}

PlanFragment::~PlanFragment() {
}

const std::string & PlanFragment::getName() const {
  return name_;
}

void PlanFragment::print(std::ostream& into) const {
  into << getName();
}

PlanFragmentWithHome::PlanFragmentWithHome(const sm::value_store::ValueStoreRef& vs, std::string name)
    : PlanFragment(name) {
  homeFileName_ = vs.getString("homeFile", std::string());
  if (homeFileName_.empty()) {
    homeFileName_ = getName() + ".home_";
  }
  LOG(INFO)<< "Using home_ file " << homeFileName_ << ".";
  std::ifstream homeFile(homeFileName_);
  if (homeFile.is_open()) {
    std::getline(homeFile, home_);
    homeFile.close();
    LOG(INFO)<<"Read home " << getHome() << " from " << homeFileName_;
  } else {
    LOG(INFO) << "Could not read home_ file (" << homeFileName_ << ")! Going to mark home_ instead.";
  }
}


bool PlanFragmentWithHome::hasHome() const {
  return !home_.empty();
}

const std::string& PlanFragmentWithHome::getHome() const {
  return home_;
}

} /* namespace plan */
} /* namespace calibration */
} /* namespace aslam */
