#include "aslam/calibration/ros/InputFeederFactoryRegistry.h"

namespace aslam {
namespace calibration {
namespace ros {

InputFeederFactoryRegistry::InputFeederFactoryRegistry() {

}

InputFeederFactoryRegistry::~InputFeederFactoryRegistry() {
}

InputFeederFactoryRegistry& InputFeederFactoryRegistry::getInstance() {
  static InputFeederFactoryRegistry instance;
  return instance;
}

InputFeederFactoryRegistry::RegistryEntry::RegistryEntry(std::unique_ptr<InputFeederFactoryI> iff) {
  InputFeederFactoryRegistry::getInstance().add(std::move(iff));
}

InputFeederFactoryRegistry::RegistryEntry::~RegistryEntry() {
}

void InputFeederFactoryRegistry::add(std::unique_ptr<InputFeederFactoryI> iff) {
  inputFeederFactories_.emplace_back(std::move(iff));
}

} /* namespace ros */
} /* namespace calibration */
} /* namespace aslam */

