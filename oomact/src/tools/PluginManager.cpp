#include <aslam/calibration/tools/PluginManager.h>

#include <glog/logging.h>

namespace aslam {
namespace calibration {

PluginI::~PluginI() {
}

PluginManager::PluginManager() {
}

PluginManager::~PluginManager() {
}

void PluginManager::addPlugin_(std::type_index i, PluginI* plugin) {
  CHECK(plugin);
  plugins_.emplace(i, std::unique_ptr<PluginI>(plugin));
}

PluginI& PluginManager::getPlugin_(std::type_index i) const {
  return *plugins_.at(i);
}

PluginI* PluginManager::getPluginPtr_(std::type_index i) const {
  auto it = plugins_.find(i);
  return it == plugins_.end() ? nullptr : it->second.get();
}

} /* namespace calibration */
} /* namespace aslam */
