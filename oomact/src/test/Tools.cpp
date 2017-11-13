#include <aslam/calibration/test/Tools.h>

#include <glog/logging.h>
#include <sm/BoostPropertyTreeLoader.hpp>

namespace aslam {
namespace calibration {
namespace test {

struct PT : public sm::BoostPropertyTreeLoader {
  PT(){
    sm::BoostPropertyTree::setHumanReadableInputOutput(true);
    getSearchPaths() = { "acceptance" };
  }

 private:
  void goingToMergeIn(const std::string & updatePath) override {
    LOG(INFO) << "Merging in configuration file " << updatePath;
  }
  void startingWith(const std::string & path) override {
    LOG(INFO) << "Starting with configuration file " << path;
  }
  void warn(const std::string & message) override {
    LOG(WARNING) << message;
  }

  bool fileExists(const std::string & path) const override {
    LOG(INFO) << "Probing path '" << path << "'.";
    return sm::BoostPropertyTreeLoader::fileExists(path);
  }
} ptLoader;

sm::value_store::ValueStoreRef readConfig(const std::string & fileList){
  sm::BoostPropertyTree pt = ptLoader.readFilesAndMergeIntoPropertyTree(fileList);
//  pt.saveXml("test-config.xml");
  return pt;
}

std::string SimpleConfigPathResolver::resolve(const std::string& path) const
{
  return ptLoader.resolveFullFilePath(path, "");
}

}
}
}
