#ifndef H1CFE6498_3DBA_4DF7_9A86_562867E72B05
#define H1CFE6498_3DBA_4DF7_9A86_562867E72B05

#include <memory>
#include <typeindex>
#include <unordered_map>

#include <aslam/calibration/tools/PluginI.h>

namespace aslam {
namespace calibration {

class PluginManager {
 public:
  PluginManager();
  virtual ~PluginManager();

  template <typename T> bool hasPlugin() { return plugins_.count(typeid(T)) > 0; }
  template <typename T> T& getPlugin() { return static_cast<T&>(getPlugin_(typeid(T))); }
  template <typename T> const T& getPlugin() const { return const_cast<PluginManager&>(*this).getPlugin<T>(); }
  template <typename T> T* getPluginPtr() { return static_cast<T*>(getPluginPtr_(typeid(T))); }
  template <typename T> const T* getPluginPtr() const { return const_cast<PluginManager&>(*this).getPluginPtr<T>(); }

 protected:
  template <typename T> void addPlugin(T * plugin) { addPlugin_(typeid(*plugin), plugin); }

  template <typename T, typename Func> T& getOrCreatePlugin(Func factory) {
    auto p = getPluginPtr<T>();
    if(!p){
      p = factory();
      addPlugin(p);
    }
    return *p;
  }

 private:
  void addPlugin_(std::type_index i, PluginI * plugin);
  PluginI * getPluginPtr_(std::type_index i) const;
  PluginI & getPlugin_(std::type_index i) const;
  std::unordered_map<std::type_index, std::unique_ptr<PluginI>> plugins_;
};

} /* namespace calibration */
} /* namespace aslam */

#endif /* H1CFE6498_3DBA_4DF7_9A86_562867E72B05 */
