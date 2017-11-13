#ifndef H04B3F29E_096B_4A87_9151_D45C16D87882
#define H04B3F29E_096B_4A87_9151_D45C16D87882


#include <aslam/calibration/tools/ConfigPathResolver.h>
#include <sm/value_store/ValueStore.hpp>

namespace aslam {
namespace calibration {
namespace test {


class SimpleConfigPathResolver : public ConfigPathResolver {
  std::string resolve(const std::string& path) const override;
};

sm::value_store::ValueStoreRef readConfig(const std::string & fileList);

}
}
}

#endif /* H04B3F29E_096B_4A87_9151_D45C16D87882 */
