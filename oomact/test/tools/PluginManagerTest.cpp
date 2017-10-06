#include <aslam/calibration/tools/PluginManager.h>

#include <stdexcept>

#include <gtest/gtest.h>

namespace aslam {
namespace calibration {

class MockPM;

template <int i>
class MockPlugin : public PluginI {
 public:
  MockPlugin(MockPM & mpm) : mpm_(mpm) {}
  MockPM & mpm_;
};
typedef MockPlugin<0> MPA;
typedef MockPlugin<1> MPB;

class MockPM : public PluginManager {
 public:
  template <typename T> T& getOrCreatePlugin() {
    return PluginManager::getOrCreatePlugin<T>([this](){ return new T{*this}; });
  }
};

TEST(PluginManager, basicFunctionality) {
  MockPM mpm;

  EXPECT_FALSE(mpm.hasPlugin<MPA>());
  EXPECT_THROW(mpm.getPlugin<MPA>(), std::out_of_range);
  EXPECT_FALSE(mpm.getPluginPtr<MPA>());

  MPA* p;
  EXPECT_NO_THROW(p = &mpm.getOrCreatePlugin<MPA>());

  EXPECT_TRUE(mpm.hasPlugin<MPA>());
  EXPECT_NO_THROW(mpm.getPlugin<MPA>());
  EXPECT_EQ(p, &mpm.getPlugin<MPA>());
  EXPECT_EQ(p, mpm.getPluginPtr<MPA>());
}

}
}
