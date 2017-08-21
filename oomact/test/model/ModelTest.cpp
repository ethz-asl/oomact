#include <aslam/calibration/model/Model.h>

#include <gtest/gtest.h>
#include <sstream>

#include "../test_tools/SimpleModel.hpp"
#include "../test_tools/Tools.hpp"

using namespace aslam::calibration;

class MockModule : public Module {
  using Module::Module;
};

TEST(Module, modelUid) {
  ValueStoreRef config;
  Model m(config, std::make_shared<SimpleConfigPathResolver>());
  MockModule mm1(m, "A", config);
  MockModule mm2(m, "A", config);

  m.add(mm1);
  ASSERT_EQ(mm1.getUid(), "A");
  m.add(mm2);
  ASSERT_EQ(mm2.getUid(), "A1");
}


