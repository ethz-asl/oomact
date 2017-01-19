#include <cmath>

#include <gtest/gtest.h>

#include "../test_tools/SimpleModel.hpp"
#include "../test_tools/Tools.hpp"

namespace aslam {
namespace calibration {

TEST(AslamCalibrationAcceptance, testSimpleModelInit) {
  auto config = readConfig("testSimple.xml");
  SimpleModel m(config.getChild("test/model"));

  EXPECT_EQ(5, m.getCalibrationVariables().size());

  LOG(INFO) << m;
}

}
}
