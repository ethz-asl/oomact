#include <cmath>

#include <gtest/gtest.h>


#include "SimpleModel.hpp"
#include "Tools.hpp"

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
