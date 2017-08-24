#include <cmath>

#include <gtest/gtest.h>

#include <aslam/calibration/test/SimpleModel.hpp>
#include <aslam/calibration/test/Tools.hpp>

using namespace aslam::calibration;
using namespace aslam::calibration::test;

TEST(AslamCalibrationAcceptance, testSimpleModelInit) {
  auto config = readConfig("testSimple.xml");
  SimpleModel m(config.getChild("test/model"));

  EXPECT_EQ(5, m.getCalibrationVariables().size());

  LOG(INFO) << m;
}
