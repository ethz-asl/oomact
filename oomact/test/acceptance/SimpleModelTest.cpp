#include <cmath>

#include <gtest/gtest.h>

#include <aslam/calibration/test/SimpleModel.h>
#include <aslam/calibration/test/Tools.h>

using namespace aslam::calibration;
using namespace aslam::calibration::test;

TEST(AslamCalibrationAcceptance, testSimpleModelInit) {
  auto config = readConfig("testSimple.xml");
  SimpleModel m(config.getChild("test/model"));

  EXPECT_EQ(5u, m.getCalibrationVariables().size());

  LOG(INFO) << m;
}
