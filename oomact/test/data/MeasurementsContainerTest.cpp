#include <cmath>

#include <gtest/gtest.h>

#include <aslam/calibration/data/MeasurementsContainer.h>

TEST(MeasurementContainerTestSuite, testOrder) {
  using namespace aslam::calibration;

  std::vector<Duration> expected = {0., 3., 4., 5., 6.};

  MeasurementsContainer<int> c;
  EXPECT_EQ(Duration(0.0), c.getMaximalTimeGap());

  c.emplace_back(3., 0);
  EXPECT_EQ(Duration(0.0), c.getMaximalTimeGap());

  c.emplace_back(5., 0);
  EXPECT_EQ(Duration(2.0), c.getMaximalTimeGap());

  c.emplace_back(6., 0);
  EXPECT_EQ(Duration(2.0), c.getMaximalTimeGap());

  c.emplace_back(4., 0);
  EXPECT_EQ(Duration(1.0), c.getMaximalTimeGap());

  c.emplace_back(0., 0);
  EXPECT_EQ(Duration(3.0), c.getMaximalTimeGap());

  try {
    EXPECT_EQ(expected.size(), c.size());
    int i = 0;
    for(auto e : expected){
      EXPECT_EQ(e, c[i++].first);
    }
  }
  catch (const std::exception& e) {
    FAIL() << e.what();
  }
}
