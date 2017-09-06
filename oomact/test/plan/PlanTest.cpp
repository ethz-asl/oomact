#include <aslam/calibration/plan/Plan.h>

#include <gtest/gtest.h>
#include <sstream>

using namespace aslam::calibration::plan;

std::string toString(PlanFragment & pf){
  std::stringstream ss;
  pf.print(ss);
  return ss.str();
}

TEST(AslamCalibrationPlan, testLoad) {
  Plan p;
  p.loadFromString("<plan><id>ID</id></plan>");
  EXPECT_EQ("ID", p.getId());
  EXPECT_TRUE(p.isFinished());

  p.loadFromString("<plan><id>ID2</id><Test>test1</Test><Test>test2</Test></plan>");
  EXPECT_EQ("ID2", p.getId());
  EXPECT_FALSE(p.isFinished());

  EXPECT_EQ("Test(val=test1)", toString(p.getCurrentFragment()));

  p.step();
  EXPECT_FALSE(p.isFinished());

  EXPECT_EQ("Test(val=test2)", toString(p.getCurrentFragment()));

  p.step();
  EXPECT_TRUE(p.isFinished());
}
