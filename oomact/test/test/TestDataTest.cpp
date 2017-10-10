#include <aslam/calibration/test/TestData.h>

#include <gtest/gtest.h>
#include <sstream>

using namespace aslam::calibration::test;

TEST(TestData, testIsAvailable) {
  EXPECT_TRUE(TestData::getInstance().isAvailable());
  EXPECT_TRUE(TestData::getInstance().isAvailable(".."));
  EXPECT_FALSE(TestData::getInstance().isAvailable("missing_test.data"));
}

TEST(TestData, testIsAvailableAndSkip) {
  const std::string testFile="README.md";
  const int skipCount = TestData::getInstance().getSkipCount();
  OOMACT_SKIP_IF_TESTDATA_UNAVAILABLE(testFile);
  EXPECT_EQ(skipCount, TestData::getInstance().getSkipCount());
  EXPECT_TRUE(TestData::getInstance().isAvailable("README.md"));
}
