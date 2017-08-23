#include <gtest/gtest.h>
#include <aslam/calibration/data/MapStorage.h>

TEST(StorageTestSuite, basics) {
  using namespace aslam::calibration;

  typedef MapStorage<const std::string *> MyStorage;

  MyStorage storage;

  std::string key1;

  MyStorage::Connector<std::string> sc(&key1);

  EXPECT_FALSE(storage.has(&key1));
  EXPECT_EQ(nullptr, sc.getDataPtrFrom(storage, false));
  EXPECT_NE(nullptr, sc.getDataPtrFrom(storage, true));
  EXPECT_NE(nullptr, sc.getDataPtrFrom(storage, false));

  const std::string testValue = "TEST";
  sc.getDataFrom(storage) = testValue;

  EXPECT_TRUE(storage.has(&key1));
  EXPECT_EQ(testValue, *sc.getDataPtrFrom(storage));
  EXPECT_EQ(testValue, sc.getDataFrom(storage));

  storage.clear();

  EXPECT_EQ(nullptr, sc.getDataPtrFrom(storage, false));

  sc.getDataPtrFrom(storage);
  EXPECT_TRUE(storage.has(&key1));

  storage.remove(&key1);

  EXPECT_FALSE(storage.has(&key1));
}
