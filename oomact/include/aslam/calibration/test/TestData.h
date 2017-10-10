#ifndef HF5024BEB_2F93_4C23_8681_9BA43D68E713
#define HF5024BEB_2F93_4C23_8681_9BA43D68E713

#include <iosfwd>
#include <string>

namespace aslam {
namespace calibration {
namespace test {

class TestData {
 public:
  bool isAvailable(const std::string& relativePath = std::string()) const;

  std::string getPath(const std::string& relativePath) const;

  void incrementSkipCount();

  int getSkipCount() const {
    return skipCount_;
  }

  void printSkipMessageAncIncSkipCounter(const std::string& name, const std::string& file) const;

  void printSkipCountMessageAndReset(std::ostream & out);

  static TestData & getInstance();
 private:
  TestData() = default;
  ~TestData();
  int skipCount_ = 0;
};

inline bool isTestDataAvailable(const std::string& relativePath){
  return TestData::getInstance().isAvailable(relativePath);
}
inline std::string getTestDataPath(const std::string& relativePath){
  return TestData::getInstance().getPath(relativePath);
}

#define OOMACT_SKIP_IF_TESTDATA_UNAVAILABLE(FILE) if (!::aslam::calibration::test::isTestDataAvailable(FILE)) {\
  std::string name;\
  if(auto * p = ::testing::UnitTest::GetInstance()->current_test_info()){\
    name = name + p->test_case_name() + "." + p->name(); \
  } else { \
    name = __func__; \
  } \
  ::aslam::calibration::test::TestData::getInstance().printSkipMessageAncIncSkipCounter(name, FILE); \
  return; \
};

} /* namespace test */
} /* namespace calibration */
} /* namespace aslam */

#endif /* HF5024BEB_2F93_4C23_8681_9BA43D68E713 */
