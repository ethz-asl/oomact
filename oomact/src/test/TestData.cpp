#include <aslam/calibration/test/TestData.h>

#include <iostream>

#include <boost/filesystem/operations.hpp>
#include <glog/logging.h>

namespace aslam {
namespace calibration {
namespace test {

TestData& TestData::getInstance() {
  static TestData instance;
  return instance;
}

std::string BasePath = "oomact_test_data";

bool TestData::isAvailable(const std::string& relativePath) const {
  return boost::filesystem::exists(getPath(relativePath));
}

std::string TestData::getPath(const std::string& relativePath) const {
  using namespace boost::filesystem;
  const std::string pwd = current_path().string();
  auto i = pwd.rfind("oomact");
  if(i == std::string::npos){
    throw std::runtime_error(std::string("Could not locate test data starting from '") + pwd + "'. Unit test using this functions must currently be executed from within their source folder.");
  } else {
    return (path(pwd.substr(0, i)) / BasePath / relativePath).string();
  }
}

void TestData::incrementSkipCount() {
  ++skipCount_;
}

void TestData::printSkipCountMessageAndReset(std::ostream& out) {
  if(skipCount_){
    out << "Skipped " << skipCount_ << " test due to missing test data. Checkout the test_data submodule to enable these test." << std::endl;
    skipCount_ = 0;
  }
}

void TestData::printSkipMessageAncIncSkipCounter(const std::string& name, const std::string& file) const {
  std::cout << "Skipping this test (" << name << ") due to missing test data (" << file << ")" << std::endl;
  test::TestData::getInstance().incrementSkipCount();
}

TestData::~TestData() {
  if(skipCount_){
    TestData::getInstance().printSkipCountMessageAndReset(LOG(WARNING));
  }
}

} /* namespace test */
} /* namespace calibration */
} /* namespace aslam */
