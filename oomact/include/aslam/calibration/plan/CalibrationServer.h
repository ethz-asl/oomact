#ifndef HA5460CEF_8694_4D7B_936A_E25129A41393
#define HA5460CEF_8694_4D7B_936A_E25129A41393

#include <vector>
#include <functional>

#include <aslam/calibration/model/Module.h>
#include <aslam/calibration/model/ModuleList.h>

#include <boost/serialization/serialization.hpp>

namespace aslam {
namespace calibration {
namespace plan {

class ServerException : public std::exception {
 public:
  ServerException(std::string what) : what_(what) { }
  const char * what() const noexcept (true) override { return what_.c_str();}
 private:
  const std::string what_;
};

class CalibrationServer {
 public:
  CalibrationServer(){}
  virtual void load(std::string configFileList, std::vector<std::string> configStrings, std::string outputFolder, std::string logFile) = 0;

  virtual void ping() = 0;

  virtual void startDataCollection(const ModuleList & moduleList) = 0;
  virtual void endDataCollection(const ModuleList & moduleList) = 0;

  virtual void startCalibration() = 0;
  virtual void waitForCalibration() = 0;

  virtual ~CalibrationServer(){}
};

} /* namespace plan */
} /* namespace calibration */
} /* namespace aslam */

#endif /* HA5460CEF_8694_4D7B_936A_E25129A41393 */
