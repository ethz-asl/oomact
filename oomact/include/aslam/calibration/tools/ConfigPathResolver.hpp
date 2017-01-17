#ifndef HFAEB3B14_531E_472A_9443_1888D697FD9B
#define HFAEB3B14_531E_472A_9443_1888D697FD9B

#include <string>

namespace aslam {
namespace calibration {

class ConfigPathResolver {
 public:
  virtual std::string resolve(const std::string & path) const = 0;
  virtual ~ConfigPathResolver(){}
};

}
}


#endif /* HFAEB3B14_531E_472A_9443_1888D697FD9B */
