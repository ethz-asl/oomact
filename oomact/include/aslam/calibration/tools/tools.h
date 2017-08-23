#ifndef INCLUDE_ASLAM_CALIBRATION_EUROPA_TOOLS_TOOLS_H_
#define INCLUDE_ASLAM_CALIBRATION_EUROPA_TOOLS_TOOLS_H_

#include <type_traits>
#include <boost/lexical_cast.hpp>
#include <sm/value_store/ValueStore.hpp>
#include <string>
#include <vector>

namespace aslam {
namespace backend {
class MEstimator;
}
namespace calibration {

template<typename T>
inline std::string toString(const T & v){
  return boost::lexical_cast<std::string>(v);
}

inline std::string operator + (const std::string & s , double t){
  return s + toString(t);
}

template <typename T, typename = typename std::enable_if<std::is_integral<T>::value>::type >
std::string operator + (const std::string & s , T t){
  return s + toString(t);
}

std::vector<std::string> splitString(const std::string & s, const std::string & byAnyOf);

void openStream(std::ofstream & outputFile, std::string pathPrefix);
void writeToFile(const std::string & fileName, std::function<void(std::ostream &o)> writer);
void writeStringToFile(const std::string & fileName, const std::string & content);

void createDirs(const std::string & path, bool ignoreErrors = false);

boost::shared_ptr<aslam::backend::MEstimator> getMestimator(const std::string & name, const sm::value_store::ValueStoreRef config, int dim = -1);

}
}


#endif /* INCLUDE_ASLAM_CALIBRATION_EUROPA_TOOLS_TOOLS_H_ */
