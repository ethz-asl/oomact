#ifndef H61102552_F2B3_4E09_AD6E_F774D85DED5F
#define H61102552_F2B3_4E09_AD6E_F774D85DED5F

#include <string>
#include <iosfwd>

#include <boost/shared_ptr.hpp>

#include <aslam/backend/ErrorTerm.hpp>

#include "aslam/calibration/error-terms/ConditionalErrorTerm.h"
namespace aslam {
namespace calibration {

class ErrorTermStatistics {
 public:
  ErrorTermStatistics(std::string name, bool evaluateError = true) : name(name), evaluateError(evaluateError) {}

  bool add(const boost::shared_ptr<aslam::backend::ErrorTerm> & e, bool ignoreInactive = true){
    return add(*e, ignoreInactive);
  }
  bool add(aslam::backend::ErrorTerm& e, bool ignoreInactive = true);
  void add(double squaredError);

  std::ostream& printInto(std::ostream & out) const;
  std::ostream& printShortInto(std::ostream & out) const;

  friend std::ostream & operator << (std::ostream & o, const ErrorTermStatistics & es){
    es.printShortInto(o); return o;
  }

  double getCost() const {
    return cost;
  }

  size_t getCounter() const {
    return counter;
  }

  const std::string& getName() const {
    return name;
  }

  void skip() {
    skipCounter ++;
  }

  size_t getSkipCount() const {
    return skipCounter;
  }

 private:
  std::string name;
  size_t counter = 0;
  size_t skipCounter = 0;
  size_t inactiveCounter = 0;
  double cost = 0;
  bool evaluateError;
};

} /* namespace calibration */
} /* namespace aslam */

#endif /* H61102552_F2B3_4E09_AD6E_F774D85DED5F */
