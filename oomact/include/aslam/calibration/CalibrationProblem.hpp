#ifndef HF3DE647E_64C8_4D32_8DF7_7D746D9ACD23
#define HF3DE647E_64C8_4D32_8DF7_7D746D9ACD23

#include <aslam/backend/DesignVariable.hpp>
#include <aslam/backend/OptimizationProblemBase.hpp>

namespace aslam {
namespace calibration {
class CalibrationVariable;

class CalibrationProblem : public backend::ErrorTermReceiver {
 public:
  virtual ~CalibrationProblem() {}
  virtual void addCalibrationVariable(CalibrationVariable *) = 0;
  virtual void addStateVariable(backend::DesignVariable *) = 0;

  virtual const std::vector<boost::shared_ptr<backend::ErrorTerm>> & getErrorTerms() const = 0;
  virtual void getErrors(const backend::DesignVariable* dv, std::set<backend::ErrorTerm*>& outErrorSet) const = 0;

  virtual size_t getDimCalibrationVariables() const = 0;
  virtual size_t getDimStateVariables() const = 0;
  virtual size_t getNumErrorTerms() const = 0;
};

}
}

#endif /* HF3DE647E_64C8_4D32_8DF7_7D746D9ACD23 */
