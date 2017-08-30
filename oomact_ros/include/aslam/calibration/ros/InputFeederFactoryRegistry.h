#ifndef H5FCABEDE_7DD2_4FF0_B920_6D60395B2D5F
#define H5FCABEDE_7DD2_4FF0_B920_6D60395B2D5F
#include "InputFeederFactoryI.h"

namespace aslam {
namespace calibration {
namespace ros {

class InputFeederFactoryRegistry {
 public:
  virtual ~InputFeederFactoryRegistry();

  static InputFeederFactoryRegistry & getInstance();

  template <typename SensorContainer, typename Func, typename Sensor = typename std::remove_reference<decltype(SensorContainer().begin())>::type >
  static void applyToMatching(const SensorContainer & sensors, Func func, std::vector<Sensor> * remainingSensors = nullptr) {
    for(auto & s : sensors){
      int c = 0;
      for(auto & ff : getInstance().inputFeederFactories_){
        if(ff->matches(s)){
          func(s, ff->createFeeder(s));
          c++;
        }
      }
      if(remainingSensors && !c){
        remainingSensors->push_back(s);
      }
    }
  }

  void add(std::unique_ptr<InputFeederFactoryI>);

  class RegistryEntry {
   public:
    RegistryEntry(std::unique_ptr<InputFeederFactoryI> iff);
    RegistryEntry(InputFeederFactoryI * iff) : RegistryEntry(std::unique_ptr<InputFeederFactoryI>(iff)) {}
    ~RegistryEntry();
  };
 private:
  InputFeederFactoryRegistry();
  std::vector<std::unique_ptr<InputFeederFactoryI>> inputFeederFactories_;
};

} /* namespace ros */
} /* namespace calibration */
} /* namespace aslam */

#endif /* H5FCABEDE_7DD2_4FF0_B920_6D60395B2D5F */
