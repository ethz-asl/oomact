#ifndef H67703C26_CF1F_457A_A1CA_B22BA7C675D8
#define H67703C26_CF1F_457A_A1CA_B22BA7C675D8
#include <unordered_map>

namespace aslam {
namespace calibration {
namespace plan {

class PlanFragmentRegistry {
 public:
  struct IllegalFragmentName : public std::runtime_error {
    using std::runtime_error::runtime_error;
  };

  static PlanFragmentRegistry & getInstance();

  template <typename X>
  void registerFragment(const std::string & name) {
    factories.emplace(name, [](const sm::value_store::ValueStoreRef & vs){ return new X(vs); });
  }

  template <typename X>
  void registerFragment() {
    registerFragment<X>(X::Name);
  }

  std::shared_ptr<PlanFragment> createFragment(const std::string key, const sm::value_store::ValueStoreRef& vs);
 private:
  PlanFragmentRegistry();
  std::unordered_map<std::string, std::function<PlanFragment*(const sm::value_store::ValueStoreRef & vs)>> factories;
};

} /* namespace plan */
} /* namespace calibration */
} /* namespace aslam */

#endif /* H67703C26_CF1F_457A_A1CA_B22BA7C675D8 */
