#ifndef HCA288077_190C_48B6_8248_4F256563F38D
#define HCA288077_190C_48B6_8248_4F256563F38D
namespace aslam {
namespace calibration {


template <typename Modulebase, ActionBase>
class ModuleToActionEntry {
  virtual ~ModuleToActionEntry() = default;
  virtual bool matches(const Modulebase & s) const = 0;
  virtual Result apply(const Modulebase & s) const = 0;
};

template <typename Input, typename Result>
class ModuleToActionMap {
 private:
  std::vector<std::unique_ptr<ModuleToActionEntry<ModuleBase>>> inputFeederFactories_;
};

} /* namespace calibration */
} /* namespace aslam */



#endif /* HCA288077_190C_48B6_8248_4F256563F38D */
