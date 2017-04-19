#ifndef INCLUDE_ASLAM_CALIBRATION_MODULE_H_
#define INCLUDE_ASLAM_CALIBRATION_MODULE_H_

#include <string>
#include <iosfwd>

#include <aslam/calibration/plan/Printable.h>
#include <sm/value_store/ValueStore.hpp>
namespace boost {
  template<typename T> class shared_ptr;
}

namespace aslam {
namespace backend {
class ErrorTermReceiver;
}
namespace calibration {
using backend::ErrorTermReceiver;
class CalibratorI;
class DesignVariableReceiver;
class BatchStateReceiver;
class Sensor;
class Model;

class Named : public Printable {
 public:
  virtual const std::string& getName() const = 0;
  virtual ~Named() = default;
  virtual void print(std::ostream & o) const override {
    o << getName();
  }
};

class NamedMinimal : virtual public Named {
 public:
  NamedMinimal(const std::string & name) : name(name) {}
  const std::string& getName() const override { return name; }

  virtual ~NamedMinimal() = default;
 private:
  const std::string name;
};

std::string getUnnamedObjectName(const void *o);

template <typename T>
std::string getObjectName(const T & o) {
  if(auto p = dynamic_cast<const Named*>(&o)){
    return p->getName();
  } else {
    return getUnnamedObjectName(&o);
  }
}

class Used {
 public:
  virtual bool isUsed() const = 0;
  virtual ~Used() = default;
};

class Observer {
 public:
  virtual bool isObserveOnly() const = 0;
  virtual void setObserveOnly(bool observeOnly) = 0;
  virtual ~Observer() = default;
};

class Calibratable {
 public:
  virtual bool isToBeCalibrated() const = 0;
  virtual ~Calibratable() = default;
};


class Activatable : public virtual Named {
 public:
  bool operator == (const Activatable & other) const { return this == & other; }
  virtual ~Activatable(){}
};
class Activator {
 public:
  virtual bool isActive(const Activatable & a) const = 0;
  virtual ~Activator() {}
};

extern const Activator & AllActiveActivator;

class EstConf : virtual public Printable {
 public:
  virtual const Activator& getCalibrationActivator() const = 0;

  virtual const Activator& getStateActivator() const = 0;

  virtual const Activator& getErrorTermActivator() const = 0;

  virtual bool isSpatialActive() const = 0;
  virtual bool isTemporalActive() const = 0;

  virtual std::string getOutputFolder(size_t segmentIndex = 0) const = 0;

  virtual bool getUseCalibPriors() const = 0;

  virtual ~EstConf(){}
};

struct ModuleLinkBase;
struct ModuleRegistry;

template <typename Derived>
class IsA {
 public:
  template <typename T>
  bool isA() const {
    return dynamic_cast<const T*>(getDerived()) != nullptr;
  }
  template <typename T>
  const T& as() const {
    return *dynamic_cast<const T*>(getDerived());
  }
  template <typename T>
  T& as() {
    return *dynamic_cast<T*>(getDerived());
  }
  template <typename T>
  const T* ptrAs() const {
    return dynamic_cast<const T*>(getDerived());
  }
  template <typename T>
  T* ptrAs() {
    return dynamic_cast<T*>(getDerived());
  }
 private:
  Derived * getDerived() {
    return static_cast<Derived*>(this);
  }
  const Derived * getDerived() const {
    return static_cast<const Derived*>(this);
  }
};

std::string normalizeName(const char * parameter);

class Module : public virtual Named, public virtual Used, public IsA<Module> {
 public:
  Module(Model & model, const std::string & name, sm::value_store::ValueStoreRef config, bool isUsedByDefault = true);
  Module(const Module & m);

  virtual void registerWithModel();

  virtual bool initState(CalibratorI & calib);
  virtual void setCalibrationActive(const EstConf & ec);
  virtual void addToBatch(const Activator & stateActivator, BatchStateReceiver & batchStateReceiver, DesignVariableReceiver & problem);
  virtual void clearMeasurements();
  virtual void addErrorTerms(CalibratorI & calib, const EstConf & ec, ErrorTermReceiver & errorTermReceiver) const;
  virtual void preProcessNewWindow(CalibratorI & calib);

  virtual void estimatesUpdated(CalibratorI & calib) const;

  virtual bool hasTooFewMeasurements() const;

  void writeInfo(std::ostream & out) const;

  virtual ~Module() = default;

  const std::string& getName() const override {
    return name_;
  }
  const Model& getModel() const {
    return model_;
  }
  Model& getModel() {
    return model_;
  }

  const std::string& getUid() const {
    return uid_;
  }

  bool isUsed() const override { return used_; }

  const sm::value_store::ValueStoreRef& getMyConfig() const {
    return myConfig;
  }


  template <typename CV, typename ... Args>
  boost::shared_ptr<CV> createCVIfUsed(sm::value_store::ValueStoreRef config, const std::string variableNamePostifx, Args ... args) const {
    if(isUsed()){
      if(config.getBool("used", true)){
        return boost::shared_ptr<CV>(new CV(getName() + "_" + variableNamePostifx, config, args...));
      }
    }
    return nullptr;
  }


  template <typename CV, typename ... Args>
  boost::shared_ptr<CV> createCVIfUsed(const std::string & configElementName, const std::string variableNamePostifx, Args ... args) const {
    return createCVIfUsed<CV>(getMyConfig().getChild(configElementName), variableNamePostifx, args...);
  }

  bool shouldObserveOnly(const EstConf& ec) const;

  void resolveLinks(ModuleRegistry & reg);

  bool operator == (const Module & other) const {
    return this == &other;
  }
 protected:
  virtual void writeConfig(std::ostream & out) const;

#define MODULE_WRITE_PARAMETER(param) out << (", " + normalizeName(#param) + "=") << param;

  virtual void setActive(bool spatial, bool temporal);
  virtual bool isCalibrationIntended(const EstConf & ec) const;

  sm::value_store::ValueStoreRef myConfig;

 private:
  virtual void addMeasurementErrorTerms(CalibratorI & calib, const EstConf & ec, ErrorTermReceiver & problem, bool observeOnly) const;
  friend Model;
  void setUid(const std::string& uid) {
    uid_ = uid;
  }
  Model & model_;
  const std::string name_;
  const bool used_ = false;
  std::string uid_;
  bool isRegistered_ = false;
  std::vector<std::reference_wrapper<ModuleLinkBase>> moduleLinks_;
  friend ModuleLinkBase;
};

class ObserverMinimal : public Observer {
 public:
  ObserverMinimal(const Module * module);

  bool isObserveOnly() const override {
    return observeOnly_;
  }
  virtual void setObserveOnly(bool observeOnly) override {
    this->observeOnly_ = observeOnly;
  }
  virtual ~ObserverMinimal() = default;
 private:
  bool observeOnly_;
};

class CalibratableMinimal : public Calibratable {
 public:
  CalibratableMinimal(const Module * module);

  bool isToBeCalibrated() const override {
    return toBeCalibrated_;
  }
  virtual ~CalibratableMinimal() = default;
 private:
  bool toBeCalibrated_;
};

struct ModuleRegistry {
  virtual Module * getModulePtr(const std::string & uid) const = 0;
  virtual ~ModuleRegistry() = default;
};

struct ModuleLinkBase : public NamedMinimal {
  ModuleLinkBase(Module & owner, const std::string & linkName, bool required = false);
  ModuleLinkBase(const std::string & ownerName, const std::string & linkName, const std::string targetUid, bool required = false);
  ModuleLinkBase(const ModuleLinkBase &) = delete;

  virtual void resolve(const ModuleRegistry & reg) = 0;
  virtual ~ModuleLinkBase() = default;

  virtual bool isResolved() const = 0;
  operator bool () { return isResolved(); }
  bool isSet() const { return !targetUid.empty(); }
 private:
 protected:
  virtual void print(std::ostream & o) const override;
  void checkAndAnnounceResolvedLink(const Module * to, bool convertsToRequiredType);
  std::string targetUid;
  bool required;
};

template <typename Interface>
struct ModuleLink : public ModuleLinkBase {
  using ModuleLinkBase::ModuleLinkBase;

  operator Interface & () { return get(); }
  operator const Interface & () const { return get(); }

  Interface & get() {
    if(!isResolved()){
      throw std::runtime_error("Resolving an unresolved link : " + toString());
    }
    return *ptr;
  }
  const Interface & get() const {
    return const_cast<ModuleLink*>(this)->get();
  }

  void resolve(const ModuleRegistry & reg) override {
    if(!targetUid.empty()){
      Module * mp = reg.getModulePtr(targetUid);
      if(mp) ptr = mp->ptrAs<Interface>();
      checkAndAnnounceResolvedLink(mp, ptr != nullptr);
    }
  }
  virtual bool isResolved() const override {
    return ptr;
  }
 private:
  Interface * ptr = nullptr;
};


} /* namespace calibration */
} /* namespace aslam */

namespace std {
template <typename> class hash;
template <>
struct hash<std::reference_wrapper<aslam::calibration::Activatable> > {
 public :
  size_t operator()(const std::reference_wrapper<aslam::calibration::Activatable> & ref) const {
    return hash<size_t>()(reinterpret_cast<size_t>(&ref));
  }
};
};

#endif /* INCLUDE_ASLAM_CALIBRATION_MODULE_H_ */
