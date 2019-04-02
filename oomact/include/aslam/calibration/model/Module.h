#ifndef INCLUDE_ASLAM_CALIBRATION_MODULE_H_
#define INCLUDE_ASLAM_CALIBRATION_MODULE_H_

#include <functional>
#include <iosfwd>
#include <string>

#include <sm/value_store/ValueStore.hpp>

#include <aslam/calibration/calibrator/CalibratorRef.h>
#include <aslam/calibration/data/StorageI.h>
#include <aslam/calibration/tools/Named.h>

namespace boost {
  template<typename T> class shared_ptr;
}

namespace aslam {
namespace backend {
class ErrorTermReceiver;
}
namespace calibration {
using backend::ErrorTermReceiver;
class Activator;
class BatchStateReceiver;
class CalibrationConfI;
class CalibratorI;
class DesignVariableReceiver;
class Model;
class Sensor;

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

std::string normalizeParamName(const char * parameter);

class Module;

class ModuleStorage : public StorageI<const Module*>, public CalibratorRef {
 public:
  template <typename Value> using Connector = StorageConnector<Value, ModuleStorage>;
  using CalibratorRef::CalibratorRef;
};

class ModuleBase {
 public:
  virtual ~ModuleBase();
  virtual const Module & getModule() const = 0;
  Module & getModule();
};

class Module : public virtual ModuleBase, public virtual Named, public virtual Used, public IsA<Module> {
 public:
  Module(Model & model, const std::string & name, sm::value_store::ValueStoreRef config, bool isUsedByDefault = true);
  Module(const Module & m);

  virtual void setCalibrationActive(const CalibrationConfI & ec);
  virtual bool initState(CalibratorI & calib);
  virtual void addToBatch(const Activator & stateActivator, BatchStateReceiver & batchStateReceiver, DesignVariableReceiver & problem);

  virtual void clearMeasurements(ModuleStorage & storage);
  virtual void clearMeasurements(); //TODO Deprecate in favor of clearMeasurements(ModuleStorage & storage); and make that one const. AND remove all the non storage compat functions.
  virtual void addErrorTerms(CalibratorI & calib, const ModuleStorage & storage, const CalibrationConfI & ec, ErrorTermReceiver & errorTermReceiver) const;
  virtual void preProcessNewWindow(CalibratorI & calib);
  virtual void writeSnapshot(const CalibrationConfI & ec, bool stateWasUpdatedSinceLastTime) const;

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

  virtual const Module & getModule() const override;

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

  bool shouldObserveOnly(const CalibrationConfI& ec) const;

  bool operator == (const Module & other) const {
    return this == &other;
  }
 protected:
  virtual void writeConfig(std::ostream & out) const;
  virtual void registerWithModel();

  virtual void setActive(bool spatial, bool temporal);
  virtual bool isCalibrationIntended(const CalibrationConfI & ec) const;

 private:
  friend class Model;

  sm::value_store::ValueStoreRef myConfig;

  void resolveLinks(ModuleRegistry & reg);

  virtual void addErrorTerms(CalibratorI & calib, const CalibrationConfI & ec, ErrorTermReceiver & errorTermReceiver) const;
  virtual void addMeasurementErrorTerms(CalibratorI & calib, const ModuleStorage & storage, const CalibrationConfI & ec, ErrorTermReceiver & problem, bool observeOnly) const;
  virtual void addMeasurementErrorTerms(CalibratorI & calib, const CalibrationConfI & ec, ErrorTermReceiver & problem, bool observeOnly) const;
  void setUid(const std::string& uid) { uid_ = uid; }

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
  ModuleLinkBase(const std::string & ownerName, const std::string & linkName,
                 const std::string targetUid, bool required = false);
  ModuleLinkBase(const std::string& ownerName, sm::value_store::ValueStoreRef config,
                 const std::string& linkName, bool required = false);
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
class ModuleLink : public ModuleLinkBase {
 public:
  using ModuleLinkBase::ModuleLinkBase;

  operator Interface & () { return get(); }
  operator const Interface & () const { return get(); }

  Interface & get() {
    if(!isResolved()){
      throw std::runtime_error("Attempt to follow an unresolved link : " + toString());
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

#endif /* INCLUDE_ASLAM_CALIBRATION_MODULE_H_ */
