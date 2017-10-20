#include <aslam/calibration/model/Module.h>

#include <memory>

#include <glog/logging.h>
#include <sm/assert_macros.hpp>
#include <sm/value_store/LayeredValueStore.hpp>
#include <sm/value_store/PrefixedValueStore.hpp>
#include <sm/value_store/VerboseValueStore.hpp>

#include <aslam/calibration/calibrator/CalibrationConfI.h>
#include <aslam/calibration/data/StorageI.h>
#include <aslam/calibration/model/Model.h>
#include <aslam/calibration/model/ModuleTools.h>
#include <aslam/calibration/tools/tools.h>
#include <aslam/calibration/tools/TypeName.h>

namespace aslam {
namespace calibration {

std::string normalizeParamName(const char * parameter){
  static const std::string IgnorePrefixCandiates [] {
      "_", "get", "is"
  };

  std::string p(parameter);

  const auto dotPos = p.rfind(".");
  const auto arrowPos = p.rfind("->");
  const bool hasDot = dotPos != p.npos;
  const bool hasArrow = arrowPos != p.npos;
  if (hasDot || hasArrow) {
    auto pos = hasDot ? dotPos + 1 : arrowPos + 2;
    if (hasDot && hasArrow) {
      pos = std::max(dotPos + 1, arrowPos + 2);
    }
    p = p.substr(pos);
  }
  for(const auto & prefix : IgnorePrefixCandiates){
    if(p.compare(0, prefix.size(), prefix) == 0){
      if(p.size() > prefix.size()){
        if(!isalpha(*prefix.rbegin()) || isupper(p[prefix.size()])) {
          p = p.substr(prefix.size());
          p[0] = tolower(p[0]);
        }
      }
    }
  }
  const auto poseCall = p.find("(");
  if(poseCall != p.npos){
    p = p.substr(0, poseCall);
  }
  return (p.empty() || p.back() != '_') ? p : p.substr(0, p.size() - 1);
}

std::string stripFolder(std::string path){
  boost::filesystem::path p(path);
  return p.filename().string();
}

std::string getSibling(std::string path, std::string name){
  boost::filesystem::path p(path);
  return (p.parent_path() / name).normalize().string();
}

sm::value_store::ValueStoreRef addVerboseLayer(const std::string & name, sm::value_store::ValueStoreRef vs, bool force = false){
  if(force || vs.getBool("debugConfig", false)){
    if(!force){
      LOG(INFO) << "Adding verbosity layer to " + stripFolder(name) + "'s configuration.";
    }
    return sm::value_store::ValueStoreRef(std::make_shared<sm::value_store::VerboseValueStore>(vs, [name](std::string v){
      LOG(INFO) << name << " configuration: "<< v;
    }));
  }
  return vs;
}

std::shared_ptr<VerboseValueStore> getVerboseValueStore(const std::shared_ptr<ValueStore> & vs){
  return std::dynamic_pointer_cast<VerboseValueStore>(vs);
}


Module::Module(Model & model, const std::string & name, sm::value_store::ValueStoreRef config, bool isUsedByDefault) :
    model_(model),
    name_(stripFolder(name)),
    myConfig_(addVerboseLayer(getName(), (config.isEmpty() ? model.getConfig() : config).getChild(name))),
    used_(myConfig_.getBool("used", isUsedByDefault))
{
  std::string parent, layeredNameStr, lastName = getName();
  std::shared_ptr<sm::value_store::LayeredValueStore> lvs;
  auto currentConfig = getMyConfig();
  while (!(parent = currentConfig.getString("parent", std::string())).empty()) {
    CHECK(config.isEmpty()) << "Configuration inheritance is not supported for module specific configuration."; // TODO D support configuration inheritance for module special configurations
    LOG(INFO) << "Module " << getName() << " inherits configuration from " << parent << ".";
    if(!lvs) {
      lvs = std::make_shared<LayeredValueStore>();
      auto myVs = getMyConfig().getValueStoreSharedPtr();
      if(auto p = getVerboseValueStore(myVs)){
        myVs = p->getUnderlyingValueStore();
      }
      lvs->add(myVs);
      layeredNameStr = getName();
    }
    currentConfig = model.getConfig().getChild(getSibling(name, parent));
    lastName = stripFolder(parent);
    layeredNameStr += "|" + lastName;
    lvs->add(currentConfig.getValueStoreSharedPtr());
  }
  if(lvs){
    if(getVerboseValueStore(getMyConfig())){
      myConfig_ = addVerboseLayer(layeredNameStr, ValueStoreRef(lvs), true);
    } else {
      myConfig_ = ValueStoreRef(lvs);
    }
  }
}

Module::Module(const Module& m) :
  model_(m.model_),
  name_(m.name_),
  myConfig_(m.myConfig_),
  used_(m.used_)
{
  LOG(WARNING) << "Module " << m << " got copied!";
  CHECK(!m.isRegistered_) << "A registered module must not be copied!";
}


bool Module::initState(CalibratorI& /*calib*/) {
  return true;
}

void Module::addToBatch(const Activator & /*stateActivator*/, BatchStateReceiver & /*batchStateReceiver*/, DesignVariableReceiver & /*problem*/) {
}

void Module::registerWithModel() {
  CHECK(isUsed());
  SM_ASSERT_TRUE(std::runtime_error, !isRegistered_, "Only register a module once! (name=" + name_ + ")");
  isRegistered_ = true;
}

void Module::clearMeasurements(ModuleStorage& storage) {
  storage.remove(this);
  clearMeasurements();
}

void Module::clearMeasurements() {
}

bool Module::shouldObserveOnly(const CalibrationConfI& ec) const {
  const bool observeOnly = isA<Observer>() && as<Observer>().isObserveOnly();
  const bool errorTermsInactive = isA<Activatable>() && !ec.getErrorTermActivator().isActive(as<Activatable>());
  LOG(INFO) << getName() <<  " shouldObserveOnly: observe only=" << observeOnly << ", error terms inactive=" << errorTermsInactive;
  return observeOnly || errorTermsInactive;
}


void Module::addErrorTerms(CalibratorI& /*calib*/, const CalibrationConfI& /*ec*/, ErrorTermReceiver& /*errorTermReceiver*/) const {
}

void Module::addErrorTerms(CalibratorI& calib, const ModuleStorage & storage, const CalibrationConfI & ec, ErrorTermReceiver & problem) const {
  if(isUsed()){
    addErrorTerms(calib, ec, problem);
    const bool observeOnly = shouldObserveOnly(ec);
    LOG(INFO) << "Adding measurement" << (observeOnly ? " observer" : "") << " error terms for module " << getName() << ".";
    addMeasurementErrorTerms(calib, storage, ec, problem, observeOnly);
    addMeasurementErrorTerms(calib, ec, problem, observeOnly);
  }
}

void Module::addMeasurementErrorTerms(CalibratorI& /*calib*/, const ModuleStorage & /*storage*/, const CalibrationConfI & /*ec*/, ErrorTermReceiver & /*problem*/, bool /*observeOnly*/) const {
}
void Module::addMeasurementErrorTerms(CalibratorI& /*calib*/, const CalibrationConfI & /*ec*/, ErrorTermReceiver & /*problem*/, bool /*observeOnly*/) const {
}

void Module::writeInfo(std::ostream& out) const {
  out << getName() << "(uid=" << getUid();
  MODULE_WRITE_PARAM(used_);
  if(auto p = ptrAs<Observer>()){ MODULE_WRITE_PARAM(p->isObserveOnly());}
  if(auto p = ptrAs<Calibratable>()){MODULE_WRITE_PARAM(p->isToBeCalibrated());}
  writeConfig(out);
  out << ")";
}
void Module::writeConfig(std::ostream& /*out*/) const {
}

ObserverMinimal::ObserverMinimal(const Module * module) :
    observeOnly_(module->getMyConfig().getBool("observeOnly", false))
{
}

//TODO C move the next three to Calibrateable interface
void Module::setCalibrationActive(const CalibrationConfI& ec) {
  const auto & activator = ec.getCalibrationActivator();
  const bool active =
         (!isA<Activatable>() || activator.isActive(as<Activatable>()))
      && (!isA<Observer>() || !as<Observer>().isObserveOnly())
      && (!isA<Calibratable>() || as<Calibratable>().isToBeCalibrated())
      && isCalibrationIntended(ec);
  setActive(active && ec.isSpatialActive(), active && ec.isTemporalActive());
  getModel().updateCVIndices();
}
bool Module::isCalibrationIntended(const CalibrationConfI& /*ec*/) const {
  return true;
}
void Module::setActive(bool /*spatial*/, bool /*temporal*/) {
}

CalibratableMinimal::CalibratableMinimal(const Module * module) :
  toBeCalibrated_(module->getMyConfig().getBool("estimate", true))
{
}

class AllActiveActivatorImpl : public Activator{
  virtual bool isActive(const Activatable & ) const {
    return true;
  }
} AllActiveActivatorObj;

const Activator & AllActiveActivator = AllActiveActivatorObj;

void ModuleLinkBase::checkAndAnnounceResolvedLink(const Module * to, bool convertsToRequiredType) {
  if(to && to->isUsed() && convertsToRequiredType){
    LOG(INFO) << *this << " successfully resolved to " << *to;
  } else {
    const char * problem = to ? (to->isUsed()? " resolves to unused module!" : " resolves to used module but of wrong type!" ) : " could not be resolved!";
    if(required){
      LOG(ERROR) << *this << problem;
      throw std::runtime_error(toString() + problem);
    } else {
      LOG(INFO) << *this << problem;
    }
  }
}
ModuleLinkBase::ModuleLinkBase(const std::string& ownerName, const std::string & linkName, const std::string targetUid, bool required) :
  NamedMinimal(ownerName + "." + linkName), targetUid(targetUid), required(required)
{
  if(required && targetUid.empty()){
    throw std::runtime_error("Empty required Link : " + toString());
  }
}

ModuleLinkBase::ModuleLinkBase(Module & owner, const std::string& linkName, bool required) :
  ModuleLinkBase(owner.getName(), linkName, required ? owner.getMyConfig().getString(linkName) : owner.getMyConfig().getString(linkName, std::string()), required)
{
  owner.moduleLinks_.push_back(*this);
}

void ModuleLinkBase::print(std::ostream& o) const {
  o << "ModuleLink(" << getName() << "->" << (targetUid.empty() ? "NONE" : targetUid ) << ")";
}

void Module::resolveLinks(ModuleRegistry & reg) {
  for(ModuleLinkBase & l : moduleLinks_){
    l.resolve(reg);
  }
}

bool Module::hasTooFewMeasurements() const {
  return false;
}

void Module::estimatesUpdated(CalibratorI& /*calib*/) const {
}

ModuleBase::~ModuleBase() {
}

Module& ModuleBase::getModule() {
  return const_cast<Module&>(static_cast<const ModuleBase*>(this)->getModule());
}

const Module& Module::getModule() const {
  return *this;
}

void Module::writeSnapshot(const CalibrationConfI & /*ec*/, bool /*stateWasUpdatedSinceLastTime*/) const {
}

} /* namespace calibration */
} /* namespace aslam */
