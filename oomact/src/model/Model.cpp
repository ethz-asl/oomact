#include <aslam/calibration/model/Model.h>

#include <atomic>
#include <Eigen/Core>
#include <glog/logging.h>
#include <iostream>
#include <vector>

#include <aslam/backend/DesignVariable.hpp>
#include <aslam/backend/OptimizationProblemBase.hpp>

#include <aslam/calibration/CommonTypes.hpp>
#include <aslam/calibration/model/fragments/Gravity.h>
#include <aslam/calibration/model/Joint.h>
#include <aslam/calibration/model/Module.h>
#include <aslam/calibration/model/Sensor.hpp>

using aslam::calibration::OptimizationProblem;

namespace aslam {
namespace calibration {

class SimpleGravity : public Gravity, public Module, public CalibratableMinimal {
 public:
  SimpleGravity(Model & model, ValueStoreRef vs) :
    Module(model, "Gravity", vs),
    CalibratableMinimal(this),
    g_m(createCVIfUsed<ScalarCv>("magnitude", "m")),
    gravityVectorExpression(isUsed() ? aslam::backend::EuclideanExpression(Eigen::Vector3d::UnitZ().eval()) * g_m->toExpression() : aslam::backend::EuclideanExpression())
  {
  }
  virtual ~SimpleGravity() = default;

  aslam::backend::EuclideanExpression & getVectorExpression() override {
    return gravityVectorExpression;
  }

 protected:
  void setActive(bool spatial, bool /*temporal*/) override {
//  TODO C USE this  const bool active = ec.getCalibrationActivator().isActive(imu); //TODO B this should be nicer (depend on Imu s)
    g_m->setActive(isToBeCalibrated() && spatial);
  }
  virtual void writeConfig(std::ostream & out) const override {
    out << (", " "g_m" "=") << g_m->getValue();
  }
  void registerWithModel() override {
    Module::registerWithModel();
    getModel().addCalibrationVariables({g_m});
  }
 private:
  /// Gravity vector in mapping frame (Reasonably should be around (0,0,9.81)
  ScalarCvSp g_m;
  aslam::backend::EuclideanExpression gravityVectorExpression;
};

Model::Model(ValueStoreRef config, std::shared_ptr<ConfigPathResolver> configPathResolver, const std::vector<const Frame *> frames) :
  configPathResolver(configPathResolver)
{
  for(auto f : frames){
    if(f) addFrame(*f);
  }

  aslam::calibration::SimpleGravity* simpleGravity = new SimpleGravity(*this, config);
  gravity.reset(simpleGravity);
  add(*simpleGravity);
}

std::ostream & Model::printCalibrationVariables(std::ostream& out) const {
  for (auto& c : calibrationVariables) {
    c->printValuesNiceInto(out);
  }
  return out;
}

void Model::print(std::ostream& out) const {
  out << typeid(*this).name() << ":" << std::endl;
  for(Module & m : getModules()){
    m.writeInfo(out );
    out << std::endl;
  }
  out << "Calibration:" << std::endl;
  printCalibrationVariables(out);
}

void Model::init() {
  for (Module& m : getModules()) {
    m.resolveLinks(*this);
  }
}

void Model::registerModule(Module & m){
  CHECK_EQ(&m.getModel(), this) << " : Module " << m.getName() << " was created with a different model!";

  modules.emplace_back(m);
  m.setUid(m.getName()); //TODO A ensure uniqueness for modules uids!!
  id2moduleMap.emplace(m.getUid(), m);

  m.registerWithModel();

  if(auto p = m.ptrAs<Sensor>()){
    registerSensor(*p);
  }
  if(auto p = m.ptrAs<Joint>()){
    registerJoint(*p);
  }
}

void Model::registerSensor(Sensor& s) {
  id2sensorMap.emplace(s.getId(), s);
  sensors.emplace_back(s);
}

void Model::registerJoint(Joint& j) {
  joints.emplace_back(j);
}

std::vector<std::reference_wrapper<const Sensor>> Model::getSensors(SensorType type) const
{
  std::vector<std::reference_wrapper<const Sensor> > ret;
  for (const Sensor& s: getSensors()) {
    if (s.getType() == type)
    ret.emplace_back(s);
  }
  return ret;
}

Sensor& Model::getSensor(SensorId id) {
  auto it = id2sensorMap.find(id);
  if (it == id2sensorMap.end()){
    std::stringstream s;
    s << "Illegal sensor id used:" << id << "!";
    throw std::runtime_error(s.str());
  }
  return it->second;
}

const std::string& Model::getSensorName(SensorId id) const {
  return getSensor(id).getName();
}

SensorId Model::createNewSensorId() {
  static std::atomic<size_t> sensorCounter(0);
  return SensorId(sensorCounter++);
}

struct GenericFrame : public Frame, public NamedMinimal {
  using NamedMinimal::NamedMinimal;
};


void Model::addFrame(const Frame& frame) {
  const auto & name = frame.getName();
  if(id2framesMap.count(name)){
    throw std::runtime_error("A frame with name " + name + " already exists.");
  }
  frames.emplace_back(frame);
  id2framesMap.emplace(name, frame);
}

const Frame & Model::createFrame(const std::string& name) {
  Frame * f = new GenericFrame(name);
  addFrame(*f);

  ownedNOs.emplace_back(f);
  return *f;
}

const Frame & Model::getOrCreateFrame(const std::string& name) {
  if(!id2framesMap.count(name)){
    return createFrame(name);
  } else {
    return getFrame(name);
  }
}

const Frame & Model::getFrame(const std::string& name) const {
  if(!id2framesMap.count(name)){
    throw std::runtime_error("A frame with name " + name + " doesn't exists.");
  }
  return id2framesMap.at(name);
}


const std::string Model::resolveConfigPath(const std::string& path) const {
  if(configPathResolver){
    std::string ret = configPathResolver->resolve(path);
    LOG(INFO) << "Resolving config path '" << path << "' to '" << ret;
    return ret;
  } else {
    LOG(INFO) << "No resolver for config path '" << path << "'";
    return path;
  }
}



void Model::updateCVIndices() {
  int i = 0;
  for(auto & c : calibrationVariables){
    if(c->getDesignVariable().isActive()){
      c->setIndex(i);
      i += c->getDimension();
    }
    else{
      c->setIndex(-1);
    }
  }
}

void Model::addCalibrationVariables(const std::initializer_list<boost::shared_ptr<CalibrationVariable>> cvs){
  for(const auto & cv: cvs){
    if(cv && !std::count(calibrationVariables.begin(), calibrationVariables.end(), cv)){
      calibrationVariables.push_back(cv);
    }
  }
  updateCVIndices();
}


void Model::addToBatch(std::function<void(CalibrationVariable*)> addCalibrationVariable) {
  for(auto & c: calibrationVariables){
    addCalibrationVariable(c.get());
  }
}


void Model::addCalibPriors(backend::ErrorTermReceiver& errorTermReceiver) {
  for(auto & c: calibrationVariables){
    if(c->isActivated()){
      boost::shared_ptr<aslam::backend::ErrorTerm> e = c->createPriorErrorTerm();
      errorTermReceiver.addErrorTerm(e);
      Eigen::MatrixXd C;
      e->getInvR(C);
      LOG(INFO) << "Prior for " << c->getName() << ": current error=" << e->evaluateError() << " with covariance :\n" << C.inverse() << ".\n";
    }
  }
}

Eigen::VectorXd Model::getParameters() const {
  int numOdometryParameters = 0;
  for(auto & c: calibrationVariables){
    numOdometryParameters += c->getNumParams();
  }

  Eigen::VectorXd params(numOdometryParameters);
  int index = 0;
  for(auto & c: calibrationVariables){
    Eigen::MatrixXd v;
    c->getDesignVariable().getParameters(v);
    assert(v.cols() == 1);
    assert(index + v.rows() <= int(numOdometryParameters));
    params.block(index, 0, v.rows(), 1);
    index += v.rows();
  }
  assert(index == int(numOdometryParameters));
  return params;
}

ModelAtTime Model::getAtTime(sm::timing::NsecTime, int, const ModelSimplification&) const {
  LOG(FATAL) << __PRETTY_FUNCTION__ << " not implemented!";
}

ModelAtTime Model::getAtTime(const BoundedTimeExpression&, int, const ModelSimplification&) const {
  LOG(FATAL) << __PRETTY_FUNCTION__ << " not implemented!";
}

void Model::add(Module& module) {
  if(module.isUsed()){
    registerModule(module);
    CHECK(module.isRegistered_) << module.getName() << " did not register itself!";
  }
}

void Model::addModulesAndInit() {
  init();
}

}
}
