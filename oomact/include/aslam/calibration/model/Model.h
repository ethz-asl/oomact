#ifndef H9DEA9F60_8E57_4BD5_979F_FDAFFC66C3AD
#define H9DEA9F60_8E57_4BD5_979F_FDAFFC66C3AD
#include <map>
#include <vector>
#include <functional>

#include <aslam/calibration/CommonTypes.hpp>
#include <aslam/calibration/model/CalibrationVariable.h>
#include <aslam/calibration/model/Module.h>
#include <aslam/calibration/tools/ConfigPathResolver.hpp>
#include <aslam/backend/TransformationExpression.hpp>
#include <aslam/calibration/model/fragments/Gravity.h>
#include <sm/value_store/ValueStore.hpp>

namespace aslam {
namespace backend {
class ErrorTermReceiver;
}
namespace calibration {
class Sensor;
class Joint;
class OptimizationProblem;
class BoundedTimeExpression;

struct ModelSimplification {
  ModelSimplification(bool needGlobalPosition = true, bool needGlobalOrientation = true) : needGlobalPosition(needGlobalPosition), needGlobalOrientation(needGlobalOrientation) {
  }
  bool needGlobalPosition;
  bool needGlobalOrientation;
};

class ModelAtTimeImpl;

struct Frame : public virtual Named {
  bool operator == (const Frame & other) const { return this == &other; }
};

class ModelAtTimeImpl {
 public:
  virtual aslam::backend::TransformationExpression getTransformationToFrom(const Frame & to, const Frame & from) const = 0;
  virtual aslam::backend::EuclideanExpression getAcceleration(const Frame & to, const Frame & frame) const = 0;
  virtual aslam::backend::EuclideanExpression getVelocity(const Frame & to, const Frame & frame) const = 0;
  virtual aslam::backend::EuclideanExpression getAngularVelocity(const Frame & to, const Frame & frame) const = 0;
  virtual aslam::backend::EuclideanExpression getAngularAcceleration(const Frame & to, const Frame & frame) const = 0;

  virtual ~ModelAtTimeImpl() = default;

  template <typename T> T & as() { assert(dynamic_cast<T*>(this)); return static_cast<T&>(*this); };
  template <typename T> const T & as() const { assert(dynamic_cast<const T*>(this)); return static_cast<const T&>(*this); };
};

class ModelAtTime {
 public:
  aslam::backend::TransformationExpression getTransformationToFrom(const Frame & to, const Frame & from) const {
    return impl_->getTransformationToFrom(to, from);
  }
  aslam::backend::EuclideanExpression getAcceleration(const Frame & to, const Frame & frame) const {
    return impl_->getAcceleration(to, frame);
  }
  aslam::backend::EuclideanExpression getVelocity(const Frame & to, const Frame & frame) const {
    return impl_->getVelocity(to, frame);
  }
  aslam::backend::EuclideanExpression getAngularAcceleration(const Frame & to, const Frame & frame) const {
    return impl_->getAngularAcceleration(to, frame);
  }
  aslam::backend::EuclideanExpression getAngularVelocity(const Frame & to, const Frame & frame) const {
    return impl_->getAngularVelocity(to, frame);
  }

  template <typename T> T & as() { assert(impl_); return impl_->as<T>(); };
  template <typename T> const T & as() const { assert(impl_); return impl_->as<T>(); };

  ModelAtTime(std::unique_ptr<ModelAtTimeImpl> && impl) : impl_(std::move(impl)) {}
 private:
  std::unique_ptr<ModelAtTimeImpl> impl_;
};

class Model : public ModuleRegistry, public Printable, public IsA<Model> {
 public:
  typedef sm::value_store::ValueStoreRef ValueStoreRef;

  /// Batch shared pointer
  typedef boost::shared_ptr<OptimizationProblem> BatchSP;

  Model(ValueStoreRef config, std::shared_ptr<ConfigPathResolver> configPathResolver, const std::vector<const Frame *> frames = {});

  Sensor & getSensor(SensorId id);
  const Sensor & getSensor(SensorId id) const { return const_cast<Model*>(this)->getSensor(id); }

  SensorId createNewSensorId();

  void addFrame(const Frame& frame);
  const Frame& createFrame(const std::string & name);
  const Frame& getOrCreateFrame(const std::string & name);
  const Frame& getFrame(const std::string & name) const;

  const std::vector<boost::shared_ptr<CalibrationVariable>> & getCalibrationVariables() const { return calibrationVariables; }
  const std::vector<std::reference_wrapper<Sensor>> & getSensors() const { return sensors; }
  std::vector<std::reference_wrapper<const Sensor> > getSensors(SensorType type) const;

  const std::vector<std::reference_wrapper<Joint>> & getJoints() const { return joints; }
  const std::vector<std::reference_wrapper<Module>> & getModules() const { return modules; }
  const Module & getModule(const std::string & uid) const { return id2moduleMap.at(uid); }
  Module & getModule(const std::string & uid) { return id2moduleMap.at(uid); }
  Module * getModulePtr(const std::string & uid) const override {
    auto it = id2moduleMap.find(uid);
    return it == id2moduleMap.end() ? nullptr : &it->second.get();
  }

  virtual void resolveAllLinks();

  const std::string& getSensorName(SensorId id) const;

  void registerSensor(Sensor& s);

  void registerJoint(Joint & s){
    joints.emplace_back(s);
  }
  virtual void registerModule(Module & m){
    modules.emplace_back(m);
    m.setUid(m.getName()); //TODO A ensure uniqueness for modules uids!!
    id2moduleMap.emplace(m.getUid(), m);
  }

  void addCalibrationVariables(std::initializer_list<boost::shared_ptr<CalibrationVariable>> cvs);
  void updateCVIndices();

  const std::string resolveConfigPath(const std::string & path) const;

  /// Adds the odometry design variables to the batch
  void addToBatch(std::function<void(CalibrationVariable*)> addCalibrationVariable);

  void addCalibPriors(backend::ErrorTermReceiver & errorTermReceiver);

  Eigen::VectorXd getParameters() const;

  void print(std::ostream& stream) const override;

  virtual ModelAtTime getAtTime(sm::timing::NsecTime timestamp, int maximalDerivativeOrder, const ModelSimplification & simplification) const;
  virtual ModelAtTime getAtTime(const BoundedTimeExpression & boundedTimeExpresion, int maximalDerivativeOrder, const ModelSimplification & simplification) const;

  const Gravity& getGravity() const {
    assert(gravity);
    return *gravity;
  }
  Gravity& getGravity() {
    assert(gravity);
    return *gravity;
  }

  std::ostream & printCalibrationVariables(std::ostream& out) const;

 protected:
  std::vector<boost::shared_ptr<CalibrationVariable>> calibrationVariables;
 private:
  std::vector<std::unique_ptr<Named>> ownedNOs;

  const std::shared_ptr<ConfigPathResolver> configPathResolver;

  std::vector<std::reference_wrapper<Sensor>> sensors;
  std::map<SensorId, std::reference_wrapper<Sensor>> id2sensorMap;

  std::vector<std::reference_wrapper<const Frame>> frames;
  std::map<std::string, std::reference_wrapper<const Frame>> id2framesMap;

  std::vector<std::reference_wrapper<Joint>> joints;

  std::vector<std::reference_wrapper<Module>> modules;
  std::map<std::string, std::reference_wrapper<Module>> id2moduleMap;

  std::unique_ptr<Gravity> gravity;
};

}
}

#endif /* H9DEA9F60_8E57_4BD5_979F_FDAFFC66C3AD */