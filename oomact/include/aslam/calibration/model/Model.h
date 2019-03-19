#ifndef H9DEA9F60_8E57_4BD5_979F_FDAFFC66C3AD
#define H9DEA9F60_8E57_4BD5_979F_FDAFFC66C3AD
#include <map>
#include <vector>
#include <functional>

#include <sm/value_store/ValueStore.hpp>
#include <aslam/backend/TransformationExpression.hpp>

#include <aslam/calibration/model/CalibrationVariable.h>
#include <aslam/calibration/model/Module.h>
#include <aslam/calibration/model/fragments/Gravity.h>
#include <aslam/calibration/SensorId.h>
#include <aslam/calibration/Timestamp.h>
#include <aslam/calibration/tools/ConfigPathResolver.h>

namespace aslam {
namespace backend {
class ErrorTermReceiver;
}
namespace calibration {
class Sensor;
class Joint;
class OptimizationProblem;
struct BoundedTimeExpression;

struct ModelSimplification {
  ModelSimplification(bool needGlobalPosition = true, bool needGlobalOrientation = true) : needGlobalPosition(needGlobalPosition), needGlobalOrientation(needGlobalOrientation) {
  }
  bool needGlobalPosition;
  bool needGlobalOrientation;
};

class ModelAtTimeImpl;

class Frame : public virtual Named {
 public:
  bool operator == (const Frame & other) const { return this == &other; }
};

class ModelAtTimeImpl {
 public:
  virtual aslam::backend::TransformationExpression getTransformationToFrom(const Frame & to, const Frame & in) const = 0;
  virtual aslam::backend::EuclideanExpression getAcceleration(const Frame & of, const Frame & in) const = 0;
  virtual aslam::backend::EuclideanExpression getVelocity(const Frame & of, const Frame & in) const = 0;
  virtual aslam::backend::EuclideanExpression getAngularVelocity(const Frame & of, const Frame & in) const = 0;
  virtual aslam::backend::EuclideanExpression getAngularAcceleration(const Frame & of, const Frame & in) const = 0;

  virtual ~ModelAtTimeImpl() = default;

  template <typename T> T & as() { assert(dynamic_cast<T*>(this)); return static_cast<T&>(*this); };
  template <typename T> const T & as() const { assert(dynamic_cast<const T*>(this)); return static_cast<const T&>(*this); };
};

class ModelAtTime {
 public:
  aslam::backend::TransformationExpression getTransformationToFrom(const Frame & to, const Frame & from) const {
    return impl_->getTransformationToFrom(to, from);
  }
  aslam::backend::EuclideanExpression getAcceleration(const Frame & of, const Frame & in) const {
    return impl_->getAcceleration(of, in);
  }
  aslam::backend::EuclideanExpression getVelocity(const Frame & of, const Frame & in) const {
    return impl_->getVelocity(of, in);
  }
  aslam::backend::EuclideanExpression getAngularAcceleration(const Frame & of, const Frame & in) const {
    return impl_->getAngularAcceleration(of, in);
  }
  aslam::backend::EuclideanExpression getAngularVelocity(const Frame & of, const Frame & in) const {
    return impl_->getAngularVelocity(of, in);
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
  template <typename Sensor_>
  std::vector<std::reference_wrapper<const Sensor_> > getSensors() const;
  template <typename Sensor_>
  std::vector<std::reference_wrapper<Sensor_> > getSensors();

  const std::vector<std::reference_wrapper<Joint>> & getJoints() const { return joints; }
  const std::vector<std::reference_wrapper<Module>> & getModules() const { return modules; }
  const Module & getModule(const std::string & uid) const { return id2moduleMap.at(uid); }
  Module & getModule(const std::string & uid) { return id2moduleMap.at(uid); }
  Module * getModulePtr(const std::string & uid) const override {
    auto it = id2moduleMap.find(uid);
    return it == id2moduleMap.end() ? nullptr : &it->second.get();
  }

  virtual void init();

  const std::string& getSensorName(SensorId id) const;

  void addModule(Module & module);
  template <typename Module_, typename ... Modules_>
  void addModules(Module_ & module, Modules_ & ... modules){
    addModule(static_cast<Module&>(module));
    addModules(modules...);
  }
  template <typename ... Modules_>
  void addModulesAndInit(Modules_ & ... modules){
    addModules(modules...);
    init();
  }

  void addCalibrationVariables(std::initializer_list<boost::shared_ptr<CalibrationVariable>> cvs);

  void updateCVIndices();

  const std::string resolveConfigPath(const std::string & path) const;

  /// Adds the odometry design variables to the batch
  void addToBatch(std::function<void(CalibrationVariable*)> addCalibrationVariable);

  void addCalibPriors(backend::ErrorTermReceiver & errorTermReceiver);

  Eigen::VectorXd getParameters() const;

  void print(std::ostream& stream) const override;

  virtual ModelAtTime getAtTime(Timestamp timestamp, int maximalDerivativeOrder, const ModelSimplification & simplification) const;
  virtual ModelAtTime getAtTime(const BoundedTimeExpression & boundedTimeExpresion, int maximalDerivativeOrder, const ModelSimplification & simplification) const;

  const Gravity& getGravity() const;
  Gravity& getGravity();

  const ValueStoreRef& getConfig() const;

  std::ostream & printCalibrationVariables(std::ostream& out) const;
 protected:
  std::vector<boost::shared_ptr<CalibrationVariable>> calibrationVariables;
  virtual void registerModule(Module & m);
 private:
  void registerSensor(Sensor& s);
  void registerJoint(Joint & j);

  void addModules() {}

  std::vector<std::unique_ptr<Named>> ownedNOs;

  const std::shared_ptr<ConfigPathResolver> configPathResolver;

  ValueStoreRef config_;

  std::vector<std::reference_wrapper<Sensor>> sensors;
  std::map<SensorId, std::reference_wrapper<Sensor>> id2sensorMap;

  std::vector<std::reference_wrapper<const Frame>> frames;
  std::map<std::string, std::reference_wrapper<const Frame>> id2framesMap;

  std::vector<std::reference_wrapper<Joint>> joints;

  std::vector<std::reference_wrapper<Module>> modules;
  std::map<std::string, std::reference_wrapper<Module>> id2moduleMap;

  std::unique_ptr<Gravity> gravity;
};

template<typename Sensor_>
inline std::vector<std::reference_wrapper<Sensor_>> Model::getSensors()
{
  static_assert(std::is_base_of<Sensor, Sensor_>::value, "Only sensors may be given as type parameter for getSensors<>()!");
  std::vector<std::reference_wrapper<Sensor_> > ret;
  for (Module& s: getModules()) {
    if (auto p = s.ptrAs<Sensor_>()) {
      ret.emplace_back(*p);
    }
  }
  return ret;
}
template<typename Sensor_>
inline std::vector<std::reference_wrapper<const Sensor_>> Model::getSensors() const
{
  return const_cast<Model*>(this)->getSensors<const Sensor_>();
}

}
}

#endif /* H9DEA9F60_8E57_4BD5_979F_FDAFFC66C3AD */
