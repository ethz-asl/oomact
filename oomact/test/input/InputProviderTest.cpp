#include <gtest/gtest.h>
#include <memory>

#include <aslam/calibration/calibrator/CalibratorI.h>
#include <aslam/calibration/input/InputProviderI.h>
#include <aslam/calibration/data/ObservationManagerI.h>
#include <aslam/calibration/model/FrameGraphModel.h>
#include <aslam/calibration/model/Model.h>
#include <aslam/calibration/model/PoseTrajectory.h>
#include <aslam/calibration/model/fragments/So3R3Trajectory.h>
#include <aslam/calibration/model/sensors/PoseSensor.h>
#include <aslam/calibration/test/MockMotionCaptureSource.h>
#include <aslam/calibration/tools/SmartPointerTools.h>

using namespace aslam;
using namespace aslam::calibration;

class MockFeederI {
 public:
  virtual ~MockFeederI() = default;
  virtual void feed(Timestamp at, ModuleStorage & storage) const = 0;
  virtual const Sensor & getSensor() const = 0;
};

class SimpleMockFeeder : public MockFeederI {
 public:
  SimpleMockFeeder(const Sensor & sensor, std::function<void(Timestamp, ModuleStorage &)> feeder) : sensor_(sensor), feeder_(feeder){}
  virtual void feed(Timestamp at, ModuleStorage & storage) const override {
    feeder_(at, storage);
  }
  virtual const Sensor & getSensor() const override {
    return sensor_;
  }
 private:
  const Sensor & sensor_;
  std::function<void(Timestamp, ModuleStorage &)> feeder_;
};

class FeederFactoryI {
 public:
  virtual ~FeederFactoryI() = default;
  virtual bool matches(const Sensor & s) const = 0;
  virtual std::unique_ptr<MockFeederI> create(const Sensor & s) const = 0;
};

class SimpleFeederFactory : public FeederFactoryI {
 public:
  SimpleFeederFactory(std::function<std::unique_ptr<MockFeederI>(const Sensor &s)> sensorFeederCreator) : sensorFeederCreator_(sensorFeederCreator) {}
  virtual ~SimpleFeederFactory() = default;
  bool matches(const Sensor & s) const override {
    return sensorFeederCreator_(s) != nullptr;
  }
  std::unique_ptr<MockFeederI> create(const Sensor & s) const override {
    return sensorFeederCreator_(s);
  }
 private:
  std::function<std::unique_ptr<MockFeederI>(const Sensor &s)> sensorFeederCreator_;
};

class MockInputProvider : public InputProviderI {
 public:
  MockInputProvider(std::shared_ptr<const Model> model) : model_(model)
  {
  }

  void add(std::unique_ptr<FeederFactoryI> ptr) {
    feederFactories_.emplace_back(std::move(ptr));
  }
  void add(FeederFactoryI * ptr) {
    feederFactories_.emplace_back(std::move(ptr));
  }
  void add(std::function<MockFeederI *(const Sensor &s)> feederFactory) {
    feederFactories_.emplace_back(new SimpleFeederFactory([feederFactory](const Sensor &s) {
      return std::unique_ptr<MockFeederI>(feederFactory(s));
    }));
  }

  static constexpr Timestamp TimeInc = 0.01;

  void init(){
    auto && sensors = model_->getSensors();
    for(auto & ff : feederFactories_){
      for(auto s : sensors){
        if(ff->matches(s)){
          feeder_.emplace_back(ff->create(s));
        }
      }
    }
  }
  void simulateData(Timestamp fromTime, Timestamp toTime, ObservationManagerI & obsManager) const {
    auto && storage = obsManager.getCurrentStorage();
    for(auto t = fromTime; t <= toTime + TimeInc; t += TimeInc){
      for(auto & f : feeder_){
        f->feed(t, storage);
        obsManager.addMeasurementTimestamp(t, f->getSensor());
      }
    }
  }
 private:
  std::shared_ptr<const Model> model_;

  std::vector<std::unique_ptr<FeederFactoryI>> feederFactories_;
  std::vector<std::unique_ptr<MockFeederI>> feeder_;
};

constexpr Timestamp MockInputProvider::TimeInc;

TEST(InputProviderSuite, testEasy) {
  auto vs = ValueStoreRef::fromString(
      "Gravity{used=false}, frames=body:world,"
      "a{referenceFrame=body,targetFrame=world,rotation/used=false,translation{used=true,x=0,y=5,z=0},delay/used=false}"
      "traj{frame=body,referenceFrame=world,McSensor=a,initWithPoseMeasurements=true,splines{knotsPerSecond=5,rotSplineOrder=4,rotFittingLambda=0.001,transSplineOrder=4,transFittingLambda=0.001}}"
    );
  auto vsCalib = ValueStoreRef::fromString(
      "verbose=true\n"
      "acceptConstantErrorTerms=true\n"
      "timeBaseSensor=a\n"
    );

  FrameGraphModel m(vs);
  PoseSensor mcSensorA(m, "a", vs);
  PoseTrajectory traj(m, "traj", vs);
  m.addModulesAndInit(mcSensorA, traj);

  EXPECT_EQ(1u, m.getCalibrationVariables().size());
  EXPECT_DOUBLE_EQ(5.0, mcSensorA.getTranslationToParent()[1]);

  auto spModel = to_local_shared_ptr(m);
  auto c = createBatchCalibrator(vsCalib, spModel);
  MockInputProvider ip(spModel);

  ip.add([](const Sensor & s) -> SimpleMockFeeder* {
    auto ptr = s.ptrAs<const PoseSensor>();
    return (ptr && s.getName() == "a") ?
        new SimpleMockFeeder(s, [ptr](Timestamp at, ModuleStorage & storage) {
          auto p = test::MmcsRotatingStraightLine.getPoseAt(at);
          ptr->addMeasurement(p.time, p.q, p.p, storage);
        })
      : nullptr;
  });

  ip.init();
  ip.simulateData(0.0, 1.0, *c);

  c->calibrate();
  EXPECT_NEAR(5.0, mcSensorA.getTranslationToParent()[1], 0.0001);
  EXPECT_NEAR(-5.0, traj.getCurrentTrajectory().getTranslationSpline().template getEvaluatorAt<0>(0).eval()[1], 0.1);
}
