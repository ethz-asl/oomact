#include <aslam/calibration/plan/Plan.h>

#include <atomic>
#include <chrono>
#include <exception>
#include <fstream>
#include <future>
#include <memory>
#include <mutex>
#include <sstream>
#include <thread>
#include <unordered_map>
#include <aslam/calibration/plan/HomingDriver.h>

#include <glog/logging.h>
#include <sm/BoostPropertyTree.hpp>
#include <sm/value_store/PropertyTreeValueStore.hpp>
#include <sm/value_store/ValueStore.hpp>

#include <aslam/calibration/plan/PlanFragmentRegistry.h>
#include "aslam/calibration/tools/tools.h"

namespace aslam {
namespace calibration {
namespace plan {

Plan::Plan() {}

void sleep(double seconds){
  std::this_thread::sleep_for(std::chrono::microseconds((size_t)(seconds * 1e6)));
}

ModuleList staticPointSensors;
ModuleList dynamicPointSensors;
ModuleList motionSensors;

void startLogging(const PlanFragment & frag, Logger& l) {
  std::string fragString = frag.toString();
  std::replace(fragString.begin(), fragString.end(), ' ', '_');
  std::replace(fragString.begin(), fragString.end(), '(', '_');
  std::replace(fragString.begin(), fragString.end(), ')', '_');
  l.start(fragString);
  if (!l.waitForLoggerToBecomeReady(1000)) {
    throw std::runtime_error("Logger timed out!");
  }
}
namespace fragment {

struct Test : public PlanFragment {
  static constexpr const char * Name = "Test";

  Test(const sm::value_store::ValueStoreRef & vs) : PlanFragment(Name) {
    val = vs.getString("");
  }
  void execute(CalibrationServer &, Driver &, Logger &) const override { }

  void print(std::ostream & into) const override {
    PlanFragment::print(into);
    into << "(val=" << val << ")";
  }

  std::string val;
};

void takeStaticPointClouds(CalibrationServer &cs, SmartDriver & d) {
  d.stop();
  sleep(1);

  LOG(INFO) << "Recording point clouds.";

  cs.startDataCollection(staticPointSensors);
  sleep(0.3);
  cs.endDataCollection(staticPointSensors);

  sleep(0.1);
}

void takeDynamicPointClouds(CalibrationServer &cs, SmartDriver & d) {
  d.setVelocity(0.0, 0.5);

  sleep(0.6);
  cs.startDataCollection(dynamicPointSensors);
  sleep(1);
  cs.endDataCollection(dynamicPointSensors);
  sleep(0.1);
  d.stop();
}

struct HomeTest : public PlanFragmentWithHome {
  static constexpr const char * Name = "Test";

  HomeTest(const sm::value_store::ValueStoreRef &vs) : PlanFragmentWithHome(vs, Name)  {}

  void execute(CalibrationServer &cs, Driver &du, Logger & l) const override {
    HomingDriver d(*this, cs, du, l);
    d.forward(1);
    d.turn(0.5);
  }
};

struct GoHome : public PlanFragmentWithHome {
  static constexpr const char * Name = "GoHome";

  GoHome(const sm::value_store::ValueStoreRef &vs) : PlanFragmentWithHome(vs, Name)  {}

  void execute(CalibrationServer &, Driver &du, Logger &) const override {
    if(getHome().empty()){
      SM_THROW(std::runtime_error, "No home given!");
    }
    HomingDriver d(*this, du);
    sleep(0.5);
  }
};

struct GetHome : public PlanFragment {
  std::string targetName;
  GetHome(const sm::value_store::ValueStoreRef &vs) : PlanFragment("GetHome") {
    targetName = vs.getString("name");
    if(targetName.empty()){
      SM_THROW(std::runtime_error, "Target name empty!");
    }
  }

  void execute(CalibrationServer &, Driver &du, Logger &) const override {
    sleep(0.2);
    du.markHome();
    std::string s = du.getHome();
    if(!s.empty()){
      std::stringstream ss;
      ss << s;
      ss << "\n";
      writeStringToFile(targetName, ss.str());
      LOG(INFO) << "Wrote home " << s << " to " << targetName;
    }
  }

  void print(std::ostream & into) const override {
    PlanFragment::print(into);
    into << "(targetName=" << targetName << ")";
  }
};

struct PointCloudTaker : public PlanFragmentWithHome {
  double forwardDist = 0.2;
  double turnAngle = 0.20;
  int repetitions = 2;
  int turnModulo = 1;
  PointCloudTaker(const sm::value_store::ValueStoreRef &vs, std::string name) : PlanFragmentWithHome(vs, name) {
    forwardDist = vs.getDouble("dist", forwardDist);
    turnAngle = vs.getDouble("angle", turnAngle);
    turnModulo = vs.getInt("turnModulo", turnModulo);
    repetitions = vs.getInt("rep", repetitions);
  }

  void execute(CalibrationServer &cs, Driver &du, Logger & l) const override {
    HomingDriver d(*this, cs, du, l);

    double yaw = du.getHomeYaw();

    cs.startDataCollection(motionSensors);

    double accumAngle = 0, yawNorm = 0;

    takePointCloud(cs, d, yaw);
    for(int i = 0; i < repetitions; i ++){
      d.forward(forwardDist);
      takePointCloud(cs, d, yaw);
      if(i != repetitions - 1){
        LOG(INFO) << "i % turnModulo=" << i % turnModulo;
        if((i % turnModulo == 0) && (turnModulo == 1|| i != 0)){
          yaw += (turnAngle * 2 * M_PI);
          accumAngle += turnAngle;
          yawNorm += turnAngle;
          LOG(INFO) << "Incrementing the angle by " << turnAngle << " to " << yawNorm << " w.r.t home.";
          if(du.hasHome()){
            d.turnTo(yaw);
          } else {
            d.turn(turnAngle);
          }
          takePointCloud(cs, d, yaw);
        }
      }
      if(accumAngle > 1){
        accumAngle = 0;
        d.goHome();
      }
    }

    cs.endDataCollection(motionSensors);
  }

  void print(std::ostream & into) const override {
    PlanFragment::print(into);
    into << "(angle=" << turnAngle << ", turnModulo="<< turnModulo << ", dist=" << forwardDist << ", rep=" << repetitions << ")";
  }

  virtual void takePointCloud(CalibrationServer& cs, HomingDriver& d, double currentYaw) const = 0;
};


struct Static : public PointCloudTaker {
  Static(const sm::value_store::ValueStoreRef &vs)
  : PointCloudTaker(vs, "Static") {}

  virtual void takePointCloud(CalibrationServer& cs, HomingDriver& d, double ) const override {
    takeStaticPointClouds(cs, d);
  }
};

struct Dynamic : public PointCloudTaker {
  Dynamic(const sm::value_store::ValueStoreRef &vs)
  : PointCloudTaker(vs, "Dynamic") {}

  virtual void takePointCloud(CalibrationServer& cs, HomingDriver& d, double currentYaw) const override {
    takeDynamicPointClouds(cs, d);
    if(hasHome()) d.turnTo(currentYaw);
  }
};


struct Straight : public PlanFragmentWithHome {
  double forwardDist = 3;
  int repetitions = 2;
  Straight(const sm::value_store::ValueStoreRef &vs) : PlanFragmentWithHome(vs, "Forward") {
    forwardDist = vs.getDouble("dist", forwardDist);
    repetitions = vs.getDouble("rep", repetitions);
  }

  void execute(CalibrationServer &cs, Driver &du, Logger & l) const override {
    HomingDriver d(*this, cs, du, l);

    cs.startDataCollection(motionSensors);

    takeStaticPointClouds(cs, d);

    d.forward(forwardDist);

    takeStaticPointClouds(cs, d);

    cs.endDataCollection(motionSensors);
  }

  void print(std::ostream & into) const override {
    PlanFragment::print(into);
    into << "(dist=" << forwardDist << ", rep=" << repetitions << ")";
  }
};

struct TurningStatic : public PlanFragmentWithHome {
  double yawInc = 0;
  double duration = 0.1;
  int rep;

  TurningStatic(const sm::value_store::ValueStoreRef &vs) : PlanFragmentWithHome(vs, "TurningStatic") {
    yawInc = vs.getDouble("yawInc", yawInc);
    duration = vs.getDouble("dur", duration);
    rep = vs.getInt("rep", rep);
  }

  void execute(CalibrationServer &cs, Driver &du, Logger & l) const override {
    HomingDriver d(*this, cs, du, l, false);

    double yawBase = 0;
    if(hasHome()){
      du.setHome(getHome());
    }
    if(du.hasHome()){
      yawBase = du.getHomeYaw();
    }
    LOG(INFO) << "Using " << yawBase << " as 0 yaw reference.";

    for(int i = 0; i < rep; i ++){
      if(i > 0 || hasHome()){
        d.turnTo(yawInc * i * 2 * M_PI + yawBase);
      }
      d.setVelocity(0, 0);
      sleep(2);
      cs.startDataCollection(staticPointSensors);
      cs.startDataCollection(dynamicPointSensors);
      sleep(duration);
      cs.endDataCollection(dynamicPointSensors);
      cs.endDataCollection(staticPointSensors);
    }
  }

  void print(std::ostream & into) const override {
    PlanFragment::print(into);
    into << "(yawInc=" << yawInc << ", duration=" << duration << ", rep=" << rep << ")";
  }
};



struct ForwardStatic : public PlanFragmentWithHome {
  double dist = 0;
  double duration = 0.1;
  int rep;

  ForwardStatic(const sm::value_store::ValueStoreRef &vs) : PlanFragmentWithHome(vs, "ForwardStatic") {
    dist = vs.getDouble("dist", dist);
    duration = vs.getDouble("dur", duration);
    rep = vs.getInt("rep", rep);
  }

  void execute(CalibrationServer &cs, Driver &du, Logger & l) const override {
    HomingDriver d(*this, cs, du, l, false);

    for(int i = 0; i < rep; i ++){
      if(i > 0){
        d.forward(dist);
      }
      d.setVelocity(0, 0);
      sleep(2);
      cs.startDataCollection(staticPointSensors);
      cs.startDataCollection(dynamicPointSensors);
      sleep(duration);
      cs.endDataCollection(dynamicPointSensors);
      cs.endDataCollection(staticPointSensors);
    }
  }

  void print(std::ostream & into) const override {
    PlanFragment::print(into);
    into << "(dist=" << dist << ", duration=" << duration << ", rep=" << rep << ")";
  }
};


struct Imu : public PlanFragmentWithHome {
  int repetitions = 2;
  double duration = 0.9;

  Imu(const sm::value_store::ValueStoreRef &vs) : PlanFragmentWithHome(vs, "Imu") {
    repetitions = vs.getInt("rep", repetitions);
    duration = vs.getDouble("dur", duration);
  }

  void execute(CalibrationServer &cs, Driver &du, Logger & l) const override {
    HomingDriver d(*this, cs, du, l);

    cs.startDataCollection(motionSensors);

    for(int i = 0; i < repetitions; i ++){
      d.setVelocity(1, 0);
      sleep(duration);
      d.setVelocity(0, 0);
      sleep(0.2);
      if((i + 1) % 3 == 0){
        d.goHome();
      }
    }
    d.setVelocity(0, 0);

    cs.endDataCollection(motionSensors);
  }

  void print(std::ostream & into) const override {
    PlanFragment::print(into);
    into << "(duration=" << duration << ", rep=" << repetitions << ")";
  }
};

struct ImuManual : public PlanFragmentWithHome {
  double duration = 0.9;

  ImuManual(const sm::value_store::ValueStoreRef &vs) : PlanFragmentWithHome(vs, "ImuManual") {
    duration = vs.getDouble("dur", duration);
  }

  void execute(CalibrationServer &cs, Driver &du, Logger & l) const override {
    SmartDriver d(du);
    startLogging(*this, l);
    cs.startDataCollection(motionSensors);

    d.rampTo(0, 1, 1);
    sleep(duration);
    d.rampTo(0, 0, 1);
    d.turnTo(hasHome()? du.getHomeYaw() : 0.0);

    sleep(duration);

    cs.endDataCollection(motionSensors);
    sleep(0.1);
    l.stop();
  }
};


struct Odom : public PlanFragmentWithHome {
  int repetitions = 2;
  double tV= 0.5, rV = 0.8;
  double durationR = 10;
  double durationT = 2;

  Odom(const sm::value_store::ValueStoreRef &vs) : PlanFragmentWithHome(vs, "Odom") {
    repetitions = vs.getInt("rep", repetitions);
    durationR = vs.getDouble("durR", durationR);
    durationT = vs.getDouble("durT", durationT);
    tV = vs.getDouble("tv", tV);
    rV = vs.getDouble("rv", rV);
  }

  void execute(CalibrationServer &cs, Driver &du, Logger & l) const override {
    HomingDriver d(*this, cs, du, l);

    cs.startDataCollection(motionSensors);

    for(int i = 0; i < repetitions; i ++){
      if(i % 2 == 0){
        d.rampTo(tV, rV, 2);
        sleep(durationR);
        d.rampTo(0, 0, 1);
        d.goHome();
        d.rampTo(tV, -rV, 2);
        sleep(durationR);
      } else {
        d.rampTo(tV, 0, 2);
        sleep(durationT);
        d.rampTo(0, 0, 1);
        d.goHome();
        d.rampTo(tV, 0, 2);
        sleep(durationT);
      }
      if(i != repetitions - 1){
        d.rampTo(0, 0, 1);
        d.goHome();
      }
    }
    cs.endDataCollection(motionSensors);
    d.rampTo(0, 0, 1);
  }

  void print(std::ostream & into) const override {
    PlanFragment::print(into);
    into << "(durationR=" << durationR << ", rep=" << repetitions << ", tV=" << tV << ", rV=" << rV << ", durationT=" << durationT << ")";
  }
};
}

PlanFragmentRegistry::PlanFragmentRegistry(){
  using namespace fragment;

  registerFragment<Test>();
  registerFragment<GoHome>();

#define AddFragment(X) registerFragment<X>(#X)
  AddFragment(Static);
  AddFragment(Dynamic);
  AddFragment(Straight);
  AddFragment(HomeTest);
  AddFragment(GetHome);
  AddFragment(Imu);
  AddFragment(ImuManual);
  AddFragment(Odom);
  AddFragment(TurningStatic);
  AddFragment(ForwardStatic);
#undef AddFragment
}

std::shared_ptr<PlanFragment> PlanFragmentRegistry::createFragment(const std::string key, const sm::value_store::ValueStoreRef& vs) {
  auto f = factories.find(key);
  if (f != factories.end()) {
    return std::shared_ptr<PlanFragment>(f->second(vs));
  } else {
    throw IllegalFragmentName("Illegal fragment name " + key);
  }
}


PlanFragmentRegistry & PlanFragmentRegistry::getInstance() {
  static PlanFragmentRegistry instance;
  return instance;
}


void Plan::loadFromString(std::string content) {
  sm::BoostPropertyTree pt;
  pt.loadXmlFromString(content);
  load(sm::value_store::PropertyTreeValueStore(pt));
}

void Plan::loadFromFile(std::string fileName) {
  sm::BoostPropertyTree pt;
  pt.loadXml(fileName);
  load(sm::value_store::PropertyTreeValueStore(pt));
}

void Plan::load(sm::value_store::ValueStoreRef in) {
  auto planNode = in.getChild("plan");
  id = planNode.getString("id");
  fragmentIndex = 0;
  fragments.clear();

  for(auto & child : planNode.getChildren()){
    if(child.getKey() != "id"){
      fragments.push_back(PlanFragmentRegistry::getInstance().createFragment(child.getKey(), child));
    }
  }
}

Plan::~Plan() {
}

bool aslam::calibration::plan::Plan::step() {
  if(fragmentIndex < fragments.size()){
    fragmentIndex ++;
  }
  return isFinished();
}

PlanFragment& aslam::calibration::plan::Plan::getCurrentFragment() {
  assert(fragmentIndex < fragments.size());
  return *fragments[fragmentIndex];
}

bool Plan::isFinished() const {
  return fragmentIndex >= fragments.size();
}

void Plan::print(std::ostream& into) const {
  into << "Plan(id=" << id << ")"  << std::endl;
  int i = 0;
  for(auto f : fragments){
    into << i << ":" << *f << std::endl;
  }
}

} /* namespace plan */
} /* namespace calibration */
} /* namespace aslam */

