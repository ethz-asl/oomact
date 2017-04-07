#include <aslam/calibration/plan/Plan.h>
#include <sm/value_store/ValueStore.hpp>
#include <sm/value_store/PropertyTreeValueStore.hpp>
#include <sm/BoostPropertyTree.hpp>

#include <memory>
#include <sstream>
#include <exception>
#include <unordered_map>
#include <thread>
#include <mutex>
#include <mutex>
#include <atomic>

#include <chrono>

#include <glog/logging.h>
#include <future>

#include "aslam/calibration/tools/tools.h"
#include <fstream>

namespace aslam {
namespace calibration {
namespace plan {

Plan::Plan() {}

void sleep(double seconds){
  std::this_thread::sleep_for(std::chrono::milliseconds((size_t)(seconds * 1e3)));
}

namespace fragment {

struct Test : public PlanFragment {
  Test(const sm::value_store::ValueStoreRef & vs) {
    val = vs.getString("");
  }
  void execute(CalibrationServer &, Driver &, Logger &) const override { }

  void print(std::ostream & into) const override {
    into << val;
  }

  std::string val;
};

struct SmartDriver {
  SmartDriver(Driver & d) :
    closing(false),
    dontRepeatLastVelocity(false),
    d(d)
  {
    stop();
    repeatCommandThread = std::move(std::thread([this](){
      while(!closing){
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        {
          if(!dontRepeatLastVelocity){
            std::lock_guard<std::mutex> m(velocitesMutex);
            if(lastTv || lastRv){
              this->d.setVelocity(lastTv, lastRv);
            }
          }
        }
      }
    }));
  }

  ~SmartDriver(){
    closing = true;
    repeatCommandThread.join();
  }

  void stop() {
    d.stop();
    lastTv = 0;
    lastRv = 0;
    sleep(0.1);
  }

  void markHome() {
    d.markHome();
  }

  template <typename F>
  void dontRepeatLastVelocityWhile(F f) {
    if(lastTv || lastRv){
      LOG(WARNING) << "Requested to not repeat nonzero velocity!";
    }
    dontRepeatLastVelocity = true;
    struct guard{
      decltype(dontRepeatLastVelocity) & myDontRepeatLastVelocity;
      ~guard(){
        myDontRepeatLastVelocity = false;
      }
    } g{dontRepeatLastVelocity};
    f();
  }

  void goHome() {
    dontRepeatLastVelocityWhile([&](){d.goHome();});
  }

  template <typename F>
  void goHomeAndDo(F f) {
    auto future = std::async(std::launch::async, f);
    goHome();
    future.get();
  }

  void goHomeAndStopLogging(Logger & l){
    goHomeAndDo([&](){sleep(0.1); l.stop();});
  }

  void setVelocity(double tv, double rv){
    std::lock_guard<std::mutex> m(velocitesMutex);
    d.setVelocity(lastTv = tv, lastRv = rv);
    VLOG(1) << "New tv = " << tv << ", rv = " << rv;
  }

  double logisticZeroOne(double t){
    return 1.0 / ( 1 + exp(-(t * 12 - 6)));
  }

  double interpol(double start, double end, double zeroOnePos){
    return (end - start) * zeroOnePos + start;
  }

  double addSocket(double current, double goal){
    if(fabs(current) < 0.1 && fabs(goal) > 0.2 && ((current >= 0) == (goal == 0))){
      current = goal >= 0 ? 0.1 : -0.1;
    }
    return current;
  }
  void rampTo(double tvGoal, double rvGoal, double duration, const double step = 0.05) {
    const double startTv = addSocket(lastTv, tvGoal);
    const double startRv = addSocket(lastRv, rvGoal);

    const int steps = duration / step;
    assert(steps > 1);
    const double orgDuration = duration;
    duration = steps * step;

    if(fabs(duration - orgDuration) > 0.001){
      LOG(WARNING) << "Truncating duration from " << orgDuration << " to " << duration;
    }

    sleep(step);

    for(double i = 1; i < steps; i++){
      const double v = logisticZeroOne((double) i / steps);
      setVelocity(interpol(startTv, tvGoal, v), interpol(startRv, rvGoal, v));
      sleep(step);
    }

    setVelocity(tvGoal, rvGoal);
    sleep(step);
  }

  void turn(double deltaAngle, double vel = .8)
  {
    LOG(INFO) << "Turning (deltaAngle=" << deltaAngle << ", vel=" << vel << ")";
    assert(vel >= 0.3);
    if(deltaAngle < 0){
      vel *= -1.0;
    }
    const double duration = (2 * M_PI * deltaAngle) / vel;
    rampTo(0.1, vel, 0.5);
    sleep(duration);
    rampTo(0, 0, 0.5);
    sleep(0.1);
  }

  void turnTo(double yaw){
    LOG(INFO) << "Turning to " << yaw;
    dontRepeatLastVelocityWhile([&](){d.turnTo(yaw);});
  }

  void forward(double deltaDistance, double vel = .4)
  {
    LOG(INFO) << "Moving forward (deltaDistance=" << deltaDistance << ", vel=" << vel << ")";

    assert(vel >= 0.3);
    if(deltaDistance < 0){
      vel *= -1.0;
    }

    const double duration = deltaDistance / vel;
    rampTo(vel, 0, 0.4);
    sleep(duration);
    rampTo(0, 0, 0.4);
    sleep(0.1);
  }
 private:
  std::thread repeatCommandThread;
  std::atomic<bool> closing, dontRepeatLastVelocity;
  std::mutex velocitesMutex;

  double lastTv = 0;
  double lastRv = 0;

  Driver & d;
};

void startLogging(const PlanFragment & frag, Logger& l) {
  std::string fragString = frag.toString();
  std::replace(fragString.begin(), fragString.end(), ' ', '_');
  std::replace(fragString.begin(), fragString.end(), '(', '_');
  std::replace(fragString.begin(), fragString.end(), ')', '_');
  l.start(fragString);
  if (!l.waitForLoggerToBecomeReady(1000)) {
    throw std::runtime_error("EuLogger timed out!");
  }
}

struct PlanFragmentWithHome : public PlanFragment{
  std::string homeFileName;
  std::string home;
  std::string name;
  PlanFragmentWithHome(const sm::value_store::ValueStoreRef &vs, std::string name) : name(name) {
    homeFileName = vs.getString("homeFile", std::string());
    if(homeFileName.empty()){
      homeFileName = getName() + ".home";
    }
    LOG(INFO) << "Using home file " << homeFileName << ".";

    std::ifstream homeFile(homeFileName);
    if(homeFile.is_open()){
      std::getline(homeFile, home);
      homeFile.close();
      LOG(INFO) << "Read home " << home << " from " << homeFileName;
    } else {
      LOG(INFO) << "Could not read home file (" << homeFileName << ")! Going to mark home instead.";
    }
  }

  virtual std::string getName() const {
    return name;
  }

  bool hasHome() const
  {
    return !home.empty();
  }

  const std::string& getHome() const
  {
    return home;
  }

  void print(std::ostream & into) const override {
    into << getName();
  }
};

struct HomingDriver : public SmartDriver {
  HomingDriver(const HomingDriver&) = delete;

  explicit HomingDriver(const PlanFragmentWithHome & planFrag, Driver & d, bool useHome = true) : HomingDriver(planFrag, nullptr, d, nullptr, useHome) {}
  explicit HomingDriver(const PlanFragmentWithHome & planFrag, CalibrationServer &cs, Driver & d, Logger & l, bool useHome = true) : HomingDriver(planFrag, &cs, d, &l, useHome) {}
  explicit HomingDriver(const PlanFragmentWithHome & planFrag, CalibrationServer * cs, Driver & d, Logger * l = nullptr, bool useHome = true) : SmartDriver(d), l(l), cs(cs), useHome(useHome) {
    if(useHome && planFrag.hasHome()){
      d.setHome(planFrag.getHome());
      goHome();
    } else {
      d.markHome();
    }
    if(l) startLogging(planFrag, *l);
  }
  ~HomingDriver(){
    stop();
    if(cs) cs->startCalibration();
    sleep(0.1);
    if(useHome){
      LOG(INFO) << "Going home";
      if(l) goHomeAndStopLogging(*l);
      else goHome();
    } else {
      if(l) l->stop();
    }
    if(cs) cs->waitForCalibration();
  }
 private:
  Logger * l;
  CalibrationServer * cs;
  bool useHome = true;
};

ModuleList velodyneSensors ({"Velodyne"});
ModuleList staticPointSensors ({"Velodyne", "BumbleBee", "LMS_FRONT", "LMS_REAR"});
ModuleList dynamicPointSensors ({"Velodyne", "LMS_DOWN", "UTM_FRONT", "dynamixel", "LMS_FRONT", "LMS_REAR"});
ModuleList motionSensors{"imu", "WheelOdometry", "ControlInput"};

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

struct Exp1 : public PlanFragment {
  Exp1(const sm::value_store::ValueStoreRef &) {}

  void execute(CalibrationServer &cs, Driver &du, Logger & l) const override {
    SmartDriver d(du);

    startLogging(*this, l);

    cs.startDataCollection(motionSensors);

    sleep(1);
    d.turn(0.25);
    sleep(1);
    d.turn(-0.25);
    sleep(1);

    d.stop();

    cs.endDataCollection(motionSensors);

    cs.startCalibration();

    sleep(1);

    l.stop();

    cs.waitForCalibration();
  }

  void print(std::ostream & into) const override {
    into << "Exp1";
  }
};

struct HomeTest : public PlanFragmentWithHome {
  HomeTest(const sm::value_store::ValueStoreRef &vs) : PlanFragmentWithHome(vs, "HomeTest")  {}

  void execute(CalibrationServer &cs, Driver &du, Logger & l) const override {
    HomingDriver d(*this, cs, du, l);
    d.forward(1);
    d.turn(0.5);
  }
};

struct GoHome : public PlanFragmentWithHome {
  GoHome(const sm::value_store::ValueStoreRef &vs) : PlanFragmentWithHome(vs, "GoHome")  {}

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
  GetHome(const sm::value_store::ValueStoreRef &vs) {
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
    into << "GetHome(targetName=" << targetName << ")";
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
    into << getName() << " (angle=" << turnAngle << ", turnModulo="<< turnModulo << ", dist=" << forwardDist << ", rep=" << repetitions << ")";
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
    into << getName() << "(dist=" << forwardDist << ", rep=" << repetitions << ")";
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
    into << getName() << "(yawInc=" << yawInc << ", duration=" << duration << ", rep=" << rep << ")";
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
    into << getName() << "(dist=" << dist << ", duration=" << duration << ", rep=" << rep << ")";
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
    into << getName() << "(duration=" << duration << ", rep=" << repetitions << ")";
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
    into << getName() << "(durationR=" << durationR << ", rep=" << repetitions << ", tV=" << tV << ", rV=" << rV << ", durationT=" << durationT << ")";
  }
};


}

#define AddFragment(X) factories[#X] = [](const sm::value_store::ValueStoreRef & vs){ return new fragment::X(vs); };
template <typename FragmentT>
struct Registry {

  struct IllegalFragmentName : public std::runtime_error {
    using std::runtime_error::runtime_error;
  };

  std::unordered_map<std::string, std::function<FragmentT*(const sm::value_store::ValueStoreRef & vs)>> factories;

  Registry(){
    using namespace fragment;
    AddFragment(Test);
    AddFragment(Exp1);
    AddFragment(Static);
    AddFragment(Dynamic);
    AddFragment(Straight);
    AddFragment(HomeTest);
    AddFragment(GetHome);
    AddFragment(GoHome);
    AddFragment(Imu);
    AddFragment(ImuManual);
    AddFragment(Odom);
    AddFragment(TurningStatic);
    AddFragment(ForwardStatic);
  }

  std::shared_ptr<FragmentT> createFragment(const std::string key, const sm::value_store::ValueStoreRef & vs){
    auto f = factories.find(key);

    if(f != factories.end()){
      return std::shared_ptr<FragmentT>(f->second(vs));
    } else {
      throw IllegalFragmentName("Illegal fragment name " + key);
    }
  }
};

Registry<PlanFragment> registry;


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
      fragments.push_back(registry.createFragment(child.getKey(), child));
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

