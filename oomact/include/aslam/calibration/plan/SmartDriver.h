#ifndef H1B320617_5D21_42DF_AEF7_E610B72C91F0
#define H1B320617_5D21_42DF_AEF7_E610B72C91F0
#include <atomic>
#include <future>
#include <mutex>
#include <thread>
#include <math.h>

#include <glog/logging.h>

#include <aslam/calibration/plan/Driver.h>
#include <aslam/calibration/plan/Plan.h>

namespace aslam {
namespace calibration {
namespace plan {

class SmartDriver {
 public:
  SmartDriver(Driver & d) :
    closing(false),
    dontRepeatLastVelocity(false),
    d(d)
  {
    stop();
    repeatCommandThread = std::thread([this](){
      while(!closing){
        sleep(0.1);
        {
          if(!dontRepeatLastVelocity){
            std::lock_guard<std::mutex> m(velocitesMutex);
            if(lastTv || lastRv){
              this->d.setVelocity(lastTv, lastRv);
            }
          }
        }
      }
    });
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
    CHECK(steps > 1);
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
    CHECK(vel >= 0.3);
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

    CHECK(vel >= 0.3);
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

} /* namespace plan */
} /* namespace calibration */
} /* namespace aslam */

#endif /* H1B320617_5D21_42DF_AEF7_E610B72C91F0 */
