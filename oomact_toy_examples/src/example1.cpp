#include <glog/logging.h>

#include <sm/boost/null_deleter.hpp>

#include <aslam/calibration/model/FrameGraphModel.h>
#include <aslam/calibration/model/sensors/MotionCaptureSensor.h>
#include <aslam/calibration/model/PoseTrajectory.h>
#include "aslam/calibration/calibrator/CalibratorI.h"

#include "aslam/calibration/input/MotionCaptureSource.h"

using namespace aslam::calibration;
class SimpleModelFrame : public Frame, public NamedMinimal {
  using NamedMinimal::NamedMinimal;
};
SimpleModelFrame world("world");
SimpleModelFrame body("body");

class MockMotionCaptureSource : public MotionCaptureSource {
 public:
  MockMotionCaptureSource(std::function<void(Timestamp start, Timestamp now, PoseStamped & p)> func) : func(func){}
 private:
  virtual std::vector<PoseStamped> getPoses(Timestamp from, Timestamp till) const override {
    Timestamp inc(1e-2);
    std::vector<PoseStamped> poses;
    for(auto t = from; t <= till + inc; t += inc){
      poses.resize(poses.size() + 1);
      poses.back().time = t;
      func(from, t, poses.back());
    }
    return poses;
  }

  std::function<void(Timestamp start, Timestamp now, PoseStamped & p)> func;
};

int main(int argc, char **argv) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);
  google::SetStderrLogging(FLAGS_v > 0 ? google::INFO : google::WARNING);

  auto vs = ValueStoreRef::fromString(
      "Gravity{used=false}"
      "o{frame=world,rotation/used=false,translation/used=false,delay/used=false}"
      "a{frame=body,rotation/used=false,translation/used=false,delay/used=false}"
      "b{frame=body,rotation/used=false,translation{used=true,x=0,y=5,z=0},delay/used=false}"
      "traj{frame=body,referenceFrame=world,McSensor=a,initWithPoseMeasurements=true,splines{knotsPerSecond=5,rotSplineOrder=4,rotFittingLambda=0.001,transSplineOrder=4,transFittingLambda=0.001}}"
    );

  FrameGraphModel m(vs, nullptr, {&world, &body});
  MotionCaptureSystem observer(m, "o", vs);
  MotionCaptureSensor mcSensorA(observer, "a", vs);
  MotionCaptureSensor mcSensorB(observer, "b", vs);
  PoseTrajectory traj(m, "traj", vs);
  m.addModulesAndInit(observer,mcSensorA, mcSensorB, traj);

  MockMotionCaptureSource mmcs([](Timestamp start, Timestamp now, MotionCaptureSource::PoseStamped & p){
    p.q = sm::kinematics::quatIdentity();
    p.p = Eigen::Vector3d::UnitX() * (now - start);
  });

  mcSensorA.setMotionCaptureSource(std::shared_ptr<MotionCaptureSource>(&mmcs, sm::null_deleter()));
  mcSensorB.setMotionCaptureSource(std::shared_ptr<MotionCaptureSource>(&mmcs, sm::null_deleter()));

  auto vsCalib = ValueStoreRef::fromString(
      "acceptConstantErrorTerms=true\n"
      "timeBaseSensor=a\n"
    );

  auto c = createBatchCalibrator(vsCalib, std::shared_ptr<Model>(&m, sm::null_deleter()));

  c->addMeasurementTimestamp(0.0, mcSensorA); // add timestamps to determine the batch interval
  c->addMeasurementTimestamp(1.0, mcSensorA);

  c->calibrate();

  std::cout << "Finished calibration: mcSensorB.getTranslationToParent()=" << std::endl << mcSensorB.getTranslationToParent() << std::endl;
}
