#include <glog/logging.h>
#include <gflags/gflags.h>

#include <sm/boost/null_deleter.hpp>

#include <aslam/calibration/model/FrameGraphModel.h>
#include <aslam/calibration/model/sensors/PoseSensor.h>
#include <aslam/calibration/model/sensors/PositionSensor.h>
#include <aslam/calibration/model/PoseTrajectory.h>
#include "aslam/calibration/calibrator/CalibratorI.h"

#include "aslam/calibration/input/MotionCaptureSource.h"

using namespace aslam::calibration;
class SimpleModelFrame : public Frame, public NamedMinimal {
  using NamedMinimal::NamedMinimal;
};
SimpleModelFrame imu("imu");
SimpleModelFrame gps("gps");  // TODO: Fix such that trajectories are not w.r.t world

/* TODO: Get data from a rosbag in a separate package
class MockMotionCaptureSource : public MotionCaptureSource {
 public:
  MockMotionCaptureSource(std::function<void(Timestamp start, Timestamp now, PoseStamped & p)> func) : func(func){}
 private:
  virtual std::vector<PoseStamped> getPoses(sm::timing::NsecTime from, sm::timing::NsecTime till) const override {
    sm::timing::NsecTime inc = 1e7;
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
*/

int main(int argc, char **argv) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);
  google::SetStderrLogging(FLAGS_v > 0 ? google::INFO : google::WARNING);

  auto vs = ValueStoreRef::fromString(
      "Gravity{used=false}"
      "rovio{frame=gps, targetFrame=imu, rotation{used=true,yaw=0,pitch=0,roll=0},translation{used=true,x=0,y=0,z=0},delay/used=false}"
      "gps{frame=gps, targetFrame=imu, rotation/used=false,translation{used=false,x=0,y=0,z=0},delay/used=false}"
      "traj{frame=imu, referenceFrame=gps, initWithPoseMeasurements=true,McSensor=rovio,splines{knotsPerSecond=5,rotSplineOrder=4,rotFittingLambda=0.001,transSplineOrder=4,transFittingLambda=0.001}}"
      );

  FrameGraphModel model(vs, nullptr, {&imu, &gps});
  PoseSensor rovioSensor(model, "rovio", vs);
  PositionSensor gpsSensor(model, "gps", vs);
  PoseTrajectory traj(model, "traj", vs);
  model.addModulesAndInit(rovioSensor, gpsSensor, traj);

/* TODO read example data from file
    MockMotionCaptureSource mmcs([](Timestamp start, Timestamp now, MotionCaptureSource::PoseStamped & p){
      p.q = sm::kinematics::quatIdentity();
      p.p = Eigen::Vector3d::UnitX() * (now - start);
    });
*/
  auto vsCalib = ValueStoreRef::fromString(
      "acceptConstantErrorTerms=true\n"
      "timeBaseSensor=rovio\n"
    );

  auto c = createBatchCalibrator(vsCalib, std::shared_ptr<Model>(&model, sm::null_deleter()));

  c->addMeasurementTimestamp(0.0, rovioSensor); // add timestamps to determine the batch interval
  c->addMeasurementTimestamp(1.0, rovioSensor);

  c->calibrate();

  std::cout << "Finished calibration: gpsSensor.getTranslationToParent()=" << std::endl << gpsSensor.getTranslationToParent() << std::endl;
}
