#include <aslam/calibration/SensorId.hpp>
#include <sm/kinematics/quaternion_algebra.hpp>

#include <aslam/calibration/test/MockMotionCaptureSource.h>

namespace aslam {
namespace calibration {
namespace test {

constexpr Timestamp MockMotionCaptureSource::StartTime;

MockMotionCaptureSource::~MockMotionCaptureSource() {
}

MotionCaptureSource::PoseStamped MockMotionCaptureSource::getPoseAt(Timestamp at) const {
  PoseStamped p;
  p.time = at;
  func(at, p);
  return p;
}

std::vector<MotionCaptureSource::PoseStamped> MockMotionCaptureSource::getPoses(Timestamp from, Timestamp till) const {
  Timestamp inc(1e-2);
  std::vector<PoseStamped> poses;
  for(auto t = from; t <= till + inc; t += inc){
    poses.emplace_back(getPoseAt(t));
  }
  return poses;
}

MockMotionCaptureSource MmcsStraightLine([](Timestamp now, MotionCaptureSource::PoseStamped & p){
  p.q = sm::kinematics::quatIdentity();
  p.p = Eigen::Vector3d::UnitX() * (now - MockMotionCaptureSource::StartTime);
});

MockMotionCaptureSource MmcsRotatingStraightLine([](Timestamp now, MotionCaptureSource::PoseStamped & p){
  const double deltaTime = now - MockMotionCaptureSource::StartTime;
  p.q = sm::kinematics::axisAngle2quat({deltaTime, 0, 0});
  p.p = Eigen::Vector3d::UnitX() * deltaTime;
});

} /* namespace test */
} /* namespace calibration */
} /* namespace aslam */

