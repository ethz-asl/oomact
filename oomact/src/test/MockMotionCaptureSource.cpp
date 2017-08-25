#include <aslam/calibration/SensorId.hpp>
#include <sm/kinematics/quaternion_algebra.hpp>

#include <aslam/calibration/test/MockMotionCaptureSource.h>

namespace aslam {
namespace calibration {
namespace test {

MockMotionCaptureSource::~MockMotionCaptureSource() {
}

MotionCaptureSource::PoseStamped MockMotionCaptureSource::getPoseAt(Timestamp start, Timestamp at) const {
  PoseStamped p;
  p.time = at;
  func(start, at, p);
  return p;
}

std::vector<MotionCaptureSource::PoseStamped> MockMotionCaptureSource::getPoses(Timestamp from, Timestamp till) const {
  Timestamp inc(1e-2);
  std::vector<PoseStamped> poses;
  for(auto t = from; t <= till + inc; t += inc){
    poses.emplace_back(getPoseAt(from, t));
  }
  return poses;
}

MockMotionCaptureSource mmcsStraightLine([](Timestamp start, Timestamp now, MotionCaptureSource::PoseStamped & p){
  p.q = sm::kinematics::quatIdentity();
  p.p = Eigen::Vector3d::UnitX() * (now - start);
});

MockMotionCaptureSource mmcsRotatingStraightLine([](Timestamp start, Timestamp now, MotionCaptureSource::PoseStamped & p){
  p.q = sm::kinematics::axisAngle2quat({double(now - start), 0, 0});
  p.p = Eigen::Vector3d::UnitX() * (now - start);
});

} /* namespace test */
} /* namespace calibration */
} /* namespace aslam */

