#include <sm/kinematics/quaternion_algebra.hpp>

#include <aslam/calibration/CommonTypes.hpp>
#include <aslam/calibration/test/MockMotionCaptureSource.h>

namespace aslam {
namespace calibration {
namespace test {

MockMotionCaptureSource::~MockMotionCaptureSource() {
}

std::vector<MotionCaptureSource::PoseStamped> MockMotionCaptureSource::getPoses(Timestamp from, Timestamp till) const {
  Timestamp inc(1e-2);
  std::vector<PoseStamped> poses;
  for(auto t = from; t <= till + inc; t += inc){
    poses.resize(poses.size() + 1);
    poses.back().time = t;
    func(from, t, poses.back());
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
