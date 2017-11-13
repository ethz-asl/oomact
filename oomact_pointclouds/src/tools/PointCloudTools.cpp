#include <aslam/calibration/tools/PointCloudTools.h>

#include <aslam/calibration/model/sensors/Lidar2d.h>
#include <aslam/calibration/model/sensors/Lidar3d.h>
#include <aslam/calibration/model/sensors/StereoCamera.h>
namespace aslam {
namespace calibration {

bool is3dPointCloudSensor(const Sensor& s){
  return s.isA<Lidar3d>() || s.isA<StereoCamera>();
}
bool is2dPointCloudSensor(const Sensor& s){
  return s.isA<Lidar2d>();
}

} /* namespace calibration */
} /* namespace aslam */
