#include <aslam/calibration/tools/PointCloudTools.h>

#include <aslam/calibration/model/sensors/Lidar2d.hpp>
#include <aslam/calibration/model/sensors/Lidar3d.hpp>
#include <aslam/calibration/model/sensors/StereoCamera.hpp>
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
